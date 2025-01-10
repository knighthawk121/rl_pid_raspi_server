#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rl_pid_uros.action import TunePID
import RPi.GPIO as GPIO
import time
from threading import Lock
import math
import socket
import netifaces as ni
import asyncio

class NetworkConfig:
    @staticmethod
    def get_ip_address():
        # Get IP address of the wireless interface
        try:
            # Try wlan0 first (most common WiFi interface name)
            ip = ni.ifaddresses('wlan0')[ni.AF_INET][0]['addr']
        except (ValueError, KeyError):
            try:
                # Try wifi0 as fallback
                ip = ni.ifaddresses('wifi0')[ni.AF_INET][0]['addr']
            except (ValueError, KeyError):
                # If both fail, use hostname resolution
                hostname = socket.gethostname()
                ip = socket.gethostbyname(hostname)
        return ip

    @staticmethod
    def check_network_connection():
        try:
            # Try to connect to Google's DNS to check internet connectivity
            socket.create_connection(("8.8.8.8", 53), timeout=3)
            return True
        except OSError:
            return False
class MotorControl:
    def __init__(self):
        # Pin Definitions
        self.ENCA = 16  # Encoder A
        self.ENCB = 18  # Encoder B
        self.PWM = 12   # PWM Pin
        self.IN2 = 13   # Motor Input 2
        self.IN1 = 11   # Motor Input 1
        self.STBY = 15  # Standby Pin

        # PID parameters
        self.position = 0
        self.target = 0
        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.01
        self.eprev = 0
        self.eintegral = 0
        self.prev_time = time.time()
        self.goal_active = False
        
        # Constants
        self.MIN_DELTA_T = 0.001
        self.EPSILON = 1.0e-6
        self.DEADBAND = 5
        self.MAX_INTEGRAL = 100.0
        self.MAX_PWM_VALUE = 100.0
        
        self.position_lock = Lock()
        self.pid_lock = Lock()
        
        # Initialize encoder state
        self.prev_state = 0
        self.encoder_running = False
        
        self.setup_gpio()
        self.start_encoder_polling()

    def setup_gpio(self):
        # No cleanup needed at start, we'll handle it at shutdown
        try:
            GPIO.cleanup()
        except:
            pass
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        
        # Setup encoder pins
        GPIO.setup(self.ENCA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.ENCB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Setup motor control pins
        GPIO.setup(self.PWM, GPIO.OUT)
        GPIO.setup(self.IN1, GPIO.OUT)
        GPIO.setup(self.IN2, GPIO.OUT)
        GPIO.setup(self.STBY, GPIO.OUT)
        
        # Initialize outputs
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.STBY, GPIO.LOW)
        
        # Setup PWM
        self.pwm = GPIO.PWM(self.PWM, 1000)
        self.pwm.start(0)

    def poll_encoder(self):
        """Polling function to run in separate thread"""
        while self.encoder_running:
            current_state = GPIO.input(self.ENCA)
            if current_state != self.prev_state and current_state == 1:
                b = GPIO.input(self.ENCB)
                with self.position_lock:
                    self.position = self.position + 1 if b > 0 else self.position - 1
            self.prev_state = current_state
            time.sleep(0.0001)  # 10kHz polling rate

    def start_encoder_polling(self):
        """Start the encoder polling thread"""
        self.encoder_running = True
        import threading
        self.encoder_thread = threading.Thread(target=self.poll_encoder, daemon=True)
        self.encoder_thread.start()

    def cleanup(self):
        """Clean up GPIO and stop polling"""
        self.encoder_running = False
        if hasattr(self, 'encoder_thread'):
            self.encoder_thread.join(timeout=1.0)
        self.pwm.stop()
        GPIO.cleanup()

    # Rest of the methods remain the same
    def get_position(self):
        with self.position_lock:
            return self.position

    def set_motor(self, direction, pwm_value):
        GPIO.output(self.STBY, GPIO.HIGH)
        self.pwm.ChangeDutyCycle(pwm_value)
        GPIO.output(self.IN1, GPIO.HIGH if direction == 1 else GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW if direction == 1 else GPIO.HIGH)

class PIDActionServer(Node):
    def __init__(self):
        super().__init__('pid_action_server')
        
        # Network configuration
        self.network_config = NetworkConfig()
        self.ip_address = self.network_config.get_ip_address()
        
        self.get_logger().info(f'Server IP address: {self.ip_address}')
        
        if not self.network_config.check_network_connection():
            self.get_logger().warning('No network connection detected!')
        
        self.motor = MotorControl()
        self.callback_group = ReentrantCallbackGroup()
            
        self._action_server = ActionServer(
            self,
            TunePID,
            'tune_pid_action',
            self.execute_callback,
            callback_group=self.callback_group
            #QoSProfile=qos_profile
        )

         
        # Create timer for motor control loop
        self.create_timer(0.02, self.motor_control_callback)  # 50Hz control loop    


        # Initialize asyncio event loop
        self.loop = asyncio.get_event_loop()
        if not self.loop.is_running():
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            

        self.get_logger().info('PID Action Server has been started')
        self.get_logger().info(f'Listening on {self.ip_address}')

    def check_network_status(self):
        if not self.network_config.check_network_connection():
            self.get_logger().warning('Network connection lost!')
        
    def motor_control_callback(self):
        
        if not self.motor.goal_active:
            return None, None

        with self.motor.pid_lock:
            # Get current time and position
            curr_time = time.time()
            delta_t = curr_time - self.motor.prev_time
            
            if delta_t <= 0:
                self.get_logger().error('Invalid time delta')
                return None, None
                
            delta_t = max(delta_t, self.motor.MIN_DELTA_T)
            self.motor.prev_time = curr_time

            current_pos = self.motor.get_position()
            error = self.motor.target - current_pos

            # PID calculations
            dedt = (error - self.motor.eprev) / (delta_t + self.motor.EPSILON)
            self.motor.eintegral += error * delta_t
            self.motor.eintegral = max(min(self.motor.eintegral, 
                                         self.motor.MAX_INTEGRAL), 
                                     -self.motor.MAX_INTEGRAL)

            # Calculate control signal
            pid_output = ((self.motor.kp * error) + 
                         (self.motor.ki * self.motor.eintegral) + 
                         (self.motor.kd * dedt))

            # Convert to motor commands
            power = min(abs(pid_output), self.motor.MAX_PWM_VALUE)
            direction = 1 if pid_output >= 0 else -1

            # Apply deadband
            if power < self.motor.DEADBAND:
                power = 0
                direction = 0

            # Update motor
            self.motor.set_motor(direction, power)
            self.motor.eprev = error

            return float(error), int(current_pos)

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = TunePID.Feedback()
        result = TunePID.Result()
      
        try:
            # Update PID parameters
            with self.motor.pid_lock:
                self.motor.kp = float(goal_handle.request.kp)
                self.motor.ki = float(goal_handle.request.ki)
                self.motor.kd = float(goal_handle.request.kd)
                self.motor.target = int(goal_handle.request.target_position)
                self.motor.eprev = 0
                self.motor.eintegral = 0
                self.motor.goal_active = True

            stable_count = 0
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.motor.goal_active = False
                    result.final_error = float(feedback_msg.current_error)
                    return result

                error, current_pos = self.motor_control_callback()
                
                if error is not None and current_pos is not None:
                    # Update feedback
                    feedback_msg.current_error = float(error)
                    feedback_msg.current_position = int(current_pos)
                    feedback_msg.target_position = float(self.motor.target)
                    goal_handle.publish_feedback(feedback_msg)

                    if abs(error) < self.motor.DEADBAND:
                        stable_count += 1
                        if stable_count >= 5:
                            break
                    else:
                        stable_count = 0

                await asyncio.sleep(0.1)  # 10Hz feedback rate

            self.motor.goal_active = False
            result.final_error = float(error)
            goal_handle.succeed()
            
            return result
        
        except Exception as e:
            self.get_logger().error(f'Error in execute callback: {str(e)}')
            self.motor.goal_active = False
            goal_handle.abort()
            result.final_error = 0.0
            return result 

    def cleanup(self):
        self.motor.cleanup()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        # Create and initialize the action server
        action_server = PIDActionServer()
        
        # Create executor
        executor = MultiThreadedExecutor()
        executor.add_node(action_server)

        # Run the executor in a separate thread
        import threading
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        # Run the asyncio event loop in the main thread
        loop = asyncio.get_event_loop()
        if not loop.is_running():
            loop.run_forever()
            
    except KeyboardInterrupt:
        GPIO.cleanup()
    finally:
        # Cleanup
        if 'action_server' in locals():
            action_server.destroy_node()
        rclpy.shutdown()
        
        # Stop the event loop
        loop = asyncio.get_event_loop()
        if loop.is_running():
            loop.stop()

if __name__ == '__main__':
    main()