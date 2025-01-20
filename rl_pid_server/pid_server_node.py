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
import sys

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
        
        #new status monitoring parameters
        self.velocity = 0.0
        self.last_position = 0
        self.last_velocity_time = time.time()
        self.stable_time = 0.0
        self.stall_count = 0
        self.MAX_STALL_COUNT = 10  # Number of consecutive low-velocity readings to detect stall
        self.VELOCITY_THRESHOLD = 1.0  # Threshold for stall detection
        self.STABLE_TIME_THRESHOLD = 0.5  # Time error must be stable for completion
        self.ERROR_STABILITY_THRESHOLD = 2.0  # Error range considered stable


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

    def update_velocity(self):
        current_time = time.time()
        delta_t = current_time - self.last_velocity_time
        if delta_t > 0:
            with self.position_lock:
                current_pos = self.position
                self.velocity = (current_pos - self.last_position) / delta_t
                self.last_position = current_pos
                self.last_velocity_time = current_time
    
    def check_motor_status(self, error):
        # Update velocity
        self.update_velocity()
        
        # Check for stall condition
        if abs(self.velocity) < self.VELOCITY_THRESHOLD:
            self.stall_count += 1
        else:
            self.stall_count = 0
            
        is_stalled = self.stall_count >= self.MAX_STALL_COUNT
        
        # Check for stable error
        if abs(error) < self.ERROR_STABILITY_THRESHOLD:
            if self.stable_time == 0:
                self.stable_time = time.time()
        else:
            self.stable_time = 0
            
        is_stable = (self.stable_time > 0 and 
                    (time.time() - self.stable_time) >= self.STABLE_TIME_THRESHOLD)
                    
        return is_stalled, is_stable, self.velocity

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
        )

        self.feedback_msg = TunePID.Feedback()
        # Create timer for motor control loop
        self.last_control_update = self.get_clock().now()
        self.create_timer(0.02, self.motor_control_callback)  # 50Hz control loop    

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
            curr_time = self.get_clock().now()
            delta_t = (curr_time - self.last_control_update).nanoseconds / 1e9
            self.last_control_update = curr_time
            
            if delta_t <= 0:
                self.get_logger().error('Invalid time delta')
                return None, None
                
            delta_t = max(delta_t, self.motor.MIN_DELTA_T)
            

            current_pos = self.motor.get_position()
            error = current_pos - self.motor.target 

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
            is_stalled, is_stable, velocity = self.motor.check_motor_status(error)

            return float(error), int(current_pos), is_stalled, is_stable, velocity

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Received new goal request - kp: {goal_handle.request.kp}, '
                              f'ki: {goal_handle.request.ki}, kd: {goal_handle.request.kd}, '
                              f'target: {goal_handle.request.target_position}')
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
                self.motor.stable_time = 0
                self.motor.stall_count = 0

            rate = self.create_rate(10)  # 10 Hz feedback rate
            max_iterations = 100  # 10 seconds maximum
            iteration_count = 0

            while rclpy.ok() and iteration_count < max_iterations:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.motor.goal_active = False
                    result.final_error = float(feedback_msg.current_error)
                    return result

                error, current_pos, is_stalled, is_stable, velocity = self.motor_control_callback()
                
                if error is not None and current_pos is not None:
                    # Update feedback
                    feedback_msg.current_error = float(error)
                    feedback_msg.current_position = int(current_pos)
                    feedback_msg.target_position = int(self.motor.target)
                    feedback_msg.motor_stopped = is_stalled
                    feedback_msg.error_stable = is_stable
                    feedback_msg.velocity = float(velocity)
                    goal_handle.publish_feedback(feedback_msg)

                    # Check completion conditions
                    if is_stable:
                        self.get_logger().info('Goal completed - Error stabilized')
                        break
                    elif is_stalled and not is_stable:
                        self.get_logger().warn('Goal completed - Motor stalled')
                        break
                        
                iteration_count += 1
                if iteration_count >= max_iterations:
                    self.get_logger().warn('Goal completed - Maximum time exceeded')

                rate.sleep()

            self.motor.goal_active = False
            result.final_error = float(error)
            goal_handle.succeed()
            self.get_logger().info(f'Goal completed with final error: {error}')
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
    # Guard to prevent multiple initializations
    if not rclpy.ok():
        try:
            rclpy.init(args=args)
        except RuntimeError as e:
            print(f"Error initializing ROS 2: {e}")
            return 1
    
    try:
        # Create and initialize the action server
        action_server = PIDActionServer()
        
        # Create executor
        executor = MultiThreadedExecutor()
        executor.add_node(action_server)

        try:
            executor.spin()
        except KeyboardInterrupt:
            print("Received keyboard interrupt, shutting down...")
        finally:
            # Clean up GPIO first
            GPIO.cleanup()
            # Then clean up ROS 2 resources
            executor.shutdown()
            action_server.cleanup()
            action_server.destroy_node()
            
    except Exception as e:
        print(f"Error during execution: {e}")
        return 1
    finally:
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"Error during shutdown: {e}")
            return 1
    
    return 0

if __name__ == '__main__':
    sys.exit(main())