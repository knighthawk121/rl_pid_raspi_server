#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time
from threading import Lock
import signal
import sys
import math
import numpy as np

# Pin Definitions
ENCA = 16  # Encoder A
ENCB = 18  # Encoder B
PWM = 12   # PWM Pin
IN2 = 13   # Motor Input 2
IN1 = 11   # Motor Input 1
STBY = 15  # Standby Pin

class PIDMotorControl:
    def __init__(self, use_interrupts=False):
        # Initialize GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.cleanup()
        
        # PID variables
        self.target = 0
        self.encoder_pos = 0
        self.prev_time = time.time_ns() / 1e9
        self.prev_error = 0
        self.integral = 0
        
        # Adjusted PID constants for better control
        self.kp = 1.0    # Reduced proportional gain
        self.kd = 0.001  # Reduced derivative gain
        self.ki = 0.0    # Start with no integral gain
        
        # Added maximum speed limit
        self.max_speed = 65  # Limit maximum speed to 30% duty cycle
        
        # Control variables
        self.running = True
        self.encoder_lock = Lock()
        
        # Setup pins
        GPIO.setup(ENCA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(ENCB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(PWM, GPIO.OUT)
        GPIO.setup(IN1, GPIO.OUT)
        GPIO.setup(IN2, GPIO.OUT)
        GPIO.setup(STBY, GPIO.OUT)
        
        # Initialize PWM
        self.pwm = GPIO.PWM(PWM, 1000)  # 1000 Hz frequency
        self.pwm.start(0)
        
        # Setup encoder reading
        if use_interrupts:
            try:
                GPIO.remove_event_detect(ENCA)
                time.sleep(0.1)
                GPIO.add_event_detect(ENCA, GPIO.RISING, 
                                    callback=self.encoder_callback,
                                    bouncetime=50)
                print("Using interrupt-based encoder reading")
            except Exception as e:
                print(f"Failed to set up encoder interrupt: {str(e)}")
                print("Falling back to polling mode")
                use_interrupts = False
        
        if not use_interrupts:
            from threading import Thread
            self.poll_thread = Thread(target=self.poll_encoder, daemon=True)
            self.poll_thread.start()
            print("Using polling-based encoder reading")

    def pid_control(self):
        # Get current time and position
        current_time = time.time_ns() / 1e9
        with self.encoder_lock:
            current_pos = self.encoder_pos
        
        # Time difference
        dt = current_time - self.prev_time
        if dt <= 0:  # Avoid division by zero
            return current_pos, self.target, 0, 0
        
        # Error calculation
        error = current_pos - self.target
         
        # Derivative (with filtering)
        derivative = (error - self.prev_error) / dt
        
        # Integral (with anti-windup)
        if abs(error) < 100:  # Only integrate when error is small
            self.integral += error * dt
        
        # Limit integral term
        self.integral = max(min(self.integral, 100), -100)
        
        # PID control signal
        u = (self.kp * error + 
             self.kd * derivative + 
             self.ki * self.integral)
        
        # Convert control signal to motor commands with improved scaling
        power = min(abs(u), self.max_speed)  # Limit to max_speed
        
        # Add deadband compensation
        if power < 5 and power > 0:  # If power is very low but not zero
            power = 5  # Minimum power to overcome friction
        
        direction = 1 if u >= 0 else -1
        
        # If error is very small, stop the motor
        if abs(error) < 5:
            power = 0
            direction = 0
        
        # Update motor
        self.set_motor(direction, power)
        
        # Store previous values
        self.prev_time = current_time
        self.prev_error = error
        
        return current_pos, self.target, power, direction

    def test_pid_sine(self, duration=10, amplitude=100, frequency=0.2):
        """Test PID control with sine wave target"""
        print("\nTesting PID control with sine wave target")
        print("Press Ctrl+C to stop")
        print(f"Using max speed of {self.max_speed}%")
        
        start_time = time.time()
        
        try:
            while (time.time() - start_time) < duration:
                # Calculate target position as sine wave
                t = time.time() - start_time
                self.target = amplitude * math.sin(2 * math.pi * frequency * t)
                
                # Run PID control
                pos, target, power, direction = self.pid_control()
                
                # Print status
                print(f"Time: {t:.2f}s, Target: {target:.1f}, Position: {pos}, "
                      f"Power: {power:.1f}%, Dir: {direction}", end='\r')
                
                time.sleep(0.02)  # 50Hz update rate
                
        except KeyboardInterrupt:
            print("\nTest stopped by user")
        finally:
            self.set_motor(0, 0)

    def test_pid_step(self, duration=5, step_size=100):
        """Test PID control with step input"""
        print("\nTesting PID control with step input")
        print("Press Ctrl+C to stop")
        
        start_time = time.time()
        step_time = start_time
        step_state = True
        
        try:
            while (time.time() - start_time) < duration:
                # Toggle target every 2 second
                if (time.time() - step_time) >= 1.0:
                    step_state = not step_state
                    self.target = step_size if step_state else 0
                    step_time = time.time()
                
                # Run PID control
                pos, target, power, direction = self.pid_control()
                
                # Print status
                t = time.time() - start_time
                print(f"Time: {t:.2f}s, Target: {target:.1f}, Position: {pos}, "
                      f"Power: {power:.1f}%, Dir: {direction}", end='\r')
                
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\nTest stopped by user")
        finally:
            self.set_motor(0, 0)

    def set_pid_constants(self, kp=None, ki=None, kd=None):
        """Update PID constants"""
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
        print(f"PID constants updated - Kp: {self.kp}, Ki: {self.ki}, Kd: {self.kd}")

    # Encoder reading methods
    def encoder_callback(self, channel):
        with self.encoder_lock:
            b = GPIO.input(ENCB)
            if b > 0:
                self.encoder_pos += 1
            else:
                self.encoder_pos -= 1

    def poll_encoder(self):
        last_a = GPIO.input(ENCA)
        while self.running:
            a = GPIO.input(ENCA)
            b = GPIO.input(ENCB)
            
            if a != last_a and a == 1:
                with self.encoder_lock:
                    if b:
                        self.encoder_pos += 1
                    else:
                        self.encoder_pos -= 1
            
            last_a = a
            time.sleep(0.0001)

    def set_motor(self, direction, speed):
        """
        Set motor direction and speed
        direction: 1 for forward, -1 for reverse, 0 for stop
        speed: PWM duty cycle (0-100)
        """
        GPIO.output(STBY, GPIO.HIGH)
        
        if direction == 1:
            GPIO.output(IN1, GPIO.HIGH)
            GPIO.output(IN2, GPIO.LOW)
        elif direction == -1:
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.HIGH)
        else:
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.LOW)
        
        self.pwm.ChangeDutyCycle(speed)

    def cleanup(self):
        self.running = False
        time.sleep(0.1)
        self.set_motor(0, 0)
        self.pwm.stop()
        GPIO.cleanup()

def main():
    print("PID Motor Control Test Program")
    print("Pin Configuration:")
    print(f"ENCA: GPIO{ENCA}")
    print(f"ENCB: GPIO{ENCB}")
    print(f"PWM:  GPIO{PWM}")
    print(f"IN2:  GPIO{IN2}")
    print(f"IN1:  GPIO{IN1}")
    print(f"STBY: GPIO{STBY}")
    
    try:
        controller = PIDMotorControl(use_interrupts=True)
    except Exception as e:
        print(f"Failed to initialize with interrupts: {e}")
        print("Retrying with polling mode...")
        controller = PIDMotorControl(use_interrupts=False)
    
    while True:
        print("\nSelect test option:")
        print("1. Run sine wave test")
        print("2. Run step response test")
        print("3. Set PID constants")
        print("4. Exit")
        
        choice = input("Enter choice (1-4): ")
        
        if choice == '1':
            controller.test_pid_sine()
        elif choice == '2':
            controller.test_pid_step()
        elif choice == '3':
            kp = float(input("Enter Kp (current: {}): ".format(controller.kp)))
            ki = float(input("Enter Ki (current: {}): ".format(controller.ki)))
            kd = float(input("Enter Kd (current: {}): ".format(controller.kd)))
            controller.set_pid_constants(kp, ki, kd)
        elif choice == '4':
            print("Exiting...")
            controller.cleanup()
            break
        else:
            print("Invalid choice, please try again")

if __name__ == "__main__":
    main()