#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time
from threading import Lock
import signal
import sys
import math
import numpy as np

# Pin Definitions
# Motor A
ENCA = 16
ENCB = 18
PWMA = 12
IN2A = 13
IN1A = 11

# Motor B
ENCA2 = 29
ENCB2 = 31
PWMB = 35
IN1B = 36
IN2B = 37

STBY = 15  # Shared standby pin

class DualPIDMotorControl:
    def __init__(self, use_interrupts=False):
        # Initialize GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.cleanup()
        
        # PID variables for both motors
        self.target_a = 0
        self.target_b = 0
        self.encoder_pos_a = 0
        self.encoder_pos_b = 0
        self.prev_time_a = time.time_ns() / 1e9
        self.prev_time_b = time.time_ns() / 1e9
        self.prev_error_a = 0
        self.prev_error_b = 0
        self.integral_a = 0
        self.integral_b = 0
        
        # PID constants for both motors
        self.kp_a = 1.0
        self.kd_a = 0.001
        self.ki_a = 0.0
        
        self.kp_b = 1.0
        self.kd_b = 0.001
        self.ki_b = 0.0
        
        # Speed limits
        self.max_speed_a = 65
        self.max_speed_b = 65
        
        # Control variables
        self.running = True
        self.encoder_lock_a = Lock()
        self.encoder_lock_b = Lock()
        
        # Setup pins for Motor A
        GPIO.setup(ENCA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(ENCB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(PWMA, GPIO.OUT)
        GPIO.setup(IN1A, GPIO.OUT)
        GPIO.setup(IN2A, GPIO.OUT)
        
        # Setup pins for Motor B
        GPIO.setup(ENCA2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(ENCB2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(PWMB, GPIO.OUT)
        GPIO.setup(IN1B, GPIO.OUT)
        GPIO.setup(IN2B, GPIO.OUT)
        
        # Setup standby pin
        GPIO.setup(STBY, GPIO.OUT)
        
        # Initialize PWM for both motors
        self.pwm_a = GPIO.PWM(PWMA, 1000)
        self.pwm_b = GPIO.PWM(PWMB, 1000)
        self.pwm_a.start(0)
        self.pwm_b.start(0)
        
        if use_interrupts:
            try:
                # Setup interrupts for Motor A
                GPIO.remove_event_detect(ENCA)
                GPIO.add_event_detect(ENCA, GPIO.RISING, 
                                    callback=lambda x: self.encoder_callback(x, 'A'),
                                    bouncetime=50)
                                    
                # Setup interrupts for Motor B
                GPIO.remove_event_detect(ENCA2)
                GPIO.add_event_detect(ENCA2, GPIO.RISING, 
                                    callback=lambda x: self.encoder_callback(x, 'B'),
                                    bouncetime=50)
                print("Using interrupt-based encoder reading")
            except Exception as e:
                print(f"Failed to set up encoder interrupts: {str(e)}")
                print("Falling back to polling mode")
                use_interrupts = False
        
        if not use_interrupts:
            from threading import Thread
            self.poll_thread = Thread(target=self.poll_encoders, daemon=True)
            self.poll_thread.start()
            print("Using polling-based encoder reading")

    def pid_control(self, motor):
        """PID control for specified motor"""
        current_time = time.time_ns() / 1e9
        
        if motor == 'A':
            with self.encoder_lock_a:
                current_pos = self.encoder_pos_a
            target = self.target_a
            prev_time = self.prev_time_a
            prev_error = self.prev_error_a
            integral = self.integral_a
            kp, ki, kd = self.kp_a, self.ki_a, self.kd_a
            max_speed = self.max_speed_a
        else:  # motor == 'B'
            with self.encoder_lock_b:
                current_pos = self.encoder_pos_b
            target = self.target_b
            prev_time = self.prev_time_b
            prev_error = self.prev_error_b
            integral = self.integral_b
            kp, ki, kd = self.kp_b, self.ki_b, self.kd_b
            max_speed = self.max_speed_b
        
        dt = current_time - prev_time
        if dt <= 0:
            return current_pos, target, 0, 0
        
        error = current_pos - target
        derivative = (error - prev_error) / dt
        
        if abs(error) < 100:
            integral += error * dt
        
        integral = max(min(integral, 100), -100)
        
        u = (kp * error + kd * derivative + ki * integral)
        
        power = min(abs(u), max_speed)
        
        if power < 5 and power > 0:
            power = 5
        
        direction = 1 if u >= 0 else -1
        
        if abs(error) < 5:
            power = 0
            direction = 0
        
        self.set_motor(motor, direction, power)
        
        if motor == 'A':
            self.prev_time_a = current_time
            self.prev_error_a = error
            self.integral_a = integral
        else:
            self.prev_time_b = current_time
            self.prev_error_b = error
            self.integral_b = integral
        
        return current_pos, target, power, direction

    def test_pid_sine(self, duration=10, amplitude=100, frequency=0.2):
        """Test PID control with sine wave target for both motors"""
        print("\nTesting PID control with sine wave target (both motors)")
        print("Press Ctrl+C to stop")
        print(f"Using max speed of {self.max_speed_a}% for Motor A")
        print(f"Using max speed of {self.max_speed_b}% for Motor B")
        
        start_time = time.time()
        
        try:
            while (time.time() - start_time) < duration:
                t = time.time() - start_time
                
                # Set targets with phase difference between motors
                self.target_a = amplitude * math.sin(2 * math.pi * frequency * t)
                self.target_b = amplitude * math.sin(2 * math.pi * frequency * t + math.pi)  # 180° phase shift
                
                # Run PID control for both motors
                pos_a, target_a, power_a, dir_a = self.pid_control('A')
                pos_b, target_b, power_b, dir_b = self.pid_control('B')
                
                print(f"Time: {t:.2f}s | Motor A - Target: {target_a:.1f}, Pos: {pos_a}, "
                      f"Power: {power_a:.1f}%, Dir: {dir_a} | Motor B - Target: {target_b:.1f}, "
                      f"Pos: {pos_b}, Power: {power_b:.1f}%, Dir: {dir_b}", end='\r')
                
                time.sleep(0.02)
                
        except KeyboardInterrupt:
            print("\nTest stopped by user")
        finally:
            self.set_motor('A', 0, 0)
            self.set_motor('B', 0, 0)

    def test_pid_step(self, duration=5, step_size=100):
        """Test PID control with step input for both motors"""
        print("\nTesting PID control with step input (both motors)")
        print("Press Ctrl+C to stop")
        
        start_time = time.time()
        step_time = start_time
        step_state = True
        
        try:
            while (time.time() - start_time) < duration:
                if (time.time() - step_time) >= 1.0:
                    step_state = not step_state
                    self.target_a = step_size if step_state else 0
                    self.target_b = step_size if not step_state else 0  # Opposite phase
                    step_time = time.time()
                
                pos_a, target_a, power_a, dir_a = self.pid_control('A')
                pos_b, target_b, power_b, dir_b = self.pid_control('B')
                
                t = time.time() - start_time
                print(f"Time: {t:.2f}s | Motor A - Target: {target_a:.1f}, Pos: {pos_a}, "
                      f"Power: {power_a:.1f}%, Dir: {dir_a} | Motor B - Target: {target_b:.1f}, "
                      f"Pos: {pos_b}, Power: {power_b:.1f}%, Dir: {dir_b}", end='\r')
                
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\nTest stopped by user")
        finally:
            self.set_motor('A', 0, 0)
            self.set_motor('B', 0, 0)

    def set_pid_constants(self, motor, kp=None, ki=None, kd=None):
        """Update PID constants for specified motor"""
        if motor == 'A':
            if kp is not None:
                self.kp_a = kp
            if ki is not None:
                self.ki_a = ki
            if kd is not None:
                self.kd_a = kd
            print(f"Motor A PID constants - Kp: {self.kp_a}, Ki: {self.ki_a}, Kd: {self.kd_a}")
        else:  # motor == 'B'
            if kp is not None:
                self.kp_b = kp
            if ki is not None:
                self.ki_b = ki
            if kd is not None:
                self.kd_b = kd
            print(f"Motor B PID constants - Kp: {self.kp_b}, Ki: {self.ki_b}, Kd: {self.kd_b}")

    def encoder_callback(self, channel, motor):
        if motor == 'A':
            with self.encoder_lock_a:
                b = GPIO.input(ENCB)
                if b > 0:
                    self.encoder_pos_a += 1
                else:
                    self.encoder_pos_a -= 1
        else:  # motor == 'B'
            with self.encoder_lock_b:
                b = GPIO.input(ENCB2)
                if b > 0:
                    self.encoder_pos_b += 1
                else:
                    self.encoder_pos_b -= 1

    def poll_encoders(self):
        last_a = GPIO.input(ENCA)
        last_b = GPIO.input(ENCA2)
        
        while self.running:
            # Poll Motor A
            a = GPIO.input(ENCA)
            b = GPIO.input(ENCB)
            if a != last_a and a == 1:
                with self.encoder_lock_a:
                    if b:
                        self.encoder_pos_a += 1
                    else:
                        self.encoder_pos_a -= 1
            last_a = a
            
            # Poll Motor B
            a2 = GPIO.input(ENCA2)
            b2 = GPIO.input(ENCB2)
            if a2 != last_b and a2 == 1:
                with self.encoder_lock_b:
                    if b2:
                        self.encoder_pos_b += 1
                    else:
                        self.encoder_pos_b -= 1
            last_b = a2
            
            time.sleep(0.0001)

    def test_encoder_a(self, duration=5):
        """Test encoder A reading"""
        print("\nTesting Encoder A")
        print("Rotate the motor shaft and watch the encoder counts")
        print("Press Ctrl+C to stop")
        
        start_time = time.time()
        last_pos = 0
        
        try:
            while (time.time() - start_time) < duration:
                with self.encoder_lock_a:
                    current_pos = self.encoder_pos_a
                
                if current_pos != last_pos:
                    print(f"Encoder A Position: {current_pos}")
                    last_pos = current_pos
                
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\nTest stopped by user")

    def test_encoder_b(self, duration=5):
        """Test encoder B reading"""
        print("\nTesting Encoder B")
        print("Rotate the motor shaft and watch the encoder counts")
        print("Press Ctrl+C to stop")
        
        start_time = time.time()
        last_pos = 0
        
        try:
            while (time.time() - start_time) < duration:
                with self.encoder_lock_b:
                    current_pos = self.encoder_pos_b
                
                if current_pos != last_pos:
                    print(f"Encoder B Position: {current_pos}")
                    last_pos = current_pos
                
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\nTest stopped by user")

    def test_encoders(self, duration=5):
        """Test both encoders simultaneously"""
        print("\nTesting Both Encoders")
        print("Rotate both motor shafts and watch the encoder counts")
        print("Press Ctrl+C to stop")
        
        start_time = time.time()
        last_pos_a = 0
        last_pos_b = 0
        
        try:
            while (time.time() - start_time) < duration:
                with self.encoder_lock_a:
                    current_pos_a = self.encoder_pos_a
                with self.encoder_lock_b:
                    current_pos_b = self.encoder_pos_b
                
                if current_pos_a != last_pos_a or current_pos_b != last_pos_b:
                    print(f"Encoder A: {current_pos_a}, Encoder B: {current_pos_b}")
                    last_pos_a = current_pos_a
                    last_pos_b = current_pos_b
                
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\nTest stopped by user")

    def set_motor(self, motor, direction, speed):
        """Set direction and speed for specified motor"""
        GPIO.output(STBY, GPIO.HIGH)
        
        if motor == 'A':
            pwm, in1, in2 = self.pwm_a, IN1A, IN2A
        else:  # motor == 'B'
            pwm, in1, in2 = self.pwm_b, IN1B, IN2B
        
        if direction == 1:
            GPIO.output(in1, GPIO.HIGH)
            GPIO.output(in2, GPIO.LOW)
        elif direction == -1:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.HIGH)
        else:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.LOW)
        
        pwm.ChangeDutyCycle(speed)

    def cleanup(self):
        self.running = False
        time.sleep(0.1)
        self.set_motor('A', 0, 0)
        self.set_motor('B', 0, 0)
        self.pwm_a.stop()
        self.pwm_b.stop()
        GPIO.cleanup()

def main():
    print("Dual Motor PID Control Test Program")
    print("Pin Configuration:")
    print("\nMotor A:")
    print(f"ENCA: GPIO{ENCA}")
    print(f"ENCB: GPIO{ENCB}")
    print(f"PWMA: GPIO{PWMA}")
    print(f"IN1A: GPIO{IN1A}")
    print(f"IN2A: GPIO{IN2A}")
    print("\nMotor B:")
    print(f"ENCA2: GPIO{ENCA2}")
    print(f"ENCB2: GPIO{ENCB2}")
    print(f"PWMB: GPIO{PWMB}")
    print(f"IN1B: GPIO{IN1B}")
    print(f"IN2B: GPIO{IN2B}")
    print(f"\nSTBY: GPIO{STBY}")
    
    try:
        controller = DualPIDMotorControl(use_interrupts=True)
    except Exception as e:
        print(f"Failed to initialize with interrupts: {e}")
        print("Retrying with polling mode...")
        controller = DualPIDMotorControl(use_interrupts=False)
    
    while True:
        print("\nSelect test option:")
        print("1. Run sine wave test (both motors)")
        print("2. Run step response test (both motors)")
        print("3. Configure PID constants")
        print("4. Run custom position test")
        print("5. Run motor calibration")
        print("6. Test Encoder A")
        print("7. Test Encoder B")
        print("8. Test Both Encoders")
        print("9. Exit")
        choice = input("\nEnter choice (1-9): ")
        
        try:
            if choice == '1':
                duration = float(input("Enter test duration (seconds): "))
                amplitude = float(input("Enter amplitude: "))
                frequency = float(input("Enter frequency (Hz): "))
                controller.test_pid_sine(duration, amplitude, frequency)
                
            elif choice == '2':
                duration = float(input("Enter test duration (seconds): "))
                step_size = float(input("Enter step size: "))
                controller.test_pid_step(duration, step_size)
                
            elif choice == '3':
                motor = input("Select motor (A/B): ").upper()
                if motor not in ['A', 'B']:
                    print("Invalid motor selection")
                    continue
                    
                print("Enter new PID constants (press Enter to keep current value)")
                kp = input(f"Enter Kp (current={'A' if motor=='A' else 'B'}): ")
                ki = input(f"Enter Ki (current={'A' if motor=='A' else 'B'}): ")
                kd = input(f"Enter Kd (current={'A' if motor=='A' else 'B'}): ")
                
                controller.set_pid_constants(
                    motor,
                    kp=float(kp) if kp else None,
                    ki=float(ki) if ki else None,
                    kd=float(kd) if kd else None
                )
                
            elif choice == '4':
                motor = input("Select motor (A/B): ").upper()
                if motor not in ['A', 'B']:
                    print("Invalid motor selection")
                    continue
                    
                position = float(input("Enter target position: "))
                duration = float(input("Enter duration (seconds): "))
                
                print(f"\nMoving motor {motor} to position {position}")
                start_time = time.time()
                
                try:
                    while (time.time() - start_time) < duration:
                        if motor == 'A':
                            controller.target_a = position
                            pos, target, power, direction = controller.pid_control('A')
                        else:
                            controller.target_b = position
                            pos, target, power, direction = controller.pid_control('B')
                            
                        print(f"Time: {time.time() - start_time:.2f}s | "
                              f"Target: {target:.1f}, Position: {pos}, "
                              f"Power: {power:.1f}%, Direction: {direction}", end='\r')
                        time.sleep(0.02)
                        
                except KeyboardInterrupt:
                    print("\nTest stopped by user")
                finally:
                    controller.set_motor(motor, 0, 0)
                    
            elif choice == '5':
                print("\nMotor Calibration Test")
                motor = input("Select motor (A/B): ").upper()
                if motor not in ['A', 'B']:
                    print("Invalid motor selection")
                    continue
                
                print("\nTesting motor response at different power levels...")
                for power in [20, 40, 60, 80, 100]:
                    print(f"\nTesting at {power}% power")
                    controller.set_motor(motor, 1, power)
                    time.sleep(2)
                    controller.set_motor(motor, 0, 0)
                    time.sleep(1)
                    
                    controller.set_motor(motor, -1, power)
                    time.sleep(2)
                    controller.set_motor(motor, 0, 0)
                    time.sleep(1)
                
                print("\nCalibration complete")

            elif choice == '6':
                duration = float(input("Enter test duration (seconds): "))
                controller.test_encoder_a(duration)
                
            elif choice == '7':
                duration = float(input("Enter test duration (seconds): "))
                controller.test_encoder_b(duration)
                
            elif choice == '8':
                duration = float(input("Enter test duration (seconds): "))
                controller.test_encoders(duration) 
            
            elif choice == '9':
                print("Exiting...")
                break
                
            else:
                print("Invalid choice")
                
        except ValueError as e:
            print(f"Invalid input: {e}")
        except KeyboardInterrupt:
            print("\nTest stopped by user")
        finally:
            controller.set_motor('A', 0, 0)
            controller.set_motor('B', 0, 0)
            
    controller.cleanup()
    sys.exit(0)

if __name__ == "__main__":
    main()