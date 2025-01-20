#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time
from threading import Lock
import signal
import sys

# Pin Definitions for both motors
# Motor A
PWMA = 12
IN1A = 11
IN2A = 13
ENCA = 16
ENCB = 18

# Motor B
PWMB = 35
IN1B = 36
IN2B = 37
ENCA2 = 29
ENCB2 = 31

STBY = 15  # Standby Pin (shared between both motors)

class DualMotorTest:
    def __init__(self, use_interrupts=False):
        # Initialize GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.cleanup()
        
        # Initialize encoder positions and locks
        self.encoder_pos_a = 0
        self.encoder_pos_b = 0
        self.encoder_lock_a = Lock()
        self.encoder_lock_b = Lock()
        self.use_interrupts = use_interrupts
        self.running = True
        
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
                print("Successfully added event detection to both encoders")
            except Exception as e:
                print(f"Failed to set up encoder interrupts: {str(e)}")
                print("Falling back to polling mode")
                self.use_interrupts = False
        
        if not self.use_interrupts:
            from threading import Thread
            self.poll_thread = Thread(target=self.poll_encoders, daemon=True)
            self.poll_thread.start()
            print("Started encoder polling thread")

    def poll_encoders(self):
        last_a = GPIO.input(ENCA)
        last_a2 = GPIO.input(ENCA2)
        
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
            if a2 != last_a2 and a2 == 1:
                with self.encoder_lock_b:
                    if b2:
                        self.encoder_pos_b += 1
                    else:
                        self.encoder_pos_b -= 1
            last_a2 = a2
            
            time.sleep(0.0001)

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

    def set_motor(self, motor, direction, speed):
        """
        Set motor direction and speed
        motor: 'A' or 'B' to specify which motor
        direction: 1 for forward, -1 for reverse, 0 for stop
        speed: PWM duty cycle (0-100)
        """
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

    def test_sequence(self):
        try:
            print("Starting dual motor test sequence...")
            
            # Test both motors forward
            print("\nTesting both motors forward (3 seconds)...")
            start_pos_a = self.encoder_pos_a
            start_pos_b = self.encoder_pos_b
            self.set_motor('A', 1, 50)
            self.set_motor('B', 1, 50)
            
            start_time = time.time()
            while (time.time() - start_time) < 3:
                pos_a = self.encoder_pos_a
                pos_b = self.encoder_pos_b
                elapsed = time.time() - start_time
                print(f"Time: {elapsed:.1f}s, Motor A: {pos_a - start_pos_a}, Motor B: {pos_b - start_pos_b}", end='\r')
                time.sleep(0.1)
            
            self.set_motor('A', 0, 0)
            self.set_motor('B', 0, 0)
            print("\nForward test complete")
            time.sleep(1)
            
            # Test both motors reverse
            print("\nTesting both motors reverse (3 seconds)...")
            start_pos_a = self.encoder_pos_a
            start_pos_b = self.encoder_pos_b
            self.set_motor('A', -1, 50)
            self.set_motor('B', -1, 50)
            
            start_time = time.time()
            while (time.time() - start_time) < 3:
                pos_a = self.encoder_pos_a
                pos_b = self.encoder_pos_b
                elapsed = time.time() - start_time
                print(f"Time: {elapsed:.1f}s, Motor A: {pos_a - start_pos_a}, Motor B: {pos_b - start_pos_b}", end='\r')
                time.sleep(0.1)
            
            self.set_motor('A', 0, 0)
            self.set_motor('B', 0, 0)
            print("\nReverse test complete")
            
        except Exception as e:
            print(f"Error during test: {e}")
            self.cleanup()
            sys.exit(1)

    def cleanup(self):
        self.running = False
        time.sleep(0.1)
        self.pwm_a.stop()
        self.pwm_b.stop()
        GPIO.cleanup()

def main():
    print("GPIO Test Program for Dual Motor Setup")
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
        print("\nAttempting to initialize with interrupts...")
        tester = DualMotorTest(use_interrupts=True)
    except Exception as e:
        print(f"Failed to initialize with interrupts: {e}")
        print("\nRetrying with polling mode...")
        tester = DualMotorTest(use_interrupts=False)
    
    while True:
        print("\nSelect test option:")
        print("1. Run full test sequence (both motors)")
        print("2. Test Motor A forward")
        print("3. Test Motor A reverse")
        print("4. Test Motor B forward")
        print("5. Test Motor B reverse")
        print("6. Test both motors")
        print("7. Exit")
        
        choice = input("Enter choice (1-7): ")
        
        if choice == '1':
            tester.test_sequence()
        elif choice == '2':
            print("Running Motor A forward for 3 seconds...")
            tester.set_motor('A', 1, 50)
            time.sleep(3)
            tester.set_motor('A', 0, 0)
        elif choice == '3':
            print("Running Motor A reverse for 3 seconds...")
            tester.set_motor('A', -1, 50)
            time.sleep(3)
            tester.set_motor('A', 0, 0)
        elif choice == '4':
            print("Running Motor B forward for 3 seconds...")
            tester.set_motor('B', 1, 50)
            time.sleep(3)
            tester.set_motor('B', 0, 0)
        elif choice == '5':
            print("Running Motor B reverse for 3 seconds...")
            tester.set_motor('B', -1, 50)
            time.sleep(3)
            tester.set_motor('B', 0, 0)
        elif choice == '6':
            print("Running both motors forward for 3 seconds...")
            tester.set_motor('A', 1, 50)
            tester.set_motor('B', 1, 50)
            time.sleep(3)
            tester.set_motor('A', 0, 0)
            tester.set_motor('B', 0, 0)
        elif choice == '7':
            print("Exiting...")
            tester.cleanup()
            break
        else:
            print("Invalid choice, please try again")

if __name__ == "__main__":
    main()