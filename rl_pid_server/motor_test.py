#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time
from threading import Lock
import signal
import sys

# Pin Definitions
ENCA = 16  # Encoder A
ENCB = 18  # Encoder B
PWM = 12   # PWM Pin
IN2 = 13   # Motor Input 2
IN1 = 11   # Motor Input 1
STBY = 15  # Standby Pin

class EncoderTest:
    def __init__(self, use_interrupts=False):
        # Initialize GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.cleanup()
        
        # Initialize encoder position and other variables
        self.encoder_pos = 0
        self.encoder_lock = Lock()
        self.use_interrupts = use_interrupts
        self.running = True
        
        # Setup pins
        GPIO.setup(ENCA, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        print(f"Successfully set up ENCA (GPIO{ENCA})")
        
        GPIO.setup(ENCB, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        print(f"Successfully set up ENCB (GPIO{ENCB})")
        
        GPIO.setup(PWM, GPIO.OUT)
        GPIO.setup(IN1, GPIO.OUT)
        GPIO.setup(IN2, GPIO.OUT)
        GPIO.setup(STBY, GPIO.OUT)
        
        # Initialize PWM
        self.pwm = GPIO.PWM(PWM, 1000)  # 1000 Hz frequency
        self.pwm.start(0)
        
        if use_interrupts:
            try:
                GPIO.remove_event_detect(ENCA)
                time.sleep(0.1)
                GPIO.add_event_detect(ENCA, GPIO.RISING, 
                                    callback=self.encoder_callback,
                                    bouncetime=50)
                print("Successfully added event detection to ENCA")
            except Exception as e:
                print(f"Failed to set up encoder interrupt: {str(e)}")
                print("Falling back to polling mode")
                self.use_interrupts = False
                
        if not self.use_interrupts:
            # Start polling thread
            from threading import Thread
            self.poll_thread = Thread(target=self.poll_encoder, daemon=True)
            self.poll_thread.start()
            print("Started encoder polling thread")

    def test_sequence(self):
        try:
            print("Starting GPIO test sequence...")
            
            # Test 1: Standby Toggle
            print("\nTest 1: Testing STBY pin")
            print("Toggling STBY pin 5 times...")
            for i in range(5):
                GPIO.output(STBY, GPIO.HIGH)
                time.sleep(0.5)
                GPIO.output(STBY, GPIO.LOW)
                time.sleep(0.5)
            
            # Test 2: Enhanced Encoder Reading
            print("\nTest 2: Testing Encoder with Motor Movement")
            print("Testing encoder in both directions...")
            
            # Forward test
            print("\nTesting forward direction (3 seconds)...")
            start_pos = self.encoder_pos
            start_time = time.time()
            self.set_motor(1, 50)  # 50% speed forward
            
            while (time.time() - start_time) < 3:
                current_pos = self.encoder_pos
                elapsed = time.time() - start_time
                print(f"Time: {elapsed:.1f}s, Position: {current_pos}, Change: {current_pos - start_pos}", end='\r')
                time.sleep(0.1)
            
            self.set_motor(0, 0)  # Stop
            final_pos = self.encoder_pos
            print(f"\nForward test complete. Total change: {final_pos - start_pos}")
            time.sleep(1)
            
            # Reverse test
            print("\nTesting reverse direction (3 seconds)...")
            start_pos = self.encoder_pos
            start_time = time.time()
            self.set_motor(-1, 50)  # 50% speed reverse
            
            while (time.time() - start_time) < 3:
                current_pos = self.encoder_pos
                elapsed = time.time() - start_time
                print(f"Time: {elapsed:.1f}s, Position: {current_pos}, Change: {current_pos - start_pos}", end='\r')
                time.sleep(0.1)
            
            self.set_motor(0, 0)  # Stop
            final_pos = self.encoder_pos
            print(f"\nReverse test complete. Total change: {final_pos - start_pos}")
            time.sleep(1)
            
            # Test 3: Motor Control
            print("\nTest 3: Testing Motor Control")
            
            # Forward
            print("Testing forward direction (3 seconds)")
            self.set_motor(1, 50)  # 50% speed forward
            time.sleep(3)
            self.set_motor(0, 0)  # Stop
            time.sleep(1)
            
            # Reverse
            print("Testing reverse direction (3 seconds)")
            self.set_motor(-1, 50)  # 50% speed reverse
            time.sleep(3)
            self.set_motor(0, 0)  # Stop
            time.sleep(1)
            
            # Speed Ramp
            print("Testing speed ramp up and down")
            for speed in range(0, 101, 10):
                print(f"Speed: {speed}%")
                self.set_motor(1, speed)
                time.sleep(0.5)
            
            for speed in range(100, -1, -10):
                print(f"Speed: {speed}%")
                self.set_motor(1, speed)
                time.sleep(0.5)
            
            # Final stop
            self.set_motor(0, 0)
            
            print("\nTest sequence completed!")
            
        except Exception as e:
            print(f"Error during test: {e}")
            self.cleanup()
            sys.exit(1)

    def test_encoder(self, duration=5, speed=50):
        """
        Test encoder while running motor
        duration: test duration in seconds
        speed: motor speed (0-100)
        """
        print(f"\nTesting encoder for {duration} seconds at {speed}% speed")
        print("Testing forward direction...")
        start_pos = self.encoder_pos
        start_time = time.time()
        
        # Run motor forward
        self.set_motor(1, speed)
        
        # Monitor encoder position while motor runs
        while (time.time() - start_time) < duration:
            current_pos = self.encoder_pos
            elapsed = time.time() - start_time
            print(f"Time: {elapsed:.1f}s, Position: {current_pos}, Change: {current_pos - start_pos}", end='\r')
            time.sleep(0.1)
        
        # Stop motor
        self.set_motor(0, 0)
        final_pos = self.encoder_pos
        
        print(f"\nForward test complete. Total change: {final_pos - start_pos}")
        time.sleep(1)
        
        # Test reverse direction
        print("\nTesting reverse direction...")
        start_pos = self.encoder_pos
        start_time = time.time()
        
        # Run motor in reverse
        self.set_motor(-1, speed)
        
        # Monitor encoder position while motor runs
        while (time.time() - start_time) < duration:
            current_pos = self.encoder_pos
            elapsed = time.time() - start_time
            print(f"Time: {elapsed:.1f}s, Position: {current_pos}, Change: {current_pos - start_pos}", end='\r')
            time.sleep(0.1)
        
        # Stop motor
        self.set_motor(0, 0)
        final_pos = self.encoder_pos
        
        print(f"\nReverse test complete. Total change: {final_pos - start_pos}")

    # Keep all other methods the same...
    def poll_encoder(self):
        last_a = GPIO.input(ENCA)
        while self.running:
            a = GPIO.input(ENCA)
            b = GPIO.input(ENCB)
            
            if a != last_a and a == 1:  # Rising edge on A
                with self.encoder_lock:
                    if b:
                        self.encoder_pos += 1
                    else:
                        self.encoder_pos -= 1
            
            last_a = a
            time.sleep(0.0001)  # 100µs delay

    def encoder_callback(self, channel):
        with self.encoder_lock:
            b = GPIO.input(ENCB)
            if b > 0:
                self.encoder_pos += 1
            else:
                self.encoder_pos -= 1

    def set_motor(self, direction, speed):
        """
        Set motor direction and speed
        direction: 1 for forward, -1 for reverse, 0 for stop
        speed: PWM duty cycle (0-100)
        """
        GPIO.output(STBY, GPIO.HIGH)  # Enable motor driver
        
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
        time.sleep(0.1)  # Give polling thread time to stop
        self.pwm.stop()
        GPIO.cleanup()

        
def main():
    print("GPIO Test Program for Motor Setup")
    print("Pin Configuration:")
    print(f"ENCA: GPIO{ENCA}")
    print(f"ENCB: GPIO{ENCB}")
    print(f"PWM:  GPIO{PWM}")
    print(f"IN2:  GPIO{IN2}")
    print(f"IN1:  GPIO{IN1}")
    print(f"STBY: GPIO{STBY}")
    
    try:
        # First try with interrupts
        print("\nAttempting to initialize with interrupts...")
        tester = EncoderTest(use_interrupts=True)
    except Exception as e:
        print(f"Failed to initialize with interrupts: {e}")
        print("\nRetrying with polling mode...")
        tester = EncoderTest(use_interrupts=False)
    
    print("\nEncoder test initialized successfully!")
    # ... rest of main function remains the same ...
    
    while True:
        print("\nSelect test option:")
        print("1. Run full test sequence")
        print("2. Test encoder (5 seconds)")
        print("3. Test motor forward")
        print("4. Test motor reverse")
        print("5. Test speed ramp")
        print("6. Exit")
        
        choice = input("Enter choice (1-6): ")
        
        if choice == '1':
            tester.test_sequence()
        elif choice == '2':
            print("Reading encoder for 5 seconds...")
            start_pos = tester.encoder_pos
            tester.test_encoder()
            time.sleep(5)
            print(f"Encoder change: {tester.encoder_pos - start_pos}")
        elif choice == '3':
            print("Running motor forward for 3 seconds...")
            tester.set_motor(1, 50)
            time.sleep(3)
            tester.set_motor(0, 0)
        elif choice == '4':
            print("Running motor reverse for 3 seconds...")
            tester.set_motor(-1, 50)
            time.sleep(3)
            tester.set_motor(0, 0)
        elif choice == '5':
            print("Testing speed ramp...")
            for speed in range(0, 101, 10):
                print(f"Speed: {speed}%")
                tester.set_motor(1, speed)
                time.sleep(0.5)
            tester.set_motor(0, 0)
        elif choice == '6':
            print("Exiting...")
            tester.cleanup()
            break
        else:
            print("Invalid choice, please try again")

if __name__ == "__main__":
    main()