import RPi.GPIO as GPIO
import time

def test_gpio():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    GPIO.cleanup()
    
    pin = 16  # ENCA pin
    
    try:
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        print(f"Successfully setup GPIO{pin}")
        
        # Read the pin value for 5 seconds
        print("Reading pin value for 5 seconds...")
        for _ in range(5):
            value = GPIO.input(pin)
            print(f"Pin value: {value}")
            time.sleep(1)
            
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    test_gpio()