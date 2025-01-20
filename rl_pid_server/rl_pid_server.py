#!/usr/bin/env python3

import os
import sys
import subprocess

class MotorControlLauncher:
    def __init__(self):
        self.scripts = {
            "1": {
                "name": "RL PID SERVER",
                "file": "pid_server_claude.py",
                "description": "Control and tune a motor PID control using ROS2 client"
            },

            "2": {
                "name": "Single Motor PID Control",
                "file": "pid_test.py",
                "description": "Control and test a motor with PID"
            },

            "3": {
                "name": "Dual Motor PID Control",
                "file": "dual_pid_test.py",
                "description": "Control and test two motors with PID"
            },

            "4": {
                "name": "Single Motor test",
                "file": "motor_test.py",
                "description": "Control and test a single motor connection and PWM"
            },

            "5": {
                "name": "Dual Motor test",
                "file": "dual_motor_test.py",
                "description": "Control and test two motors with PID"
            }
        }

    def clear_screen(self):
        os.system('clear' if os.name == 'posix' else 'cls')

    def print_header(self):
        print("="*50)
        print("Motor Control Test Suite")
        print("="*50)
        print("\nAvailable Scripts:")
        print("-"*20)

    def print_menu(self):
        for key, script in self.scripts.items():
            print(f"\n{key}. {script['name']}")
            print(f"   Description: {script['description']}")

    def run_script(self, script_path):
        try:
            subprocess.run([sys.executable, script_path], check=True)
        except subprocess.CalledProcessError as e:
            print(f"\nError running script: {e}")
        except FileNotFoundError:
            print(f"\nError: Script file '{script_path}' not found!")
        except KeyboardInterrupt:
            print("\nScript execution cancelled by user")
        
        input("\nPress Enter to return to main menu...")

    def check_script_exists(self, script_name):
        script_path = os.path.join(os.path.dirname(__file__), script_name)
        return os.path.exists(script_path)

    def run(self):
        while True:
            self.clear_screen()
            self.print_header()
            self.print_menu()
            
            print("\nOptions:")
            print("6. Quit")
            
            choice = input("\nEnter your choice(1-6): ").lower()
            
            if choice == '6':
                print("\nExiting Motor Control Test Suite...")
                break
                
            if choice not in self.scripts:
                print("\nInvalid choice! Press Enter to continue...")
                input()
                continue
            
            script_file = self.scripts[choice]["file"]
            
            if not self.check_script_exists(script_file):
                print(f"\nError: Script file '{script_file}' not found!")
                print("Make sure all script files are in the same directory as this launcher.")
                input("\nPress Enter to continue...")
                continue
            
            script_path = os.path.join(os.path.dirname(__file__), script_file)
            self.run_script(script_path)

def main():
    launcher = MotorControlLauncher()
    launcher.run()

if __name__ == "__main__":
    main()