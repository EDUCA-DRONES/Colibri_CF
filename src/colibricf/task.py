# Information: https://clover.coex.tech/programming

import rospy
from abc import ABC, abstractmethod
from .drone import Drone, DroneMode
from .camera import Camera
from .servo import Servo
from typing import Union

class Task(ABC):
    '''
    An abstract class to write mission.
    '''

    RTL_ALTITUDE = 10

    def __init__(self, gpio: Union[int, None] = None) -> None:
        if gpio != None:
            self.servo = Servo(gpio)

        self.drone = Drone()
        self.camera = Camera()

    @abstractmethod
    def mission(self):
        raise Exception("Need implementation.")

    def run(self):
        '''
        A secure method to run a mission. Useful in most cases.
        '''

        try:
            self.mission()

        except KeyboardInterrupt:
            print("warning: aborting task")

        except Exception as e:
            print(f"ERROR: {e}")

        finally:
            print('note: landing')
            self.drone.land_wait()

    def return_to_launch_confim(self):
        '''
        Use if you need to confirm the return.
        '''
        confirm = input("Return to launch position? (Y/n): ").strip().lower()
        if confirm == 'y':
            self.drone.set_mode(DroneMode.RTL)

    def change_servo_pin(self, gpio:int):
        '''
        Change the servo gpio.
        '''

        self.servo.gpio = gpio

