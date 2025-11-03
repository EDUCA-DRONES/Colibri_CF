# Information: https://clover.coex.tech/programming

import rospy
from abc import ABC, abstractmethod
from .drone import Drone
from .camera import Camera
from .servo import Servo

class Task(ABC):
    '''
    An abstract class to write mission.
    '''

    drone = Drone()
    camera = Camera()

    def __init__(self, gpio:(int | None) = None) -> None:
        if gpio != None:
            self.servo = Servo(gpio)
    
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

        pass

    def change_servo_pin(self, gpio:int):
        '''
        Change the servo gpio.
        '''

        self.servo.gpio = gpio

