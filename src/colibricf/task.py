# Information: https://clover.coex.tech/programming

import rospy
from abc import ABC, abstractmethod
from drone import Drone
from camera import Camera
from servo import Servo

class Task(ABC):
    '''
    An abstract class to write mission.
    '''

    drone = Drone()
    camera = Camera()

    def __init__(self, servo:int = 14):
        self.servo = Servo(servo)
    
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
            print("Aborting")

        except Exception as e:
            print(f"Error: {e}")

        finally:
            print('Landing')
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

