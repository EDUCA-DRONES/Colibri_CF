# Information: https://clover.coex.tech/programming
#
import rospy
from abc import ABC, abstractmethod
from .drone import Drone, DroneMode
from .camera import Camera
from .servo import Servo
from typing import Union
from .files.logger import Logger

class Task(ABC):
    '''
    An abstract class to write mission.
    '''

    def __init__(self, gpio: Union[int, None] = None) -> None:
        self.drone = Drone()
        self.logger = Logger()
        self.logger.start()
        rospy.sleep(3)

        if gpio != None:
            self.servo = Servo(gpio)

        self.camera = Camera()

    @abstractmethod
    def mission(self) -> None:
        raise Exception("Need implementation.")

    def run(self) -> None:
        '''
        A secure method to run a mission. Useful in most cases.
        '''

        try:
            rospy.logwarn('Starting task.')
            self.mission()

        except KeyboardInterrupt:
            rospy.logwarn('Aborting task.')
            rospy.sleep(0.5)

        except Exception as e:
            rospy.logerr(e)

        finally:
            self.drone.land_wait()
            self.camera.stop()
            rospy.sleep(3)
            self.logger.stop()
