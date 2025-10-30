import pigpio
import time

class Servo():
    def __init__(self, gpio: int):
        self.gpio = gpio
        self.pi = pigpio.pi()

        if not self.pi.connected:
            raise Exception("Pi not connected")

        self.pi.set_mode(self.gpio, pigpio.OUTPUT)

    def pwm_neutral(self, sleep=0.45):
        '''
        Set servo to neutral position.
        '''

        try:
            self.pi.set_servo_pulsewidth(self.gpio, 1000)
            time.sleep(sleep)
            self.pi.write(self.gpio, 0)
        except KeyboardInterrupt:
            self.pi.write(self.gpio, 0)

    def pwm_high(self, sleep=0.45):
        '''
        Set servo to the highest secure pulsewidth.
        '''

        try:
            self.pi.set_servo_pulsewidth(self.gpio, 2000)
            time.sleep(sleep)
            self.pi.write(self.gpio, 0)
        except KeyboardInterrupt:
            self.pi.write(self.gpio, 0)

    def pwm_low(self, sleep=0.45):
        '''
        Set servo to the lowest secure pulsewidth.
        '''

        try:
            self.pi.set_servo_pulsewidth(self.gpio, 500)
            time.sleep(sleep)
            self.pi.write(self.gpio, 0)
        except KeyboardInterrupt:
            self.pi.write(self.gpio, 0)

    def set_pulsewidth(self, sleep=0.45, pulsewidth=1500):
        '''
        Set servo to any pulsewidth. Not recommended in most cases.
        '''

        try:
            self.pi.set_servo_pulsewidth(self.gpio, pulsewidth)
            time.sleep(sleep)
            self.pi.write(self.gpio, 0)
        except KeyboardInterrupt:
            self.pi.write(self.gpio, 0)


