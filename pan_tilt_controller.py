import RPi.GPIO as gpio
from gpiozero import AngularServo
import pigpio
import time

class PanTiltControllerPiGPIO:

    def __init__(self):
        # Uses Broadcom numbering to match pigpio library
        gpio.setmode(gpio.BCM)
        self.pan_pin = 24
        self.tilt_pin = 11
        self.im_width = 640
        self.im_height = 480
        self.pi_control = pigpio.pi()
        self.center_point = [self.im_width/2,self.im_height/2]
        self.init_pins_and_servos()
        self.stop()

    def init_pins_and_servos(self):
	# Initializes Pins
        self.pi_control.set_mode(self.pan_pin,pigpio.OUTPUT)
        self.pi_control.set_mode(self.tilt_pin,pigpio.OUTPUT)
        self.setServoAngle(self.pan_pin,20)
        self.setServoAngle(self.tilt_pin,60)
        self.pan_angle = 20
        self.tilt_angle = 60

    def stop(self):
        time.sleep(2)

    def setServoAngle(self,servo_pin, angle):
        dutyCycle = angle / 18. + 3.
        self.pi_control.set_PWM_dutycycle(servo_pin,dutyCycle)
        time.sleep(1)
        self.pi_control.set_PWM_dutycycle(servo_pin,dutyCycle)

    def set_angle_pan(self,angle):
        self.setServoAngle(self.pan_pin,angle)
        self.pan_angle = angle

    def set_angle_tilt(self,angle):
        self.setServoAngle(self.tilt_pin,angle)
        self.tilt_angle = angle

    def get_angle_pan(self):
        return self.pan_angle

    def get_angle_tilt(self):
        return self.tilt_angle

    def close(self):
        self.stop()
        gpio.cleanup()

    def get_offset(self,curr_coords):
        off_center = [curr_coords[0] - self.im_width,curr_coords[1] - self.im_height]
        return off_center

class PanTiltController:

    def __init__(self):
        gpio.setmode(gpio.BOARD)
        self.pan_pin = 18
        self.tilt_pin = 23
        self.im_width = 640
        self.im_height = 480
        self.center_point = [self.im_width/2,self.im_height/2]
        self.init_pins_and_servos()
        self.stop()


    def init_pins_and_servos(self):
	    # Initializes Pins
        gpio.setup(self.pan_pin,gpio.OUT)
        gpio.setup(self.tilt_pin,gpio.OUT)
        self.setServoAngle(self.pan_pin,20)
        self.setServoAngle(self.tilt_pin,60)
        self.pan_angle = 20
        self.tilt_angle = 60

    def stop(self):
        time.sleep(2)

    def set_angle(self,servo_pin,angle):
        #value = angle/90 - 1
        servo_pwm = gpio.PWM(servo_pin,50)
        duty = angle/18 + 2
        servo_pwm.start(duty)
        time.sleep(1)
        servo_pwm.stop()

    def setServoAngle(self,servo_pin, angle):
        #assert angle >=30 and angle <= 150
        servo_pwm = gpio.PWM(servo_pin,50)
        servo_pwm.start(8)
        dutyCycle = angle / 18. + 3.
        servo_pwm.ChangeDutyCycle(dutyCycle)
        time.sleep(1)
        servo_pwm.stop()

    def set_angle_pan(self,angle):
        self.setServoAngle(self.pan_pin,angle)
        self.pan_angle = angle

    def set_angle_tilt(self,angle):
        self.setServoAngle(self.tilt_pin,angle)
        self.tilt_angle = angle

    def get_angle_pan(self):
        return self.pan_angle

    def get_angle_tilt(self):
        return self.tilt_angle

    def close(self):
        self.stop()
        gpio.cleanup()

    def get_offset(self,curr_coords):
        off_center = [curr_coords[0] - self.im_width,curr_coords[1] - self.im_height]
        return off_center
