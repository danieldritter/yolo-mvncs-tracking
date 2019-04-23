import RPi.GPIO as gpio
import time

class PanTiltController:
    def __init__(self):
        gpio.setmode(gpio.BOARD)
        self.pan_pin = 23
        self.tilt_pin = 18
        self.im_width = 640
        self.im_height = 480
        self.center_point = [self.im_width/2,self.im_height/2]
        self.pan_pwm = None
        self.tilt_pwm = None
        self.init_pins_and_servos()
        self.stop()
    def init_pins_and_servos(self):
	# Initializes Pins
        gpio.setup(self.pan_pin,gpio.OUT)
        gpio.setup(self.tilt_pin,gpio.OUT)
        self.pan_pwm = gpio.PWM(self.pan_pin,50)
        self.tilt_pwm = gpio.PWM(self.tilt_pin,50)
        self.set_angle(self.pan_pin,self.pan_pwm,0)
        self.set_angle(self.tilt_pin,self.tilt_pwm,0)
        self.pan_angle = 0
        self.tilt_angle = 0
    def stop(self):
        gpio.output(self.pan_pin,False)
        gpio.output(self.tilt_pin,False)
        time.sleep(2)
    def set_angle(self,servo_pin,servo_pwm,angle):
        duty = angle/18 + 2
        gpio.output(servo_pin,True)
        servo_pwm.ChangeDutyCycle(duty)
        time.sleep(1)
        gpio.output(servo_pin,False)
        servo_pwm.ChangeDutyCycle(0)
    def set_angle_pan(self,angle):
        self.set_angle(self.pan_pin,self.pan_pwm,angle)
        self.pan_angle = angle
    def set_angle_tilt(self,angle):
        self.set_angle(self.tilt_pin,self.tilt_pwm,angle)
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


