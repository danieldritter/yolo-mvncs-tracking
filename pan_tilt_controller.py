import RPi.GPIO as gpio
from gpiozero import AngularServo
import time

class PanTiltController:
    def __init__(self):
        gpio.setmode(gpio.BOARD)
        self.pan_pin = 18 
        self.tilt_pin = 23
        self.im_width = 640
        self.im_height = 480
        self.center_point = [self.im_width/2,self.im_height/2]
        self.pan_pwm = None
        self.tilt_pwm = None
        #self.pan_pwm = AngularServo(self.pan_pin,min_angle=-70,max_angle=70)
        #self.tilt_pwm = AngularServo(self.tilt_pin,min_angle=-50,max_angle=50)
        self.init_pins_and_servos()
        self.stop()
    def init_pins_and_servos(self):
	# Initializes Pins
        gpio.setup(self.pan_pin,gpio.OUT)
        gpio.setup(self.tilt_pin,gpio.OUT)
        self.pan_pwm = gpio.PWM(self.pan_pin,50)
        self.tilt_pwm = gpio.PWM(self.tilt_pin,50)
        self.setServoAngle(self.pan_pwm,45)
        self.setServoAngle(self.tilt_pwm,45)
        self.pan_angle = 45
        self.tilt_angle = 45
    def stop(self):
        self.pan_pwm.stop()
        self.tilt_pwm.stop()
        #gpio.output(self.pan_pin,False)
        #gpio.output(self.tilt_pin,False)
        time.sleep(2)
    
    def set_angle(self,servo_pin,servo_pwm,angle):
        #value = angle/90 - 1
        
        duty = angle/18 + 2
        #gpio.output(servo_pin,True)
        # servo_pwm.ChangeDutyCycle(duty)
        servo_pwm.start(duty)
        time.sleep(1)
        #gpio.output(servo_pin,False)
        #servo_pwm.ChangeDutyCycle(0)
        servo_pwm.stop()
    
    def setServoAngle(self,servo_pwm, angle):
        assert angle >=30 and angle <= 150
        servo_pwm.start(8)
        dutyCycle = angle / 18. + 3.
        servo_pwm.ChangeDutyCycle(dutyCycle)
        time.sleep(0.3)
        servo_pwm.stop()

    def set_angle_pan(self,angle):
        self.setServoAngle(self.pan_pwm,angle)
        #self.pan_pwm.angle = angle
        self.pan_angle = angle
    
    def set_angle_tilt(self,angle):
        self.setServoAngle(self.tilt_pwm,angle)
        #self.tilt_pwm.angle = angle
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


