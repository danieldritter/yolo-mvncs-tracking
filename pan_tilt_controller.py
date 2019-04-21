import RPi.GPIO as gpio
import time

class PanTiltController:

	def __init__(self):
		self.pan_pin = 23
		self.tilt_pin = 18
		self.im_width = 640
		self.im_height = 480
		self.center_point = [self.im_width/2,self.im_height/2]
		self.init_pins()
		self.stop()
	def init_pins(self):
		# Initializes Pins
		gpio.setmode(gpio.BOARD)
		gpio.setup(self.pan_pin,gpio.OUT)
		gpio.setup(self.tilt_pin,gpio.OUT)	
		
	def move(self):
		self.stop()
		gpio.output(self.tilt_pin,True)
		time.sleep(0.1)
		self.stop()
		time.sleep(0.3)
	def stop(self):
		gpio.output(self.tilt_pin,False)
		time.sleep(2)
	
        def get_direction(curr_coords):
        	dir_vector = [curr_coords[0] - self.im_width,curr_coords[1]-self.im_height]
        	return dir_vector

	def adjust_position():
		
		off_center = dir_vector
        
        def close(self):
		self.stop()
		gpio.cleanup()

def __main__():
	controller = PanTiltController()
	controller.move()
	controller.close()

__main__()
