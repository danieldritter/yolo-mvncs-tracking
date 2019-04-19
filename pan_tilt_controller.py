import RPi.GPIO as gpio
import time

class PanTiltController:

	def __init__(self):
		self.init_pins()
		self.stop()

	def init_pins(self):
		gpio.setmode(gpio.BOARD)
		gpio.setup(8,gpio.OUT)

	def move(self):
		self.stop()
		gpio.output(8,True)
		time.sleep(0.3)
		self.stop()
		time.sleep(0.3)
	def stop(self):
		gpio.output(8,False)
		time.sleep(2)
	
        def get_direction(prev_coords,curr_coords):
            """ 
            Takes in the coordinates of a previous classification from YOLO and set of current coordinates 
            for the same classification and determines which direction the pan tilt has to move to track the object.            
            :Params: prev_coords = [x,y], curr_coords = [x,y]
            :Returns: One of {"Left","Right","Up","Down"}
            """
            pass
            
        
        
        def close(self):
		self.stop()
		gpio.cleanup()

def __main__():
	controller = PanTiltController()
	controller.move()
	controller.close()

__main__()
