import RPi.GPIO as gpio
import time

class PanTiltController:

	def __init__(self):
		self.init_pins()
		self.stop()

	def init_pins(self):
		gpio.setmode(gpio.BOARD)
		gpio.setup(8,gpio.OUT)
		gpio.setup(10,gpio.OUT)
		gpio.setup(11,gpio.OUT)
		gpio.setup(13,gpio.OUT)

	def move(self):
		self.stop()
		gpio.output(8,True)
		gpio.output(10,True)
		gpio.output(11,True)
		gpio.output(13,False)
		time.sleep(0.3)
		self.stop()
		time.sleep(0.3)
	def stop(self):
		gpio.output(8,False)
		gpio.output(10,False)
		gpio.output(11,False)
		gpio.output(13,False)
		time.sleep(2)
	
	def close(self):
		self.stop()
		gpio.cleanup()

def __main__():
	controller = PanTiltController()
	controller.move()
	controller.close()

__main__()
