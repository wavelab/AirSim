from AirSimClient import *

client = MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

def pos():
	return client.getPosition()

def rot():
    return client.getOrientation()

def gps():
    return  client.getGpsLocation()