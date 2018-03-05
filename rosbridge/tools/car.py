from AirSimClient import *

client = CarClient()
client.confirmConnection()
client.enableApiControl(True)
car_controls = CarControls()
car_state = client.getCarState()

def refresh():
    car_state = client.getCarState()
    client.setCarControls(car_controls)

def steer(val): #-1 to 1
    car_controls.steering = val
    refresh()

def brake(val):
    car_controls.brake = val
    refresh()

def gear(val):
    car_controls.is_manual_gear = True;
    car_controls.manual_gear = val
    refresh()

def throttle(val):
    car_controls.throttle = val
    refresh()

def stop():
    brake(1)
    throttle(0)
    refresh()
    brake(0)
    refresh()

def go():
    brake(0)
    throttle(1)
    refresh()

def pos():
    refresh()
    return car_state.kinematics_true.position

def rot():
    refresh()
    return car_state.kinematics_true.orientation

def imu():
    #refresh()
    car_state= client.getCarState()
    return (car_state.kinematics_true.linear_velocity, car_state.kinematics_true.angular_velocity, car_state.kinematics_true.linear_acceleration, car_state.kinematics_true.linear_velocity)

def gps():
    refresh()
    #car_state = client.getCarState()
    return  client.getGpsLocation()
