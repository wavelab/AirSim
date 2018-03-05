from AirSimClient import *

client = CarClient()
client.confirmConnection()
client.enableApiControl(True)
car_controls = CarControls()

home_geo_point = client.getHomeGeoPoint()

def sendControls():
    client.setCarControls(car_controls)

def getState():
    return client.getCarState()

def steer(val): #-1 to 1
    car_controls.steering = val
    sendControls()

def brake(val):
    car_controls.brake = val
    sendControls()

def gear(val):
    car_controls.is_manual_gear = True;
    car_controls.manual_gear = val
    sendControls()

def throttle(val):
    car_controls.throttle = val
    sendControls()

def stop():
    brake(1)
    throttle(0)
    sendControls()
    brake(0)
    sendControls()

def go():
    brake(0)
    throttle(1)
    sendControls()

def pos():
    car_state = getState()
    return car_state.kinematics_true.position

def rot():
    car_state = getState()
    return car_state.kinematics_true.orientation

def imu():
    car_state= getState()
    return (car_state.kinematics_true.linear_velocity, car_state.kinematics_true.angular_velocity, car_state.kinematics_true.linear_acceleration, car_state.kinematics_true.linear_velocity)

# ~ ~ ~ ~ ~ 

earthRadius = 6378137
originShift = 2 * math.pi * earthRadius / 2

def metersToLatLon(m):
    vx = (m[0] / originShift) * 180
    vy = (m[1] / originShift) * 180
    vy = 180 / math.pi * (2 * math.atan(math.exp(vy * math.pi / 180)) - math.pi / 2)
  
    return (vy, vx, m[2])

def latLonToMeters(lat, lon, alt):
    posx = lon * originShift / 180
    posy = math.log(math.tan((90 + lat) * math.pi / 360)) / (math.pi / 180)
    posy = posy * originShift / 180
    
    return (posx, posy, alt)

def gps():
    homeInMeters = latLonToMeters(home_geo_point.latitude, home_geo_point.longitude, home_geo_point.altitude)
    posInMeters = pos()
    posInMeters = ( posInMeters.x_val, posInMeters.y_val, posInMeters.z_val )
    x = homeInMeters[0] + posInMeters[0]
    y = homeInMeters[1] + posInMeters[1]
    z = homeInMeters[2] + posInMeters[2]
    return metersToLatLon((x,y,z))

gps_location = gps()
