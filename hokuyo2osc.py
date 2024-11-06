import numpy as np
from time import sleep
import math
import serial
import sys
import os
import json
import serial.tools.list_ports
from hokuyo.driver import hokuyo
from hokuyo.tools import serial_port
from pythonosc.udp_client import SimpleUDPClient # pip install python-osc

# ---------- Classes ------------
class Point:
    def __init__(self, angle, distance) -> None:
        self.angle = angle
        self.distance = distance

    def __str__(self):
        return f"{self.angle} Deg - {self.distance} mm"
    
    def x(self):
        return (self.distance * math.cos((self.angle-angleOffset+90) * math.pi/180))+(touchWidth/2)+widthOffset

    def xy(self):
        return (((self.distance * math.cos((self.angle-angleOffset+90) * math.pi/180))+(touchWidth/2)+widthOffset), ((self.distance * math.sin((self.angle-angleOffset+90) * math.pi/180))-heightOffset))

    def y(self):
        return (self.distance * math.sin((self.angle-angleOffset+90) * math.pi/180))-heightOffset

# ---------- Functions ----------
def distBetweenPoints(x1, y1, x2, y2):
	return math.sqrt(((x2-x1)**2)+((y2-y1)**2))

def getDist(x, y):
    return math.sqrt(x**2+y**2)

def getAngle(x, y):
	return math.atan(x/y)+((angleOffset+90)*math.pi/180)

def readConfig(settingsFile):
	if os.path.isfile(settingsFile):
		with open(settingsFile) as json_file:
			data = json.load(json_file)
	else:
		data = {
				"uartPort": "COM4",
				"uartSpeed": 19200,
				"debug": False,
				"minDist": 100,
				"maxDist": 4000,
				"minAng": -90,
				"maxAng": 90,
				"touchWidth": 4000,
				"touchHeight": 2250,
				"widthOffset": 0,
				"heightOffset": 0,
				"angleOffset": 0,
				"time2Scan": 0.1,
				"sendSpeed": 1,
				"tcpPort": 65432,
				"minSize": 20,
				"maxSize": 300,
				"oscPort": 9000,
				"oscServer": "127.0.0.1"
		}
		# Serializing json
		json_object = json.dumps(data, indent=4)

		# Writing to config.json
		with open(settingsFile, "w") as outfile:
			outfile.write(json_object)
	return data

def connectRadar():
	#Clean Range Finder settings
	try:
		#print(laser.get_sensor_state())
		laser.laser_on()
		return True
	except:
		laser.laser_off()
		print("Error Conecting to Laser, Reseting")
		return False

try:
	this_file = __file__
except NameError:
	this_file = sys.argv[0]
this_file = os.path.abspath(this_file)
if getattr(sys, 'frozen', False):
	cwd = os.path.dirname(sys.executable)
else:
	cwd = os.path.dirname(this_file)

# Get the current working
# directory (CWD)
#print("Current working directory:", cwd)

#Read Configuration File
settingsFile = os.path.join(cwd, "appconfig.json")
config = readConfig(settingsFile)
uartPort = config["uartPort"]
uartSpeed = config["uartSpeed"]
minDist = config["minDist"]
maxDist = config["maxDist"]
minAng = config["minAng"]
maxAng = config["maxAng"]
touchWidth = config["touchWidth"]
touchHeight = config["touchHeight"]
widthOffset = config["widthOffset"]
heightOffset = config["heightOffset"]
angleOffset = config["angleOffset"]
time2Scan = config["time2Scan"]
sendSpeed = config["sendSpeed"]
minSize = config["minSize"]
maxSize = config["maxSize"]
oscPort = config["oscPort"]
oscServer = config["oscServer"]
debug = config["debug"]
oscAddress = config["oscAddress"]


# Laser Settings
laser_serial = serial.Serial(port=uartPort, baudrate=uartSpeed, timeout=0.5)
port = serial_port.SerialPort(laser_serial)
laser = hokuyo.Hokuyo(port)
laserOn = False

# Connect to laser
while not laserOn: # Block Program if Not Conecting to Laser
	laserOn = connectRadar()

#OSC Client Connection
oscClient = SimpleUDPClient(oscServer, oscPort)

print(laser.get_version_info())

print("laser Ready")

try:
	while laserOn:
		#ang = []
		#dist = []
		#lastDist = 1000
		touchPoints = list()
		scan = laser.get_single_scan()
		#Create array for radar points
		radarPoints = list()
		for ang in scan:
			dist = scan[ang]
			if minDist < dist <= maxDist and minAng < ang < maxAng:
				radarPoints.append(Point(ang, dist))
				#Get Points in TouchArea
				if (0 < radarPoints[-1].x() < touchWidth) and (0 < radarPoints[-1].y() < touchHeight):
					#print(f"{int(x)},{int(y)}")
					touchPoints.append(Point(ang, dist))
		#Create Arrays ro make zones to detectar objects in area. A zone is made up area close points in scucession
		points = list()
		zones = list()
		lastPoint = Point(0, 0)
		newZone = False
		for point in touchPoints:
			distance = distBetweenPoints(point.x(), point.y(), lastPoint.x(), lastPoint.y())
			lastPoint = point
			if distance < minSize:
				points.append(point)
				newZone = True
			else:
				if newZone:
					zones.append(points)
					newZone = False
					points = list()
		# iterate over all Zones to get the mediam point which will be converted to Touch Points			
		for zone in zones:
			if len(zone) > 2:
				xx1 = zone[0].x()
				xx2 = zone[-1].x()
				yy1 = zone[0].y()
				yy2 = zone[-1].y()
				#Get distance of the extremes to test if the zones are useble
				distOfExtremes = distBetweenPoints(xx1, yy1, xx2, yy2)
				if minSize < distOfExtremes < maxSize:
					medX = (xx2+xx1)/2
					medY = (yy2+yy1)/2
					medAng = (zone[0].angle + zone[-1].angle)/2
					medDist = (zone[0].distance + zone[-1].distance)/2
					#Point to send to Services
					curX = medX/touchWidth
					curY = medY/touchHeight
					print(f"{curX}, {curY}")
					oscClient.send_message(oscAddress, [curX, curY])
			sleep(time2Scan)
except Exception as error:
    print("An exception occurred:", error)
    laser.laser_off()