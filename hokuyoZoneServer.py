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
import socket
import threading
from hokuyoZoneServer_tcpserver_part import TCPServer
import pyautogui  # pip install pyautogui

# ---------- Classes ------------
class Zone:
	def __init__(self, angle, distance, radius, in_event, on_event, out_event, name=""):
		self.angle = angle
		self.distance = distance
		self.radius = radius
		self.in_event = in_event
		self.on_event = on_event
		self.out_event = out_event
		self.name = name

	def __str__(self):
		return f"Zone at ang: {self.angle}, dist: {self.distance} with radius {self.radius}mm"

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
				"oscServer": "192.168.60.159",
				"oscAddress": "/zones",
				"manual_zones": [
					{
						"name": "Left",
						"angle": -30.0,
						"distance": 400.0,
						"radius": 200.0,
						"in_event": "a",
						"on_event": "b",
						"out_event": "c"
					},
					{
						"name": "Right",
						"angle": 30.0,
						"distance": 1000.0,
						"radius": 200.0,
						"in_event": "d",
						"on_event": "e",
						"out_event": "f"
					}
				],
				"outputType": "Zones"
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

def distBetweenPolar(r1, theta1, r2, theta2):
	"""
	Calculate distance between two points in polar coordinates (r, theta in degrees)
	"""
	# Convert degrees to radians
	theta1_rad = math.radians(theta1)
	theta2_rad = math.radians(theta2)
	# Convert polar to cartesian
	x1 = r1 * math.cos(theta1_rad)
	y1 = r1 * math.sin(theta1_rad)
	x2 = r2 * math.cos(theta2_rad)
	y2 = r2 * math.sin(theta2_rad)
	# Calculate Euclidean distance
	return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

# ---------- Main Program ----------
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
outputType = config["outputType"]
oscPort = config["oscPort"]
oscServer = config["oscServer"]
debug = config["debug"]
oscAddress = config["oscAddress"]

# Load manual zones from config if present
manual_zones = []
if "manual_zones" in config:
	for zone_data in config["manual_zones"]:
		zone = Zone(
			angle=zone_data.get("angle", 0),
			distance=zone_data.get("distance", 0),
			radius=zone_data.get("radius", 0),
			in_event=zone_data.get("in_event", ""),
			on_event=zone_data.get("on_event", ""),
			out_event=zone_data.get("out_event", ""),
			name=zone_data.get("name", "")
		)
		manual_zones.append(zone)
else:
	manual_zones = []

# Laser Settings
laser_serial = serial.Serial(port=uartPort, baudrate=uartSpeed, timeout=0.5)
port = serial_port.SerialPort(laser_serial)
laser = hokuyo.Hokuyo(port)
laserOn = False

# Connect to laser
while not laserOn: # Block Program if Not Conecting to Laser
	laserOn = connectRadar()

if outputType == "OSC":
	#OSC Client Connection
	oscClient = SimpleUDPClient(oscServer, oscPort)
elif outputType == "TCP":
	#TCP Server Connection
	tcpServer = TCPServer(port=oscPort)
	tcpServer.start()

print(laser.get_version_info())

print("laser Ready")
if outputType == "Keyboard":
    print("Keyboard will be activatedin 20 seconds.")
    sleep(20)  # Give time to switch to the target application
    print("Keyboard mode activated. Ready to send keypresses.")

#Create zone state list
last_zone_states = [None for _ in range(len(manual_zones))]
try:
	while laserOn:
    	# Read laser data
		#ang = []
		#dist = []
		if debug:
			print("zone_states")
			print(last_zone_states)
		scan = laser.get_single_scan()
		#Create empty Zone States
		zone_states = [None for _ in range(len(manual_zones))]
		for i, zone in enumerate(manual_zones):
			for ang in scan:
				dist = scan[ang]
				if distBetweenPolar(dist, ang, zone.distance, zone.angle) < zone.radius:
					
					if last_zone_states[i] is None:
						zone_states[i] = zone.in_event
					elif last_zone_states[i] is not None:
						zone_states[i] = zone.on_event
					if debug:
						print(f"{zone.name} - {zone_states[i]}")
					
					break
			#check if the was no interaction and the last state was not None
			print(f"{zone_states[i]} - {last_zone_states[i]}")
			if (zone_states[i] is None) and (last_zone_states[i] is not None):
				zone_states[i] = zone.out_event
				#last_zone_states[i] = None
			if zone_states[i] is not None:
				if outputType == "OSC":
					oscClient.send_message(oscAddress, zone_states[i])
				elif outputType == "TCP":
					tcpServer.send(zone_states[i].encode())
				elif outputType == "Keyboard":
					# Simulate keypresses of the message string
					pyautogui.typewrite(zone_states[i])

			if zone_states[i] == zone.out_event:
				last_zone_states[i] = None
			else:
				last_zone_states[i] = zone_states[i]
			
		#print("waiting")
		sleep(time2Scan)
except Exception as error:
	print("An exception occurred:", error)
	laser.laser_off()
	tcpServer.stop()
	laser_serial.close()
	print("Laser turned off and serial port closed.")