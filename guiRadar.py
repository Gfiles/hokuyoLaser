import cv2
import statistics
import numpy as np
from time import sleep
import math
import threading
import random
import serial
import sys
import os
import json
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import serial.tools.list_ports
from hokuyo.driver import hokuyo
from hokuyo.tools import serial_port

# ---------- Functions ----------
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
				"touchWidth": 8000,
				"touchHeight": 4000,
				"widthOffset": 0,
				"heightOffset": 0,
				"angleOffset": 0,
				"time2Scan": 0.1,
				"sendSpeed": 1,
				"tcpPort": 65432,
				"minSize": 20,
				"maxSize": 300,
				"oscPort": 9000,
				"oscServer": "172.25.202.34"
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

def on_closing():
	if messagebox.askokcancel("Quit", "Do you want to quit?"):
		if laserOn:
			laser.laser_off()
			#lidar.stop_motor()
			#lidar.disconnect()
		stopEvent.set()
		thread.join()
		cv2.destroyAllWindows()
		tkWindow.destroy()
		#sys.exit()

def saveJson():
    if uartPort_text.get() != "None":
        config["uartPort"] = uartPort_text.get()
    config["uartSpeed"] = int(uartSpeed_text.get())
    config["minDist"] = int(heightOffset_text.get())
    config["maxDist"] = int(radarMaxDist_text.get())
    config["minAng"] = int(radarMinAng_text.get())
    config["maxAng"] = int(radarMaxAng_text.get())
    config["touchWidth"] = int(touchWidth_text.get())
    config["touchHeight"] = int(touchHeight_text.get())
    config["widthOffset"] = int(widthOffset_text.get())
    config["heightOffset"] = int(heightOffset_text.get())
    config["angleOffset"] = int(angleOffset_text.get())
    config["minSize"] = int(minSize_text.get())
    config["maxSize"] = int(maxSize_text.get())
    
    # Serializing json
    json_object = json.dumps(config, indent=4)
    
    with open(settingsFile, "w") as outfile:
        outfile.write(json_object)
    print("Saved")

def animate_radar(stopEvent):
	while not stopEvent.is_set():
		minDist = int(radarMinDist_text.get())
		maxDist = int(radarMaxDist_text.get())
		minAng = int(radarMinAng_text.get())
		maxAng = int(radarMaxAng_text.get())
		touchWidth = int(touchWidth_text.get())
		touchHeight = int(touchHeight_text.get())
		widthOffset = int(widthOffset_text.get())
		heightOffset = int(heightOffset_text.get())
		angleOffset = int(angleOffset_text.get())
		minSize = int(minSize_text.get())
		maxSize = int(maxSize_text.get())
  		
		if maxDist < 1000:
			maxDist = 1000
		
		meters = int(maxDist/1000)
		meterInPX = (canvasSizeX/2) / meters
		dist2PX = maxDist/midPointX
		#print(f"meterInPX - {meterInPX}, dist2PX - {dist2PX}")
		#print(f"midPoint - {midPointX}, canvasSizeX - {canvasSizeX}")
		backGround = np.zeros((canvasSizeY, canvasSizeX, 3), np.uint8)

		# Draw Cross Lines
		image = cv2.line(backGround, (0, midPointY), (canvasSizeX, midPointY), color, thickness)
		image = cv2.line(backGround, (midPointX, 0), (midPointX, canvasSizeY), color, thickness)
		# Draw a circle of red color of thickness -1 px
		image = cv2.circle(backGround, (midPointX, midPointY), 5, color, -1)
		#draw Meter Circles
		for i in range(1, int(meters)+1):
			image = cv2.circle(backGround, (midPointX, midPointY), int(meterInPX*i), color, thickness)

		#Draw Text
		#image = cv2.putText(backGround, "180", (midPointX, 15), font, 0.5, color, thickness, cv2.LINE_AA)
		image = cv2.putText(backGround, "0", (midPointX+2, canvasSizeY-5), font, 0.5, color, thickness, cv2.LINE_AA)
		image = cv2.putText(backGround, "90", (2, midPointY-5), font, 0.5, color, thickness, cv2.LINE_AA)
		image = cv2.putText(backGround, "-90", (canvasSizeX-35, midPointY-5), font, 0.5, color, thickness, cv2.LINE_AA)

		#Draw TouchWindow
		x1 = int((touchWidth/2)+widthOffset)
		y1 = heightOffset
  
		x2 = int((-1*touchWidth/2)+widthOffset)
		y2 = y1

		x3 = x2
		y3 = heightOffset+touchHeight

		x4 = x1
		y4 = y3
		
		dist1 = math.sqrt(x1**2+(y1)**2)
		if y1 == 0:
			y1 = 1
		ang1 = math.atan(x1/y1)+((angleOffset+90)*math.pi/180)
		x1 = int(dist1/dist2PX * math.cos((ang1)))+midPointX
		y1 = int(dist1/dist2PX * math.sin((ang1)))+midPointY

		dist2 = math.sqrt(x2**2+(y2)**2)
		if y2 == 0:
			y2 = 1
		ang2 = math.atan(x2/y2)+((angleOffset+90)*math.pi/180)
		x2 = int(dist2/dist2PX * math.cos((ang2)))+midPointX
		y2 = int(dist2/dist2PX * math.sin((ang2)))+midPointY
		
		dist3 = math.sqrt(x3**2+(y3)**2)
		if y3 == 0:
			y3 = 1
		ang3 = math.atan(x3/y3)+((angleOffset+90)*math.pi/180)
		x3 = int(dist3/dist2PX * math.cos((ang3)))+midPointX
		y3 = int(dist3/dist2PX * math.sin((ang3)))+midPointY

		dist4 = math.sqrt(x4**2+(y4)**2)
		if y4 == 0:
			y4 = 1
		ang4 = math.atan(x4/y4)+((angleOffset+90)*math.pi/180)
		x4 = int(dist4/dist2PX * math.cos((ang4)))+midPointX
		y4 = int(dist4/dist2PX * math.sin((ang4)))+midPointY
		#print(f"{x1} x {x2} - {x2} x {y2}")
		image = cv2.line(backGround, (x1, y1), (x2, y2), green, 3)
		image = cv2.line(backGround, (x2, y2), (x3, y3), green, 3)
		image = cv2.line(backGround, (x3, y3), (x4, y4), green, 3)
		image = cv2.line(backGround, (x4, y4), (x1, y1), green, 3)

		# end Square Draw

		ang = []
		dist = []
		lastDist = 1000
		lineArray = [(midPointX, midPointY)]
		if not laserOn:
			for i in range(minAng, maxAng):
				ang.append(i)
				lastDist = lastDist + random.randrange(-100,100)
				if lastDist < 0:
					lastDist = 0
				if lastDist > meters*1000:
					lastDist = meters*1000
				dist.append(lastDist)
			# End Test Data
			i = 0
			for item in dist:
				x = int((item/dist2PX * math.cos((ang[i]+90) * math.pi/180))+midPointX)
				y = int((item/dist2PX * math.sin((ang[i]+90) * math.pi/180))+midPointY)
				lineArray.append((x, y))

				image = cv2.line(backGround, lineArray[i], (x, y), red, 3)
				i += 1
		else:
			scan = laser.get_single_scan()
			i = 0
			pointsAng = list()
			pointsDist = list()
			lastAng = 0
			lastDist = 0
			for ang in scan:
				dist = scan[ang]
				if dist > maxDist:
					dist = maxDist
				if minDist < dist <= maxDist and minAng < ang < maxAng:
					radAng = ang * math.pi/180
					x = int((dist/dist2PX * math.cos((ang+90) * math.pi/180))+midPointX)
					y = int((dist/dist2PX * math.sin((ang+90) * math.pi/180))+midPointY)
					lineArray.append((x, y))
					image = cv2.line(backGround, lineArray[i], (x, y), red, 2)
					i += 1
				# Create Points
				if abs(dist - lastDist) < maxSize:
					pointsAng.append(ang)
					pointsDist.append(dist)
				else:
					if	100 > len(pointsAng) > 5:
						medAng = statistics.median(pointsAng)
						medDist = statistics.median(pointsDist)
						x = int((medDist/dist2PX * math.cos((medAng+90) * math.pi/180))+midPointX)
						y = int((medDist/dist2PX * math.sin((medAng+90) * math.pi/180))+midPointY)
						cv2.circle(backGround, (x, y), 5, (255, 0, 0), -1)
					pointsAng = list()
					pointsDist = list()
				lastDist = dist

		image = cv2.line(backGround, lineArray[i], lineArray[0], red, thickness)
		# Displaying the image
		#image = cv2.line(backGround,(600,600), (300, 300), red, thickness)

		cv2.imshow(window_name, image)
		sleep(time2Scan)
		if cv2.waitKey(1) == ord('q'):
			break

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
debug = config["debug"]
#print(debug)

# Get COM Ports
comlist = serial.tools.list_ports.comports()
comPortList = []
if len(comlist) > 0:
	jsonPort = False
	for element in comlist:
		comPortList.append(element.device)
	if len(comlist) == 1:
		uartPort = comPortList[0]
	else:
		for item in comPortList:
			if uartPort == item:
				jsonPort = True
				break
		if not jsonPort:
			uartPort = comPortList[0]
	#print(uartPort)

# Laser Settings
laser_serial = serial.Serial(port=uartPort, baudrate=uartSpeed, timeout=0.5)
port = serial_port.SerialPort(laser_serial)
laser = hokuyo.Hokuyo(port)
laserOn = False

# Connect to laser
while not laserOn: # Block Program if Not Conecting to Laser
	laserOn = connectRadar()

print(laser.get_version_info())

print("laser Ready")

# Reading an image in default mode
# Window name in which image is displayed
window_name = 'Radar Scan'
# --- Variables

#Create TK window
tkWindow = tk.Tk()

tkWindow.protocol("WM_DELETE_WINDOW", on_closing)

tkWindow.title("Radar Detection")
# get the screen dimension
screenWidth = tkWindow.winfo_screenwidth()
screenHeight = tkWindow.winfo_screenheight()
windowWidth = 300
windowHeight = 700
canvasSizeX = screenWidth - windowWidth
canvasSizeY = int(canvasSizeX/2+20)
showMeters = True
showAngles = True
midPointX = int(canvasSizeX/2)
midPointY = 20
stopThread = False

# color in BGR
color = (255, 255, 255)
red = (0, 0, 255)
green = (0, 255, 0)

# Line thickness of -1 px
thickness = 1
font = cv2.FONT_HERSHEY_SIMPLEX

# find the center point
#center_x = int(screen_width/2 - windowWidth / 2)
center_x = 0
#center_y = int(screen_height/2 - windowHeight / 2)
center_y = 0

tkWindow.geometry(f'{windowWidth}x{windowHeight}+{center_x}+{center_y}')

frame1 = ttk.Frame(tkWindow)
frame1['padding'] = 10

ttk.Label(frame1, text='Radar settings', font=("Arial", 12)).pack()
ttk.Label(frame1, text='COM Port', font=("Arial", 12)).pack()
uartPort_text = tk.StringVar()
if len(comPortList) == 0:
	uartPort_text.set("None")
else:
	uartPort_text.set(uartPort)
if len(comPortList) < 2:
	uartEntry = ttk.Entry(frame1, textvariable=uartPort_text, font=("Arial", 12))
	uartEntry["state"] = "readonly"
	uartEntry.pack()
else:
	comboComPort = ttk.Combobox(frame1, textvariable=uartPort_text, font=("Arial", 12))
	comboComPort["values"] = comPortList
	comboComPort["state"] = 'readonly'
	comboComPort.pack()

ttk.Label(frame1, text='Port Speed', font=("Arial", 12)).pack()
uartSpeed_text = tk.StringVar(value=uartSpeed)
comboSpeed = ttk.Combobox(frame1, textvariable=uartSpeed_text, font=("Arial", 12))
comboSpeed["values"] = ["19200", "115200", "256000"]
comboSpeed["state"] = 'readonly'
comboSpeed.pack()

ttk.Label(frame1, text='Radar Min Distance (mm)', font=("Arial", 12)).pack()
radarMinDist_text = tk.StringVar(value=minDist)
tk.Spinbox(frame1, textvariable=radarMinDist_text, font=("Arial", 12), from_=20, to=7000, increment=100).pack()

ttk.Label(frame1, text='Radar Max Distance (mm)', font=("Arial", 12)).pack()
radarMaxDist_text = tk.StringVar(value=maxDist)
tk.Spinbox(frame1, textvariable=radarMaxDist_text, font=("Arial", 12), from_=1000, to=7000, increment=1000).pack()

ttk.Label(frame1, text='Radar Min Angle', font=("Arial", 12)).pack()
radarMinAng_text = tk.StringVar(value=minAng)
tk.Spinbox(frame1, textvariable=radarMinAng_text, font=("Arial", 12), from_=-360, to=360, increment=10).pack()

ttk.Label(frame1, text='Radar Max Angle', font=("Arial", 12)).pack()
radarMaxAng_text = tk.StringVar(value=maxAng)
tk.Spinbox(frame1, textvariable=radarMaxAng_text, font=("Arial", 12), from_=-360, to=360, increment=10).pack()


ttk.Label(frame1, text='Touch Area settings', font=("Arial", 12)).pack()
ttk.Label(frame1, text='Width (mm)', font=("Arial", 12)).pack()
touchWidth_text = tk.StringVar(value=touchWidth)
tk.Spinbox(frame1, textvariable=touchWidth_text, font=("Arial", 12), from_=0, to=14000, increment=100).pack()

ttk.Label(frame1, text='Height (mm)', font=("Arial", 12)).pack()
touchHeight_text = tk.StringVar(value=touchHeight)
tk.Spinbox(frame1, textvariable=touchHeight_text, font=("Arial", 12), from_=0, to=7000, increment=100).pack()

ttk.Label(frame1, text='Width Offset (mm)', font=("Arial", 12)).pack()
widthOffset_text = tk.StringVar(value=widthOffset)
tk.Spinbox(frame1, textvariable=widthOffset_text, font=("Arial", 12), from_=-7000, to=7000, increment=100).pack()

ttk.Label(frame1, text='Height Offset (mm)', font=("Arial", 12)).pack()
heightOffset_text = tk.StringVar(value=heightOffset)
tk.Spinbox(frame1, textvariable=heightOffset_text, font=("Arial", 12), from_=0, to=7000, increment=100).pack()

ttk.Label(frame1, text='Angle Offset', font=("Arial", 12)).pack()
angleOffset_text = tk.StringVar(value=angleOffset)
tk.Spinbox(frame1, textvariable=angleOffset_text, font=("Arial", 12), from_=-360, to=360).pack()

ttk.Label(frame1, text='Min Detectable Size', font=("Arial", 12)).pack()
minSize_text = tk.StringVar(value=minSize)
tk.Spinbox(frame1, textvariable=minSize_text, font=("Arial", 12), from_=10, to=1000, increment=10).pack()

ttk.Label(frame1, text='Max Detectable Size', font=("Arial", 12)).pack()
maxSize_text = tk.StringVar(value=maxSize)
tk.Spinbox(frame1, textvariable=maxSize_text, font=("Arial", 12), from_=20, to=2000, increment=100).pack()

tk.Button(frame1, text="Save Settings", command = saveJson, font=("Arial", 14)).pack(pady=20)

frame1.pack(side = tk.LEFT)

stopEvent = threading.Event()
thread = threading.Thread(target=animate_radar, daemon=True, args=(stopEvent,))

thread.start()

tkWindow.mainloop()
