#!/usr/bin/env python3
import json
from pythontuio import TuioServer #pip install python-tuio
from pythontuio import Cursor
from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import BlockingOSCUDPServer

def leftCursor(address, *args):
    #print(f"{address} {args}")
    tuioCursor.position = (args[0]/2, args[1])
    print(f"{args[0]/2}, {args[1]}")
    tuioServer.send_bundle()

def rightCursor(address, *args):
    #print(f"{address} {args}")
    tuioCursor.position = ((args[0]/2)+0.5, args[1])
    print(f"{(args[0]/2)+0.5}, {args[1]}")
    tuioServer.send_bundle()

def readConfig():
    with open('appconfig.json') as json_file:
        data = json.load(json_file)
    return data

# Read Config File
config = readConfig()
oscPort = config["oscPort"]

#TUIO Server
tuioServer = TuioServer()
tuioCursor = Cursor(123) # sets session_id to 123

tuioCursor.velocity = (0.2,0.1)
tuioCursor.motion_acceleration  = 0.1

tuioServer.cursors.append(tuioCursor)

dispatcher = Dispatcher()
dispatcher.map("/cursorLeft", leftCursor)
dispatcher.map("/cursorRight", rightCursor)

ip = "0.0.0.0"
port = oscPort
print("Ready")
oscServer = BlockingOSCUDPServer((ip, port), dispatcher)
oscServer.serve_forever()  # Blocks forever
