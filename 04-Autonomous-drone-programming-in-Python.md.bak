# [Introduction](#introduction)

Please check our [online course](https://github.com/guermonprez/intel-aero-documents/tree/master/course) (slides or videos) to understand:
* how to connect to the flight controller
* what's the general software architecture
* how to use yocto or ubuntu or the linux OS of choice in a container

## [Installation](#installation)

On this page, I suppose you're using Ubuntu installed [as described on this page](90-(References)-OS-user-Installation). Install the packages:
```bash
sudo apt-get install python-pip python-opencv python-opencv-apps python-zbar zbar-tools vim-python-jedi vim-python-jedi python-editor eric idle vim-nox-py2
pip install Cython numpy
pip install pyrealsense
```

## [Local / Remote editing](#)

With Ubuntu running on Intel Aero, you have a full desktop running locally.
When you have a keyboard, mouse and hdmi screen connected on Intel Aero, you can launch regular graphical Ubuntu editors (eric, idle, ...). Or you can launch a terminal on your development station and run a command line editor (vim, nano) over ssh.

* example: vim over ssh

![vim](/guermonprez/intel-aero-documents/raw/master/doc_photos/python_vim.png)

* example: IDLE

![idle](/guermonprez/intel-aero-documents/raw/master/doc_photos/python_idle.png)

* example: eric

![eric](/guermonprez/intel-aero-documents/raw/master/doc_photos/python_eric.png)

# [Flight control](#flight-control)
## [Hello world in PyMAVLINK](#hello-world-pymavlink)
Here's a simple python script using the basic pymavlink wrapper to arm the motors for 3 seconds.
Arming the motors is the simplest action we can test to show everything is connected.
Note: we're using `tcp:127.0.0.1:5760` to connect to the flight controller, as we'll do for all the following examples.

__UNPLUG THE PROPELLERS BEFORE RUNNING THIS CODE. WE INSIST.__

```python
#!/usr/bin/python
from __future__ import print_function

import pymavlink.mavutil as mavutil
import sys
import time

mav = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
mav.wait_heartbeat()
mav.mav.command_long_send(mav.target_system, mav.target_component,
                          mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1,
                          0, 0, 0, 0, 0, 0)
time.sleep(3)
mav.mav.command_long_send(mav.target_system, mav.target_component,
                          mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0,
                          0, 0, 0, 0, 0, 0)
```
The motors should spin for 3 seconds.
If they don't, it may be because:
* you have not calibrated your flight controller with QGroundControl
* the radio remote is not connected
* your wall power supply does not provide enough power

## [Hello world in DroneKit](#hello-world-dronekit)
It’s important to know the basics of MAVLINK, as it the base of all communications with the Flight Controllers.
But coding frames with python-mavlink is not developer friendly.
DroneKit, developed by [3D Robotics](http://3drobotics.com), is one of the friendly python abstractions available under Apache v2 Licence : [python.dronekit.io](http://python.dronekit.io) 
To install on Intel Aero, first connect your drone to a [WiFi network (in client mode)](08-Aero-Network-and-System-Administration#networking-internet-access) with internet access and install Dronekit:
```shell
pip install dronekit
```
__UNPLUG THE PROPELLERS BEFORE RUNNING THIS CODE. WE INSIST.__

Here's the code, still arming the motors for 5 seconds:
```python
#!/usr/bin/python
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

vehicle = connect('tcp:127.0.0.1:5760', wait_ready=False)
print "Arming motors:"
vehicle.mode    = VehicleMode("GUIDED")
vehicle.armed   = True
while not vehicle.armed:
        print "  Waiting for arming to be finished"
        time.sleep(1)
print "Keeping motors armed for 5s"
time.sleep(5)
print "Disarming"
vehicle.armed   = False

```

## [Hello world in PyMAVProxy](#hello-world-PyMAVProxy)

On top of being a developer friendly layer on top of MAVLINK, MAVProxy was designed to bridge the gap between programming-only libraries like DroneKit and graphical-only tools like QGroundControl.
Check: [ardupilot.github.io/MAVProxy](http://ardupilot.github.io/MAVProxy)
Some people use it on a remote computer to control the drone and have optional user interfaces to complex functions, but you can use on the drone itself for autonomous drone development.

MAVProxy is using the MAVLINK protocol, but is focused on the Ardupilot variant, not the PX4 variant. IF you're interested in MAVProxy, you'll have to flash the Flight Controller with the Ardupilot firmware.
To install on Intel Aero, first connect your drone to a [WiFi network (in client mode)](08-Aero-Network-and-System-Administration#networking-internet-access) with internet access and install MAVProxy:
```shell
pip install MAVProxy
```

__UNPLUG THE PROPELLERS BEFORE RUNNING THIS CODE. WE INSIST.__

Launch the shell
```shell
mavproxy.py --master=tcp:127.0.0.1:5760 --quadcopter
```
And type a few commands to arm/disarm the motors:
```shell
arm throttle
disarm
bat
```

# [Cameras](#cameras)

## [Intel RealSense](#intel-realsense)

We're covering Python programming with Intel RealSense SDK (pyrealsense) with model R200, included in the Intel Aero Ready To Fly Drone. If you have a newer camera like the D430, you'll need [another SDK](90-(References)-OS-user-Installation#intel-realsense-sdk).

```python
## setup logging
import logging
logging.basicConfig(level = logging.INFO)

## import the package
import pyrealsense as pyrs

## start the service - also available as context manager
serv = pyrs.Service()

## create a device from device id and streams of interest
cam = serv.Device(device_id = 0, streams = [pyrs.stream.ColorStream(fps = 60)])

## retrieve 60 frames of data
for _ in range(60):
    cam.wait_for_frames()
    print(cam.color)

## stop camera and service
cam.stop()
serv.stop()
```

## [OpenCV and RealSense](#opencv-and-realsense)

You can consider the depth data coming from the RealSense camera as an image, and manipulate it with OpenCV.
[Here's an example](https://github.com/toinsson/pyrealsense/blob/master/examples/show_cv2.py).

## [Barcodes](#barcodes)

Let's use zbar to detect barcodes from the camera:
```python
from sys import argv
import zbar
proc = zbar.Processor()
proc.parse_config('enable')

# set the correct device number for your system
device = '/dev/video13'
if len(argv) > 1:
    device = argv[1]
proc.init(device)
proc.process_one()
for symbol in proc.results:
    print 'barcode type=', symbol.type, ' data=', '"%s"' % symbol.data

```
And here's a [test sheet with various sizes of barcodes](/guermonprez/intel-aero-documents/raw/master/doc_photos/barcode_test.png).

# [Networking](#networking)

## [Networked drone in WebSockets](#network-ws)
Please refer to the module `D2 - Software - Networked Drone` of the course for more information about the following codes.

__UNPLUG THE PROPELLERS BEFORE RUNNING THIS CODE. WE INSIST.__

The Python websocket API on Intel Aero:
```
pip install websocket-server
```

A [WebSocket client in your browser](https://chrome.google.com/webstore/detail/smart-websocket-client/omalebghpgejjiaoknljcfmglgbpocdp), to simulate a web call:
 
Optionally, a Library on your development station to send the request from a script instead of your browser:
```
pip install websocket-client
```

And here's the server code running on Intel Aero:
```python
#!/usr/bin/python
from websocket_server import WebsocketServer
import re
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
vehicle = connect('tcp:127.0.0.1:5760', wait_ready=False)
vehicle.mode    = VehicleMode("GUIDED")
print("Flight Controller Connected")
def new_client(client, server):
        print("Client connected")
        server.send_message_to_all("Client connected")
def message_received(client, server, message):
	if len(message) > 200:
		message = message[:200]+'..'
	print("Arming motors")
	vehicle.armed   = True
	while not vehicle.armed:
		time.sleep(1)
	time.sleep(5)
	print("Disarming")
	vehicle.armed   = False
server = WebsocketServer(8080, '0.0.0.0')
server.set_fn_new_client(new_client)
server.set_fn_message_received(message_received)
server.run_forever()
```

And the optional client code, running on a remote computer (change the IP 192.168.0.100 with Aero's IP on YOUR NETWORK):
```python
#!/usr/bin/python
from websocket import create_connection
ws = create_connection("ws://192.168.0.100:8080")
ws.send("Alert, send drone")
result = ws.recv()
print("Received '%s'" % result)
ws.close()
```
## [Networked drone in MQTT](#network-mqtt)
Please refer to the module `D2 - Software - Networked Drone` of the course for more information about the following codes.

To install the Python MQTT API on Intel Aero (should be there already):
```
pip install paho-mqtt 
```
A [MQTT client in your browser](https://chrome.google.com/webstore/detail/mqttlens/hemojaaeigabkbcookmlgmdigohjobjm?hl=en), to generate a message posting.

__UNPLUG THE PROPELLERS BEFORE RUNNING THIS CODE. WE INSIST.__

Here's the client code, running on Intel Aero:

```python
#!/usr/bin/python
import paho.mqtt.client as mqtt
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
vehicle = connect('tcp:127.0.0.1:5760', wait_ready=False)
vehicle.mode    = VehicleMode("GUIDED")
print("Flight Controller Connected")

def on_connect(client, userdata, rc):
	print("Client connected ")
	client.subscribe("aero-paul")
def on_message(client, userdata, msg):
	print("Arming motors ("+msg.topic+"/"+str(msg.payload)+")")
	vehicle.armed   = True                                   
	while not vehicle.armed:                                 
		time.sleep(1)                           
	time.sleep(5)                                   
	print("Disarming")                            
	vehicle.armed   = False                       

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect("test.mosquitto.org", 1883, 60)
client.loop_forever()
```

# [Deep Learning with Intel Movidius NCS](#deep-learning-with-intel-movidius-ncs)

After installing the Intel Movidius Neural Compute Stick [as described on this page](07-Movidius-Neural-Compute-Stick) and make sure the `make examples` result is ok, go to the first Python NCS code sample:
```
cd ~/workspace/ncsdk/examples/apps/hello_ncs_py
python hello_ncs.py
```
it should output
```
Hello NCS! Device opened normally.
Goodbye NCS! Device closed normally.
NCS device working.
```

For a real life example:
```
cd ~/workspace/ncsdk/examples/caffe/AlexNet
make run_py
```
it should output:
```
...
------- predictions --------
prediction 0 (probability 94.970703125%) is n03272010 electric guitar  label index is: 546
prediction 1 (probability 4.76684570312%) is n02676566 acoustic guitar  label index is: 402
prediction 2 (probability 0.102138519287%) is n02787622 banjo  label index is: 420
prediction 3 (probability 0.0409126281738%) is n04517823 vacuum, vacuum cleaner  label index is: 882
prediction 4 (probability 0.0347137451172%) is n04141076 sax, saxophone  label index is: 776
```

# [Peripherals](#peripherals)

## Intel Aero board LEDs
There's a multicolor LED on top of the board (if the board is in the enclosure, you can see the light from  the white cable hole), and an orange LED under the board. As the LEDs are enclosed in the Ready-To-Fly design, it is not very useful. But if you build your own drone design or enclosing you may want to let the LEDs visible and use them.
To install the IO module:
```shell
pip install python-periphery
```

And here is a sample code to test all the LED colors:

```python
#!/usr/bin/python
import time
from periphery import GPIO

print "Top LED Blue"
gpio = GPIO(403, "out")
gpio.write(bool(1))
time.sleep(1)
gpio.write(bool(0))
gpio.close()

print "Top LED Green"
gpio = GPIO(397, "out")
gpio.write(bool(1))
time.sleep(1)
gpio.write(bool(0))
gpio.close()

print "Top LED Red"
gpio = GPIO(437, "out")
gpio.write(bool(1))
time.sleep(1)
gpio.write(bool(0))
gpio.close()

print "Bottom LED Orange"
gpio = GPIO(507, "out")
gpio.write(bool(1))
time.sleep(1)
gpio.write(bool(0))
gpio.close()
```

## CAN
The Intel Aero Compute Board includes a MCP2515 CAN controller and MCP2562 CAN transceiver. The controller is connected to the Atom processor via the SPI interface on bus 1 (SPI1) chip select 0 (CS0). It can be accessed via spidev as /dev/spidev1.0. [Python spidev libraries](https://github.com/doceme/py-spidev)

