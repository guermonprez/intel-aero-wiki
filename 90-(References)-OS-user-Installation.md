# [Choice of OSes](#choice-of-oses)

Intel is shipping Intel Aero (RTF Drone and Compute Board) with Yocto linux preinstalled and you should download and flash the updated version of this image ([details here](02-Initial-Setup)) to update the hardware when you receive it. This Yocto build is preconfigured and highly customized for Intel Aero.

In parallel to this Intel supported Yocto image, full native Ubuntu* with Intel drivers are now verified by Intel (as user installation). Intel is providing documentation and packages to let you install Ubuntu yourself, manually. We do NOT provide a prepackaged and customized version of Ubuntu. Compared to Yocto, Ubuntu can be interesting for rapid prototyping and software development.

Yocto for a Ready-To-Fly experience or Ubuntu for prototyping and development, you have the choice.

Video recording of the install procedure described in this page: [Video: Autonomous Drone Engineer Course - Module C3 - Installing Ubuntu](https://www.youtube.com/watch?v=14DZ18dzoEc&list=PLTQSXsG86pGfyZm5ac6-ZtQsEniUJIE9o&index=9)

# [Installing Ubuntu on Intel Aero](#installing-ubuntu-on-intel-aero)

## [Upgrade Yocto first](#upgrade-yocto-first)

To keep it simple, [please upgrade to the latest version of Yocto](02-Initial-Setup) and flash the BIOS, FPGA, Flight Controller. Check everything works, then install Ubuntu. It will replace Yocto and keep the BIOS, FPGA and Flight Controller.

## [OS](#os)

* Download [Ubuntu 16.04.3 x64 Desktop](http://old-releases.ubuntu.com/releases/16.04.3/). Note: do not try a more recent version of Ubuntu.
* Create a bootable disk (refer to the Ubuntu documentation)
* Plug Intel Aero to the wall power supply, the USB-OTG adapter, bootable USB key, hub, keyboard and mouse.
Power on
* Type ESC to enter the BIOS
* Select boot manager, select your USB key and press Enter
* Install Ubuntu as you would on a computer
* In terms of options, my preferences are:
  * I went with a full disk install (erase Yocto and use all space for Ubuntu)
  * I do not install third party proprietary software (flash, mp3)
  * I choose to have my session opening automatically (it has networking consequences).

## [Intel Aero repository](#intel-aero-repository)

* Connect Intel Aero to the network with internet access (using Ubuntu's network manager)
* Open a terminal
* Add the Intel Aero repository (and key) to your sources:
```bash
echo 'deb https://download.01.org/aero/deb xenial main' | sudo tee /etc/apt/sources.list.d/intel-aero.list
wget -qO - https://download.01.org/aero/deb/intel-aero-deb.key | sudo apt-key add -
sudo apt-get update
sudo apt-get upgrade
sudo apt-get -y install gstreamer-1.0 libgstreamer-plugins-base1.0-dev libgstrtspserver-1.0-dev gstreamer1.0-vaapi gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-libav ffmpeg v4l-utils python-pip
sudo pip install pymavlink
sudo apt-get -y install aero-system
sudo reboot
```

## [Intel Aero Maintenance tools](#intel-aero-maintenance-tools)

### QGroundControl

QGroundControl is required to calibrate the Flight Controller and useful to pilot the drone.

When you're using Yocto, the drone is a DHCP server (and WiFi hotspot). So the drone knows the IP of your laptop and is able to send the MAVLink telemetry feed to the right IP. But if you're using Ubuntu, you are probably connected to the same network with variable IPs. Get your laptop IP and run on Aero:
* `sudo mkdir /etc/mavlink-router/config.d`
* create this file: `sudo gedit /etc/mavlink-router/config.d/qgc.conf` and fill it with:
```
[UdpEndpoint wifi]
Mode = Normal
Address = 192.168.1.147
```
(my laptop IP is 192.168.1.147 but your IP will be different)
* restart the router: `sudo systemctl restart mavlink-router`
* launch QGroundControl on your laptop. It should receive automatically the telemetry feed from the drone.
* Follow the [setup procedure](02-Initial-Setup) to calibrate your Flight Controller, unless you've done it already.

### Get Version

```bash
sudo aero-get-version.py
```
will return BIOS, FPGA and OS information, but not yet the Flight Controller version. We'll update the tool very soon.
Airmap is not installed yet, so version not reported is expected. 

### BIOS

To flash the BIOS that's part of the Aero repo, type

```bash
sudo aero-bios-update
sudo reboot
```

OR to flash the latest BIOS release from the Intel Download Center [[Link](https://downloadcenter.intel.com/download/27399/Intel-Aero-Platform-for-UAVs-Installation-Files)], download the RPM and extract it.

```bash
  rpm2cpio aero-bios-01.00.16-r1.corei7_64.rpm | cpio -idmv
  sudo mv BIOSUPDATE.fv /boot
  sudo reboot
```

### FPGA

Similar to Yocto: `sudo jam -aprogram /etc/fpga/aero-rtf.jam` (`aero-rtf.jam` if you are using the RTF drone, `aero-compute-board.jam` if you are using the board only).

### Flight Controller

That's valid only for the Ready To Fly drone.
```bash
cd /etc/aerofc/px4/
sudo aerofc-update.sh nuttx-aerofc-v1-default.px4
```
For more details, refer to the [main setup page](02-Initial-Setup#flashing-the-flight-controller-rtf-only).

## [Intel RealSense SDK](#intel-realsense-sdk)

Different hardware sensors:
* If you have the Intel Aero Ready To Fly Drone, you have Intel RealSense R200 Camera included.
* If you buy a third party RealSense Camera, you may have a newer model such as D435 and the special [Intel USB3-OTG cable](https://click.intel.com/usb-3-0-otg-cable-for-the-intel-aero-platform.html).

Different libraries: The R200 requires the `legacy` branch of the SDK, and D435 the main branch.

Different kernel drivers: the kernel driver for Intel RealSense R200 is already included in the Intel Aero repository. Nothing else to do. Do NOT try to install the module from the SDK or with snap.

### Intel RealSense SDK - legacy (for R200)
```bash
cd
sudo apt-get -y install git libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev cmake
git clone -b legacy --single-branch https://github.com/IntelRealSense/librealsense.git
cd librealsense
mkdir build && cd build
cmake ../ -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true
make
sudo make install
```

### Intel RealSense SDK - main branch (for D435)
```bash
cd
sudo apt-get -y install git libusb-1.0-0-dev pkg-config libgtk-3-dev libglfw3-dev cmake
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
mkdir build && cd build
cmake ../ -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true
make
sudo make install
```
Note: For D435, you will need the special [Intel USB3-OTG cable](https://click.intel.com/usb-3-0-otg-cable-for-the-intel-aero-platform.html). No other USB3-OTG cable will work.

## [Intel Camera Streaming Daemon](#intel-camera-streaming-daemon)

Camera streaming daemon gets installed with `aero-system`.
To verify, check: 
```
systemctl status csd
```
You should see something like:
```bash
csd.service - Camera Streaming Daemon
   Loaded: loaded (/lib/systemd/system/csd.service; disabled; vendor preset: enabled)
   Active: active (running) since Wed 2017-11-08 17:28:00 PST; 6min ago
 Main PID: 14616 (csd)
   CGroup: /system.slice/csd.service
           └─14616 /usr/bin/csd

Nov 08 17:28:00 frelon systemd[1]: Started Camera Streaming Daemon.
Nov 08 17:28:00 frelon csd[14616]: Could not open conf file '/etc/csd/main.conf' (No such file or directory)
Nov 08 17:28:01 frelon csd[14616]: Failed to connect to Mir: Failed to connect to server socket: No such file or directory
Nov 08 17:28:01 frelon csd[14616]: Unable to init server: Could not connect: Connection refused
Nov 08 17:28:01 frelon csd[14616]: (gst-plugin-scanner:14618): Clutter-CRITICAL **: Unable to initialize Clutter: Could not initialize 
Nov 08 17:28:01 frelon csd[14616]: (gst-plugin-scanner:14618): Clutter-Gst-CRITICAL **: Unable to initialize Clutter
Nov 08 17:28:02 frelon csd[14616]: AVAHI START
```
We have started CSD without configuring it manually: it will find the video devices automatically and propose feeds. As a test, use VLC on your computer and open the video2 feed `rtsp://192.168.1.4:8554/video13`. On my network, Intel Aero has IP 192.168.1.4 but yours will be different of course (type `ifconfig wlp1s0` to see your IP).
![VLC RTSP](https://raw.githubusercontent.com/guermonprez/intel-aero-documents/raw/master/doc_photos/vlc_rtsp.png)

For more understanding, visit [https://github.com/01org/camera-streaming-daemon](https://github.com/01org/camera-streaming-daemon)

## [Intel Optical Flow](#intel-optical-flow)
Optical flow requires you buy and plug a vertical range meter, like a vertical laser. For more details, visit: [https://github.com/intel-aero/aero-optical-flow](https://github.com/intel-aero/aero-optical-flow).

Optical Flow also requires a software running on linux. This package gets installed with `aero-system`. Optical flow service is disabled by default, enable using:
```
systemctl enable aero-optical-flow
systemctl start aero-optical-flow
```
Note: do NOT enable the service if you have not installed the vertical range finder (laser).

Check status:
```
systemctl status aero-optical-flow
```
The output would look like:
```
aero-optical-flow.service - Aero optical flow
   Loaded: loaded (/lib/systemd/system/aero-optical-flow.service; enabled; vendor preset: enabled)
   Active: active (running) since Wed 2018-01-03 11:32:35 IST; 1s ago
 Main PID: 2629 (aero-optical-fl)
   CGroup: /system.slice/aero-optical-flow.service
           └─2629 /usr/bin/aero-optical-flow

Jan 03 11:32:35 aero-CherryTrail systemd[1]: Started Aero optical flow. 
```



## [Checks](#checks)

### Kernel version

Check the kernel version with `uname -a`, you should see:
```
Linux frelon 4.4.76-aero-1.2 #1 SMP PREEMPT Mon Nov 6 19:42:57 UTC 2017 x86_64 x86_64 x86_64 GNU/Linux
```
  It should have aero in the kernel name.

### Number of video devices

To check if the camera drivers are correctly installed, list the video devices with
```bash
ls /dev/video*
```
You should see a list like `/dev/video0  /dev/video1  /dev/video2 ...` up to 13. If less than 13, it's a problem (If you see only 3 video devices, it means you booted the wrong kernel).

The order of devices is also important. Check the details with
```
sudo v4l2-ctl --list-devices
```

You should see a long listing, including:
```
Intel RealSense 3D Camera R200 (usb-0000:00:14.0-4):
	/dev/video11
	/dev/video12
	/dev/video13
```
R200 should have the devices 11, 12, 13.

### MAVLink router

Check if the system is listening for MAVLink messages on port 5760 thanks to MAVLink router (for the Ready To Fly Drone only):
```bash
netstat -l|grep 5760
```
you should see:
```
tcp        0      0 *:5760                  *:*                     LISTEN     
```
If not, it's a problem. (You may have booted the wrong kernel, or something else is wrong)

### RealSense

Test: run `cpp-enumerate-devices` to see if your camera is detected. On the Ready To Fly Drone, you should see:
```
Device 0 - Intel RealSense R200:
 Serial number: 2481009843
 Firmware version: 1.0.71.06
 USB Port ID: 2-4
 Camera info: 
    DEVICE_NAME         : 	Intel RealSense R200
    DEVICE_SERIAL_NUMBER: 	2481009843
    CAMERA_FIRMWARE_VERSION: 	1.0.71.06
    CAMERA_TYPE         : 	PRQ-Ready
    OEM_ID              : 	OEM None
    ISP_FW_VERSION      : 	0x0
    CONTENT_VERSION     : 	12
    MODULE_VERSION      : 	4.2.5.0
    IMAGER_MODEL_NUMBER : 	31
    CALIBRATION_DATE    : 	2014-07-04 08:18:35 UTC
    EMITTER_TYPE        : 	Laser Driver 3
    FOCUS_VALUE         : 	0
    LENS_TYPE           : 	Newmax 58.9 x 45.9 degs in VGA
    3RD_LENS_TYPE       : 	Newmax 71.7 x 44.2 degs in 1080p
    LENS_COATING__TYPE  : 	Visible-light block / IR pass 43 nm width
    3RD_LENS_COATING_TYPE: 	IR coating
    NOMINAL_BASELINE    : 	70 mm
    3RD_NOMINAL_BASELINE: 	58 mm
 Supported options:                                    min        max       step  default  
    COLOR_BACKLIGHT_COMPENSATION                       : 0    ... 4           1     1         
    COLOR_BRIGHTNESS                                   : 0    ... 255         1     56        
...
```

To see the output of a camera, use the SDK tools. Examples:
* `cpp-tutorial-1-depth` (command line)
* `cpp-capture` (graphical). You should something like:

![RS output](https://raw.githubusercontent.com/guermonprez/intel-aero-documents/raw/master/doc_photos/rs_cpp-capture_chairs.png)

### CSD

Check the port 8554 is listening with `netstat -l|grep 8554`. You should see:
```bash
tcp        0      0 *:8554                  *:*                     LISTEN
```

# [Next steps](#next-steps)

You now have a working Ubuntu setup working on Intel Aero.
If you want to start coding, check the [Autonomous Drone Programming with Python](04-Autonomous-drone-programming-in-Python) page.
