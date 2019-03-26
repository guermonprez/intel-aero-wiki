# [Video streaming](#video-streaming)

## [RTSP Streaming with the Camera Streaming Daemon](#rtsp-streaming-with-the-camera-streaming-daemon)
Camera Streaming Daemon will stream the video feeds over the network with RTSP, a standard protocol. RTSP video can be received by QGroundControl, VLC and typical video players on most platforms. It cover the RGB sensor of Intel RealSense, but also the black and white downward facing camera, the depth and Infrared sensors.
* If you're using the Yocto build provided by Intel (v1.6 or newer), the Camera Streaming Daemon is already installed and configured.
* If you're using Ubuntu, you'll need to [install it manually](#intels-camera-streaming-daemon).

On Yocto (v1.6 minimum), Intel Aero is proposing 5 RTSP video feeds:
* RealSense R200, HD camera: `rtsp://192.168.8.1:8554/video13`
* RealSense R200, depth sensor: `rtsp://192.168.8.1:8554/rsdepth`
* RealSense R200, infrared first camera: `rtsp://192.168.8.1:8554/rsir`
* RealSense R200, infrared second camera: `rtsp://192.168.8.1:8554/rsir2`
* Bottom facing black and white global shutter: `rtsp://192.168.8.1:8554/bottom`

On Ubuntu, Intel Aero is proposing 5 RTSP video feeds:
* RealSense R200, HD camera: `rtsp://AERO_IP_ON_YOUR_NETWORK:8554/video13`
* RealSense R200, depth sensor: `rtsp://AERO_IP_ON_YOUR_NETWORK:8554/rsdepth`
* RealSense R200, infrared first camera: `rtsp://AERO_IP_ON_YOUR_NETWORK:8554/rsir`
* RealSense R200, infrared second camera: `rtsp://AERO_IP_ON_YOUR_NETWORK:8554/rsir2`
* Bottom facing black and white global shutter: `rtsp://AERO_IP_ON_YOUR_NETWORK:8554/bottom`

## [RTSP video streaming from Yocto using QGroundControl](#rtsp-video-streaming-from-yocto-using-qgroundcontrol)

Procedure:
* Following the calibration procedure, you should have QGroundControl already installed and connected to Intel Aero.
* Make sure you have disabled any sort of firewall/VPN on your Host PC.
* Launch QGroundControl on Host PC. It will open into the flight path mode. You need to click on the QGC Icon to see the configuration and scroll down:

   ![](https://raw.githubusercontent.comhttps://raw.githubusercontent.com/guermonprez/intel-aero-documents/master/doc_photos/qgc_rtsp.png)

* Video settings: RTSP
* URL on Yocto: `rtsp://192.168.8.1:8554/video13`
* URL on Ubuntu: `rtsp://AERO_IP_ON_YOUR_NETWORK:8554/video13`

* Go back to the flight view. The bottom left view should show the video.

   ![](https://raw.githubusercontent.comhttps://raw.githubusercontent.com/guermonprez/intel-aero-documents/master/doc_photos/qgc_rtsp_video.png)

_**Note**: It may be possible that video stream is blocked by firewall/VPN. In that case, you may want to disable the firewall/VPN on the host PC_

## [RTSP video streaming from Yocto using Gstreamer](#rtsp-video-streaming-from-yocto-using-gstreamer)

You can access the RTSP feeds from QGroundControl or any compatible video player.
If you have a linux PC, try gstreamer with the latency=0 setting to receive video from CSD running on Intel Aero:
```bash
gst-launch-1.0 rtspsrc location=rtsp://192.168.8.1:8554/bottom latency=0 ! decodebin ! autovideosink
```

_**Note**: It may be possible that video stream is blocked by firewall/VPN. In that case, you may want to disable the firewall/VPN on the host PC_

## [RTSP video streaming from Yocto using VLC](#rtsp-video-streaming-from-yocto-using-vlc)
Using Yocto, open the network video `rtsp://192.168.8.1:8554/video13`.

![VLC RTSP](https://raw.githubusercontent.com/guermonprez/intel-aero-documents/raw/master/doc_photos/vlc_rtsp.png)

_**Note**: It may be possible that video stream is blocked by firewall/VPN. In that case, you may want to disable the firewall/VPN on the host PC_

## [Network streaming with gstreamer](#network-streaming-with-gstreamer)
We covered the Camera Streaming Daemon to stream RTSP feeds. But you can also use gstreamer directly to encode (using the GPU) and send video over the network. gstreamer is not as easy to use as CSD but very flexible. Here an example using Ubuntu, launching a gstreamer command to get video from the RGB sensor of the R200 camera (/dev/video13), select a VGA resolution at 15fps, use the hardware accelerated h264 encoder and send it to my laptop's IP (my laptop is 192.168.1.147 and I have a RTSP software ready to receive the video on this machine).
```
sudo gst-launch-1.0 v4l2src  device=/dev/video13 do-timestamp=true ! video/x-raw, format=YUY2, width=640, height=480, framerate=15/1 ! autovideoconvert ! vaapih264enc ! rtph264pay !  udpsink host=192.168.1.147 port=5600
```
It is an advanced topic posted for reference, don't worry if it looks too complex you won't need it.

# [Video and photo devices on Intel Aero](#video-and-photo-devices-on-intel-aero)

## [Linux setup](#linux-setup)
Let's focus on the Intel RealSense R200 camera included in the Intel Aero Ready-To-Fly Drone. A single R200 camera has 3 feeds. The first two are for depth, the third for RGB. Other than that, it's a standard video device on linux, and the R200 feeds are `/dev/video[11-13]`.

## [Take a RGB photo](#take-a-rgb-photo)
gstreamer is a great tool to work with video devices:
```
sudo gst-launch-1.0 v4l2src device=/dev/video13 num-buffers=1 ! jpegenc ! filesink location="rs.jpg"
identify rs.jpg 
```
should see a photo file created with the following FullHD specs:
```
rs.jpg JPEG 1920x1080 1920x1080+0+0 8-bit sRGB 50.4KB 0.000u 0:00.000
```

Here is an example of a photo of the R200 color camera (resized):

![](https://raw.githubusercontent.com/guermonprez/intel-aero-documents/raw/master/doc_photos/rs_color_intel.png)

And one from the bottom black and white camera:

![](https://raw.githubusercontent.com/guermonprez/intel-aero-documents/raw/master/doc_photos/camera_bottom.png)

## [Take a photo with the black and white camera](#take-a-photo-with-the-black-and-white-camera)

You can access the camera with `/dev/video2` but can't stream this device with gstreamer. Instead, you can download a sample code from https://github.com/intel-aero/sample-apps/tree/master/capturev4l2
to see how you can capture a few frames to raw format, then output jpg frames:
```bash
git clone https://github.com/intel-aero/sample-apps.git
cd sample-apps/capturev4l2/
make
C=10 INPUT=1 MODE=PREVIEW ./capture_v4l2 --userp -d /dev/video2
ffmpeg -f rawvideo -s 640x480 -pix_fmt yuv420p -i Image-video<*> test.jpg
```
Typical use for the camera include [optical flow](https://github.com/intel-aero/aero-optical-flow) and [Camera Streaming Daemon](https://github.com/intel/camera-streaming-daemon/blob/master/src/stream_aero_bottom.cpp)

Notes :
* Sensor is an [Omnivision OV7251 CMOS](http://www.ovt.com/sensors/OV7251).
* Driver is not fully v4l2 compliant. Using opencv or gstreamer with the camera device node to grab camera frames will not work. The sequence of operation as shown in the sample needs to be done. Polling on device file descriptor times out after certain interval. This requires the device to be restarted.
* It works on Yocto and Ubuntu
* How can I use the camera using GStreamer?: Capture frame from VGA Camera as shown in examples. Captured frames can be used by appsrc element of gstreamer. V4L2 Source plugin of gstreamer cannot be used for capturing frames from VGA Camera
* Is there any APIs to use the camera?: System Calls and IOCTLs as shown in examples must be used
* Which device corresponds to the camera?: /dev/video2

## [Take a depth/infrared photo](#take-a-depthinfrared-photo)

You can use the sample tools from the Intel RealSense SDK
```bash
cpp-headless
```
It will create 4 files from the R200 Camera: color, depth, infrared left, infrared right.
```
There are 1 connected RealSense devices.

Using device 0, an Intel RealSense R200
    Serial number: 2481009843
    Firmware version: 1.0.71.06
Writing cpp-headless-output-DEPTH.png, 480 x 360 pixels
Writing cpp-headless-output-COLOR.png, 640 x 480 pixels
Writing cpp-headless-output-INFRARED.png, 480 x 360 pixels
Writing cpp-headless-output-INFRARED2.png, 480 x 360 pixels
wrote frames to current working directory.
```

## [Record video](#record-video)
let's use vaapih264enc to take advantage of the hardware accelerated video encoding:
```
sudo gst-launch-1.0 -e v4l2src device=/dev/video13 num-buffers=2000 ! autovideoconvert format=i420 width=1920 height=1080 framerate=30/1 ! vaapih264enc rate-control=cbr tune=high-compression ! qtmux ! filesink location=encoded_video.mp4
ffmpeg -i encoded_video.mp4
```
you should see as output the specs of the FullHD video encoded:
```
Input #0, mov,mp4,m4a,3gp,3g2,mj2, from 'encoded_video.mp4':
  Metadata:
    major_brand     : qt  
    minor_version   : 537199360
    compatible_brands: qt  
    creation_time   : 2017-11-09 02:01:08
  Duration: 00:00:05.70, start: 0.000000, bitrate: 11712 kb/s
    Stream #0:0(eng): Video: h264 (High) (avc1 / 0x31637661), yuv420p, 1920x1080 [SAR 1:1 DAR 16:9], 11705 kb/s, 30 fps, 30 tbr, 3k tbn, 60 tbc (default)
    Metadata:
      creation_time   : 2017-11-09 02:01:08
      handler_name    : VideoHandler
```

## [Control the camera settings](#control-the-camera-settings)
On Ubuntu, we'll use the standard V4L2 tools: ``sudo v4l2-ctl --list-devices`` :
```
Intel RealSense 3D Camera R200 (usb-0000:00:14.0-4):
	/dev/video11
	/dev/video12
	/dev/video13
```

List possible controls with ``sudo v4l2-ctl --list-ctrls -d /dev/video13``:
```
                     brightness (int)    : min=0 max=255 step=1 default=56 value=56
                       contrast (int)    : min=16 max=64 step=1 default=32 value=32
                     saturation (int)    : min=0 max=255 step=1 default=128 value=128
                            hue (int)    : min=-2200 max=2200 step=1 default=0 value=0
 white_balance_temperature_auto (bool)   : default=1 value=1
                          gamma (int)    : min=100 max=280 step=1 default=220 value=220
                           gain (int)    : min=0 max=256 step=1 default=32 value=32
           power_line_frequency (menu)   : min=0 max=2 default=0 value=0
      white_balance_temperature (int)    : min=2000 max=8000 step=1 default=6500 value=6500 flags=inactive
                      sharpness (int)    : min=0 max=7 step=1 default=0 value=0
         backlight_compensation (int)    : min=0 max=4 step=1 default=1 value=1
                  exposure_auto (menu)   : min=0 max=3 default=3 value=3
              exposure_absolute (int)    : min=1 max=666 step=1 default=1 value=1 flags=inactive
```
And change a setting, as a test:
```
sudo v4l2-ctl -d /dev/video13 -c brightness=56
```

## [gstreamer graphical output](#gstreamer-graphical-output)
If you are connected to Intel Aero graphically, with a HDMI cable, you can run a graphical tool.
The simplest tool is gstreamer:
```
sudo gst-launch-1.0 v4l2src device=/dev/video13 ! xvimagesink
```
You can play around with the settings, like changing to VGA, 30fps:
```
sudo gst-launch-1.0 v4l2src device=/dev/video13 ! video/x-raw,width=640,height=480,framerate=30/1  ! xvimagesink
```

## [guvcview](#guvcview)
For a more user friendly tool, install guvcview with ```sudo apt install guvcview``` on Ubuntu.
Remember, there's 3 feeds for the R200 Camera. Select the third one for the color output.

![](https://raw.githubusercontent.comhttps://raw.githubusercontent.com/guermonprez/intel-aero-documents/master/doc_photos/guvcview.png)

# [Third party USB cameras](#third-party-usb-cameras)

## [Camera choice](#camera-choice)
To plug another camera, you have to consider the type of IO and the drivers.
* We have lots of possible ports, including. SPI, CAN, USB2 or USB3.
* Intel Aero is linux based (yocto as factory install or Ubuntu 16.04.3 as manual installation), you should target cameras with drivers for this selection of linux distributions. Typically it means UVC, USB Video Class.
Note: most action cameras do NOT have linux drivers publicly available.

## [Connecting and first camera shot](#connecting-and-first-camera-shot)
Here's an example with e-con systems [See3Cam USB3 camera](https://www.e-consystems.com/UltraHD-USB-Camera.asp).
It is not an Intel recommended/supported hardware. We just provide the documentation as a typical example of linux camera integration.

Plug the camera on the USB3 with the USB3-OTG cable (sold by Intel as an accessory).

Note: we're working with Ubuntu 16.04.3 in this example.

Run ``lsusb``:
```
Bus 002 Device 002: ID 8086:0a80 Intel Corp. 
Bus 002 Device 003: ID 2560:c1d0  
Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
```
There's a new line (2nd line) corresponding to the camera, showing it's correctly connected as a USB device.

Run ``dmesg``:
```
...
[ 2580.288919] usb 2-1: new SuperSpeed USB device number 3 using xhci_hcd
[ 2580.302449] usb 2-1: LPM exit latency is zeroed, disabling LPM.
[ 2580.305441] uvcvideo: Found UVC 1.00 device See3CAM_CU130 (2560:c1d0)
[ 2580.307009] uvcvideo 2-1:1.0: Entity type for entity Extension 3 was not initialized!
[ 2580.307023] uvcvideo 2-1:1.0: Entity type for entity Processing 2 was not initialized!
[ 2580.307032] uvcvideo 2-1:1.0: Entity type for entity Camera 1 was not initialized!
[ 2580.307580] input: See3CAM_CU130 as /devices/pci0000:00/0000:00:14.0/usb2/2-1/2-1:1.0/input/input5
[ 2580.310773] hid-generic 0003:2560:C1D0.0001: hiddev0: USB HID v1.11 Device [e-con systems See3CAM_CU130] on usb-0000:00:14.0-1/input2
```
The camera is recognized as a UVC device, as expected (and as a HID device, too).

Run ``ls /dev/video*`` to list the video devices:
```
/dev/video0  /dev/video1  /dev/video2  /dev/video3 ...
```
Before connecting the camera, we could see video0,1,2 corresponding to the Intel RealSense R200 camera (depth and RGB sensors). Plus other cameras listed. We now have a new video device /dev/video14. Depending on how the device is found by linux, it may have a different index number. Use `sudo v4l2-ctl --list-devices` to get the details about the camera-number association. Note: this number may change at the next boot.

It means this camera is seen as a video device by linux and can be used for video or pictures.

To capture a frame from the /dev/video14 device, you could use a variety of methods, but gstreamer is incredibly versatile:
```bash
sudo gst-launch-1.0 v4l2src device=/dev/video14 num-buffers=1 ! jpegenc ! filesink location="test.jpg"
```

A 13Mpixels, 2.4Mo jpg file is created, here are the ``identify test.jpg`` results:
```
test.jpg JPEG 4224x3156 4224x3156+0+0 8-bit sRGB 2.398MB 0.000u 0:00.000
```
The quality of the photo may not be optimal, as we have not played with the camera settings yet.

## [Camera settings](#camera-settings)
As with Intel RealSense R200, you can use guvcview and v4l2-ctl.
`sudo v4l2-ctl --list-devices`:
```
See3CAM_CU130 (usb-0000:00:14.0-1):
	/dev/video14
...
Intel RealSense 3D Camera R200 (usb-0000:00:14.0-4):
	/dev/video11
	/dev/video12
	/dev/video13
```

List possible controls with ``sudo v4l2-ctl --list-ctrls -d /dev/video14``:
```
                     brightness (int)    : min=-15 max=15 step=1 default=0 value=0
                       contrast (int)    : min=0 max=30 step=1 default=10 value=10
                     saturation (int)    : min=0 max=60 step=1 default=16 value=16
 white_balance_temperature_auto (bool)   : default=1 value=1
                          gamma (int)    : min=40 max=500 step=1 default=220 value=220
                           gain (int)    : min=0 max=100 step=1 default=0 value=0
      white_balance_temperature (int)    : min=1000 max=10000 step=50 default=5000 value=5000 flags=inactive
                      sharpness (int)    : min=0 max=127 step=1 default=16 value=16
                  exposure_auto (menu)   : min=0 max=3 default=1 value=0
              exposure_absolute (int)    : min=1 max=10000 step=1 default=156 value=312 flags=inactive
                   pan_absolute (int)    : min=-648000 max=648000 step=3600 default=0 value=0
                  tilt_absolute (int)    : min=-648000 max=648000 step=3600 default=0 value=0
                  zoom_absolute (int)    : min=100 max=800 step=1 default=100 value=100
```
And change one, as a test:
```
sudo v4l2-ctl -d /dev/video14 -c brightness=56
```


