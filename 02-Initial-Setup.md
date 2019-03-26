# [Hardware](#hardware)

## Video

Video recording of the install procedure described in this page:
[Video: Autonomous Drone Engineer Course - Module C1 - Out of Box and Flashing](https://www.youtube.com/watch?v=e9MLnRbMDHo&index=7&list=PLTQSXsG86pGfyZm5ac6-ZtQsEniUJIE9o)

## Power
If you use the wall socket power adapter, connect the cable as described in [the first page](https://github.comhttps://raw.githubusercontent.com/guermonprez/meta-intel-aero/wiki/01-About-Intel-Aero#wall-socket-power-adapter).
If you use a LiPo battery, refer to the manual of your charger and battery to safely charge your LiPo battery. Then connect the battery to Intel Aero XT60 connector.

A short press of the power button should switch on Intel Aero. You'll hear a beep and see light coming from the adjacent LEDs.

## Connecting cables
You have the choice between 2 options:
* recommended method: connect a USB key, hub, keyboard, mouse and HDMI screen to flash the system and have full local graphical login
* alternate method: connect a USB key only and flash the system

Recommended method: Connect all devices
* OTG USB cable to Aero (you can reuse an old USB2 OTG cable, but if you're buying a new one pick a USB3 - [check the cable reference notes](https://github.comhttps://raw.githubusercontent.com/guermonprez/meta-intel-aero/wiki/08-Aero-Network-and-System-Administration#sysadmin-ubuntu-installation))
* USB hub to OTG cable (optional, only needed if keyboard and USB thumb drive are used at the same time)
* keyboard, mouse and USB thumb drive (see next chapter on how to build this key)
* micro HDMI cable to Aero

Alternate method: Connect a USB key only
* OTG USB cable to Aero (you can reuse an old USB2 OTG cable, but if you're buying a new one pick a USB3 - [check the cable reference notes](https://github.comhttps://raw.githubusercontent.com/guermonprez/meta-intel-aero/wiki/08-Aero-Network-and-System-Administration#sysadmin-ubuntu-installation))
* USB key to OTG USB cable

![USB-OTG and microHDMI connected](https://raw.githubusercontent.com/guermonprez/intel-aero-documents/raw/master/doc_photos/cables_usb-hdmi-connected_small.jpg)

[link to the hi-resolution photo](https://raw.githubusercontent.com/guermonprez/intel-aero-documents/raw/master/doc_photos/cables_usb-hdmi-connected.jpg)

## Plugging / Unplugging the fan
* If you use the Intel Compute Board, there is a passive heat-sink and no fan. It is ok as long as you don't run complex code that would cause Aero to heat and require a fan. If you do, we recommend you install a temporary fan (ex: USB powered fan) during development.
* If you use the Intel Aero Ready to Fly Drone, a fan is included. If you plan to leave the drone on your desk all day and do not plan to run complex codes that would cause the board to heat, unplugging the fan may be an option. To do so, unscrew the top screws, slightly open the lid and unplug the fan connector.

# [Flashing](#flashing)
## Different parts of the drone to flash
Intel Aero Compute Board is like your laptop or server: it has an UEFI BIOS and an operating system.

But the Compute Board also has a FPGA, in charge of routing the IOs between the processor (SoC, System-on-Chip to be precise) and the motors plus flight controller. In certain cases, the FPGA will need to be flashed too.

If you have the Intel Aero Ready to Fly Drone, you also have the Intel Aero Flight Controller in it. You will have to flash it when you first use the drone and in certain cases (if you do flight controller development or want to change some FC settings). Flashing the FC means you'll have to go though the QGroundControl sensor calibrating procedure. Intel is shipping the drone with PX4 already flashed, but you can choose between PX4 and Ardupilot as both are included in the default filesystem iso image. [ArduPilot.org](http://ardupilot.org/copter/docs/common-intel-aero-rtf.html) also has some documentation.

To summarize:
1. .iso files for the Linux operating system
2. .rpm for the BIOS, part of the iso file (also available as a separate downloadable file).
3. .jam for the FPGA, part of the iso file
4. .px4 for the Flight Controller, part of the iso file (both PX4 and Ardupilot have a PX4 extension)

Where to get the files:
* [downloadcenter.intel.com page to download the .iso linux image](https://downloadcenter.intel.com/download/26389/UAV-installation-files-for-Intel-Aero-Platform?v=t).
* .rpm .jam and .px4 are part of the .iso

Checking the .iso file:
* After downloading files, always check the md5 hash of the file matches the md5 published on Intel's download website. Do not flash if the hashes do not match.
* To get the checksum on a linux workstation type:
```shell
md5sum intel-aero-image-intel-aero-1.6.iso
```
![md5sum on linux](https://raw.githubusercontent.com/guermonprez/intel-aero-documents/raw/master/doc_photos/install_01_md5sum_linux.png)
* On windows, open [Win32 Disk Imager](https://sourceforge.net/projects/win32diskimager/) and check the "md5 hash" option:

![md5sum on windows](https://raw.githubusercontent.com/guermonprez/intel-aero-documents/raw/master/doc_photos/install_01_md5_windows.png)


## Flashing Intel Aero Linux Distribution

### Create an USB drive to flash the Operating System
Easy method: For Linux, Windows and MacOS you can use [Etcher](https://etcher.io/).
* Insert the removable/USB disk to the windows machine (it will be formated)
* Launch Etcher
* Select the .iso file, the USB drive and click on the "Flash" button
* Wait for the image to be written and verified

Alternate method: There are other tools available for specific operating systems, if it's preferred. Windows users can use [Win32 Disk Imager](https://sourceforge.net/projects/win32diskimager/) and Linux/MacOS users can use the following command in the terminal:

  ```shell
  sudo dd if=/path/to/your/image/intel-aero-image-intel-aero-1.6.iso of=/dev/sdX bs=1M
  ```

Where X is the letter of your USB key (be careful to format the USB key and not your main disk !). It may take a few minutes.

### Flash Intel Aero Linux distribution - recommended method

For the recommended method (HDMI screen connected):
* shut down Intel Aero (unplug-wait 5s-replug)
* press ESC to enter the BIOS
* select boot from USB key
* when booting from USB key, select "install"

### Flash Intel Aero Linux distribution - alternate method

For the alternate method (no screen attached):
* Connect the USB drive to Intel Aero using the OTG cable.
* If you already have a system with firmware v1.2 or more recent, type the command:
```bash
aero-reboot-update.py
```
If this command does not work, it means you have an older version. Type:
```bash
mkdir -p /tmp/{iso,newroot}
mount -o ro /dev/sda1 /tmp/iso
mount -o loop,ro /tmp/iso/rootfs.img /tmp/newroot
/tmp/newroot/usr/sbin/aero-reboot-update.py
```
The commands above will verify you have a valid USB drive connected, do some sanity checks and reboot into the update image.

Here's how to follow the flash progress during the alternate method flash:
* If you have the Aero Compute Board, the RGB LED on the board will blink and change color.
* If you have the Aero RTF Drone, the blue LEDs next to the power button to convey feedback about the update progress. The RGB LED may be difficult to see on the RTF since it's inside the case. You can look to the hole through each the FAN cable is passing in order to check the color of the RGB LED.
![RGB Led on RTF](https://github.comhttps://raw.githubusercontent.com/guermonprez/intel-aero-documents/raw/master/doc_photos/rgb-led-rtf.png)

LED colors and frequencies:
1. By default the LED is green
1. Before rebooting the RGB LED will become yellow: this means the board started the reboot progress. It takes ~20 seconds. If the BIOS was not updated yet, the reboot command may get stuck. You can remove power and power it on again.
1. When board is rebooted the RGB LED becomes green again.
1. During the update process the RGB LED and the LEDs next to the power button (on RTF) will blink slowly at 1Hz. The time taken by the update process may vary, but should take around 3 minutes.
1. When finished the LEDs will blink fast at 10Hz for 5 seconds and reboot. After that the update process is done and you should be greeted with the new version.

## Flashing the BIOS
* Although the BIOS file (.rpm) is part of the Aero image (.iso), always download the latest BIOS from the Intel Download Center [[link](https://downloadcenter.intel.com/download/27399/Intel-Aero-Platform-for-UAVs-Installation-Files?v=t )] and copy onto the Aero disk space.  

  To install, first remove the previous version (v1.00.13) and then install the latest (v1.00.16)

```
  rpm -ev aero-bios-01.00.13-r1.corei7_64
  rpm -i aero-bios-01.00.16-r1.corei7_64.rpm
  aero-bios-update
```
* Then reboot, the UEFI BIOS will detect the files and update. You should see (if you have a screen connected):
  ```
  GoTo FOTA Process Begin.
  Back up BIOS for seamless recovery. File size is: 0x3C9000
  CHTT_X64_RELEASE_RECOVERY.ROM written onto eMMC. Continue to update Firmware.
  Set FOTA Process Step:  21
  Start to update firmware 
  Updating firmware… <n%> Completed.
  Flash Update Complete Status Success. Ready to reset…
  Set FOTA Process Step:  FF
  AFU: Delete File CHTT_X64_RELEASE_RECOVERY.ROM on eMMC.
  AFU: Delete File BIOSUpdate.FV on eMMC.
  Finish FOTA Capsule Update Process.
  ```
* If the Intel Aero Compute Board reboots automatically and endlessly after the completion of the BIOS update process, a cold reboot is required.
* Check the BIOS version:
  ```shell
  aero-get-version.py
  ```
  You should see a message like this (or newer):
  ```shell
  BIOS_VERSION = Aero-01.00.13
  OS_VERSION = Poky Aero (Intel Aero linux distro) v1.6.0 (pyro)
  AIRMAP_VERSION = 1.8
  FPGA_VERSION = 0xc2
  Aero FC Firmware Version = 1.6.5
  ```


## Flashing the FPGA

Aero comes with 3 FPGA firmwares that can be used, all of them under the `/etc/fpga/` directory.
* `aero-rtf.jam`: this is for use with RTF
* `aero-rtf-recovery.jam`: this is for use with the RTF under special circunstances: it allows Aero to instruct the flight controller to stop on bootloader so we can flash new versions of the firmware even if the firmware stops responding due to a bad update
* `aero-compute-board.jam`: this should be selected if using only the compute board. Note that the labels that accompany the compute board have no meaning. Check the official documentation for each pin.

Flash the new FPGA firmware:
```bash
cd /etc/fpga/
jam -aprogram <firmware>.jam
```

In which `<firmware>` is one of the firmwares above.

## Flashing the Flight Controller (RTF only)

A Flight Controller is included in the Intel Aero Ready to Fly Drone and this FC needs to be updated too. The current release of Intel Aero includes the recommended version for PX4. The firmwares are located on `/etc/px4-fw/` directory. To update the flight controller use the following command:
```bash
cd /etc/aerofc/px4/
aerofc-update.sh nuttx-aerofc-v1-default.px4
```
Note: the script will try all the possible serial port speeds. It is usually able to sync and flash within 10 seconds, but can take longer in some cases. If it does not, let it try for a few minutes and try again.

You can also flash a new version of the firmware directly from the [PX4 repository](https://github.com/PX4/Firmware/). Follow the instructions on their [wiki](https://dev.px4.io/hardware-intel-aero.html) for that.

Note: In parallel of the Intel provided PX4 version, you can install the Ardupilot version:
```bash
cd /etc/aerofc/ardupilot/
aerofc-update.sh arducopter-aerofc-v1.px4
```
Note: the script will try all the possible serial port speeds. It is usually able to sync and flash within 10 seconds, but can take longer in some cases. If it does not, let it try for a few minutes and try again.

Note: if you see an infinite loop you can run: 
```bash
aerofc-force-bootloader-pin.py 1
```
to force flight stack go to bootloader and start the update, then update, and after the update run:
```bash
aerofc-force-bootloader-pin.py 0
```
to allow PX4/Ardupilot to boot.

After updating the flight stack, make sure to go through the calibration procedures.

## Check the BIOS-FPGA-FC-OS version:
  ```shell
  aero-get-version.py
  ```
  You should see a message like this (or newer):
  ```shell
  BIOS_VERSION = Aero-01.00.13
  OS_VERSION = Poky Aero (Intel Aero linux distro) v1.6.0 (pyro)
  AIRMAP_VERSION = 1.8
  FPGA_VERSION = 0xc2
  Aero FC Firmware Version = 1.6.5
  ```

# [Calibration](#calibration)
## Calibration?
The Inertial Motion Unit (IMU) and compass used by the Intel Aero Flight Controller to stabilize the drone need to be calibrated. When you receive the RTF Drone from the factory, it is already calibrated.
But after you flash the flight controller, the calibration settings are not valid anymore. You need to recalibrate.

You'll need a computer or tablet running [QGroundControl (QGC)](http://qgroundcontrol.com/downloads/) and connected to the Intel Aero access point (called "Aero-*", password "1234567890").

Open up QGroundControl and ensure the GPS and Battery are active “colored black” and have basic functionality in the top panel:

![QGC](https://raw.githubusercontent.com/guermonprez/intel-aero-documents/raw/master/doc_photos/calibration_01.png)

Navigate to the Airframe menu and select “Reset” to reset the drone’s airframe configuration. Select “Apply and Restart” and restart the drone

![QGC](https://raw.githubusercontent.com/guermonprez/intel-aero-documents/raw/master/doc_photos/calibration_02.png)

![QGC](https://raw.githubusercontent.com/guermonprez/intel-aero-documents/raw/master/doc_photos/calibration_03.png)

Wait 1 minute after the drone powers on, and reconnect to its AP.

QGroundControl may need to be restarted to reconnect to the drone.

Press the Bind button on the Spektrum Transmitter while turning it on and ensure a series of 3 fast continuous fast beeps are heard.

On QGroundControl, navigate to the Radio menu and select “Calibrate”.

Follow the instructions to calibrate the transmitter, selecting “Next” after each change

![QGC](https://raw.githubusercontent.com/guermonprez/intel-aero-documents/raw/master/doc_photos/calibration_04.png)

Navigate to the Sensors menu. Select “Ok” to start the sensor calibration process

![QGC](https://raw.githubusercontent.com/guermonprez/intel-aero-documents/raw/master/doc_photos/calibration_05.png)

Select Compass menu and press “Ok” to start the calibration process

![QGC](https://raw.githubusercontent.com/guermonprez/intel-aero-documents/raw/master/doc_photos/calibration_06.png)

Position then rotate the drone as indicated in each of the 6 figures

![QGC](https://raw.githubusercontent.com/guermonprez/intel-aero-documents/raw/master/doc_photos/calibration_07.png)

Select Gyroscope menu and it will start automatically. Wait until it completes

![QGC](https://raw.githubusercontent.com/guermonprez/intel-aero-documents/raw/master/doc_photos/calibration_08.png)

Select Accelerometer menu. Position the drone in each of the positions until each of the 6 are marked as green and “Completed"

![QGC](https://raw.githubusercontent.com/guermonprez/intel-aero-documents/raw/master/doc_photos/calibration_09.png)

Select the Level Horizon menu and select “Ok”. The calibration will start and stop automatically

![QGC](https://raw.githubusercontent.com/guermonprez/intel-aero-documents/raw/master/doc_photos/calibration_10.png)

Navigate to Flight Modes and set the following 3 modes:
* Flight Mode 1 – Position
* Flight Mode 4 – Altitude
* Flight Mode 5 – Manual

![QGC](https://raw.githubusercontent.com/guermonprez/intel-aero-documents/raw/master/doc_photos/calibration_11.png)

When this procedure is finished, you should be able to get data in QGC from the drone (ex: GPS coordinates) but also act on the drone (ex: ARM motors, without propellers first).

# [Next steps](#next-steps)
After flashing, you have a updated working RTF Drone or Compute Board.

By default the Drone is working:
* network: as a wifi access point, and accepting ssh connections
* login: presenting a very basic terminal on the display

If you plan to fly with the remote control, go to the [first flight page](03-First-flight).

You can also enable the [video streaming](06-Cameras-and-Video) between the drone and your computer.

But if you plan to start coding from your workstation on Aero over the network, install software on Intel Aero and access internet then you may want to change the network settings of Aero and join the [wifi network of your home/company](08-Aero-Network-and-System-Administration#networking-wifi).

After your first flight, you may want to [install Ubuntu](90-(References)-OS-user-Installation) if you prefer Ubuntu to Yocto.
