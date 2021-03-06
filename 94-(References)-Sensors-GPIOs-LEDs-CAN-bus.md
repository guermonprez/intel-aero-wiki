# Intel Aero Compute Board Hardware Features
The content found in this section augments much of the details found in the [Hardware Features and Usage] (https://www.intel.com/content/dam/support/us/en/documents/boardsandkits/aero/intel-aero-compute-board-guide.pdf) document.

# Flight Controller Block Diagram
![Aero Flight Controller](https://github.comhttps://raw.githubusercontent.com/guermonprez/intel-aero-documents/blob/master/doc_photos/fcm.png)

* The purpose of the illustration is to focus on the Aero Flight Controller connections.

* The "connectors" and components in this diagram are representational and do not necessarily reflect the actual location on the circuit board assembly. 

* Sizes are not to scale. The Aero Compute Board is larger than the Aero Flight Controller module. 

* All signals from the Aero Flight Controller (except the SDIO interface and CAN bus) are routed through the MAX10 FPGA to the IO Expansion connector.  The pin assignments can be found in the [Hardware Features and Usage](https://www.intel.com/content/www/us/en/support/products/98471/drones/development-drones/intel-aero-products/intel-aero-ready-to-fly-drone.html) document.

* The pin functions from the Aero Flight Controller may vary according to the STM32's pinmux configuration. This diagram reflects the original assignment when the Aero Ready to Fly Drone first shipped and may change in the future.

* The Flight Controller hardware contains a CAN bus transceiver. It cannot be used at the same time as USART1 (Motors) which is required by the Intel Aero Ready to Fly Drone.  Note that the Compute Board contains a separate CAN bus described below.


# CAN Bus (Flight Controller)

The Intel Aero Flight Controller includes a [TJA1051 CAN bus transceiver](http://www.nxp.com/products/automotive-products/energy-power-management/can-transceivers/high-speed-can-transceiver:TJA1051?lang_cd=en).  This CAN bus is multiplexed with the flight controller's embedded micro-controller USART1 interface.

One possible way to utilize the flight controller CAN bus is through PX4's [UAVCAN ](https://dev.px4.io/en/uavcan/)library. However, the PX4 firmware included in Aero releases currently do not have this enabled by default. The user will need to recompile the firmware and update the flight controller. 

An example patch is available [here](https://github.com/broody/Firmware/commit/55042b60480ce3dc04b5247b768087de71e68444) with the required changes to enable UAVCAN. This has been tested with Zubax Babel USB dongle. 

_Above contains information about the Intel Aero Flight Controller._

_Below contains information about the sensors, LEDs and GPIOs available on Intel Aero Compute Board._   It is useful to refer to the [Hardware Features and Usage](https://www.intel.com/content/dam/support/us/en/documents/boardsandkits/aero/intel-aero-compute-board-guide.pdf) document.


# Inertial Measurement Unit (IMU), Pressure and Magnetometer Sensors
## Architecture
In addition to the IMU built into the flight controller (Intel Aero FC or a third party FC), the Intel Aero Compute Board includes a 6-DOF IMU: [BMI 160 sensor](http://www.mouser.com/ds/2/783/BST-BMI160-DS000-07-786474.pdf). This sensor is connected to the Intel Atom processor via its _SPI interface_ on bus 3 (SPI3) chip select 0 (CS0). It can be accessed via spidev as `/dev/spidev3.0`.  The Aero Compute Board also integrates a magnetometer and a pressure sensor on its I2C.

## Command to access the IMU Sensor on the Aero Compute Board

* An example to communicate with BMI 160 is shown below:
```
        root@intel-aero~# spi_xfer -b 3 -c 0 -d 0x80 -w 2
```
where,
```
    * -b is for spi bus
    * -c is for chip select
    * -d is the data value
    * -w is for number of words
```

_Note_:
Users can also follow [linux sample program](https://git.kernel.org/cgit/linux/kernel/git/stable/linux-stable.git/tree/Documentation/spi/spidev_fdx.c?id=refs/tags/v4.4.32) showing an example on how to use spidev interface.

## Command to access the Magnetometer Sensor
Intel Aero Compute Board includes [BMM 150 sensor](http://www.mouser.com/ds/2/783/BST-BMM150-DS001-01-786480.pdf). This sensor is connected to Atom SoC via _I2C interface_ on bus 2 at slave address _0x12_. In order to communicate with this sensor, please use below linux i2c commands:
```
    root@intel-aero~# i2cdetect -y -r 2
    root@intel-aero~# i2cset -y 2 0x12 DATA-ADDRESS [VALUE] ... [MODE]
    root@intel-aero~# i2cget -y 2 0x12 [DATA-ADDRESS [MODE]]
```
## Command to access the Pressure Sensor
Intel Aero Compute Board includes [MS5611 pressure sensor](http://www.te.com/usa-en/product-CAT-BLPS0036.html). This sensor is connected to Atom SoC via _I2C interface_ on bus 2 at slave address _0x76_. In order to communicate with this sensor, please use below linux i2c commands:
```
    root@intel-aero~# i2cset -y 2 0x76 DATA-ADDRESS [VALUE] ... [MODE]
    root@intel-aero~# i2cget -y 2 0x76 [DATA-ADDRESS [MODE]]
```


# LEDS
Intel Aero Compute Board has 2 LEDs that can be controlled via the Intel Atom processor GPIO Sysfs interface: one Tri-color LED and one Orange LED.

## Tricolor LED
This LED is located on the top view of Aero Compute BoarD (refer to the [Hardware Features and Usage] (https://www.intel.com/content/dam/support/us/en/documents/boardsandkits/aero/intel-aero-compute-board-guide.pdf) document. It can be configured as:

### Red
The GPIO number for red LED is **437**. Use below commands on [Intel Aero console](https://github.comhttps://raw.githubusercontent.com/guermonprez/meta-intel-aero/wiki/Quickstart-Guide#how-to-get-to-a-console) to turn this LED on:
```
    echo 437 > /sys/class/gpio/export
    echo out > /sys/class/gpio/gpio437/direction
    echo 1 > /sys/class/gpio/gpio437/value
```
To turn it off:
```
    echo 0 > /sys/class/gpio/gpio437/value
    echo 437 > /sys/class/gpio/unexport
```
### Green
The GPIO number for green LED is **397**. Use below commands on [Intel Aero console](https://github.comhttps://raw.githubusercontent.com/guermonprez/meta-intel-aero/wiki/Quickstart-Guide#how-to-get-to-a-console) to turn this LED on:
```
    echo 341 > /sys/class/gpio/export
    echo out > /sys/class/gpio/gpio397/direction
    echo 1 > /sys/class/gpio/gpio397/value
```
To turn it off:
```
    echo 0 > /sys/class/gpio/gpio397/value
    echo 341 > /sys/class/gpio/unexport
```
### Blue
The GPIO number for blue LED is **403**. Use below commands on [Intel Aero console](https://github.comhttps://raw.githubusercontent.com/guermonprez/meta-intel-aero/wiki/Quickstart-Guide#how-to-get-to-a-console) to turn this LED on:
```
    echo 403 > /sys/class/gpio/export
    echo out > /sys/class/gpio/gpio403/direction
    echo 1 > /sys/class/gpio/gpio403/value
```
To turn it off:
```
    echo 0 > /sys/class/gpio/gpio403/value
    echo 347 > /sys/class/gpio/unexport
```
## Orange LED
This LED is located on the bottom view of [Aero Compute Board](https://software.intel.com/sites/default/files/managed/25/d5/Intel-Aero-Compute-Board-Getting-Started.pdf). 

The GPIO number for orange LED is **507**. Use below commands on [Intel Aero console](https://github.comhttps://raw.githubusercontent.com/guermonprez/meta-intel-aero/wiki/Quickstart-Guide#how-to-get-to-a-console) to turn this LED on:
```
    echo 507 > /sys/class/gpio/export
    echo out > /sys/class/gpio/gpio507/direction
    echo 1 > /sys/class/gpio/gpio507/value
```
To turn it off:
```
    echo 0 > /sys/class/gpio/gpio507/value
    echo 507 > /sys/class/gpio/unexport
```
## Sample script to toggle the LEDs
```
    #!/bin/sh

    gpios="437 397 403 507"

    for i in $gpios; do
      echo $i > /sys/class/gpio/export
      echo out > /sys/class/gpio/gpio$i/direction
      echo 1 > /sys/class/gpio/gpio$i/value
      sleep 1
      echo 0 > /sys/class/gpio/gpio$i/value
      sleep 1
      echo $i > /sys/class/gpio/unexport
    done
```
## ESC LED
There are two LEDs located under each ESC, however, these cannot be controlled. When they are blinking, this means the flight controller is operational. During update to either the flight controller or FPGA these LEDs will stop blinking.

# Processor GPIOs
The following Aero Compute Board Processor GPIOs are available:

| Processor GPIO Name | Kernel GPIO # |
|:-------------:|:-------------:|
| CPU_GPIO_01   | GPIO-350      |
| CPU_GPIO_02   | GPIO-481      |
| CPU_GPIO_03   | GPIO-485      |
| CPU_GPIO_04   | GPIO-348      |
| CPU_GPIO_05   | GPIO-487      |
| CPU_GPIO_06   | Power Off     |
| CPU_GPIO_07   | GPIO-319      |

Note: Grounding this CPU_GPIO_06 for 3 seconds will initiate shutdown on the Compute Board.  This is hardcoded in the BIOS.  It cannot be repurposed by the user.

Use below commands on [Intel Aero console](https://github.comhttps://raw.githubusercontent.com/guermonprez/meta-intel-aero/wiki/Quickstart-Guide#how-to-get-to-a-console) to toggle these processor GPIOs:

Configure CPU_GPIO_01 as an output and drive it High / Low
```
    echo 486 > /sys/class/gpio/export
    echo out > /sys/class/gpio/gpio486/direction
    echo 1 > /sys/class/gpio/gpio486/value
```
Configure CPU_GPIO_01 as an input and read the value
```
    echo 486 > /sys/class/gpio/export
    echo in > /sys/class/gpio/gpio486/direction
    cat /sys/class/gpio/gpio486/value
```
# CAN Controller (Compute Board)

The Intel Aero Compute Board includes a [MCP2515 CAN controller](http://ww1.microchip.com/downloads/en/DeviceDoc/21801e.pdf) and [MCP2562 CAN transceiver](http://ww1.microchip.com/downloads/en/DeviceDoc/20005167C.pdf).  The controller is connected to the Atom processor via the _SPI interface_ on bus 1 (SPI1) chip select 0 (CS0). It can be accessed via spidev as `/dev/spidev1.0`

### Example SocketCAN configuration
This example shows you how to setup socket based communication between two Aero boards connected via the CAN bus. **Minimum BSP requirement version is v1.6**

Make sure that the mcp251x driver is initialized. If initialized properly you should be able to see can0 network interface.

```
    root@intel-aero~# ifconfig -a
    can0      Link encap:UNSPEC  HWaddr 00-00...
```

If the interface is not see try to reload the driver.

```
    root@intel-aero~# rmmod mcp251x && modprobe mcp251x
```

Configure the ip link for the can interface to specify the can bus bitrate.

```
    root@intel-aero~# ip link set can0 type can bitrate 125000 triple-sampling on
```

Bring the network interface up.

```
    root@intel-aero~# ifconfig can0 up
```

On the Aero board that is receiving incoming CAN data use command

```
    root@intel-aero~# candump
```

On the Aero board that is sending CAN data use command

```
    root@intel-aero~# cansend 5A1#11.22.33.44.55.66.77.88
```

# 8Mpx Camera kernel support

These are the instructions to change kernel configuration options on [www.yoctoproject.org](http://www.yoctoproject.org/docs/2.3/dev-manual/dev-manual.html#configuring-the-kernel)
 
For changing the kernel without having to replace the current one what I do is to have an
hybrid approach. First part is to get the source and configuration, second part is to deploy
on the board (that I use for any distro, not only yocto).
 
* Get the source (same as in https://github.com/intel-aero/intel-aero-manifest/blob/master/default.xml#L22-L30)
```
repo init -u https://github.com/intel-aero/intel-aero-manifest.git
repo sync
TEMPLATECONF=../meta-intel-aero/conf/ . poky/oe-init-build-env
```

* Download and configure the kernel
```
bitbake linux-yocto -c kernel_configme -f
cd tmp/work/corei7-64-intel-common-poky-linux/linux-yocto/4.4.*/linux-*
```

Here you have a normal kernel tree you can make changes, change configuration, etc.
If you want to have the equivalent git tree with history and all commits we added,
just go to the "source" directory there.
 
* Deploy
 
These instructions are the same for any board and is not special to Aero. It assumes you are
building the kernel on your machine, not on Aero. After building the kernel as per (1), just
copy vmlinuz and kernel modules over to where you want to test them (in our case, aero board).
 
Over time this becomes tedious so I have some scripts to automate this:
https://github.com/lucasdemarchi/toolbox/blob/master/custom-kernel-gen
 
To use it, clone the directory above and add to your PATH. Then, after building the kernel you do:
```
$ custom-kernel-gen -x && scp linux-*.sh intel-aero.local:
$ ssh intel-aero.local
(aero) $ ./linux-.sh
```
 
The first time you do this you will need to configure grub. Just open
/boot/EFI/BOOT/grub.cfg and copy the 3 lines fom the boot menuentry, give it another name like 'test'
and change from /vmlinuz to /vmlinuz-test. You may want to change the default configuration there, but
I prefer leaving it as is and selecting the entry I want to test during boot.
 
After doing this once, if you are changing just a kernel module you can just copy the relevant .ko file to have
and even faster workflow.
