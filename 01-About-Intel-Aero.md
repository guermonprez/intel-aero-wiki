# [Is Intel Aero right for me?](#is-intel-aero-right-for-me)

Intel Aero is a set of components to help drone developers design and build their own drones:
* __Drone hardware integrators__ can use the individual components : Intel Aero Compute Board, Intel Aero Vision Accessory Kit, Intel Aero Enclosure Kit to build their own drone solution.
* __Software developers__ can use the Intel Aero Ready-To-Fly ("RTF") drone to start coding and run flight tests without assembling a drone themselves. When their software is ready and the drone hardware specifications are clear, they can work with one of the drone hardware integrators to get a custom drone built for their usage model by using the Intel Aero board and sensors (Intel RealSense, Movidius). Important: the Intel Aero Ready-To-Fly drone itself is not a consumer product but a software development kit for professionals. If you are looking for a consumer product, we highly recommend Yuneec's [Typhoon H](https://www.yuneec.com/en_US/products/typhoon/h/overview.html).
* __Professors__ can directly use the Intel Aero Ready To Fly ("RTF") drone to build their autonomous drone course. We provide an [open source reference course](https://github.com/guermonprez/intel-aero-documents/tree/master/course) for you to help build your own. We can also help with pedagogical consulting (paul.guermonprez@intel.com) to find the best way to introduce the topic in your existing course.

Typically, a professional drone project targeting production and certification would require a collaboration between hardware and software specialists. Both could start working in parallel and end up with a product using Intel Aero Compute Board, probably Intel RealSense 3D sensors, your choice of motors, propellers frame and a software you would develop for your specific needs.

As a professional, you'll have to follow the laws and regulations of your country. As an example, for US consult the [Federal Aviation Authority](https://www.faa.gov/uas/) website on Unmanned Aircraft Systems.

[Video: Autonomous Drone Engineer Course - Module A1 - Intel Aero in 5mn](https://www.youtube.com/watch?v=7t7l885g8dI&list=PLTQSXsG86pGfyZm5ac6-ZtQsEniUJIE9o)

[Video: Autonomous Drone Engineer Course - Module A2 - Usage Models](https://www.youtube.com/watch?v=sLUyQYcc4vc&list=PLTQSXsG86pGfyZm5ac6-ZtQsEniUJIE9o&index=2)


# [Intel Aero: resources, support, links](#intel-aero-ressources-support-links)

Refer to the [links page](99-External-Links)

# [OSes and SW development methods](#oses-and-sw-development-methods)

## OSes
By default, Intel Aero board and Intel Aero Ready To Fly kit are delivered with a Yocto Project build already flashed. [Yocto project](https://www.yoctoproject.org/) is an open source set of tools for embedded professionals. The UEFI BIOS is maintained by [InsydeH20](https://www.insyde.com/insydeh2o-intel-aero).

Many developers may prefer prototyping with typical linux distributions like [Ubuntu*](https://www.ubuntu.com/) and you can [install it manually on Intel Aero](90-(References)-OS-user-Installation).

To learn more about docker, check our [course modules](https://github.com/guermonprez/intel-aero-documents/tree/master/course) `B4 - Architecture - Software Architecture` and `D3 - Software - Docker Containers`.

## Development languages
You are then free to code in the language of your choice. Typical choices by drone software developers include:
* C/C++ programming
* Python high level programming and prototyping, with a proposed selection of libraries for flight control and computer vision
* ROS, the [Robotic Operating System](http://www.ros.org/). A basic installation is provided by Intel on our Yocto binary build.
We will focus on documentation on Python and ROS.

## Development environments
In terms of development environment, you can:
* Code on your workstation and upload the code to Intel Aero for execution. You can then use the IDE of your choice. This is the commended approach as you can use all the tools from your host machine rather than depending on the development libraries installed on Aero Compute Board.
* The standard installation also ships with a minimum set of development tools which makes it possible to do some experiments on the board itself. For this case use case it's suggested to connect via ssh and do changes locally or use sftp/sshfs to edit files in your editor of choice
* Code on your workstation only, if you do not have Intel Aero hardware. Simulators like Gazebo and SITL are available to simulate the drone and flight controller.


# [What’s in the box, what’s not in the box](#whats-in-the-box-whats-not-in-the-box)

## Contents of the kits

|                                | Compute Board kit | Ready to Fly kit | Vision Accessory kit | Enclosure kit |
| ------------------------------ |:-----------------:|:----------------:|:--------------------:|:---------------:|
| Compute Board                  |         YES       |         YES      |          no          | no |
| Flight Controller              |         no        |         YES      |          no          | no |
| RealSense 200 3D camera        |         no        |         YES      |          YES         | no |
| Monochome camera               |         no        |         YES      |          YES         | no |
| RGB 8Mp camera                 |         no        |         YES      |          YES         | no |
| frame, ESC, motors, propellers |         no        |         YES      |          no          | no |
| Radio emitter/receiver         |         no        |         YES      |          no          | no |
| Wifi antennas                  |         YES       |         YES      |          no          | no |
| Enclosure                      |         no        |         YES      |          no          | YES |
| Power supply                   |         no        |         no       |          no          | no |
| Power supply cable adapter     |         YES       |         no       |          no          | no |
| OTG USB 2.0 cable              |         YES       |         no       |          no          | no |
| OTG USB 3.0 cable              |         no        |         no       |          no          | no |
| Micro HDMI cable               |         no        |         no       |          no          | no |
| USB 3.0 hub                    |         no        |         no       |          no          | no |
| LiPo battery                   |         no        |         no       |          no          | no |
| LiPo charger                   |         no        |         no       |          no          | no |

## Photos of the kits
* Intel Aero Compute Board
![unboxing Intel Aero Compute Board](/guermonprez/intel-aero-documents/raw/master/doc_photos/unboxing_Intel_Aero_Compute_Board_small.jpg)
[link to the hi-resolution photo](/guermonprez/intel-aero-documents/raw/master/doc_photos/unboxing_Intel_Aero_Compute_Board.jpg)

* Intel Aero Vision Accessory kit
![unboxing Intel Aero Vision Accessory kit](/guermonprez/intel-aero-documents/raw/master/doc_photos/unboxing_Intel_Aero_Computer_Vision_kit.jpg)
[link to the hi-resolution photo](/guermonprez/intel-aero-documents/raw/master/doc_photos/unboxing_Intel_Aero_Computer_Vision_kit_small.jpg)

* Intel Aero Ready to Fly kit
![unboxing Intel Aero Vision Accessory kit](/guermonprez/intel-aero-documents/raw/master/doc_photos/unboxing_Intel_Aero_RTK_kit_small.jpg)
[link to the hi-resolution photo](/guermonprez/intel-aero-documents/raw/master/doc_photos/unboxing_Intel_Aero_RTK_kit.jpg)

* Intel Aero Enclosure kit
![unboxing Intel Aero Vision Accessory kit](/guermonprez/intel-aero-documents/raw/master/doc_photos/unboxing_Intel_Aero_Enclosure_kit_small.jpg)
[link to the hi-resolution photo](/guermonprez/intel-aero-documents/raw/master/doc_photos/unboxing_Intel_Aero_Enclosure_kit.jpg)

## Cables and USB hub

Intel Aero has two ports of interest for software developers: one micro USB 3.0 OTG and one micro HDMI.
You will need cables as soon as you receive the drone or compute baord, as it is required to update the system components. You'll need:


* USB OTG adapter: you can reuse an old USB 2.0 OTG adapter or buy a new USB 3.0 OTG adapter.
* If you plan to plug a mouse, keyboard, plus the USB thumb key used to flash the system, you'll need a USB hub. If you plan to plug on top of that some big USB devices like cameras, pick a powered USB 3.0 hub.

 
Required accessories:
* Micro HDMI to standard HDMI cable and HDMI monitor
* Support for USB 2.0 peripherals:
    * Self-powered USB 2.0 Hub
    * OTG Cable1: USB 2.0 Micro-B to Type-A female (This is the same cable supplied with the Intel Aero Compute Board)
* USB keyboard
* USB flash drive 2GB or larger for firmware updates
 
Optional accessories:
* USB mouse
* Support for USB 3.0 peripherals:
    * Self-powered USB 3.0 Hub
    * OTG cable2: (non-standard) USB3.0 Micro-B to Type-A female (Available as an accessory through Intel (coming in Q4 2017))

## Powering the Intel Aero Ready to Fly Drone for developers
If you plan to fly, you'll need a LiPo battery.
As LiPo batteries are difficult to ship and most drone developers already have their batteries and chargers, a battery is not included in the Ready to Fly kit.
LiPo battery specifications:
* capacity (4S or 3S)
* connector XT60
* size: most 4S/3S batteries will fit (the maximum battery size is 150mm x 50mm x 32mm)
* Note that a 4000 mAh battery will last about 24 minutes while hovering


Most LiPo batteries require a LiPo charger-balancer. Some "smart batteries" have a cell balancer included and only require a power adapter.

Note: LiPo batteries are very dense in energy and can be dangerous is not handled properly. If you are not already a RC Hobbyist, then working with a professional or joining a RC club may be a good idea.
In case you plan to fly with your drone, be aware that airlines have strict rules regarding the transportation of LiPo batteries.

## Wall socket power adapter
The Intel Aero Compute Board does not have a wall socket power adapter. A barrel cable adapter is provided. Recommended power adapter specs:
  * 5V DC
  * 3A (15 watts) minimum
  * cable: 5.5mm x 2.1mm barrel connector (OD / ID), center-pin positive

There is no power adapter provided for the Ready to Fly Drone, but you may want to add one if you plan to code for long periods of time and don't want to handle LiPo batteries on your desk. Recommended power adapter specs:
  * 9V - 16.8V DC at 4A minimum   (this covers the voltage ranges for 3S and 4S batteries)
  * 12V DC at 4A is recommended if using for bench testing the motors. Power draw at more than idle may exceed the power adapter specifications
  * cable: The Ready to Fly Drone already has a XT60 male connector intended for the LiPo battery, but the power supply you'll find will mostly have a 5.5mm male barrel jack plug. That's why you'll need to buy or build a XT60 female to jack 5.5 female cable.

Important: DO NOT CONNECT both a wall adapter & battery at the same time.

![Intel Aero Ready to Fly XT60 connector](/guermonprez/intel-aero-documents/raw/master/doc_photos/cables_XT60_small.jpg)

[link to the hi-resolution photo](/guermonprez/intel-aero-documents/raw/master/doc_photos/cables_XT60.jpg)

![Intel Aero Ready to Jack connector](/guermonprez/intel-aero-documents/raw/master/doc_photos/cables_jack_small.jpg)

[link to the hi-resolution photo](/guermonprez/intel-aero-documents/raw/master/doc_photos/cables_jack.jpg)




