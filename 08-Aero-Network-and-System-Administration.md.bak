# [Networking: LTE](#networking-lte)

Please refer to the [Enabling LTE Modems page](90-(References)-Enabling-LTE-Modems). You will need firmware 1.6.0 or newer with ALL PARTS FLASHED (BIOS, ...).

# [Networking: WiFi](#networking-wifi)

With Yocto: By default, Intel Aero is shipping with Yocto installed and is proposing an access point with SSID Aero-<MAC> (where <MAC> is the MAC address of Intel Aero) and password 1234567890. IP of Intel Aero is 192.168.8.1. You can ssh to this IP address with login root and no password.

If you want Intel Aero to join a WiFi network in client mode, here is the procedure:
```
nmcli c down hotspot
nmcli c modify hotspot connection.autoconnect no
# scan wifi networks
nmcli dev wifi
# connect
nmcli dev wifi connect <network name> password <password>
```

With Ubuntu*: If you install Ubuntu* manually on Intel Aero, it will work in client mode by default. You can join a WiFi network using the typical network manager tools.

# [Networking: Access Point Wifi](#networking-access-point-wifi)
Please refer to the module `C2 - Lab Setup - Network and ssh` [of the course](https://github.com/guermonprez/intel-aero-documents/tree/master/course) for more details. Here are the basic facts:

By default, Intel's Yocto for Aero is broadcasting a WiFi access point called `Aero-<MAC>` and **Wi-Fi password is 1234567890**.

The IP of Aero is 192.168.8.1. Login is root, no password.

Launch a terminal and connect to aero via SSH as follows:

```bash
ssh root@192.168.8.1
```

After connecting to Aero it's advised to change the passkey and ssid by using the following commands:
```bash
nmcli c modify hotspot 802-11-wireless-security.psk <new passkey>
nmcli c modify hotspot 802-11-wireless.ssid <new ssid>
```

Need to restart NetworkManager with below command to take effect of new SSID
```bash
systemctl restart NetworkManager
```
You can also edit the configuration file in /etc/NetworkManager/system-connections/hotspot.

On macOS and Linux it's possible to use the `link-local` name rather than fixed IP 192.168.8.1. Refer to the documentation of your distro on how to enable it; it's enabled by default on Fedora and [Archlinux's documentation](https://wiki.archlinux.org/index.php/avahi#Hostname_resolution)
works on several Linux distributions and macOS:
```bash
ssh root@intel-aero.local
```

# [Networking: Internet Access](#networking-internet-access)
Please refer to the module `C2 - Lab Setup - Network and ssh` [of the course](https://github.com/guermonprez/intel-aero-documents/tree/master/course) for more details. Here are the basic facts:

In order to have internet access on Aero board it's possible to switch it to client mode so it connects to your Access Point instead of being an Access Point itself. We'll use the standard connman.

Here's the procedure:
* First you need to disable AP mode which is the default one:
```bash
nmcli c down hotspot
Connection 'hotspot' successfully deactivated (D-Bus active path: ...
nmcli c modify hotspot connection.autoconnect no
```
* Scan WiFi networks with nmcli dev wifi command. Example:
```
nmcli dev wifi
   ABCD-C072-5         Infra  157   54 Mbit/s  79      ***   WPA1 WPA2
   MYOFFICEWLAN        Infra  44    54 Mbit/s  75      ***   WPA1 WPA2 802.1X 
   LabWLAN             Infra  44    54 Mbit/s  75      ***   WPA2 802.1X
   AndroidHotspot      Infra  44    54 Mbit/s  74      ***   WPA2 802.1X
   OfficeWLAN2         Infra  1     54 Mbit/s  74      ***   WPA2 802.1X
```
* Connect to one network by giving the desired ssid and passkey. From the example above we can connect to ABCD-C072-5 with:
```bash
nmcli dev wifi connect ABCD-C072-5 password 12mysecretpassword34
```
With this configuration it will try to autoconnect to this network every time you boot Intel Aero board.

## Switch WiFi back to Access Point mode

With Yocto: In case you switched the interface to work in client (e.g. to connect to the Internet) mode you may get it back in AP mode with the following commands:
```
nmcli c up hotspot
Connection successfully activated (D-Bus active path: ... 
nmcli c modify hotspot connection.autoconnect yes
```

## WiFi factory reset

If you changed the WiFi configuration and don't remember the passkey, or would just like to reset to the initial configuration given by the OS, you can use the following commands:

```bash
rm /etc/sysconfig/networkmanager
reboot
```

The board will reboot and re-configure itself with default configuration.

# [UART speed between Intel Aero Compute Board and Intel Aero Flight Controller](#uart-speed-between-intel-aero-compute-board-and-intel-aero-flight-controller)

This changes the baudrate that communicates between the Intel Aero Flight Controller and the Intel Aero Computer Board. In order to get a reliable communication there are some changes on the Linux side that's done here and will be part of a new version in future. Without upgrading the distro/packages these are the equivalent instructions:

* Add this line to the `/lib/systemd/system/mavlink-router.service` under `[Service]` section:
```
ExecStartPre=/bin/bash -c 'echo 4 > /sys/class/tty/ttyS1/rx_trig_bytes'
```
* Change baudrate of the uart port for mavlink-router in `/etc/mavlink-router/main.conf`
* Change baudrate of the update script in `/usr/sbin/aerofc-update.sh`


# [SysAdmin: Docker Introduction](#sysadmin-docker-intro)
If you are using Yocto but would like to deploy a separate OS on top of Yocto, you can use a docker image of this OS. But remember you can also use [Ubuntu* and install it manually on Intel Aero](90-(References)-OS-user-Installation). Docker is an advanced and complex topic, use at your risk.

Intel Aero is shipped with a Yocto Linux build: Yocto is great as an embedded build system for professionals, including in the aeronautics sector where it is widely used.
But it is not very friendly for rapid prototyping: Some may prefer an OS like Ubuntu*.

There’s several possible mechanisms to create this separation and choice of OS for deployment. On Intel Aero we support containers. A container is a set of resources (cpu, memory ...) allocated to a set of processes. It’s a lot like a virtual machine but much lighter and efficient as it works at the process level and not the OS level.
It is very popular in the server world, where the toolbox Docker is allowing you to create development environments on your station (Linux, Mac, Windows) and migrate them to servers seamlessly.
To summarize, containers behave like a virtual machine but also like a way to package and deploy apps.

Here's how to get started:

The `docker` command line tool allow the management of containers.
The `run` command with parameters `--interactive (-i)` and `--tty (-t)` opens a console session within
the container environment. The run command requires a docker image name in order to create
a new container. The following example shows the execution of a container based on Ubuntu*
16.04:

```bash
docker run --privileged -it ubuntu:16.04
```

The parameter `--privileged` grants full access to host devices.
Notice the first execution of run will download a docker image to satisfy the request for a new
container.
After that, an Ubuntu* 16.04 environment will be available, providing commands such as apt for
package management, etc.
To exit a container, simply use Ctrl+d or execute the exit ​command. When you exit a container,
all of the modifications you have done inside *will be lost* unless you explicitly save the state
(more info below).
To find a list of docker images available, check https://hub.docker.com/search/.
Useful commands:
* `docker images` will list existing docker images;
* `docker ps` will list the containers currently in execution. The values in column
“CONTAINER ID” can be used to execute other docker commands
* `docker commit <CONTAINER ID>` will save the existing state of a running container
into a new image, this needs to be run outside of a docker instance, so if you are already
inside of one you can ssh back into the drone by running: ssh 172.17.0.1. You can also
have multiple ssh connections to aero, leaving one inside the docker container and
another on the host. The screen command on the host also works, so you can multiplex
a single connection
* `docker exec -it <CONTAINER ID> bash` can be used to have multiple terminals inside
the container - or you can install screen inside the container and multiplex a single
terminal. Note the screen shortcuts for screen will conflict if you use both on host and
container

* `docker commit <CONTAINER ID> <CUSTOM_IMG_NAME>` allows you to store the current state of a container. It generates a docker image that can be used for next executions with all customization you did.

* `docker save CUSTOM_IMG_NAME > customimgname.tar` This command exports the docker image named CUSTOM_IMG_NAME to a tar file, allowing that docker image to be transferred to another system, where it can be loaded using `docker load customimgname.tar`. 

### Container network connectivity

If you are using Yocto but would like to deploy a separate OS on top of Yocto, you can use a docker image of this OS. But remember you can also use Ubuntu* and install it manually on Intel Aero. Docker is an advanced and complex topic, use at your risk.

To allow docker to forward IPv4 and thus allow a container to communicate with the network, you can choose between:

1. **enable ip_forward by default** so all containers will have access to the network: `sysctl -w net.ipv4.ip_forward=1` in the root console

2. **Keep ip_forward disabled by default** and use the parameter `--net=host` in your `docker run` calls for containers that should have access to the network.

Also, you might want to allow other machines in the **local network to have access to a container**. With `docker run`, you can use either the `-p PORT` parameter, which allows you to specify a port/list of ports to be exposed, or `-P`  which exposes all ports. For more details about these parameters, check [Bind container ports to the host](https://docs.docker.com/engine/userguide/networking/default_network/binding/)


# [SysAdmin: Docker Network Server](#sysadmin-docker-network-server)

If you are using Yocto but would like to deploy a separate OS on top of Yocto, you can use a docker image of this OS. But remember you can also use Ubuntu* and install it manually on Intel Aero. Docker is an advanced and complex topic, use at your risk.

As seen in the previous example, you can instantiate a live image from a downloaded image and modify it on the fly. But you can also define one programmatically.
In this example, I'll build an image based on Ubuntu* 16.04, will add openssh-server and set a default password.
I will then instantiate this image with the right settings to listen to a network port on Intel Aero. Users will then be able to access the docker running on Intel Aero from the network with ssh. Check the course to understand the details.

First, create a file called `Dockerfile`:
```
FROM ubuntu:16.04
MAINTAINER Paul Guermonprez <paul.guermonprez@intel.com>
RUN apt-get update && apt-get install -y openssh-server iputils-ping net-tools
RUN mkdir /var/run/sshd
RUN echo 'root:password' | chpasswd
RUN sed -i 's/PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config
RUN sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd

ENV NOTVISIBLE "in users profile"
RUN echo "export VISIBLE=now" >> /etc/profile
CMD ["/usr/sbin/sshd", "-D"]
```

Then launch the creation process:
```
docker build -t "guermonprez/aero-demo" .
```
It will download the Ubuntu* docker, if needed. Then run `apt-get install` and all the commands.
Instantiate the container, listening on port 2222:
```
docker run -d --name aero-demo -h aero-demo -p 2222:22 guermonprez/aero-demo
```
Open a direct shell on the image:
```
docker exec -i -t aero-demo bash
```
Or a ssh connection from your station, first to the Yocto layer:
(change the IP to reflect the drone's IP on YOUR NETWORK)
`ssh root@192.168.0.13`
Then to the docker instance (here ubuntu*):
```
ssh root@192.168.0.13 -p 2222
```

To list container instances, and stop ours:
```
docker ps
docker stop aero-demo
```
List images, and delete our instance and image created:
```
docker images
docker rm aero-demo
docker rmi guermonprez/aero-demo
```

# [SysAdmin: Docker ROS](#sysadmin-docker-ros)

If you are using Yocto but would like to deploy a separate OS on top of Yocto, you can use a docker image of this OS. But remember you can also use Ubuntu* and install it manually on Intel Aero. Docker is an advanced and complex topic, use at your risk.

We'll run ROS from a reference docker image called `ros`. It is based on Ubuntu*.
```
docker run -it --name aero-ros -h aero-ros --privileged ros
```
Then install the necessary packages:
```
apt-get update
apt-get install ros-kinetic-mavros iproute2 ros-kinetic-image-view openssh-client
apt-get install ros-kinetic-librealsense ros-kinetic-realsense-camera
# it may take a while ...
```
Open a second terminal on Intel Aero Yocto layer. When the previous `apt-get install` command is finished, commit the changes to create a new personalized local image:
```
docker commit aero-ros guermonprez/aero-ros
docker images
```

When this image is saved, we can launch ROS.
In the first terminal (running ubuntu-ros), launch:
```
roscore
```

In the second terminal (running yocto), launch a shell to the `ubuntu-ros` layer:
```
docker exec -i -t aero-ros /ros_entrypoint.sh bash
```
And launch the `mavros` ROS module. The only specific argument is the URL to access the flight controller:
```
rosrun mavros mavros_node _fcu_url:=tcp://172.17.0.1:5760 _system_id:=2
```
You now have a working ROS installation, with modules:
* MAVROS for the flight controller access
* Intel RealSense for the 3D sensor access
* ROS modules like `image-view` to get images from cameras

To test it, let's look at the topics available.
Let's launch a third terminal on yocto, and a connection to the ubuntu-ros layer:
```
docker exec -i -t aero-ros /ros_entrypoint.sh bash
```

Then list the topics, and get the battery level:
```
rostopic list
rostopic echo /mavros/battery
```

Now let's get a RGB image from the Intel RealSense sensor:
```
rosrun image_view image_saver image:=/camera/color/image_raw
# close with ctrl-c after a second, as it's running for ever
```

Then copy the files to your computer for display:
(change the IP with your development station's IP on YOUR NETWORK)
```
scp left00* paulguermonprez@192.168.0.12:~/
```
And display the image in your home folder.

We now have a working ROS container, with Intel RealSense working.


