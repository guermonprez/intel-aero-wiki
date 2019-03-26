# [Rebuild Yocto](#rebuild-yocto)

## [Why rebuild Yocto](#why-rebuild-yocto)

Intel Aero comes with a custom build of Linux Yocto flashed from the factory, and you should [upgrade it to the latest release](02-Initial-Setup).

You can use this preconfigured-precompiled linux system to perform task such as fly or stream video to QGroundControl on your laptop.

In the yocto project concept, you often choose to rebuild your full system from scratch. You compile your app with your OS to have full control over the total software stack. This page will show you how. Note: the process will take hours and tens of gigabits (really) of disk space.

But it is important to understand: **YOU DO NOT HAVE THE REBUILD THE IMAGE TO DEVELOP SOFTWARE ON INTEL AERO**:
* You can also access the system running on Intel Aero with ssh as root and start coding. Editors like vi, C/C++ compiler and scripting engines such as python are included. It's very quick and simple.
* You can edit your files on your development station with your favorite IDE and transfer your files to the system for execution of compilation.
* You could even edit and compile on your station and transfer the binary.
* You can use [Ubuntu on Intel Aero](90-(References)-OS-user-Installation).

## [One Time Pre-requisites](#one-time-pre-requisites)

### Linux Distribution

Yocto Project validates its tools against a list of distributions and versions, as [described here](http://www.yoctoproject.org/docs/2.3/ref-manual/ref-manual.html#detailed-supported-distros) and replicated as follows:
* Ubuntu 14.04 (LTS), 14.10, 15.04, 15.10, **16.04  (LTS)** - environment used by the development team CI system
* Fedora release 22, 23, 24
* CentOS release 7.x
* Debian GNU/Linux 8.x (Jessie)
* openSUSE 13.2, 42.1

### Dependencies

The Yocto documentation also provides [details and commands to install packages required in each supported distribution](http://www.yoctoproject.org/docs/2.3/ref-manual/ref-manual.html#required-packages-for-the-host-development-system). For Ubuntu 16.04:

```
apt-get update
apt-get install gawk wget git-core diffstat unzip texinfo gcc-multilib \
     build-essential chrpath socat cpio python python3 python3-pip python3-pexpect \
     xz-utils debianutils iputils-ping
```

### Download and add repo to PATH

```
mkdir ~/bin
curl http://commondatastorage.googleapis.com/git-repo-downloads/repo > ~/bin/repo
chmod a+x ~/bin/repo
export PATH=$HOME/bin:$PATH
```

Note that `~/bin` is just a suggestion. If you choose a different path, adjust instructions accordingly.

## [Build](#build)

### Get sources

```bash
mkdir ~/intel_aero && cd ~/intel_aero;
```
Note that `~/intel_aero` is just a suggestion. If you choose a different path, adjust instructions accordingly.

To fetch the source code matching a specific release, run:

```
repo init -u https://github.com/intel-aero/intel-aero-manifest.git -m v1.6.xml -b master
```

To fetch a the version currently under development, run:

```bash
repo init -u https://github.com/intel-aero/intel-aero-manifest.git -m default.xml -b master
```

With the repository initialized, run the command that actually fetches the layers and recipes:

```
repo sync -j4
```

### Initialize the environment

```
cd ~/intel_aero/poky
export TEMPLATECONF=../meta-intel-aero/conf/
source oe-init-build-env
```
`oe-init-build-env` configures environment variables, create config files based on templates and takes the shell to `~/intel_aero/poky/build` directory, where bitbake can be executed:

```
bitbake intel-aero-image
```

Upon build completion, the image will be available at `~/intel_aero/poky/build/tmp/deploy/images/intel-aero/`

Refer to the [setup page](02-Initial-Setup) for instructions on how to flash this image.
