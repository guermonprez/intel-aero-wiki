# [Introduction](#introduction)

Intel Movidius recently released the Neural Compute Stick.
The Movidius Neural Compute Stick (NCS) is a tiny fanless deep learning device that you can use to learn AI programming at the edge. NCS is powered by the same low power high performance Movidius Vision Processing Unit (VPU) that can be found in millions of smart security cameras, gesture controlled drones, industrial machine vision equipment, and more. A VPU is not a GPU, it is highly optimized for AI tasks.

In the drone world, Movidius is used to provide AI capabilities to the [DJI Phantom 4](https://www.youtube.com/watch?v=hX0UELNRR1I) and [DJI Spark](https://newsroom.intel.com/news/intel-movidius-myriad-2-vpu-enables-advanced-computer-vision-deep-learning-features-ultra-compact-dji-spark-drone/).

From a developer point of vue, the NCS allow you to run networks built with Berkeley Caffe and Tensorflow, two major open sources references in the field.
* You would typically start by reusing an existing network and tuning it to your needs. If it's not enough you can build your own.
* When you are happy with your network, deploy it on Intel Aero with Intel Movidius NCS connected to the USB port.
* At runtime, a simple C/C++ or Python code will send the network and the data (a webcam frame grab perhaps?) to the stick for analysis and get the result.

When you install the Intel Movidius NCS SDK on Intel Aero, you have the full stack to create and execute networks. But the creation of complex real life networks typically requires server farms or at least a workstation. That's why Intel Aero is typically used to execute the network and do AI at the edge.

You can [buy it online today](https://developer.movidius.com/buy).

The main developer page for the NCS : [developer.movidius.com](https://developer.movidius.com/)

## [Installation](#installation)

Plug the NCS to Intel Aero with a USB2-OTG cable (NOT a USB3-OTG).
The USB2-OTG cables included in the Intel Aero Compute Board box works.

```
sudo apt install git
mkdir -p ~/workspace
cd ~/workspace
git clone https://github.com/movidius/ncsdk.git
cd ~/workspace/ncsdk
make install
make examples
```

You should see an output like:
```
...
USB: Transferring Data...
Time to Execute :  123.84  ms
USB: Myriad Execution Finished
Time to Execute :  103.19  ms
USB: Myriad Execution Finished
USB: Myriad Connection Closing.
USB: Myriad Connection Closed.
...
```

If you use the wrong cable, you may see errors such as:
```
USB: Transferring Data...
Traceback (most recent call last):
  File "/usr/local/bin/mvNCProfile", line 121, in <module>
    profile_net(args.network, args.inputnode, args.outputnode, args.nshaves, args.inputsize, args.weights, args.device_no)
  File "/usr/local/bin/mvNCProfile", line 111, in profile_net
    timings, myriad_output = run_myriad(graph_file, args, file_gen=False)
  File "/usr/local/bin/ncsdk/Controllers/MiscIO.py", line 136, in run_myriad
    graph = device.AllocateGraph(blob_file)
  File "/usr/local/lib/python3.5/dist-packages/mvnc/mvncapi.py", line 201, in AllocateGraph
    raise Exception(Status(status))
Exception: mvncStatus.ERROR
```
or
```
[Error 7] Toolkit Error: USB Failure. Code: Error opening device
```
You can browse code samples from `~/workspace/ncsdk/examples/apps`

## [UI example](#ui-example)

To get started, let's plug a UVC compatible webcam (if you have the Intel Aero RTF Drone, the included R200 camera is perfect).
Then download the zoo (a repository of existing networks).

```
cd
git clone https://github.com/movidius/ncappzoo
cd ncappzoo/apps/stream_infer
make
python3 stream_infer.py
```

While connected on the hdmi graphical output of the board/drone, you should see a window analyzing images from the wbecam in real time on the NCS.
The network used is very powerful. In this example, the network running on the NCS was able to pick my dog and even give a pretty good estimate of the dog's race (it's a papillon, not a pekinese).

![](https://raw.githubusercontent.com/guermonprez/intel-aero-documents/raw/master/doc_photos/movidius_pekinese.png)


## [Going further](#going-further)

For python samples, see the [python page](04-Autonomous-drone-programming-in-Python#deep-learning-with-intel-movidius-ncs)

For more information, check the main developer page for the NCS : [developer.movidius.com](https://developer.movidius.com/)

