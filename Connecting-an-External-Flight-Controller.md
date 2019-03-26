The Aero Compute Board can connect to a variety of sensors and controllers through its I/O interfaces. A popular usage is to connect the Aero Compute Board to a Pixhawk flight controller. This can be done using the Aero Compute Board’s high-speed UART (HSUART). This article provides the necessary instructions for connecting the hardware and configuring the software.

Though these instructions are specifically for the Pixhawk flight controller, any flight controller with a UART interface can be attached.

<img src="https://github.comhttps://raw.githubusercontent.com/guermonprez/intel-aero-documents/blob/master/doc_photos/pixhawk_aero.png?raw=true"  />



### Software Configuration
1. Run the latest Aero Software

   Ensure the Aero Compute Board software is up to date by following these instructions:
   https://github.comhttps://raw.githubusercontent.com/guermonprez/meta-intel-aero/wiki/02-Initial-Setup#flashing

2. Configure the on-board Altera Max 10 FPGA

   The Aero Compute Board has an on-board Altera Max 10 FPGA which must be configured to disconnect from the HSUART. Specifically, FPGA pins B11 and B12 must be tri-stated.

   An FPGA configuration file (JAM) with the necessary changes is distributed as part of the software release and can be program via command: <br> `jam -aprogram /etc/fpga/aero-compute-board.jam`

3. Depending on your use case, you can either **disable the mavlink-router** daemon and consume HSUART (/dev/ttyS1) locally on the Aero Compute Board OR **configure mavlink-router** to route incoming messages from the flight controller to Aero’s access point, allowing you to communicate to the flight controller over WiFi. 

   **Disabling mavlink router**

    From Aero’s Linux shell, disable the mavlink-router daemon to free up the HSUART port. You can disable it with the following command. <br>`/etc/init.d/mavlink-routerd.sh stop`

   **Configuring mavlink router**

   By default, mavlink router is already setup to route packets between the flight controller and different IP endpoints. The only modification needed is to set the correct baud rate to be used on the HSUART. Normally, this is 57600 but can be different depending on how you configure Pixhawk.

   Edit the file `/etc/mavlink-router/main.conf` and update the following with the correct baud.

   _[UartEndpoint uart]_<br>
   _Device = /dev/ttyS1_<br>
   _Baud = 57600_

   Restart mavlink router

   `/etc/init.d/mavlink-routerd.sh stop`<br>
   `/etc/init.d/mavlink-routerd.sh start`

### Hardware Configuration
The Intel Aero Compute Board is shipped with a power cable that includes a separate pin connector as shown in Figure 1. This standard 0.1” 3-pin connector exposes the HSUART pins from the Aero Compute Board. 

<img src="https://github.comhttps://raw.githubusercontent.com/guermonprez/intel-aero-documents/blob/master/doc_photos/pixhawk_aero_power_cable.jpg?raw=true" width=70%/><br>
**Figure 1.** Power + HSUART pins

The Pixhawk will be connected to this 3-pin connector via its TELEM2 port, this port has a DF13 mating connector. The user is recommended to create a cable similar to the one shown below, with a 6-pin DF13 connector on one side and a 0.1" 3-pin female header on the other. 

<img src="https://github.comhttps://raw.githubusercontent.com/guermonprez/intel-aero-documents/blob/master/doc_photos/pixhawk_aero_uart_cable.png?raw=true" width=50%/><br>
**Figure 2.** 6-pin DF13 connector to 3-pin female header. Pinout of the HSUART
> The HSUART interface operates at 3.3V. Do not exceed 3.6V

When creating this cable, be mindful of the TX/RX connections. The TX of the Aero Compute Board is connected to the RX of the Pixhawk.  The RX of the Aero Compute Board connects to the TX of the Pixhawk.  As reference, Figure 4 shows the ports and pinout of the Pixhawk. 


<img src="https://github.comhttps://raw.githubusercontent.com/guermonprez/intel-aero-documents/blob/master/doc_photos/pixhawk_aero_ports.png?raw=true" width=60% />
<img src="https://github.comhttps://raw.githubusercontent.com/guermonprez/intel-aero-documents/blob/master/doc_photos/pixhawk_pinout.png?raw=true"/><br>
<strong>Figure 4.</strong> Pixhawk ports and TELEM pinout<br><br>

### Powering the Aero Compute Board and Pixhawk

The Aero Compute Board can be powered directly from the drone’s power distribution board. It requires an **input voltage of 5V** and be able to supply up to **15W**. Feed power through the power + UART connector in Figure 1.

In Figure 5, we used a XT60 power module (outputs 5.3V 3A max) to supply power to the Aero Compute Board. The power module might not be able to power both the compute board and Pixhawk, however, you can daisy chain two together and provide power separately. 

<img src="https://github.comhttps://raw.githubusercontent.com/guermonprez/intel-aero-documents/blob/master/doc_photos/pixhawk_aero_setup.jpg?raw=true"/><br>
**Figure 5.** Aero Compute Board being power by a XT60 power module and connected to Pixhawk's telemetry port.
