# [Drotek](#drotek)
Drotek is building a professional autonomous drones and 4x4 rovers based on Intel Aero to support their professional drone services strategy.

Drotek CEO Jerome Perin: *“With Intel Aero compute board and sensors, we’ve been able to move from idea to production quickly and spend more time focusing on our clients.”*

Website: [drotek.com](http://drotek.com/en)

# [Dronecode PX4](#px4)

The Intel Aero Ready to Fly Drone is shipped with Dronecode PX4 autopilot firmware.  Instructions for updating the firmware are documented [on this page.](https://dev.px4.io/en/flight_controller/intel_aero.html).

# [ArduPilot](#ardupilot)

The Intel Aero Ready to Fly Drone is officially supported by Ardupilot. Instructions for loading Arducopter firmware are documented [on this page.](http://ardupilot.org/copter/docs/common-intel-aero-rtf.html).

# [Skysense](#skysense)

Skysense is the leading drone charging solution for autonomous drone charging trusted by NASA and GoogleX. With up to 20Ah charging rate and its unique landing error-tolerant design, Skysense is the fastest, most easy to use and robust drone charging solution on the market and is compatible with Intel Aero.

Skysense was successfully integrated and tested with Intel Aero based drones.

Skysense Charging Pad specifications:
* two available sizes: 462x462mm, 924x924mm
* drone retrofit-kit weight 50-150g
* supply AC voltage 110-240V AC 50-60 Hz
* max. charging current 20Ah charging rate
* support for 2-6S LiPo batteries
* communication interface: RJ45/eth
* remote control: GUI, terminal interface, API
* available in the indoor (IP44) and outdoor (IP55) edition

![Skysense charging pad](/guermonprez/intel-aero-documents/raw/master/doc_photos/ecosystem-skysense.jpg)


You can order your Skysense development kit at [skysense.co](http://www.skysense.co/contact-us)


# [Airmap](#airmap)

From [Airmap.com](https://www.airmap.com/): *AirMap is the leading global provider of UTM technology for the drone ecosystem. Millions of drones and hundreds of partners rely on AirMap’s data and services for safe and efficient drone flight.*

[AirMap SDK](https://www.airmap.com/intel/) is available on Intel Aero Compute Board Yocto build. AirMap Services supported:
* Connect to extensive database of flight advisories
* Access drone flight restrictions from airports, national parks, power plants, schools, and many more
* Coverage of over 180 countries with local regulations
* Get up to date weather updates and no-fly zones
* Discover local flight notification requirements
* Create Flight : Send flight location and send notifications if needed
* Flight Status : Get up to date no fly zones and notification requirements
* Send automated flight operation notifications
* Share flight information
* Telemetry : Update dynamic drone position

Follow documentation at [https://www.airmap.com/intel-docs/](https://www.airmap.com/intel-docs/) to know how to use above services

Integrate with the AirMap Platform by signing up as a developer at [https://developers.airmap.com/](https://developers.airmap.com/) to create new applications and manage your AirMap credentials required to authenticate your requests

# [Dronesmith](#dronesmith)
From [Dronesmith.io](http://dronesmith.io/):

## For Enterprises
We provide fully architected custom solutions. With enhanced security and dedicated support, we offer the scalability and reliability that an enterprise demands.
Whether you are looking for an on-premise or in-cloud solution, our microservice based applications removes inconsistencies among platforms and languages.

## For Developers
Dronesmith Suite consists of a variety of drone app building tools, including flight control RESTful APIs, virtual simulator, analytics tools, and sensor APIs.
We streamline the complexity of hardware, so that you can focus on building great apps. Develop once and have it run on any MavLink-based drone.

## For Drone Makers
Connect your drone to the internet with Luci and get access to an ecosystem of drone apps built on Dronesmith tools.
Does your drone have Luci-like hardware architecture? Consult with our solutions engineer to evaluate your project’s compatibility with the rest of our Dronesmith tools.


# [FlytBase](#flytbase)
From [FlytBase](https://flytbase.com/): *FlytBase is the next generation platform for commercial drone applications. It is by design open for developers to build advanced drone applications using its open APIs. FlytBase offers FlytOS which is a software framework for drones along with FlytSIM, the simulation environment and FlytSDKs, the web and mobile app development kits.*

Instructions on how to install FlytBase are available [on this page](http://docs.flytbase.com/docs/FlytOS/GettingStarted/AeroGuide.html) but a new, simpler, docker based installation procedure is being validated.

# [OttoFly](#ottofly)

OttoFly CEO Hakki Refai: *“We partner our SDK/API package with the Intel Aero Compute board to accelerate drone innovation, integrate Intel RealSense and other sensors, and leverage the board’s highly adaptive features to inspire and drive next generation automated and autonomous drone applications in an intuitive, concept-to-product development environment.”*

OttoFly’s SDK/API incorporates a suite of features that allow anyone, from experienced drone application developers to students, startups, and other first-time developers, to build new drone applications quickly, effectively, and safely.  Our SDK/API natively supports C++ and encapsulates MavLink protocol communication with the drone within the SDK/API command functions. As a result, any developer familiar with C++ can intuitively work with OttoFly’s SDK/API, without the need to learn MavLink protocol or worry about common drone communication issues.  The easy-to-use command functions allow the developer to control common drone movements and actions with a minimal number of high level code lines instead of many low level lines of code and the SDK/API takes care of synchronization and timing issues as part of the command execution. Configuration modifications require only editing of a json-based configuration file. Check [OttoFly.com](http://ottofly.com/)

OttoFly’s SDK/API provides several other attractive features that differentiate it from many other options for developing drone applications. 
* OttoFly’s SDK/API provides seamless interaction with the Intel Aero Ready To Fly Drone.  The SDK/API interfaces directly and seamlessly with the Intel Aero Flight Controller, allowing the developer to access all of the features available within the Aero Drone Kit.
* The SDK/API supports both Autopiloting and Autonomous drone applications. For Autopiloting, the SDK/API provides clear an intuitive command and control set to accurately program the drone’s desired path. For Autonomous piloting, The SDK/API contains commands the developer can use for developing an autonomous application. OttoFly can also provide customized SDK/API solutions that implement sophisticated artificial intelligence algorithms in a carefully crafted command set, allowing the developer to implement sophisticated actions such as collision avoidance and vision analysis.
* OttoFly’s SDK/API provides platform independence.  The SDK/API is written in ISO C++ 14, eliminating the need for any middleware framework and eliminating delays in execution.  The SDK/API will operate on any platform employing ISO C++ 14 and a Boost Library. Wrappers for other languages such as Python, Java, Javascript and .NET are in development and will extend the range of platforms suitable for use with the SDK/API. Installation (Linking with the SDK) can be carried out via CMake or directly using the preferred IDE.
* The SDK/API provides extensive logging capabilities that the developer can configure to provide the level of logging needed to examine and debug the operation of the application. For example, every state sent by the Intel Aero can be collected and examined in the log files.
* OttoFly’s SDK/API contains built-in safety features that minimize risks in development and operation even for the beginning developer. Functions such as Takeoff and Arm are implemented as blocking, preventing the unintentional execution of dangerous commands or actions when executing these functions. Many common functions provide acknowledgements that let the developer determine whether or not the function executes correctly before issuing further instructions to the drone.
* OttoFly provides live support and custom configuration services to assist the developer in building the best applications to meet the specific needs of a project.
