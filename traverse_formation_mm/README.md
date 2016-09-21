## TeleKyb formation demo adapted for a multimaster setup for the TRaVERSE project
### traverse_formation_mm
hack to get the std demo formation on a multimaster platform

this package is based on ```telekyb_exp/tk_standard_demo/``` 

it is however enhanced to accomodate the desired mutlimaster setup

## Dependencies 
1. [ros mutlimaster](http://wiki.ros.org/multimaster_fkie)
2. [telekyb core]()
3. [telekyb swarm]()
4. [mk interface]()

## Startup Procedure

1. Start a roscore and the multimaster setup 
  On each component involved you would need to start the multimaster_fkie discoverer and sync node.
  To simplify this procedure you'll find a launchfile [mm_setup.launch](https://github.molgen.mpg.de/MPI-KYB-RoboGroup/tk_formation_mm/blob/master/launch/mm_setup.launch) and even a [quickstart_mm.sh](https://github.molgen.mpg.de/MPI-KYB-RoboGroup/tk_formation_mm/blob/master/launch/quickstart_mm.sh). In both files you can specify the port on which the core should run (default 11311). 

  ```bash
  roscd tk_formation_mm/launch
  ./quickstart_mm.sh
  ```
  Afterwards check if all masters are discovered. On one pc run 
  ```bash
  rosservice call /multimaster_disco_11311/list_masters
  ```
  Note. Change the port in the service name to your requirements.
  The output should look something like this:
  ```bash
  masters: 
  - 
    name: 192.168.2.2
    uri: http://192.168.2.2:11311/
    timestamp: 1455187665.71
    timestamp_local: 1455186269.42
    online: True
    discoverer_name: /multimaster_disco_11311
    monitoruri: http://192.168.2.2:11611
  - 
    name: odroid-trav-43
    uri: http://odroid-TRaV-43:11311/
    timestamp: 1455187672.12
    timestamp_local: 1455187662.51
    online: True
    discoverer_name: /multimaster_disco_11311
    monitoruri: http://192.168.2.43:11611
  - 
    name: odroid-trav-44
    uri: http://odroid-TRaV-44:11311/
    timestamp: 1455187672.38
    timestamp_local: 1455187662.65
    online: True
    discoverer_name: /multimaster_disco_11311
    monitoruri: http://192.168.2.44:11611

  ```
2. Start the vicon node
  On the desktop PC start the vicon node 
  ```bash
  roslaunch tk_formation_mm vicon_TH.launch
  ```
  This starts the vicon node connected to the vicon PC in the tracking hall. You can easily change the IP address   within the launch file to your needs. 
  __Be sure that the vicon topic is recieved on all UAVs.__
  
3. Start the formations slaves
  On the UAVs start the __mk_interface__ the __tk_core__ and the __formation_slave__. 
  You can use the launch file [start_formation_slave_with_qcID.launch](https://github.molgen.mpg.de/MPI-KYB-RoboGroup/tk_formation_mm/blob/master/launch/start_formation_slave_with_qcID.launch) 
  ```bash
  roslaunch tk_formation_mm start_formation_slave_with_qcID.launch qcID:=<somenumber>
  ```
  or the [quickstart_slave.sh](https://github.molgen.mpg.de/MPI-KYB-RoboGroup/tk_formation_mm/blob/master/launch/quickstart_slave.sh) script
  ```bash
  roscd tk_formation_mm/launch
  ./quickstart_slave <somenumber>
  ```
4. Start the formation master
  Now you can start the formation master on the desktop or other controlling PC. With the corresponding launchfile you can state the list of robots within the formation and the ones which can be influenced with he joystick.
  For example:

  ```bash 
  roslaunch formation_master.launch robotIDs:=[<idofuav1>,<idofuav2>] humanInput:=[1,0] --screen
  ```
5. Enjoy
  Now you should be able to control all the uavs in formation from the master PC. As if you would be connected directly.
