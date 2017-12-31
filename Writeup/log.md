## Recent Changes
 * Time 31.12.2017
   Issue: modified tl_detector.py so that it can set early warn distance value.

 * Time 23.12.2017
 
   Issue: [unity_simulator-3] process has died
   
   https://discussions.udacity.com/t/unity-simulator-3-process-has-died-solved/376182

 * Time 23.12.2017
 
   Issue: update tl_detector.py by set 'EARLY_WARNING_DISTANCE = 80 # > 11*11/2  2.8*2.8/2' and 'self.simulator = True'
   
 * Time 22.12.2017
 
   Issue: upload a new file waypoint_updater_12_22.py to waypoint_updater fold.
   
 * Upload a new file waypoint_updater_loop_corrected.py to waypoint_updater
 
   Time: 19.12.2017
   
   Issue: add loop feature so that the car can run around the track, depending on the global variable LOOP (False or True) of waypoint_updater_loop_corrected.py
   
   Note. To run in 'Test Lot' environment of simulater v1.3, see 1.


### 1. Settings for testing in 'Test Lot' environment in Simulator in v1.3

Since len(self.base_waypoints_list) equals 61, before testing in 'Test Lot' envirnoment we should adjust the global variable of waypoint_updater.py as follows.

* LOOKAHEAD_WPS_INIT = 20

&

Change the uploading route in waypoint_loader.py so that it uses corrent waypoints data.
```    
    source devel/setup.sh
    #cd launch && ls
    #cat styx.launch
    #cd ..
    cd src/waypoint_loader && ls
    cd launch && ls
    nano waypoint_loader.launch
    ##change 'wp_yaw_const.csv' to 'churchlot_with_cars.csv'
    ##change velocity value to 10 and save
    cat waypoint_loader.launch
    cd ../../..
    catkin_make
    source devel/setup.sh
```

### 2. clean logs
```
    rosclean check
    rosclean purge
```

### 3. General Parameters
Copied from Terminal 

    SUMMARY
    ========

    PARAMETERS
    * /dbw_node/accel_limit: 1.0
    * /dbw_node/brake_deadband: 0.2
    * /dbw_node/decel_limit: -5.0
    * /dbw_node/fuel_capacity: 0.0
    * /dbw_node/max_lat_accel: 3.0
    * /dbw_node/max_steer_angle: 8.0
    * /dbw_node/steer_ratio: 14.8
    * /dbw_node/vehicle_mass: 1080.0
    * /dbw_node/wheel_base: 3
    * /dbw_node/wheel_radius: 0.335
    * /pure_pursuit/linear_interpolate_mode: True
    * /rosdistro: kinetic
    * /rosversion: 1.12.7
    * /traffic_light_config: <...>
    * /waypoint_loader/path: /home/student/Des...
    * /waypoint_loader/velocity: 10

    NODES
    /
       dbw_node (twist_controller/dbw_node.py)
       pure_pursuit (waypoint_follower/pure_pursuit)
       styx_server (styx/server.py)
       tl_detector (tl_detector/tl_detector.py)
       unity_simulator (styx/unity_simulator_launcher.sh)
       waypoint_loader (waypoint_loader/waypoint_loader.py)
       waypoint_updater (waypoint_updater/waypoint_updater.py)

We can also use commands like `rosparam list`, `rosparam get`, `rosparam set`.

### 4. How to check the data type transfered between nodes
Check the operating ros topics
```
    rostopic list
    rostopic info
```
Check the msg info
```
    rosmsg info #msg type's name#
```
