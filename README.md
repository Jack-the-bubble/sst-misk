# INITIAL STARTUP

## Build ros workspace

1. navigate to `ros_ws` directory (`cd ros_ws`)
2. initialize workspace (`catkin init`)
3. build packages (`catkin build`)
4. export built packages (`source devel/setup.bash`)

# Run simulation

1. make sure you have 4 terminals open 
   and all of them have previously built packages exported 
   (see point 4. in previous step).
2. In first terminal run `roscore`
3. In second terminal run simulation 
    with defined world and robot specified in *.world file
    (`rosrun stage_ros stageros /workspaces/sst-misk/ros_ws/src/roomba_stage/roomba_lse_arena.world`)
4. run navigation system in third terminal (`rosrun basic_controller controller`)
5. In fourth terminal call service to order robot to move to specified position on the map.
    (`rosservice call /go_to_pose "pose:
        position:
            x: 1.0
            y: 2.0
            z: 0.0
        orientation:
            x: 0.0
            y: 0.0
            z: 0.0
            w: 0.0"
    `)

# discover robots

There is one controller started for every robot.
In order for position command to reach the robot 
it is necessary to know the exact address to send the message to.

1. In fourth terminal discover available services (`rosservice list`)
2. addresses that end on `go_to_pose` phrase represent robots waiting for order to move,
replace fragment `/go_to_pose` with full path found in order to reach specific robot.
Right now only robot_0 awaits commands, but it will change in the future.
