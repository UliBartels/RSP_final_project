# Table of Contents
1.  [Running the Simulation](#org2202a9d)
2.  [Running the Physical Hardware](#org2202b9d)
    1.  [Turtle Bringup](#org2202c9d)
    2.  [Waffle Bringup](#org2202d9d)
3.  [SLAM](#org643380c)
    1.  [Waffle bringup](#org56eff83)
    2.  [Run SLAM node](#orgbb19ce0)
    3.  [Save your map](#org2202e9d)

<a id="org2202a9d"></a>
#Running the Simulation
<a id="org2202a9d"></a>

<a id="org2202b9d"></a>
# Running the Physical Hardware

<a id="org2202c9d"></a>
## Turtle Bringup
<a id="org2202c9d"></a>

<a id="org2202d9d"></a>
## Waffle Bringup
<a id="org2202d9d"></a>

<a id="org2202b9d"></a>


<a id="org643380c"></a>

# SLAM

<a id="org56eff83"></a>

## Waffle bringup

1.  Connect your waffle from a remote PC
    
        ssh ubuntu@192.168.10.2
    
    The password is `turtlebot`.
2.  Launch the bringup file
    
        ros2 launch turtlebot3_bringup robot.launch.py
3.  On the remote PC, launch the SLAM node
    
        ros2 launch turtlebot3_cartographer cartographer.launch.py


<a id="orgbb19ce0"></a>

## Run SLAM node

On the remote PC, launch the SLAM node and the teleop node

    ros2 launch final_project turtlebot_draw_map.launch


<a id="org2202e9d"></a>

## Save your map

The following command will save your map under your home directory

    ros2 run nav2_map_server map_saver_cli -f ~/map

