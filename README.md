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
# Running the Simulation

1. Download the package
2. Run these commands, in order:
```
cd src/
vcs import < src/RSP_final_project/final_project/RSP_final_project.repos
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build
```
3. To launch the simulation for the first time you will need to create a map. For this, follow the instructions in the SLAM chapter.
4. Once you have a map of the virtual environment you can launch the environment again, but this time set-up for the virtual task:
```
ros2 launch final_project simulation_multibot.launch
```
5. After you have launched the virtual environment you then need to run the following two commands in two separate terminals:
```
ros2 run turtlebot_actionlib server
ros2 run turtlebot_actionlib client
```

<a id="org2202a9d"></a>

<a id="org2202b9d"></a>
# Running the Physical Hardware
Running the demo first requires that you've set up a maze, like so:
![Image of the Turtlebot Teaming Maze setup](/docs/Maze_Setup.jpg)
You then need to create a Maze. For this, follow the instructions in the SLAM chapter

<a id="org2202c9d"></a>
## Burger Bringup
1. SSH into Burger. It's IP should be between 192.168.10.1 and 192.168.10.10.
2. Run
```
ros2 launch final_project burger_bringup.launch.py
```
<a id="org2202c9d"></a>

<a id="org2202d9d"></a>
## Waffle Bringup
1. SSH into Waffle. It's IP should be between 192.168.10.1 and 192.168.10.10.
2. Run
```
ros2 launch final_project waffle_bringup.launch.py
```
<a id="org2202d9d"></a>

## Run the application
Once Waffle and Burger are both up and running run `ros2 launch final_project run-application`

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

