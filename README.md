# Path_Tracking_With_Obstacle_Avoiding_Robot_In_Coppeliasim-Vrep

This is a simple project in which the Pioneer robot will track the laid path and avoid the obstacle in the track and also we can switch the mode to tele-operation with the help of python GUI.The whole simulation tool places in vrep.

## Coppeliasim

The robot simulator CoppeliaSim, with integrated development environment, is based on a distributed control architecture: each object/model can be individually controlled via an embedded script, a plugin, a ROS or BlueZero node, a remote API client, or a custom solution. This makes CoppeliaSim very versatile and ideal for multi-robot applications. Controllers can be written in C/C++, Python, Java, Lua, Matlab or Octave.

CoppeliaSim is used for fast algorithm development, factory automation simulations, fast prototyping and verification, robotics related education, remote monitoring, safety double-checking, as digital twin, and much more.

## Pioneer 3dx

Pioneer 3-DX is a small lightweight two-wheel two-motor differential drive robot ideal for indoor laboratory or classroom use. The robot comes complete with front SONAR, one battery, wheel encoders, a microcontroller with ARCOS firmware, and the Pioneer SDK advanced mobile robotics software development package.Pioneer research robots are the world’s most popular intelligent mobile robots for education and research. Their versatility, reliability and durability have made them the preferred platform for advanced intelligent
robotics. Pioneers are pre-assembled, customizable, upgradeable, and rugged enough to last through years of laboratory and classroom use.

## Path Tracking/Line Following And Obstacle Avoiding Robot

The Line Following Robot is an autonomous robot that detects a path and according to the path drawn, it follows the path with the help of an IR sensor attached to the robot. The path can be either a  Blackline drawn over a white  surface or  a white  line drawn over a black surface thus avoiding any  detection error. Line  follower  robot  also consists  of  an  obstacle sensor  that detects  any obstacle  in front  of the  Robot thus  avoiding any unnecessary  accidents.  Line  follower  robot  is  designed  and programmed in such a way that it does its job perfectly without any error and detects it’s given path.

### Algorithm

```
(i)    Get the Values of Three Vision Sensors v[1],v[2],[3] (v[1] --> Left Vision Sensor, v[2] --> Middle Vision Sensor, v[3] --> Right Vision Sensor)
(ii)   Get the Values of Proximity Sensord p[1],p[2] (p[1]--> Front Proximity Sensor, p[2]--> Left Proximity Sensor)
(iii)  Assign maximum velocity of the robot as 'speed'
(iv)   if p[1] == False then 
        (i)     if v[1]<0.3 then 
                   (i)  set left_motor_speed = 0.3*speed
                   (ii) set right_motor_speed = speed
                else if v[3]<0.3 then 
                    (i)  set right_motor-speed = 0.3*speed
                    (ii) set left_motor_speed = speed
                else
                    (i)  set left_motor_speed = speed
                    (ii) set right_motor_speed = speed
                end
(v)    else if p[1] == True then
            (i)   Rotate robot 90 degree right
            (ii)  Follow the obstacle until (v[1]>0.3 and v[2]>0.3 and v[3]>0.3)
            (iii) Rotate robot 90 degree right
```

## Execution

```
(i)   Download or Clone the Repository
(ii)  Open .ttt file
(iii) Press the play button
(iv)  Run main.py 
(v)   With the help of GUI Start autonomous navigation and there is an option to switch to tele-operation mode
```

Demo Link : https://youtu.be/0KgrIhGcq90
