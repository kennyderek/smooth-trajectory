#### How it works
Uses two nodes (a path publisher and an path smoother).
The publisher creates the desired trajectory, and passes it a topic with a certain amount of random error
(which can be set inside the path_publisher code)
The the path smoother reads and stores the information published to the topic, and after a full completion of the
trajectory, displays the original trajectory and a smoothed trajectory on rviz.


#### To build project
- The program was written in Python 2.7.12 (it is run on linux)
- Create a catkin_ws if one is not already created (I used ROS kinetic).
 (follow instructions here: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
- In the src folder of your workspace, enter `https://github.com/kennyderek/smooth-trajectory.git` on terminal.
- `cd smooth-trajectory`
- `cmake .`
- `rosmake`
- `cd ..` and then one more time, to get back into the catkin_ws folder
- `catkin_make`.
- In catkin_ws, enter `source devel/setup.bash`.

#### How to run:

We will need 4 different terminal tabs to run the Runner/Observer nodes
- first terminal:
    --> `roscore`
- second terminal:
    --> `rosrun ` (if you just wish to run one bot) OR
    --> `rosrun ROS_bots runner.py _numbots:=100` (creates 100 bots to keep track of. Can change number as desired)
- third terminal:
    --> `rosrun ROS_bots observer.py`
- final terminal:
    --> `rosrun rviz rviz`

Note: if you encounter the error `[rospack] Error: package 'ROS_bots' not found`, then you need to run `source devel/setup.bash` in each new terminal window from the catkin_ws root.

#### End results
In the runner.py terminal you will see:
```
creating 10 bots
Publishing locations
[INFO] [1514675657.532913]: time elapsed: 0.002120971
[INFO] [1514675658.033852]: time elapsed: 0.502940177
[INFO] [1514675658.533420]: time elapsed: 1.002565145
[INFO] [1514675659.033450]: time elapsed: 1.502618074
...
```

In the terminal in which the observer.py was run, you will see:
```
[INFO] [1514675662.035074]: Bot 1 at (x, y, z) coordinates: (-0.9781627058982849, 20.27720832824707, 20.27720832824707)
[INFO] [1514675662.035713]: Bot 2 at (x, y, z) coordinates: (20.27720832824707, -0.9781627058982849, -0.9781627058982849)
[INFO] [1514675662.036069]: Bot 3 at (x, y, z) coordinates: (-0.9781627058982849, 20.27720832824707, -0.20784056186676025)
[INFO] [1514675662.036355]: Bot 4 at (x, y, z) coordinates: (20.27720832824707, -0.9781627058982849, -0.9781627058982849)
[INFO] [1514675662.036628]: Bot 5 at (x, y, z) coordinates: (20.27720832824707, -0.9781627058982849, -0.9781627058982849)
[INFO] [1514675662.037012]: Bot 6 at (x, y, z) coordinates: (-0.9781627058982849, -0.20784056186676025, -0.20784056186676025)
[INFO] [1514675662.037214]: Bot 7 at (x, y, z) coordinates: (-0.9781627058982849, 20.27720832824707, -0.9781627058982849)
[INFO] [1514675662.037419]: Bot 8 at (x, y, z) coordinates: (-0.9781627058982849, -0.20784056186676025, -0.9781627058982849)
[INFO] [1514675662.037608]: Bot 9 at (x, y, z) coordinates: (-0.9781627058982849, -0.20784056186676025, 20.27720832824707)
[INFO] [1514675662.037803]: Bot 10 at (x, y, z) coordinates: (20.27720832824707, -0.9781627058982849, 20.27720832824707)
********
[INFO] [1514675662.534759]: Bot 1 at (x, y, z) coordinates: (-0.9581419825553894, 25.02745819091797, 25.02745819091797)
[INFO] [1514675662.535173]: Bot 2 at (x, y, z) coordinates: (25.02745819091797, -0.9581419825553894, -0.9581419825553894)
[INFO] [1514675662.535464]: Bot 3 at (x, y, z) coordinates: (-0.9581419825553894, 25.02745819091797, 0.28629350662231445)
[INFO] [1514675662.535739]: Bot 4 at (x, y, z) coordinates: (25.02745819091797, -0.9581419825553894, -0.9581419825553894)
[INFO] [1514675662.536008]: Bot 5 at (x, y, z) coordinates: (25.02745819091797, -0.9581419825553894, -0.9581419825553894)
[INFO] [1514675662.536275]: Bot 6 at (x, y, z) coordinates: (-0.9581419825553894, 0.28629350662231445, 0.28629350662231445)
[INFO] [1514675662.536544]: Bot 7 at (x, y, z) coordinates: (-0.9581419825553894, 25.02745819091797, -0.9581419825553894)
[INFO] [1514675662.537532]: Bot 8 at (x, y, z) coordinates: (-0.9581419825553894, 0.28629350662231445, -0.9581419825553894)
[INFO] [1514675662.537860]: Bot 9 at (x, y, z) coordinates: (-0.9581419825553894, 0.28629350662231445, 25.02745819091797)
[INFO] [1514675662.538592]: Bot 10 at (x, y, z) coordinates: (25.02745819091797, -0.9581419825553894, 25.02745819091797)
********
...
```


