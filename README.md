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
- In the src folder of your workspace, enter `https://github.com/kennyderek/smooth_trajectory.git` on terminal.
- `cd smooth_trajectory`
- `cmake .`
- `cd ..` and then one more time, to get back into the catkin_ws folder
- `catkin_make`.
- In catkin_ws, enter `source devel/setup.bash`.

#### How to run:

We will need 4 different terminal tabs to run the Runner/Observer nodes
- first terminal:
    --> `roscore`
- second terminal:
    --> `rosrun smooth_trajectory path_planner`
- third terminal:
    --> `rosrun smooth_trajectory path_smoother`
- final terminal:
    --> `rosrun rviz rviz`

Note: if you encounter the error `[rospack] Error: package 'smooth_trajectory' not found`, then you need to run `source devel/setup.bash` in each new terminal window from the catkin_ws root.

#### In the Rviz window:
You will need to add `my_frame` into the Fixed Frame box of the rviz window. Additionally, find the 'ADD' button in the bottom left of the window and select the 'Marker' option. Wait a few seconds for the first path to be displayed

#### Final results:
![Example of final Rviz image with 5% error and SMOOTHNESS (in path_smoother.cpp) set to 3. Notice the '/my_frame' in Fixed Frame and the ADD button in the bottom left.] (https://github.com/kennyderek/smooth_trajectory/blob/master/Screen%20Shot%202018-01-18%20at%203.33.02%20PM.png)
![Set to 20% error and SMOOTHNESS still at 3] (https://github.com/kennyderek/smooth_trajectory/blob/master/Screen%20Shot%202018-01-18%20at%203.32.01%20PM.png)


