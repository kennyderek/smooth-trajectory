#include "ros/ros.h"
#include "std_msgs/String.h"
#include "smoothing_path/Positions.h"

#include <sstream>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <stdlib.h>

// change radius of circle
#define RADIUS 4
#define PI 3.14159
// change the frequency of location sampling
#define INCREMENT .05 
// change the error in the system
#define ERROR 5

float xLocation(float time) {
  // parameterize the x location as a function of time
  return RADIUS * cos(time);
}

float yLocation(float time) {
  // parameterize the y location as a function of time
  return RADIUS * sin(time);
}

float randomError(float value) {
  // come up with an error perentage (between .95 and 1.05)
  float percentage =  (100.0 + (rand() % (ERROR * 2) - ERROR))/100.0;

  // returns the value within +/- 5 perent
  return value * percentage;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_planner");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<smoothing_path::Positions>("chatter", 1000);

  ros::Rate loop_rate(20);

  // a function of time tells us where the trajectory is at a given time
  // time will be incremented by the INCREMENT value (so time is not the real time)
  float time;
  time = 0;

  while (ros::ok()) {
    // use the smoothing_path::Positions message
    smoothing_path::Positions msg;

    // imrement the time, if the path is complete (after 2*PI), then set completed_path to True
    // so path_smoother can publish the path to rviz
    time+= INCREMENT;
    if (time > 2*PI + INCREMENT) {
      time = 0;
      msg.completed_path = true;
    } else {
      msg.completed_path = false;
    }

    msg.x_pos = randomError(xLocation(time));
    msg.y_pos = randomError(yLocation(time));

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}


