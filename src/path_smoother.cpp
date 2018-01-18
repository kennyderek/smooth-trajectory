#include "ros/ros.h"
#include "std_msgs/String.h"
#include "smoothing_path/Positions.h"
#include <visualization_msgs/Marker.h>
#include <vector>

// change the smoothness level to affect the amount the trajectory is smoothed
#define SMOOTHNESS 3

// declare global variables that chatterCallback and main can access
std::vector<float> x_arr;
std::vector<float> y_arr;
bool completed;

void chatterCallback(const smoothing_path::Positions::ConstPtr& msg)
{
  // add the coordinates to x_arr and y_arr
  float x_pos = msg -> x_pos;
  float y_pos = msg -> y_pos;
  completed = msg -> completed_path;
  x_arr.push_back(x_pos);
  y_arr.push_back(y_pos);

  // print out the coordinates that it is reading
  printf("x found at %f, y found at %f, completed:%d \n", x_pos, y_pos, completed);
  if (completed) {
    std::copy(x_arr.begin(), x_arr.end(), std::ostream_iterator<float>(std::cout, " "));
    std::copy(y_arr.begin(), y_arr.end(), std::ostream_iterator<float>(std::cout, " "));
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_smoother");

  ros::NodeHandle n;

  // set up a subscriber and publisher on the same node
  // subscriber will read from path_planner
  // publisher will send data to rviz
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Rate loop_rate(20);

  while (ros::ok()) {

    // if the path is completed (path_planner sets completed to true when it has finished looping),
    // then publish to rviz
    if (completed) {

      // declare two visualization markers
      visualization_msgs::Marker line_strip, smoothed_strip;

      // line_strip is the original unsmoothed trajectory (blue)
      // it is labeled as '/my_frame', which needs to be typed into the 'Fixed Frame' section of rviz
      line_strip.header.frame_id = "/my_frame";
      line_strip.id = 0;
      line_strip.type = visualization_msgs::Marker::LINE_STRIP;
      line_strip.scale.x = 0.1;
      line_strip.color.b = 1.0;
      line_strip.color.a = 1.0;

      // smoothed_strip will display the smoothed trajectory (green)
      smoothed_strip.header.frame_id = "/my_frame";
      smoothed_strip.id = 1;
      smoothed_strip.type = visualization_msgs::Marker::LINE_STRIP;
      smoothed_strip.scale.x = 0.1;
      smoothed_strip.color.g = 1.0f;
      smoothed_strip.color.a = 1.0;

      // iterate over the points in the trajectory
      for(std::vector<float>::size_type i = 0; i != x_arr.size(); i++) {
        // push line_strip points to rviz untouched
        geometry_msgs::Point p;
        p.x = x_arr[i];
        p.y = y_arr[i];
        p.z = 0;

        line_strip.points.push_back(p);

        // smooth the line by taking the x & y average of the neighboring points
        float average_x = 0.0;
        float average_y = 0.0;

        // get the neighboring values, how many is determined by SMOOTHNESS (defined at top)
        for(int j = -SMOOTHNESS; j < SMOOTHNESS+1; j++) {
          int test_index = i + j;
          test_index = test_index % x_arr.size();

          average_x += x_arr[test_index];
          average_y += y_arr[test_index];
        }

        average_x = average_x / (2.0 * SMOOTHNESS + 1);
        average_y = average_y / (2.0 * SMOOTHNESS + 1);

        // publish the smoothed point into smoothed_strip trajectory
        geometry_msgs::Point p_new;
        p_new.x = average_x;
        p_new.y = average_y;
        p_new.z = 0;
        smoothed_strip.points.push_back(p_new);

      }

      // reset
      x_arr.clear();
      y_arr.clear();

      // publish to rviz
      marker_pub.publish(smoothed_strip);
      marker_pub.publish(line_strip);

    }

    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}


