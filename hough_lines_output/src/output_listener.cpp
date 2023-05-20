#include <ros/ros.h>
#include <opencv_apps/LineArrayStamped.h>
#include <geometry_msgs/Point.h>
#include <vector>

void calculateIntersection(const geometry_msgs::Point& pt1, const geometry_msgs::Point& pt2, const geometry_msgs::Point& pt3, const geometry_msgs::Point& pt4)
{
  // Check if the lines are vertical
  bool isVertical1 = (pt2.x - pt1.x) == 0;
  bool isVertical2 = (pt4.x - pt3.x) == 0;

  // Check if the lines are parallel
  bool isParallel = ((pt2.y - pt1.y) / (pt2.x - pt1.x)) == ((pt4.y - pt3.y) / (pt4.x - pt3.x));

  // Skip calculation if lines are vertical or parallel
  if (isVertical1 || isVertical2 || isParallel)
  {
    ROS_WARN("Lines are vertical or parallel. Skipping intersection calculation.");
    return;
  }

  // Calculate the slopes of the two lines
  double slope1 = (pt2.y - pt1.y) / (pt2.x - pt1.x);
  double slope2 = (pt4.y - pt3.y) / (pt4.x - pt3.x);

  // Define the threshold angle for excluding lines
  double angleThreshold = 30.0; // Set your desired angle threshold here

  // Calculate the angles of the two lines
  double angle1 = atan(slope1) * 180.0 / M_PI;
  double angle2 = atan(slope2) * 180.0 / M_PI;

  // Skip calculation if lines have angles within the threshold
  if (std::abs(angle1) < angleThreshold && std::abs(angle2) < angleThreshold)
  {
    ROS_WARN("Lines are within the angle threshold. Skipping intersection calculation.");
    return;
  }

  // Calculate the y-intercepts of the two lines
  double yIntercept1 = pt1.y - slope1 * pt1.x;
  double yIntercept2 = pt3.y - slope2 * pt3.x;

  // Calculate the x-coordinate of the intersection point
  double xIntersection = (yIntercept2 - yIntercept1) / (slope1 - slope2);

  // Calculate the y-coordinate of the intersection point
  double yIntersection = slope1 * xIntersection + yIntercept1;

  // Create a geometry_msgs::Point for the intersection point
  geometry_msgs::Point intersection;
  intersection.x = xIntersection;
  intersection.y = yIntersection;
  intersection.z = 0.0;

  // Print the intersection point
  ROS_INFO("Intersection: (%f, %f)", intersection.x, intersection.y);
}


void lineArrayCallback(const opencv_apps::LineArrayStamped::ConstPtr& msg)
{
  // Check if the lines array is not empty
  if (msg->lines.empty())
  {
    ROS_WARN("Empty lines array.");
    return;
  }

  // Initialize lists to store pt1 and pt2 coordinates of each line
  std::vector<geometry_msgs::Point> pt1_list;
  std::vector<geometry_msgs::Point> pt2_list;

  // Iterate over each line
  for (const auto& line : msg->lines)
  {
    // Convert pt1 from opencv_apps::Point2D to geometry_msgs::Point
    geometry_msgs::Point pt1;
    pt1.x = line.pt1.x;
    pt1.y = line.pt1.y;
    pt1.z = 0.0;  // Assuming z-coordinate is not provided in opencv_apps::Point2D

    // Convert pt2 from opencv_apps::Point2D to geometry_msgs::Point
    geometry_msgs::Point pt2;
    pt2.x = line.pt2.x;
    pt2.y = line.pt2.y;
    pt2.z = 0.0;  // Assuming z-coordinate is not provided in opencv_apps::Point2D

    // Calculate the slope of the line
    double slope = (pt2.y - pt1.y) / (pt2.x - pt1.x);

    // Convert the slope to degrees
    double slope_deg = atan(slope) * 180 / M_PI;

    // Check if the slope is within the desired range
    if (200 < slope_deg < 70 && 110 < slope_deg < 160)
    {
      // Store pt1 and pt2 coordinates in the lists
      pt1_list.push_back(pt1);
      pt2_list.push_back(pt2);
    }
  }

  // Call calculateIntersection for each pair of lines
  for (size_t i = 0; i < pt1_list.size(); ++i)
  {
    calculateIntersection(pt1_list[i], pt2_list[i], pt1_list[i + 1], pt2_list[i + 1]);
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "output_listener");
  ros::NodeHandle nh;

  // Subscribe to the LineArrayStamped topic
  ros::Subscriber sub = nh.subscribe("/hough_lines/lines", 1000, lineArrayCallback);

  // Set the desired loop rate (e.g., 10 Hz)
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce(); // Process callback functions

    // Your code here

    loop_rate.sleep(); // Sleep to achieve the desired loop rate
  }

  return 0;
}