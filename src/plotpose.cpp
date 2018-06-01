#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>


geometry_msgs::PoseStamped pose;
geometry_msgs::Vector3 output_pose_data;
geometry_msgs::Point optitrack, estimate;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
  pose = *msg;
}
void output_pose_cb(const geometry_msgs::Vector3::ConstPtr &msg){
  output_pose_data = *msg;
}
int main( int argc, char** argv )
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  ros::Publisher marker_pub2 = n.advertise<visualization_msgs::Marker>("visualization_marker2", 10);
  ros::Subscriber pose_sub = n.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/RigidBody1/pose", 10, pose_cb);
  ros::Subscriber output_sub = n.subscribe<geometry_msgs::Vector3>("/output_pose", 10, output_pose_cb);
  uint32_t shape = visualization_msgs::Marker::CUBE;
  ros::Rate r(30);

  float i = 0;
  while (ros::ok())
  {

    optitrack.x = pose.pose.position.x;
    optitrack.y = pose.pose.position.y;
    optitrack.z = pose.pose.position.z;

    estimate.x = output_pose_data.x;
    estimate.y = output_pose_data.y;
    estimate.z = output_pose_data.z;


    visualization_msgs::Marker points, line_strip, line_list, marker;
    visualization_msgs::Marker points2, line_strip2, line_list2;


    points.header.frame_id = points2.header.frame_id = line_strip.header.frame_id = line_strip2.header.frame_id = line_list.header.frame_id = line_list2.header.frame_id = marker.header.frame_id = "/my_frame";
    points.header.stamp = points2.header.stamp = line_strip.header.stamp = line_strip2.header.stamp = line_list.header.stamp = line_list2.header.stamp = marker.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "points_and_lines";
    points2.ns = line_strip2.ns = line_list2.ns = "points_and_lines2";
    points.action = line_strip.action = line_list.action = points2.action = line_strip2.action = line_list2.action =  visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = points2.pose.orientation.w = line_strip2.pose.orientation.w = line_list2.pose.orientation.w = 1.0;

    marker.ns = "point_and_lines";
    marker.id = 3;

    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 1;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
/*
    while (marker_pub.getNumSubscribers() < 1)
        {
          if (!ros::ok())
          {
            return 0;
          }
          ROS_WARN_ONCE("Please create a subscriber to the marker");
          sleep(1);
        }
        */
        marker_pub.publish(marker);





    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;
    points2.id = 4;
    line_strip2.id = 5;
    line_list2.id = 6;



    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    points2.type = visualization_msgs::Marker::POINTS;
    line_strip2.type = visualization_msgs::Marker::LINE_STRIP;
    line_list2.type = visualization_msgs::Marker::LINE_LIST;



    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.3;
    points.scale.y = 0.3;

    points2.scale.x = 0.3;
    points2.scale.y = 0.3;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;
    line_list.scale.x = 0.1;

    line_strip2.scale.x = 0.1;
    line_list2.scale.x = 0.1;



    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    points2.color.b = 1.0f;
    points2.color.a = 1.0;
    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    line_strip2.color.b = 1.0;
    line_strip2.color.a = 1.0;

    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;

    line_list2.color.r = 1.0;
    line_list2.color.a = 1.0;


    // Create the vertices for the points and lines

      points.points.push_back(optitrack);
      line_strip.points.push_back(optitrack);


      points2.points.push_back(estimate);
      line_strip2.points.push_back(estimate);
      //line_list.points.push_back(p);




    marker_pub.publish(points);
    marker_pub.publish(line_strip);

    marker_pub2.publish(points2);
    marker_pub2.publish(line_strip2);
    //marker_pub.publish(line_list);

    ros::spinOnce();
    r.sleep();


  }
  return 0;
}
