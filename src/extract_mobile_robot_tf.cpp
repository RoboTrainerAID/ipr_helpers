
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <ipr_helpers/Pose2DStamped.h>
#include <ipr_helpers/ArrayStamped.h>
#include <ipr_helpers/ExtractMobileRobotTFParameters.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "extract_mobile_robot_tf");

  ros::NodeHandle node;

  ros::Publisher pose_pub = node.advertise<ipr_helpers::Pose2DStamped>("mobile_robot_pose", 1);
  ros::Publisher array_pub = node.advertise<ipr_helpers::ArrayStamped>("mobile_robot_position", 1);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ipr_helpers::ExtractMobileRobotTFParameters params_{ros::NodeHandle("~")};
  params_.fromParamServer();

  ros::Rate rate(params_.rate);
  while (node.ok()){
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform(params_.root_frame, params_.child_frame,
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN_THROTTLE(5, "%s",ex.what());
      continue;
    }

    ipr_helpers::Pose2DStamped poseStamped;
    poseStamped.header = transformStamped.header;
    poseStamped.pose.x = transformStamped.transform.translation.x;
    poseStamped.pose.y = transformStamped.transform.translation.y;
    poseStamped.pose.theta = tf2::getYaw(transformStamped.transform.rotation);

    ipr_helpers::ArrayStamped arrayStamped;
    arrayStamped.header = transformStamped.header;
    arrayStamped.position.push_back(transformStamped.transform.translation.x);
    arrayStamped.position.push_back(transformStamped.transform.translation.y);
    arrayStamped.position.push_back(tf2::getYaw(transformStamped.transform.rotation));


    pose_pub.publish(poseStamped);
    array_pub.publish(arrayStamped);

    rate.sleep();
  }
  return 0;
};


