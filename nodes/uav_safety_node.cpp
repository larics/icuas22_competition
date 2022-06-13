#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

class UavSafetyNode
{
public:
  UavSafetyNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

private:
  ros::Subscriber m_position_hold_sub;
  ros::Publisher  m_position_hold_final_pub;
  void position_hold_cb(const trajectory_msgs::MultiDOFJointTrajectoryPoint& msg);

  ros::Subscriber m_carrot_pose_array_sub;
  void            carrot_pose_array_cb(const geometry_msgs::PoseArray& msg);

  ros::Subscriber m_tracker_pose_sub;
  ros::Publisher  m_tracker_pose_final_pub;
  void            tracker_pose_cb(const geometry_msgs::PoseStamped& msg);

  ros::Subscriber m_tracker_trajectory_sub;
  ros::Publisher  m_tracker_trajectory_final_pub;
  void tracker_trajectory_cb(const trajectory_msgs::MultiDOFJointTrajectory& msg);
};

UavSafetyNode::UavSafetyNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
{
  m_carrot_pose_array_sub = nh.subscribe(
    "tracker/remaining_trajectory", 1, &UavSafetyNode::carrot_pose_array_cb, this);

  m_position_hold_final_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
    "position_hold/final_trajectory", 1);
  m_position_hold_sub =
    nh.subscribe("position_hold/trajectory", 1, &UavSafetyNode::position_hold_cb, this);

  m_tracker_pose_final_pub =
    nh.advertise<geometry_msgs::PoseStamped>("tracker/final_input_pose", 1);
  m_tracker_pose_sub =
    nh.subscribe("tracker/input_pose", 1, &UavSafetyNode::tracker_pose_cb, this);

  m_tracker_trajectory_final_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
    "tracker/final_input_trajectory", 1);
  m_tracker_trajectory_sub = nh.subscribe(
    "tracker/input_trajectory", 1, &UavSafetyNode::tracker_trajectory_cb, this);
}

void UavSafetyNode::position_hold_cb(
  const trajectory_msgs::MultiDOFJointTrajectoryPoint& msg)
{
  ROS_INFO_THROTTLE(3.0, "[UavSafetyNode] Forwarding position_hold/trajectory!");
  m_position_hold_final_pub.publish(msg);
}

void UavSafetyNode::carrot_pose_array_cb(const geometry_msgs::PoseArray& msg)
{
  ROS_INFO_THROTTLE(3.0, "[UavSafetyNode] Got remaining trajectory.");
}

void UavSafetyNode::tracker_pose_cb(const geometry_msgs::PoseStamped& msg)
{
  ROS_INFO_THROTTLE(3.0, "[UavSafetyNode] Forwarding tracker/input_pose");
  m_tracker_pose_final_pub.publish(msg);
}

void UavSafetyNode::tracker_trajectory_cb(
  const trajectory_msgs::MultiDOFJointTrajectory& msg)
{
  ROS_INFO_THROTTLE(3.0, "[UavSafetyNode] Forwarding tracker/input_trajectroy");
  m_tracker_trajectory_final_pub.publish(msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "uav_safety_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  UavSafetyNode safety(nh, nh_private);
  ros::MultiThreadedSpinner spinner;
  spinner.spin();
  return 0;
}