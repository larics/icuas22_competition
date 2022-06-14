#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>

enum class SafetyStates { IDLE, OVERRIDE };

class SafetyMachine
{
public:
  SafetyStates getState()
  {
    SafetyStates state;
    {
      std::lock_guard<std::mutex> lock(m_state_mutex);
      state = m_current_state;
    }
    return state;
  }

  void setState(SafetyStates newState)
  {
    std::lock_guard<std::mutex> lock(m_state_mutex);
    m_current_state = newState;
  }

  bool isIdle()
  {
    std::lock_guard<std::mutex> lock(m_state_mutex);
    return m_current_state == SafetyStates::IDLE;
  }

  bool isOverride()
  {
    std::lock_guard<std::mutex> lock(m_state_mutex);
    return m_current_state == SafetyStates::OVERRIDE;
  }

  std::string toString()
  {
    auto curr_state = getState();
    switch (curr_state) {
    case SafetyStates::IDLE:
      return "IDLE";

    case SafetyStates::OVERRIDE:
      return "OVERRIDE";

    default:
      return "NONE";
    }
  }

private:
  std::mutex   m_state_mutex;
  SafetyStates m_current_state = SafetyStates::IDLE;
};

class UavSafetyNode
{
public:
  UavSafetyNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

private:
  SafetyMachine m_safety_sm;

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

  ros::ServiceClient m_tracker_reset_client;
  ros::ServiceServer m_safety_override_srv;
  bool               safety_override_cb(std_srvs::SetBool::Request&  req,
                                        std_srvs::SetBool::Response& resp);
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

  m_safety_override_srv =
    nh.advertiseService("safety/override", &UavSafetyNode::safety_override_cb, this);
  m_tracker_reset_client = nh.serviceClient<std_srvs::Empty>("tracker/reset");
}

bool UavSafetyNode::safety_override_cb(std_srvs::SetBool::Request&  req,
                                       std_srvs::SetBool::Response& resp)
{
  m_safety_sm.setState(req.data ? SafetyStates::OVERRIDE : SafetyStates::IDLE);

  // If we are in override now, reset the trajectory
  if (m_safety_sm.isOverride()) {
    std_srvs::Empty request;
    auto            sucess = m_tracker_reset_client.call(request);
    if (!sucess) {
      resp.success = false;
      resp.message = "Current state is: " + m_safety_sm.toString()
                     + ", unable to stop running trajectory. Beware!";
      return true;
    }
  }

  resp.success = true;
  resp.message = "Current state is: " + m_safety_sm.toString();
  return true;
}

void UavSafetyNode::position_hold_cb(
  const trajectory_msgs::MultiDOFJointTrajectoryPoint& msg)
{
  if (m_safety_sm.isOverride()) {
    ROS_WARN_THROTTLE(
      2.0, "[UavSafetyNode] Override enabled. position_hold/trajectory rejected");
    return;
  }

  ROS_INFO_THROTTLE(3.0, "[UavSafetyNode] Forwarding position_hold/trajectory!");
  m_position_hold_final_pub.publish(msg);
}

void UavSafetyNode::carrot_pose_array_cb(const geometry_msgs::PoseArray& msg)
{
  ROS_INFO_THROTTLE(3.0, "[UavSafetyNode] Got remaining trajectory.");
}

void UavSafetyNode::tracker_pose_cb(const geometry_msgs::PoseStamped& msg)
{
  if (m_safety_sm.isOverride()) {
    ROS_WARN_THROTTLE(2.0,
                      "[UavSafetyNode] Override enabled. tracker/input_pose rejected");
    return;
  }

  ROS_INFO_THROTTLE(3.0, "[UavSafetyNode] Forwarding tracker/input_pose");
  m_tracker_pose_final_pub.publish(msg);
}

void UavSafetyNode::tracker_trajectory_cb(
  const trajectory_msgs::MultiDOFJointTrajectory& msg)
{
  if (m_safety_sm.isOverride()) {
    ROS_WARN_THROTTLE(
      2.0, "[UavSafetyNode] Override enabled. tracker/input_trajectory rejected");
    return;
  }

  ROS_INFO_THROTTLE(3.0, "[UavSafetyNode] Forwarding tracker/input_trajectroy");
  m_tracker_trajectory_final_pub.publish(msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "uav_safety_node");
  ros::NodeHandle           nh;
  ros::NodeHandle           nh_private("~");
  UavSafetyNode             safety(nh, nh_private);
  ros::MultiThreadedSpinner spinner;
  spinner.spin();
  return 0;
}