#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include <larics_motion_planning/CheckStateValidity.h>
#include <uav_ros_lib/ros_convert.hpp>
#include <uav_ros_lib/param_util.hpp>

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
      return "ERROR";
    }
  }

private:
  std::mutex   m_state_mutex;
  SafetyStates m_current_state = SafetyStates::IDLE;
};

struct SafetyNodeParams
{
  bool check_position_hold        = true;
  bool check_tracker_pose         = true;
  bool check_tracker_trajectory   = true;
  bool check_remaining_trajectory = true;
};

class UavSafetyNode
{
public:
  UavSafetyNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

private:
  SafetyMachine    m_safety_sm;
  SafetyNodeParams m_safety_params;

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

  static constexpr auto WATCHDOG_RATE = 50.0;
  ros::Timer            m_traj_watchdog_timer;
  void                  trajectory_watchdog_cb(const ros::TimerEvent& /*unused*/);

  std::mutex         m_trajectory_checker_mutex;
  ros::ServiceClient m_trajectory_checker_client;
  void               check_trajectory(const trajectory_msgs::JointTrajectory& traj);
};

UavSafetyNode::UavSafetyNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
{
  param_util::getParamOrThrow(
    nh_private, "safety/check_position_hold", m_safety_params.check_position_hold);
  param_util::getParamOrThrow(
    nh_private, "safety/check_tracker_pose", m_safety_params.check_tracker_pose);
  param_util::getParamOrThrow(nh_private,
                              "safety/check_tracker_trajectory",
                              m_safety_params.check_tracker_trajectory);
  param_util::getParamOrThrow(nh_private,
                              "safety/check_remaining_trajectory",
                              m_safety_params.check_remaining_trajectory);

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

  m_traj_watchdog_timer = nh.createTimer(
    ros::Duration(1 / WATCHDOG_RATE), &UavSafetyNode::trajectory_watchdog_cb, this);

  m_trajectory_checker_client =
    nh.serviceClient<larics_motion_planning::CheckStateValidity>("validity_checker");
}

void UavSafetyNode::check_trajectory(const trajectory_msgs::JointTrajectory& traj)
{
  larics_motion_planning::CheckStateValidity state_validity;

  {
    std::lock_guard<std::mutex> lock(m_trajectory_checker_mutex);
    state_validity.request.points = traj;
    auto success                  = m_trajectory_checker_client.call(state_validity);
    if (!success) {
      ROS_FATAL(
        "[UavSafetyNode::check_trajectory] - validity_checker server unavailable.");
      return;
    }
  }

  if (!state_validity.response.valid) {
    ROS_WARN("COLLISION DETECTED, AUTOMATIC OVERRIDE!");
    std_srvs::SetBool::Request      req;
    std_srvs::SetBool::ResponseType resp;
    req.data     = true;
    auto success = safety_override_cb(req, resp);
    if (!success || !resp.success) {
      ROS_ERROR(
        "[UavSafetyNode::check_trajectory] - unable to trigger override, this shouldn't "
        "happen.");
      return;
    }
  }
}

void UavSafetyNode::trajectory_watchdog_cb(const ros::TimerEvent& /*unused*/)
{
  if (m_safety_sm.isOverride()) {
    ROS_INFO_THROTTLE(
      2.0, "[UavSafetyNode::trajectory_watchdog] Current state is OVERRIDE, no danger.");
    return;
  }

  if (!m_trajectory_checker_client.exists()) {
    ROS_WARN_THROTTLE(2.0,
                      "[UavSafetyNode::trajectory_watchdog] Checker service is missing!");
    return;
  }
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
  if (m_safety_params.check_position_hold) {
    // First check the trajectory
    trajectory_msgs::JointTrajectory joint_trajectory;
    joint_trajectory.points.emplace_back(ros_convert::pose_to_joint_trajectory_point(
      ros_convert::trajectory_point_to_pose(msg)));
    check_trajectory(joint_trajectory);
  }

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
  if (!m_safety_params.check_remaining_trajectory) { return; }
}

void UavSafetyNode::tracker_pose_cb(const geometry_msgs::PoseStamped& msg)
{
  if (m_safety_params.check_tracker_pose) {
    // Check the tracker pose
    trajectory_msgs::JointTrajectory joint_trajectory;
    joint_trajectory.points.push_back(
      ros_convert::pose_to_joint_trajectory_point(msg.pose));
    check_trajectory(joint_trajectory);
  }

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
  if (m_safety_params.check_tracker_trajectory) {
    // Check tracker trajectory
    trajectory_msgs::JointTrajectory joint_trajectory;
    std::for_each(msg.points.begin(),
                  msg.points.end(),
                  [&](const trajectory_msgs::MultiDOFJointTrajectoryPoint& point) {
                    joint_trajectory.points.emplace_back(
                      ros_convert::trajectory_point_to_joint_trajectory_point(point));
                  });
    check_trajectory(joint_trajectory);
  }

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