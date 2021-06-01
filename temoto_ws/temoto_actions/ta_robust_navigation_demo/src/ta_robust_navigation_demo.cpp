
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 *
 *  The basis of this file has been automatically generated
 *  by the TeMoto action package generator. Modify this file
 *  as you wish but please note:
 *
 *    WE HIGHLIY RECOMMEND TO REFER TO THE TeMoto ACTION
 *    IMPLEMENTATION TUTORIAL IF YOU ARE UNFAMILIAR WITH
 *    THE PROCESS OF CREATING CUSTOM TeMoto ACTION PACKAGES
 *    
 *  because there are plenty of components that should not be
 *  modified or which do not make sence at the first glance.
 *
 *  See TeMoto documentation & tutorials at: 
 *    https://temoto-telerobotics.github.io
 *
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

/* REQUIRED BY TEMOTO */
#include <class_loader/class_loader.hpp>
#include "ta_robust_navigation_demo/temoto_action.h"
#include "temoto_component_manager/component_manager_interface.h"
#include "temoto_robot_manager/robot_manager_interface.h"
#include "tf/tf.h"

/* 
 * ACTION IMPLEMENTATION of TaRobustNavigationDemo 
 */
class TaRobustNavigationDemo : public TemotoAction
{
public:

// Constructor. REQUIRED BY TEMOTO
TaRobustNavigationDemo()
{
  std::cout << __func__ << " constructed\n";
}

// REQUIRED BY TEMOTO
void executeTemotoAction()
{
  rmi_.initialize(*this);
  cmi_.initialize(*this);
  // cmi_.registerComponentStatusCallback(std::bind(&TaRobustNavigationDemo::componentStatusCallback
  // , this
  // , std::placeholders::_1
  // , std::placeholders::_2));

  std::string robot_name = "jackal";

  TEMOTO_INFO_STREAM("trying to get config of '" << robot_name << "' ...");
  YAML::Node robot_config = rmi_.getRobotConfig(robot_name);
  TEMOTO_INFO_STREAM("Config of robot '" << robot_name << "': " << robot_config);

  TEMOTO_WARN_STREAM(robot_config["description"].as<std::string>());
  std::string robot_absolute_namespace = robot_config["robot_absolute_namespace"].as<std::string>();
  std::string lidar_scan_topic = robot_absolute_namespace + "/"
    + robot_config["navigation"]["controller"]["scan_topic"].as<std::string>();
  TEMOTO_INFO_STREAM("robot '" << robot_name << "' expects lidar scans at topic '" << lidar_scan_topic << "'");

  /*
   * Load a 2D lidar for teleoperation feedback
   */
  ComponentTopicsReq requested_topics;
  temoto_component_manager::LoadComponent load_component_query;
  std::string sensor_type = "2d_lidar";

  TEMOTO_INFO_STREAM("Starting the " << sensor_type << " component ...");
  requested_topics.addOutputTopic("lidar_data_2d", lidar_scan_topic);
  ComponentTopicsRes responded_topics = cmi_.startComponent(sensor_type, requested_topics);

  /*
   * Load the robot
   */
  TEMOTO_INFO_STREAM("loading " << robot_name);
  rmi_.loadRobot(robot_name);
  TEMOTO_INFO_STREAM(robot_name << " initialized");

  //ros::Duration(10).sleep();
  /*
   * Move the robot
   */
  geometry_msgs::PoseStamped target_pose;
  target_pose.pose.position.x = -9.5;
  target_pose.pose.position.y = -1;
  target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 2.4);

  bool goal_reached = false;
  while (!goal_reached)
  try
  {
    TEMOTO_INFO_STREAM("Sending a navigation goal to " << robot_name << " ...");
    if (rmi_.navigationGoal(robot_name, "map", target_pose))
    {
      TEMOTO_INFO_STREAM("Done navigating");
      goal_reached = true;
    }
    else
    {
      TEMOTO_INFO_STREAM("The goal was not reached, requesting the same navigation goal again ... ");
    }
  }
  catch(const resource_registrar::TemotoErrorStack &e)
  {
    TEMOTO_WARN_STREAM("Caught an error, requesting the same navigation goal again ... ");
  }
}

// Destructor
~TaRobustNavigationDemo()
{
  TEMOTO_INFO("Action instance destructed");
}

void componentStatusCallback(temoto_component_manager::LoadComponent query_msg
, temoto_resource_registrar::Status status_msg)
{
  // cmi_.stopComponent(load_component_query_);
  // ros::Duration(3).sleep();
  // cmi_.startComponent(load_component_query_);
}

private:

temoto_robot_manager::RobotManagerInterface rmi_;
temoto_component_manager::ComponentManagerInterface cmi_;
temoto_component_manager::LoadComponent load_component_query_;

}; // TaRobustNavigationDemo class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaRobustNavigationDemo, ActionBase);
