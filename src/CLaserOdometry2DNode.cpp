/** ****************************************************************************************
*  This node presents a fast and precise method to estimate the planar motion of a lidar
*  from consecutive range scans. It is very useful for the estimation of the robot odometry from
*  2D laser range measurements.
*  This module is developed for mobile robots with innacurate or inexistent built-in odometry.
*  It allows the estimation of a precise odometry with low computational cost.
*  For more information, please refer to:
*
*  Planar Odometry from a Radial Laser Scanner. A Range Flow-based Approach. ICRA'16.
*  Available at: http://mapir.isa.uma.es/mapirwebsite/index.php/mapir-downloads/papers/217
*
* Maintainer: Javier G. Monroy
* MAPIR group: http://mapir.isa.uma.es/
*
* Modifications: Jeremie Deray
******************************************************************************************** */

#include "rf2o_laser_odometry/CLaserOdometry2D.h"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include <tf2/utils.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#ifndef D2R
	#define D2R 0.017453292519943
#endif


using namespace std::chrono_literals;

// namespace rf2o {

class CLaserOdometry2DNode : public rclcpp::Node
{
public:

  CLaserOdometry2DNode();
  ~CLaserOdometry2DNode() = default;

  void process();
  void publish();

  bool setLaserPoseFromTf();

public:
  using Scalar = float;

  using Pose2d = Eigen::Isometry2d;
  using Pose3d = Eigen::Isometry3d;
  using MatrixS31 = Eigen::Matrix<Scalar, 3, 1>;
  using IncrementCov = Eigen::Matrix<Scalar, 3, 3>;

  CLaserOdometry2D* rf2o_;

  bool publish_tf = true;
  bool new_scan_available;

  double freq;

  std::string         laser_scan_topic = "scan";
  std::string         odom_topic = "odom_pose";
  std::string         base_frame_id = "base_link";
  std::string         odom_frame_id = "odom";
  std::string         init_pose_from_topic = "odom_init_pose";
  std::string         scan_frame_id = "laser";

  bool verbose_param = true;
  
  sensor_msgs::msg::LaserScan      last_scan;
  bool                             GT_pose_initialized;

  // Transforms
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  
  std::shared_ptr<tf2_ros::TransformBroadcaster>    odom_broadcaster_;
  nav_msgs::msg::Odometry          initial_robot_pose;
  

  //Subscriptions & Publishers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::ConstSharedPtr laser_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr initPose_sub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

  rclcpp::TimerBase::SharedPtr process_timer_;

  bool scan_available();

  //CallBacks
  void LaserCallBack(const sensor_msgs::msg::LaserScan::SharedPtr new_scan);
  void initPoseCallBack(const nav_msgs::msg::Odometry::SharedPtr new_initPose);

  void DeclareParam();
  void GetParam();
};

void CLaserOdometry2DNode::DeclareParam(void)
{
    declare_parameter("laser_scan_topic", laser_scan_topic);
    declare_parameter("odom_topic", odom_topic);
    declare_parameter("base_frame_id", base_frame_id);
    declare_parameter("odom_frame_id", odom_frame_id);
    declare_parameter("init_pose_from_topic", init_pose_from_topic);
    declare_parameter("publish_tf", publish_tf);
    declare_parameter("verbose", verbose_param);
    declare_parameter("scan_frame_id", scan_frame_id);
}

void CLaserOdometry2DNode::GetParam(void)
{
    laser_scan_topic = get_parameter("laser_scan_topic").as_string();
    odom_topic = get_parameter("odom_topic").as_string();
    base_frame_id = get_parameter("base_frame_id").as_string();
    odom_frame_id = get_parameter("odom_frame_id").as_string();
    init_pose_from_topic = get_parameter("init_pose_from_topic").as_string();
    publish_tf = get_parameter("publish_tf").as_bool();
    verbose_param = get_parameter("verbose").as_bool();
    scan_frame_id = get_parameter("scan_frame_id").as_string();
}

CLaserOdometry2DNode::CLaserOdometry2DNode() : Node("rf2o_laser_odometry")
{
  RCLCPP_INFO(get_logger(), "Localization Configuring...");

  //Read Parameters
  //----------------
  DeclareParam();
  GetParam();

  //Publishers and Subscribers
  //--------------------------
  odom_pub  = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 5);

  laser_sub = create_subscription<sensor_msgs::msg::LaserScan>(
      laser_scan_topic, 1, std::bind(&CLaserOdometry2DNode::LaserCallBack, this, std::placeholders::_1));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  odom_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  if (init_pose_from_topic != "")
  {
    initPose_sub = create_subscription<nav_msgs::msg::Odometry>(
      init_pose_from_topic, 1, std::bind(&CLaserOdometry2DNode::initPoseCallBack, this, std::placeholders::_1));

    GT_pose_initialized  = false;
  }
  else
  {
    GT_pose_initialized = true;
    initial_robot_pose.pose.pose.position.x = 0;
    initial_robot_pose.pose.pose.position.y = 0;
    initial_robot_pose.pose.pose.position.z = 0;
    initial_robot_pose.pose.pose.orientation.w = 1;
    initial_robot_pose.pose.pose.orientation.x = 0;
    initial_robot_pose.pose.pose.orientation.y = 0;
    initial_robot_pose.pose.pose.orientation.z = 0;
  }

  rf2o_ = new CLaserOdometry2D();
  //Init variables
  rf2o_->last_increment_ = Pose3d::Identity();
  rf2o_->laser_pose_on_robot_ = Pose3d::Identity();
  rf2o_->laser_pose_on_robot_inv_ = Pose3d::Identity();
  rf2o_->laser_pose_ = Pose3d::Identity();
  rf2o_->laser_oldpose_ = Pose3d::Identity();
  rf2o_->robot_pose_ = Pose3d::Identity();
  rf2o_->robot_oldpose_ = Pose3d::Identity();
  rf2o_->last_odom_time = rclcpp::Node::now();
  
  setLaserPoseFromTf();
  
  rf2o_->verbose = verbose_param;
  rf2o_->module_initialized = false;
  rf2o_->first_laser_scan = true;
  
  process_timer_ = this->create_wall_timer(
    100ms, std::bind(&CLaserOdometry2DNode::process, this)); // 10Hz
}

bool CLaserOdometry2DNode::setLaserPoseFromTf()
{
  bool retrieved = false;
  
  // Set laser pose on the robot (through tF)
  // This allow estimation of the odometry with respect to the robot base reference system.
  
  rclcpp::Duration timeout{1, 0};
  geometry_msgs::msg::TransformStamped transformStamped;

  tf2::Quaternion laser_tf_q; 
  laser_tf_q.setRPY(0, 0, 0);
  // std::cout << "laser_att: " << laser_tf_q.w() << ", " << laser_tf_q.x() << ", " << laser_tf_q.y() << ", " << laser_tf_q.z() << std::endl;
  
  const tf2::Matrix3x3 basis(laser_tf_q);
  Eigen::Matrix3d R;

  for(int r = 0; r < 3; r++)
    for(int c = 0; c < 3; c++)
      R(r,c) = basis[r][c];

  Pose3d laser_tf(R);
  laser_tf.translation()(0) = 0.0;
  laser_tf.translation()(1) = 0.0;
  laser_tf.translation()(2) = 0.0;
  
  //TODO: tf based laser initialized

  // laser_tf.translation()(0) = transformStamped.transform.translation.x;
  // laser_tf.translation()(1) = transformStamped.transform.translation.y;
  // laser_tf.translation()(2) = transformStamped.transform.translation.z;

  rf2o_->setLaserPose(laser_tf);

  return retrieved;
}

bool CLaserOdometry2DNode::scan_available()
{
  return new_scan_available;
}

void CLaserOdometry2DNode::process()
{
  if( rf2o_->is_initialized() && scan_available() )
  {
    //Process odometry estimation
    rf2o_->odometryCalculation(last_scan);
    publish();
    new_scan_available = false; //avoids the possibility to run twice on the same laser scan
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Waiting for laser_scans....") ;
  }
}

//-----------------------------------------------------------------------------------
//                                   CALLBACKS
//-----------------------------------------------------------------------------------

void CLaserOdometry2DNode::LaserCallBack(const sensor_msgs::msg::LaserScan::SharedPtr new_scan)
{
  if (GT_pose_initialized)
  {
    //Keep in memory the last received laser_scan
    last_scan = *new_scan;
    rf2o_->current_scan_time = last_scan.header.stamp;

    //Initialize module on first scan
    if (!rf2o_->first_laser_scan)
    {
      //copy laser scan to internal variable
      for (unsigned int i = 0; i<rf2o_->width; i++)
        rf2o_->range_wf(i) = new_scan->ranges[i];
      new_scan_available = true;
    }
    else
    {
      rf2o_->last_odom_time = last_scan.header.stamp;
      rf2o_->init(last_scan, initial_robot_pose.pose.pose);
      rf2o_->first_laser_scan = false;
    }
  }
}

void CLaserOdometry2DNode::initPoseCallBack(const nav_msgs::msg::Odometry::SharedPtr new_initPose)
{
  //Initialize module on first GT pose. Else do Nothing!
  if (!GT_pose_initialized)
  {
    initial_robot_pose = *new_initPose;
    GT_pose_initialized = true;
  }
}

void CLaserOdometry2DNode::publish()
{
  //first, we'll publish the odometry over tf
  //---------------------------------------
  if (publish_tf)
  {
    //RCLCPP_INFO(get_logger(), "[rf2o] Publishing TF: [base_link] to [odom]");
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = rclcpp::Node::now();
    odom_trans.header.frame_id = odom_frame_id;
    odom_trans.child_frame_id = base_frame_id;
    odom_trans.transform.translation.x = rf2o_->robot_pose_.translation()(0);
    odom_trans.transform.translation.y = rf2o_->robot_pose_.translation()(1);
    odom_trans.transform.translation.z = 0.0;
    double odom_yaw = getYaw(rf2o_->robot_pose_.rotation());
    tf2::Quaternion odom_q;
    odom_q.setRPY(0, 0, odom_yaw);
    odom_trans.transform.rotation = tf2::toMsg(odom_q);

    //send the transform
    odom_broadcaster_->sendTransform(odom_trans);
  }

  //next, we'll publish the odometry message over ROS
  //-------------------------------------------------
  //RCLCPP_INFO(get_logger(), "[rf2o] Publishing Odom Topic");
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = rclcpp::Node::now();
  odom.header.frame_id = odom_frame_id;

  //set the position
  odom.pose.pose.position.x = rf2o_->robot_pose_.translation()(0);
  odom.pose.pose.position.y = rf2o_->robot_pose_.translation()(1);
  odom.pose.pose.position.z = 0.0;

  double odom_yaw_pub = getYaw(rf2o_->robot_pose_.rotation());
  tf2::Quaternion odom_q_pub;
  odom_q_pub.setRPY(0, 0, odom_yaw_pub);
  odom.pose.pose.orientation = tf2::toMsg(odom_q_pub);
  
  //set the velocity
  odom.child_frame_id = base_frame_id;
  odom.twist.twist.linear.x = rf2o_->lin_speed;    //linear speed
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = rf2o_->ang_speed;   //angular speed
  //publish the message
  odom_pub->publish(odom);
}

// } /* namespace rf2o */

//-----------------------------------------------------------------------------------
//                                   MAIN
//-----------------------------------------------------------------------------------
int main(int argc, char* argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CLaserOdometry2DNode>());
  rclcpp::shutdown();
   
  return 0;
}
