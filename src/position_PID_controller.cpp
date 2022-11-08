#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <geometry_msgs/Vector3.h>
#include <iostream>
#include<cmath>

#include <rotors_control/ControlConfig.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/server.h>

class PositionPID
{
public:

  double gazebo_pos_x, gazebo_pos_y, gazebo_pos_z, gazebo_yaw;
  double target_pos_x, target_pos_y, target_pos_z, target_yaw;

  double gain_P_pos_x, gain_P_pos_y, gain_P_pos_z, gain_P_yaw;

  std::vector<double> pos_x_error_, pos_y_error_, pos_z_error_, yaw_error_;

  //estos deben crearse igual para cada PID para que no se cambien de uno a otro
  ros::Time prev_time_pos_x, prev_time_pos_y, prev_time_pos_z, prev_time_yaw;
  ros::Duration delta_t_pos_x, delta_t_pos_y, delta_t_pos_z, delta_t_yaw;
  
  //inputs max
  double pos_x_upper_limit, pos_y_upper_limit, pos_z_upper_limit, yaw_upper_limit; 
  double pos_x_lower_limit, pos_y_lower_limit, pos_z_lower_limit, yaw_lower_limit; 

  //output max
  double vel_x_upper_limit, vel_y_upper_limit, vel_z_upper_limit, yaw_rate_upper_limit; 
  double vel_x_lower_limit, vel_y_lower_limit, vel_z_lower_limit, yaw_rate_lower_limit; 

  //Dynamic reconfigure
  std::unique_ptr<dynamic_reconfigure::Server<marsupial_optimizer::ControlConfig>> server;
  std::unique_ptr<dynamic_reconfigure::Server<marsupial_optimizer::ControlConfig>::CallbackType> f;


  double reference_vel_x, reference_vel_y, reference_vel_z, reference_yaw_rate;
  sensor_msgs::Joy pose_reference_msg;
  bool is_initialized;

  ros::Subscriber sub_gazebo_pos;
  ros::Subscriber sub_target_pos;
  ros::Publisher pub_actuations;

  PositionPID(ros::NodeHandle nh, ros::NodeHandle pnh);
  void gazebo_pos_callback(nav_msgs::Odometry _msg); 
  void target_pos_callback(geometry_msgs::PoseStamped _target_pos);
  void executePIDs_sumative();
  double PID(double _gain_P, double _target_value, double _current_value, std::vector<double> &error_, ros::Time &prev_time_, ros::Duration &delta_t_);
  void dynReconfCb(marsupial_optimizer::ControlConfig &config, uint32_t level);
};

PositionPID::PositionPID(ros::NodeHandle nh, ros::NodeHandle pnh)
//:filtered_error_(3,0)
{

  //Dynamic reconfigure stuff
  server.reset(new dynamic_reconfigure::Server<marsupial_optimizer::ControlConfig>);
  f.reset(new dynamic_reconfigure::Server<marsupial_optimizer::ControlConfig>::CallbackType);
  *f = boost::bind(&PositionPID::dynReconfCb, this, _1, _2);
  server->setCallback(*f);

  sub_gazebo_pos = nh.subscribe("ground_truth/odom", 1, &PositionPID::gazebo_pos_callback, this); //cambiar para que acepte distintos drones
  sub_target_pos = nh.subscribe("targetPose", 1, &PositionPID::target_pos_callback, this);

  pub_actuations = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_generic", 1); //cambiar para que acepte distintos drones

  pos_x_error_.resize(3);
  pos_y_error_.resize(3);
  pos_z_error_.resize(3);
  yaw_error_.resize(3);

  pose_reference_msg.axes.resize(4);
  pose_reference_msg.axes[0] = 0.0;
  pose_reference_msg.axes[1] = 0.0;
  pose_reference_msg.axes[2] = 0.0;
  pose_reference_msg.axes[3] = 0.0;

  reference_vel_x = 0.0;
  reference_vel_y = 0.0;
  reference_vel_z = 0.0;
  reference_yaw_rate = 0.0;

  target_pos_x = 0.0;
  target_pos_y = 0.0;
  target_pos_z = 0.0;
  target_yaw = 0.0;

  is_initialized =false;


  //maximos y mínimos de mis salidas
  pnh.param("vel_x_upper_limit", vel_x_upper_limit, 2.0);         // [m/s]
  pnh.param("vel_y_upper_limit", vel_y_upper_limit, 2.0);         // [m/s]
  pnh.param("vel_z_upper_limit", vel_z_upper_limit, 1.0);         // [m/s]
  pnh.param("yaw_rate_upper_limit", yaw_rate_upper_limit, 45.0 * M_PI / 180.0);         // [rad/s] 
  
  pnh.param("vel_x_lower_limit", vel_x_lower_limit, -vel_x_upper_limit);         // [m/s]
  pnh.param("vel_y_lower_limit", vel_y_lower_limit, -vel_y_upper_limit);         // [m/s]
  pnh.param("vel_z_lower_limit", vel_z_lower_limit, -vel_z_upper_limit);         // [m/s]  
  pnh.param("yaw_rate_lower_limit", yaw_rate_lower_limit, -yaw_rate_upper_limit);         // [rad/s]     


  //maximos y mínimos de mis entradas   
  pnh.param("pos_x_upper_limit", pos_x_upper_limit, 30.0);         // [m]  -> Check this
  pnh.param("pos_y_upper_limit", pos_y_upper_limit, 30.0);         // [m]  -> Check this
  pnh.param("pos_z_upper_limit", pos_z_upper_limit, 20.0);         // [m]  -> Check this
  pnh.param("yaw_upper_limit", yaw_upper_limit, 360.0 * M_PI / 180.0);         // [rad] -> Check this
  
  pnh.param("pos_x_lower_limit", pos_x_lower_limit, -pos_x_upper_limit);         // [m]  -> Check this
  pnh.param("pos_y_lower_limit", pos_y_lower_limit, -pos_y_upper_limit);         // [m]  -> Check this
  pnh.param("pos_z_lower_limit", pos_z_lower_limit, 0.0);         // [m]  -> Check this
  pnh.param("yaw_lower_limit", yaw_lower_limit, 0.0);         // [rad]  -> Check this




  //Proportional gains
  pnh.param("gain_P_pos_x", gain_P_pos_x, 0.0); //ajustar estos
  pnh.param("gain_P_pos_y", gain_P_pos_y, 0.0);
  pnh.param("gain_P_pos_z", gain_P_pos_z, 0.01);
  pnh.param("gain_P_yaw", gain_P_yaw, 0.0);

  
}

void PositionPID::gazebo_pos_callback(nav_msgs::Odometry _msg)
{

  gazebo_pos_x = _msg.pose.pose.position.x;
  gazebo_pos_y = _msg.pose.pose.position.y;
  gazebo_pos_z = _msg.pose.pose.position.z;
  
  tf::Quaternion q(
        _msg.pose.pose.orientation.x,
        _msg.pose.pose.orientation.y,
        _msg.pose.pose.orientation.z,
        _msg.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  gazebo_yaw = yaw;

}

void PositionPID::target_pos_callback(geometry_msgs::PoseStamped _target_pos)
{

  
  is_initialized =true;
  target_pos_x = _target_pos.pose.position.x;
  target_pos_y = _target_pos.pose.position.y;
  target_pos_z = _target_pos.pose.position.z;
  
  tf::Quaternion q(
        _target_pos.pose.orientation.x,
        _target_pos.pose.orientation.y,
        _target_pos.pose.orientation.z,
        _target_pos.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  target_yaw = yaw;
  
}

void PositionPID::executePIDs_sumative()
{

  double vel_x_increment, vel_y_increment, vel_z_increment, yaw_rate_increment; //hacemos control sumativo

  vel_x_increment = PID(gain_P_pos_x, target_pos_x, gazebo_pos_x, pos_x_error_, prev_time_pos_x, delta_t_pos_x);

  vel_y_increment = PID(gain_P_pos_y, target_pos_y, gazebo_pos_y, pos_y_error_, prev_time_pos_y, delta_t_pos_y);

  vel_z_increment = PID(gain_P_pos_z, target_pos_z, gazebo_pos_z, pos_z_error_, prev_time_pos_z, delta_t_pos_z);
  
  yaw_rate_increment = PID(gain_P_yaw, target_yaw, gazebo_yaw, yaw_error_, prev_time_yaw, delta_t_yaw);

  //Increment de reference
  reference_vel_x += vel_x_increment; //mirar estos signos
  reference_vel_y += vel_y_increment;
  reference_vel_z -= vel_z_increment;
  reference_yaw_rate += yaw_rate_increment;
  

  // //Apply saturation limits to roll
  // if (reference_roll > roll_upper_limit)
  //   reference_roll = roll_upper_limit;
  // else if (reference_roll < roll_lower_limit)
  //   reference_roll = roll_lower_limit;

  

  //Set topic output values
  pose_reference_msg.axes[0] = reference_vel_x;
  pose_reference_msg.axes[1] = reference_vel_y;
  pose_reference_msg.axes[2] = reference_vel_z;
  pose_reference_msg.axes[3] = reference_yaw_rate;
  std::cout << "gazebo pos z: " << gazebo_pos_z << std::endl;
  std::cout << "target pos z: " << target_pos_z<< std::endl;
  std::cout << "Publishing vel z: " << reference_vel_z<< std::endl;

  ros::Time update_time = ros::Time::now();
  pose_reference_msg.header.stamp = update_time;
  pose_reference_msg.header.frame_id = "joy_vel_frame";
  

}



double PositionPID::PID(double _gain_P, double _target_value, double _current_value, std::vector<double> &error_, ros::Time &prev_time_, ros::Duration &delta_t_)
{

  double control_effort_, proportional_; //estos no guardan memoria, por tanto puedo ponerlo aqui

  //solo hace falta acumular errores si se usa el derivativo
  // error_.at(1) = error_.at(0);
  error_.at(0) = _target_value - _current_value; // Current error goes to slot 0

  // calculate delta_t
  if (!prev_time_.isZero()) // Not first time through the program
  {
    delta_t_ = ros::Time::now() - prev_time_;
    prev_time_ = ros::Time::now();
    if (0 == delta_t_.toSec())
    {
      ROS_ERROR("delta_t is 0, skipping this loop. Possible overloaded cpu "
                "at time: %f",
                ros::Time::now().toSec());
      return control_effort_; 
    }
  }
  else
  {
    ROS_INFO("prev_time is 0, doing nothing");
    prev_time_ = ros::Time::now();
    return control_effort_; 
  }


  // calculate the control effort
  proportional_ = _gain_P * error_.at(0);
  control_effort_ = proportional_;


  return control_effort_;
}


void PositionPID::dynReconfCb(marsupial_optimizer::ControlConfig &config, uint32_t level)
{

  gain_P_pos_x = config.gain_P_pos_x;
  gain_P_pos_y = config.gain_P_pos_y;
  gain_P_pos_z = config.gain_P_pos_z;
  gain_P_yaw = config.gain_P_yaw;
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "position_PID_controller");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  PositionPID position_PID(nh, private_nh);

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    if (position_PID.is_initialized){
    position_PID.executePIDs_sumative();
    position_PID.pub_actuations.publish(position_PID.pose_reference_msg);
    // std::cout << "Publishing vel x: " << position_PID.pose_reference_msg.axes[0];
    // std::cout << "Publishing vel y: " << position_PID.pose_reference_msg.axes[1];
    std::cout << "Publishing vel z: " << position_PID.pose_reference_msg.axes[2];
    // std::cout << "Publishing yaw_rate: " << position_PID.pose_reference_msg.axes[3];
     }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}