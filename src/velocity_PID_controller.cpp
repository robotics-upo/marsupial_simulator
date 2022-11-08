#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <geometry_msgs/Vector3.h>
#include <iostream>
#include<cmath>

#include <rotors_control/ControlConfig.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/server.h>

class VelocityPID
{
public:

  double gazebo_vel_x, gazebo_vel_y, gazebo_vel_z;
  double target_vel_x, target_vel_y, target_vel_z, target_yaw_rate;

  double gain_P_vel_x, gain_P_vel_y, gain_P_vel_z;
  double gain_I_vel_x, gain_I_vel_y, gain_I_vel_z;
  double gain_D_vel_x, gain_D_vel_y, gain_D_vel_z;

  double c_thrust_increment;

  std::vector<double> vel_x_error_, vel_y_error_, vel_z_error_;

  double error_integral_vel_x, error_integral_vel_y, error_integral_vel_z;

  //estos deben crearse igual para cada PID para que no se cambien de uno a otro
  ros::Time prev_time_vel_x, prev_time_vel_y, prev_time_vel_z;
  ros::Duration delta_t_vel_x, delta_t_vel_y, delta_t_vel_z;
  // ros::Time last_setpoint_msg_time_;

  double roll_upper_limit, pitch_upper_limit; //thrust_upper_limit, yaw_rate_upper_limit;
  double roll_lower_limit, pitch_lower_limit; //thrust_lower_limit, yaw_rate_lower_limit;

  //Dynamic reconfigure
  std::unique_ptr<dynamic_reconfigure::Server<marsupial_optimizer::ControlConfig>> server;
  std::unique_ptr<dynamic_reconfigure::Server<marsupial_optimizer::ControlConfig>::CallbackType> f;



  //
  //Values for PID extras TODOs
  // Used in filter calculations. Default 1.0 corresponds to a cutoff frequency
  // at
  // 1/4 of the sample rate.
  // double c_;
  // Used to check for tan(0)==>NaN in the filter calculation
  // double tan_filt_;
  // Parameters for error calc. with disconinuous input
  // bool angle_error_;
  // double angle_wrap_;
  // Cutoff frequency for the derivative calculation in Hz.
  // Negative -> Has not been set by the user yet, so use a default.
  // double cutoff_frequency_;
  // Anti-windup term. Limits the absolute value of the integral term.
  // double windup_limit_;
  // Upper and lower saturation limits
  // double upper_limit_, lower_limit_;

  double reference_roll, reference_pitch, reference_thrust_z;
  geometry_msgs::Vector3 reference_thrust;
  mav_msgs::RollPitchYawrateThrust roll_pitch_yawrate_thrust_reference_msg;
  bool is_initialized;

  ros::Subscriber sub_gazebo_vel;
  ros::Subscriber sub_target_vel;
  ros::Publisher pub_actuations;

  VelocityPID(ros::NodeHandle nh, ros::NodeHandle pnh);
  void gazebo_vel_callback(nav_msgs::Odometry _msg);
  void target_vel_callback(sensor_msgs::Joy _current_joy);
  void executePIDs_sumative();
  double PID(double _gain_P, double _gain_I, double _gain_D, double _target_value, double _current_value, std::vector<double> &error_,
             double &error_integral_, ros::Time &prev_time_, ros::Duration &delta_t_);
  void dynReconfCb(marsupial_optimizer::ControlConfig &config, uint32_t level);
};

VelocityPID::VelocityPID(ros::NodeHandle nh, ros::NodeHandle pnh)
//:filtered_error_(3,0)
{
    //maximos del joy k habia -> de aqui pongo maximos de mis salidas
  pnh.param("roll_upper_limit", roll_upper_limit, 30.0 * M_PI / 180.0);         // pasamos grados a [rad]
  pnh.param("pitch_upper_limit", pitch_upper_limit, 30.0 * M_PI / 180.0);       // pasamos grados a [rad]
  // pnh.param("yaw_rate_upper_limit", yaw_rate_upper_limit, 45.0 * M_PI / 180.0); // pasamos grados a [rad]
  // pnh.param("thrust_upper_limit", thrust_upper_limit, 30.0);                    // [N]

  pnh.param("roll_lower_limit", roll_lower_limit, -roll_upper_limit);           
  pnh.param("pitch_lower_limit", pitch_lower_limit, -pitch_upper_limit);         
  // pnh.param("yaw_rate_lower_limit", yaw_rate_lower_limit, -yaw_rate_upper_limit); // pasamos grados a [rad]
  // pnh.param("thrust_lower_limit", thrust_lower_limit, 0.0);                       // [N]

  //para sacar Kp, pruebo a dar en simulacion un roll, pitch o thrust y veo la vel resultante, y de ahi saco por su definicion el Kp
  pnh.param("gain_P_vel_x", gain_P_vel_x, 0.002);
  pnh.param("gain_P_vel_y", gain_P_vel_y, 0.002);
  pnh.param("gain_P_vel_z", gain_P_vel_z, 0.1);

  //estos se inician a cero y luego se van subiendo según lo que se aprecie
  pnh.param("gain_I_vel_x", gain_I_vel_x, 0.0);
  pnh.param("gain_I_vel_y", gain_I_vel_y, 0.0);
  pnh.param("gain_I_vel_z", gain_I_vel_z, 0.0);

  //estos se inician a cero y luego se van subiendo según lo que se aprecie
  pnh.param("gain_D_vel_x", gain_D_vel_x, 0.003);
  pnh.param("gain_D_vel_y", gain_D_vel_y, 0.003);
  pnh.param("gain_D_vel_z", gain_D_vel_z, 0.1);

  pnh.param("c_thrust_increment", c_thrust_increment, 9.0);

  //Dynamic reconfigure stuff
  server.reset(new dynamic_reconfigure::Server<marsupial_optimizer::ControlConfig>);
  f.reset(new dynamic_reconfigure::Server<marsupial_optimizer::ControlConfig>::CallbackType);

  *f = boost::bind(&VelocityPID::dynReconfCb, this, _1, _2);
  server->setCallback(*f);

  sub_gazebo_vel = nh.subscribe("odom", 1, &VelocityPID::gazebo_vel_callback, this); //cambiar para que acepte distintos drones
  sub_target_vel = nh.subscribe("/dji_sdk/flight_control_setpoint_generic", 1, &VelocityPID::target_vel_callback, this);

  pub_actuations = nh.advertise<mav_msgs::RollPitchYawrateThrust>("command/roll_pitch_yawrate_thrust", 1); //cambiar para que acepte distintos drones

  reference_thrust.x = 0.0;
  reference_thrust.y = 0.0;
  reference_thrust.z = 0.0;

  error_integral_vel_x = 0;
  error_integral_vel_y = 0;
  error_integral_vel_z = 0;

  vel_x_error_.resize(3);
  vel_y_error_.resize(3);
  vel_z_error_.resize(3);

  reference_roll = 0.0;
  reference_pitch = 0.0;
  reference_thrust_z = 0.0;

  roll_pitch_yawrate_thrust_reference_msg.roll = 0.0; 
  roll_pitch_yawrate_thrust_reference_msg.pitch = 0.0;
  roll_pitch_yawrate_thrust_reference_msg.yaw_rate = 0.0;
  roll_pitch_yawrate_thrust_reference_msg.thrust = reference_thrust;


  target_vel_x = 0.0;
  target_vel_y = 0.0;
  target_vel_z = 0.0;
  target_yaw_rate = 0.0;

  gazebo_vel_x = 0.0;
  gazebo_vel_y = 0.0; 
  gazebo_vel_z = 0.0;

  is_initialized = false;

  delta_t_vel_x = ros::Duration(0.0);
  delta_t_vel_y = ros::Duration(0.0);
  delta_t_vel_z = ros::Duration(0.0);
  // prev_time_vel_x = ros::Time::Now()

  //Varaibles for PID extras TODOs
  // c_ = 1.;
  // angle_error_ = false;
  // tan_filt_ = 1.;
  // angle_wrap_ = 2.0 * 3.14159;
  // cutoff_frequency_ = -1;
  // windup_limit_ = 1000;
  // upper_limit_ = 1000;
  // lower_limit_ = -1000;
}

void VelocityPID::gazebo_vel_callback(nav_msgs::Odometry _msg)
{

  gazebo_vel_x = _msg.twist.twist.linear.x;
  gazebo_vel_y = _msg.twist.twist.linear.y;
  gazebo_vel_z = _msg.twist.twist.linear.z;
}

void VelocityPID::target_vel_callback(sensor_msgs::Joy _current_joy)
{
  is_initialized =true;
  target_vel_x = _current_joy.axes[0];
  target_vel_y = _current_joy.axes[1];
  target_vel_z = _current_joy.axes[2];
  target_yaw_rate = _current_joy.axes[3];
}

void VelocityPID::executePIDs_sumative()
{

  double pitch_increment = 0.0, roll_increment = 0.0, thrust_z_increment = 0.0; //hacemos control sumativo

  pitch_increment = PID(gain_P_vel_x, gain_I_vel_x, gain_D_vel_x, target_vel_x, gazebo_vel_x, vel_x_error_,
                        error_integral_vel_x, prev_time_vel_x, delta_t_vel_x);

  roll_increment = PID(gain_P_vel_y, gain_I_vel_y, gain_D_vel_y, target_vel_y, gazebo_vel_y, vel_y_error_,
                       error_integral_vel_y, prev_time_vel_y, delta_t_vel_y);

  thrust_z_increment = PID(gain_P_vel_z, gain_I_vel_z, gain_D_vel_z, target_vel_z, gazebo_vel_z, vel_z_error_,
                           error_integral_vel_z, prev_time_vel_z, delta_t_vel_z);

  //Increment de reference
  reference_roll -= roll_increment; //este se resta
  reference_pitch += pitch_increment;
  reference_thrust_z += thrust_z_increment;
  

  //Apply saturation limits to roll
  if (reference_roll > roll_upper_limit)
    reference_roll = roll_upper_limit;
  else if (reference_roll < roll_lower_limit)
    reference_roll = roll_lower_limit;

  //Apply saturation limits to pitch
  if (reference_pitch > pitch_upper_limit)
    reference_pitch = pitch_upper_limit;
  else if (reference_pitch < pitch_lower_limit)
    reference_pitch = pitch_lower_limit;

  // //Apply saturation limits to thrust z component
  // if (reference_thrust_z > thrust_upper_limit)
  //   reference_thrust_z = thrust_upper_limit;
  // else if (reference_thrust_z < thrust_lower_limit)
  //   reference_thrust_z = thrust_lower_limit;

  //Set topic output values
  roll_pitch_yawrate_thrust_reference_msg.roll = reference_roll;
  roll_pitch_yawrate_thrust_reference_msg.pitch = reference_pitch;

  reference_thrust.z = reference_thrust_z + (sqrt(pow(reference_roll,2) + pow(reference_pitch,2)) * c_thrust_increment); 
  roll_pitch_yawrate_thrust_reference_msg.thrust = reference_thrust;

  // //yaw_rate no pasa por PID, pero comprobamos limites
  // if (target_yaw_rate > yaw_rate_upper_limit)
  //   target_yaw_rate = yaw_rate_upper_limit;
  // else if (target_yaw_rate < yaw_rate_lower_limit)
  //   target_yaw_rate = yaw_rate_lower_limit;
  roll_pitch_yawrate_thrust_reference_msg.yaw_rate = target_yaw_rate;
}



double VelocityPID::PID(double _gain_P, double _gain_I, double _gain_D, double _target_value, double _current_value, std::vector<double> &error_,
                        double &error_integral_, ros::Time &prev_time_, ros::Duration &delta_t_)
{
  //based on doCalcs() from https://bitbucket.org/AndyZe/pid/src/master/src/pid.cpp

  double control_effort_ = 0.0, proportional_ = 0.0, integral_, derivative_ = 0.0, error_derivative_ = 0.0; //estos no guardan memoria, por tanto puedo ponerlo aqui

  // if (!((_gain_P <= 0. && _gain_I <= 0.) ||
  //         (_gain_P >= 0. && _gain_I >= 0.)))  // All 2 gains should have the same sign
  //     ROS_WARN("All gains (Kp, Ki) should have the same sign for "
  //              "stability.");

  // error_.at(2) = error_.at(1); //solo hace falta acumular errores si se usa el derivativo
  error_.at(1) = error_.at(0);
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

  //integrate the error
  error_integral_ += error_.at(0) * delta_t_.toSec();

  error_derivative_ = (error_.at(0) - error_.at(1)) / delta_t_.toSec();

  // Apply windup limit to limit the size of the integral term - TODO

  //Cutoff frequency - TODO

  //Calculate filtered error - TODO

  // calculate the control effort
  proportional_ = _gain_P * error_.at(0);
  integral_ = _gain_I * error_integral_;
  derivative_ = _gain_D * error_derivative_;
  control_effort_ = proportional_ + integral_ + derivative_;

  return control_effort_;
}


void VelocityPID::dynReconfCb(marsupial_optimizer::ControlConfig &config, uint32_t level)
{
  gain_P_vel_x = config.gain_P_vel_x;
  gain_P_vel_y = config.gain_P_vel_y;
  gain_P_vel_z = config.gain_P_vel_z;
  
  gain_I_vel_x = config.gain_I_vel_x;
  gain_I_vel_y = config.gain_I_vel_y;
  gain_I_vel_z = config.gain_I_vel_z;

  gain_D_vel_x = config.gain_D_vel_x;
  gain_D_vel_y = config.gain_D_vel_y;
  gain_D_vel_z = config.gain_D_vel_z;

  c_thrust_increment = config.c_thrust_increment;

}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "velocity_PID_controller");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  VelocityPID velocity_PID(nh, private_nh);

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    if (velocity_PID.is_initialized){
    velocity_PID.executePIDs_sumative();
    velocity_PID.roll_pitch_yawrate_thrust_reference_msg.header.stamp = ros::Time::now();
    velocity_PID.pub_actuations.publish(velocity_PID.roll_pitch_yawrate_thrust_reference_msg);
     }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}