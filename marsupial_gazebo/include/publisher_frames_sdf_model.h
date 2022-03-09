#ifndef PLUGIN_PUBLISH_TF_H
#define PLUGIN_PUBLISH_TF_H

#include <map>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo {

  class Joint;
  class Entity;

  class PublishTFModesSDF : public ModelPlugin {

    public:
        PublishTFModesSDF();
        ~PublishTFModesSDF();
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
        std::vector<std::string> link_names_;

    protected:
        virtual void UpdateChild();
        // virtual void FiniChild();

    private:
        void publishTF();
        void QueueThread();

        physics::WorldPtr world;
        physics::ModelPtr parent;
        event::ConnectionPtr update_connection_;
        std::vector<physics::LinkPtr> links_;

        // ROS STUFF
        ros::NodeHandle* rosnode_;

        boost::mutex lock;

        std::string robot_namespace_, base_frame_;

        // Custom Callback Queue
        ros::CallbackQueue queue_;
        boost::thread callback_queue_thread_;

        // DiffDrive stuff
        bool alive_;

        // Update Rate
        double update_rate_;
        double update_period_;
        common::Time last_update_time_;

  };

}

#endif