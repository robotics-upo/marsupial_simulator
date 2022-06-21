#ifndef MODELATE_TETHER_H
#define MODELATE_TETHER_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

// ROS
#include <ros/ros.h>
#include <std_msgs/Float32.h>

// Custom Callback Queue
#include <ros/callback_queue.h>

namespace gazebo
{
    class Boxscale : public ModelPlugin
    {
        public: 
            void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);
            void OnUpdate();
            void updatelength (const std_msgs::Float32::ConstPtr& length_tether_msg);
            // gazebo::msgs::Visual scaleLink(ignition::math::Vector3d scale_, int n_links);
            void scaleLink(ignition::math::Vector3d scale_, int n_links);
            void moveLinks(double move_link_, int n_links);
            void moveJoints(double move_joint_, int n_links);

            boost::shared_ptr<ros::NodeHandle> rosnode_;

            ros::Subscriber lenght_tether_subscriber_;
            boost::mutex lock;

        private:
            gazebo::rendering::ScenePtr scene_ptr ;
            gazebo::rendering::VisualPtr box_ptr ;
            gazebo::physics::ModelPtr model_;
            gazebo::physics::WorldPtr world_;
            gazebo::event::ConnectionPtr updateConnection;
            gazebo::transport::NodePtr node_;
            gazebo::transport::PublisherPtr pub_visual_;
            std::string visual_name_;
            double joint_step = 0.0001; // Must be fixed 
            double visual_step; // It is not fixed, depend on the iterations values and scale
            double length_tether, length_link, new_length_link, initial_length_link; //initial values for tether length and link length
            double length_gap ; // Fixed Gap length between links
            double max_length, min_length;
            double joint_offset; //Offset of joint when increase o reduce catenary
            int num_links;
            double scale = 1; // save value for initial and previous Visual Scale in Link
            double new_scale;  // save value for initial Visual Scale in Link
            int iterations; // calculate number of iteration to increase o reduce Visual Link and Joint Position
            double initial_joint_pos , joint_pos ;
            std::string name_link_tether, name_joint_tether;
            std::string robot_namespace_;

            // Custom Callback Queue
            ros::CallbackQueue queue_;
            boost::thread callback_queue_thread_;
            void QueueThread();
            bool alive_;

            int count, count_link;
    };
}

#endif // MODELATE_TETHER_H