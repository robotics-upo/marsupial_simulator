#include <algorithm>
#include <assert.h>

#include <publisher_frames_sdf_model.h>

#if (GAZEBO_MAJOR_VERSION < 8)
#include <gazebo/math/gzmath.hh>
#else
#include <ignition/math.hh>
#endif
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include <gazebo/gazebo_config.h>

namespace gazebo {

    PublishTFModesSDF::PublishTFModesSDF() {}

    // Destructor
    PublishTFModesSDF::~PublishTFModesSDF() 
    {
        delete rosnode_;
        // delete transform_broadcaster_;
    }

    // Load the controller
    void PublishTFModesSDF::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) 
    {
        this->parent = _parent;
        this->world = _parent->GetWorld();

        //Take the namespace
        this->robot_namespace_ = "";
        if (!_sdf->HasElement("robotNamespace")){ 
            ROS_INFO("PublishTFModesSDF Plugin missing <robotNamespace>, defaults to \"%s\"", this->robot_namespace_.c_str());
        }else{ 
            this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
            ROS_INFO_STREAM("PublishTFModesSDF <robotNamespace> set to: "<<robot_namespace_);
        }

        //Take name base_frame
        this->base_frame_ = "base_link";
        if (!_sdf->HasElement("robotBaseFrame")){ 
            ROS_INFO("PublishTFModesSDF Plugin missing <robotBaseFrame>, defaults to \"%s\"", this->base_frame_.c_str());
        }else{ 
            this->base_frame_ = _sdf->GetElement("robotBaseFrame")->Get<std::string>();
            std::vector<std::string> v_base_frame_;
            boost::split( v_base_frame_, base_frame_, boost::is_any_of("::") );
            int s_ = v_base_frame_.size();
            base_frame_ = v_base_frame_[s_ -1];
            ROS_INFO_STREAM("PublishTFModesSDF <robotBaseFrame> set to: "<<base_frame_);
        }

        // this->left_joint_names_ = "left_joint";
        if(!_sdf->HasElement("linkNames")){ 
            gzthrow("Have to specify space-separated link names via <linkNames> tag!");
        }else{
            std::string link_names_string = _sdf->GetElement("linkNames")->Get<std::string>();
            boost::split( link_names_, link_names_string, boost::is_any_of(" ") );
            ROS_INFO_STREAM("PublishTFModesSDF <linkNames> set to: "<<link_names_string);
        }
        
        //Take the rate
        this->update_rate_ = 100.0;
        if (!_sdf->HasElement("updateRate")){ 
            ROS_WARN("PublishTFModesSDF Plugin (ns = %s) missing <updateRate>, defaults to %f", this->robot_namespace_.c_str(), this->update_rate_);
        }else {
            this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();
            ROS_INFO_STREAM("PublishTFModesSDF <updateRate> set to: "<<update_rate_);
        }

        // Initialize update rate stuff
        if (this->update_rate_ > 0.0) 
        this->update_period_ = 1.0 / this->update_rate_;
        else 
        this->update_period_ = 0.0;

        #if (GAZEBO_MAJOR_VERSION >= 8)
            last_update_time_ = this->world->SimTime();
        #else
            last_update_time_ = this->world->GetSimTime();
        #endif

        for (size_t i = 0; i < link_names_.size(); ++i){
            links_.push_back(this->parent->GetLink(robot_namespace_+"::"+link_names_[i]));
            if (!links_[i]){
                char error[200];
                snprintf(error, 200, "PublishTFModesSDF Plugin (ns = %s) couldn't get hinge link named \"%s\"", this->robot_namespace_.c_str(), link_names_[i].c_str());
                gzthrow(error);
            }
        }

        alive_ = true;

        // Make sure the ROS node for Gazebo has already been initialized
        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM("A ROS node for Gazebo(PublishTFModesSDF) has not been initialized, unable to load plugin. "
                << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }

        rosnode_ = new ros::NodeHandle(this->robot_namespace_);

        ROS_INFO("Starting PublishTFModesSDF Plugin (ns = %s)!", this->robot_namespace_.c_str());

        // start custom queue for diff drive
        this->callback_queue_thread_ = boost::thread(boost::bind(&PublishTFModesSDF::QueueThread, this));

        // listen to the update event (broadcast every simulation iteration)
        this->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&PublishTFModesSDF::UpdateChild, this));
    }


    // Update the controller
    void PublishTFModesSDF::UpdateChild() {
        #if (GAZEBO_MAJOR_VERSION >= 8)
            common::Time current_time = this->world->SimTime();
        #else
            common::Time current_time = this->world->GetSimTime();
        #endif
            double seconds_since_last_update = (current_time - last_update_time_).Double();

        if (seconds_since_last_update > update_period_) {
            publishTF();
            last_update_time_+= common::Time(update_period_);
        }
    }

    void PublishTFModesSDF::QueueThread() 
    {
        static const double timeout = 0.01;

        while (alive_ && rosnode_->ok()) {
            queue_.callAvailable(ros::WallDuration(timeout));
        }
    }

    void PublishTFModesSDF::publishTF(void)
    {
        static tf::TransformBroadcaster br;
        tf::Transform t_;
        ignition::math::Pose3d  pos_l;
        std::string parent_;

        for(size_t i=0 ; i < links_.size(); i++){
            pos_l = links_[i]->RelativePose();
            parent_ = links_[i]->GetParent()->GetName();
            t_.setOrigin( tf::Vector3(pos_l.Pos().X(), pos_l.Pos().Y(), pos_l.Pos().Z()) );
            t_.setRotation(tf::Quaternion(pos_l.Rot().X(),pos_l.Rot().Y(),pos_l.Rot().Z(), pos_l.Rot().W()));  
            br.sendTransform(tf::StampedTransform(t_, ros::Time::now(), robot_namespace_+"/"+base_frame_, robot_namespace_+"/"+link_names_[i]));
// std::cout << "robot_namespace_+parent_=" << robot_namespace_ +"/"+parent_ << " , robot_namespace_+link_names_[i]= "<< robot_namespace_+"/"+link_names_[i] << std::endl; 
        }
    }

  GZ_REGISTER_MODEL_PLUGIN(PublishTFModesSDF)
}