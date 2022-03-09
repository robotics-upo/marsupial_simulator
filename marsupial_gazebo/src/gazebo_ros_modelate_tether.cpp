#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo_ros_modelate_tether.h"

namespace gazebo
{
    void Boxscale::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr sdf)
    {
        model_ = _parent;
        world_ = model_->GetWorld();

         robot_namespace_ = "";
        if (!sdf->HasElement("namespace"))
            ROS_WARN("Boxscale Plugin missing <namespace>, defaults to %s", this->robot_namespace_.c_str());
        else {
            this->robot_namespace_ = sdf->GetElement("namespace")->Get<std::string>();
            ROS_INFO_STREAM("modelateTether <namespace> set to: "<<robot_namespace_);
        }

        this->num_links = 1;
        if (!sdf->HasElement("numLinkTether")) 
            ROS_WARN("Boxscale Plugin missing <numLinkTether>, defaults to %i", this->num_links);
        else{ 
            this->num_links = sdf->GetElement("numLinkTether")->Get<int>();
            ROS_INFO_STREAM("modelateTether <numLinkTether> set to: "<<num_links);
        }

        this->name_link_tether = "link";
        if (!sdf->HasElement("nameLinkFrame")) 
            ROS_WARN("Boxscale Plugin missing <nameLinkFrame>, defaults to \"%s\"", this->name_link_tether.c_str());
        else{ 
            this->name_link_tether = sdf->GetElement("nameLinkFrame")->Get<std::string>();
            ROS_INFO_STREAM("modelateTether <nameLinkFrame> set to: "<<name_link_tether);
        }

        this->name_joint_tether = "joint";
        if (!sdf->HasElement("nameJointFrame")) 
            ROS_WARN("Boxscale Plugin missing <nameJointFrame>, defaults to \"%s\"", this->name_joint_tether.c_str());
        else{ 
            this->name_joint_tether = sdf->GetElement("nameJointFrame")->Get<std::string>();
            ROS_INFO_STREAM("modelateTether <nameJointFrame> set to: "<<name_joint_tether);
        }


        this->length_tether = -1.0;
        if (!sdf->HasElement("initTetherLength")) 
            ROS_WARN("Boxscale Plugin missing <initTetherLength>, defaults to %f", this->length_tether);
        else{ 
            this->length_tether = sdf->GetElement("initTetherLength")->Get<double>();
            ROS_INFO_STREAM("modelateTether <initTetherLength> set to: "<<length_tether);
        }

        this->length_link = 0.01;
        if (!sdf->HasElement("initLengthLink")) 
            ROS_WARN("Boxscale Plugin missing <initLengthLink>, defaults to %f", this->length_link);
        else{ 
            this->length_link = sdf->GetElement("initLengthLink")->Get<double>();
            ROS_INFO_STREAM("modelateTether <initLengthLink> set to: "<<length_link);
        }

        this->length_gap = 0.002;
        if (!sdf->HasElement("lengthGap")) 
            ROS_WARN("Boxscale Plugin missing <lengthGap>, defaults to %f", this->length_gap);
        else{ 
            this->length_gap = sdf->GetElement("lengthGap")->Get<double>();
            ROS_INFO_STREAM("modelateTether <lengthGap> set to: "<<length_gap);
        }

        this->max_length = 20.0;
        if (!sdf->HasElement("maxLengthTether")) 
            ROS_WARN("Boxscale Plugin missing <maxLengthTether>, defaults to %f", this->max_length);
        else{ 
            this->max_length = sdf->GetElement("maxLengthTether")->Get<double>();
            ROS_INFO_STREAM("modelateTether <maxLengthTether> set to: "<<max_length);
        }

        this->min_length = 0.8;
        if (!sdf->HasElement("minLengthTether")) 
            ROS_WARN("Boxscale Plugin missing <minLengthTether>, defaults to %f", this->min_length);
        else{ 
            this->min_length = sdf->GetElement("minLengthTether")->Get<double>();
            ROS_INFO_STREAM("modelateTether <minLengthTether> set to: "<<min_length);
        }

        alive_ = true;

        // Ensure that ROS has been initialized and subscribe to cmd_vel
        if (!ros::isInitialized())
        {
        ROS_FATAL_STREAM_NAMED("modelate catenary", "modelateTetherPlugin (ns = " << robot_namespace_
            << "). A ROS node for Gazebo has not been initialized, "
            << "unable to load plugin. Load the Gazebo system plugin "
            << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
        }
         rosnode_.reset( new ros::NodeHandle(this->robot_namespace_));

        // Initialize values
        initial_length_link = length_link; // Necessary value to calculate global visual scale
        initial_joint_pos = length_link/2.0 + length_gap / 2.0;
        joint_pos = initial_joint_pos;
        scale = 1;
        count = 0;

        node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
        node_->Init(world_->Name());
        pub_visual_ = node_->Advertise<gazebo::msgs::Visual>("~/visual");

        std::cerr << "Initializing with values:  initial_length_link= " << initial_length_link << " , length_link*num_links+length_gap*(num_links-1)= " << num_links* length_link+length_gap*(num_links-1) << " , length_gap=" << length_gap << " , length_tether= " <<length_tether << " , joint_step= " << joint_step << " , scale=" << scale << std::endl;

        // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
        ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32>("length_tether", 1, boost::bind(&Boxscale::updatelength,this, _1),ros::VoidPtr(),&queue_);
        lenght_tether_subscriber_ = rosnode_->subscribe(so);

        // start custom queue for diff drive
        callback_queue_thread_ = boost::thread(boost::bind(&Boxscale::QueueThread, this));
        this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&Boxscale::OnUpdate, this));
    }

    void Boxscale::OnUpdate()
    {
        if (length_tether != -1.0)
        {
            double error_ = length_tether * 0.01; // porcent error
            double sign_joint_;
            if (visual_step < 0)
                sign_joint_ = -1.0;
            else
                sign_joint_ = 1.0;

            if ( (length_tether + error_) < (num_links* length_link + (num_links - 1)*length_gap) || (length_tether- error_) > (num_links* length_link + (num_links - 1)*length_gap )){

            	std::string yy_ ;
				yy_ = "s";
				/********************* To obligate stop method and check Optimization result *********************/
					std::cout << " Press key 'y': ";
					while (yy_ != "y"){
						std::cin >> yy_ ;
					}
				/**********************************************************************************************/

                // Process to get new Visual Link
                // double old_length_link_ = length_link;
                // for (int i = 0 ; i < iterations ; i ++)
                // {
                    scale =  scale + visual_step; 
                    length_link = length_link + sign_joint_*joint_step; 

                    joint_pos = joint_pos + sign_joint_*joint_step;
                    // double move_link_ = length_link - old_length_link_; 
                    // std::cerr << "move_link_= "<< move_link_ <<" , length_link= " << length_link << " , old_length_link_= " << old_length_link_ << std::endl;

                    ignition::math::Vector3d initial_scale(1.0, 1.0, scale);
                    for (int j=1; j<= 19 ; j++ ){    // loop for to apply scale and visual_scale in each link
                        // scaleLink(initial_scale, j) ;
                        if(j < num_links)
                            moveJoints(joint_pos,j);
                        // pub_visual_->Publish( scaleLink(initial_scale, j) );
                        // if (j != 1 && j!= num_links)
                        //     moveLinks(move_link_, j);
                    }
                    std::cerr << "Setting["<< count <<"]   scale= " << scale << " , num_links= " << num_links << " , length_link= " << length_link <<" , (num_links* length_link + (num_links - 1)*length_gap): " << (num_links* length_link + (num_links - 1)*length_gap) << " , length_gap= " << length_gap << " , length_tether:" <<length_tether << " , joint_step:" << joint_step << std::endl;
                } 
            // }
            count++;
        }
    }

    void Boxscale::updatelength (const std_msgs::Float32::ConstPtr& length_tether_msg) 
    {
        boost::mutex::scoped_lock scoped_lock(lock);
        length_tether = length_tether_msg->data;

        std::cerr << "Receiving values:  length_tether= " << length_tether << " , num_links= " << num_links << " , length_gap= " <<length_gap << std::endl;
        new_length_link = ( length_tether - (num_links - 1) * length_gap ) / num_links; // Compute new value for link
        joint_offset = (new_length_link - length_link)/2.0 + length_gap / 2.0;
        new_scale = new_length_link / initial_length_link;
        iterations =  round(sqrt((new_length_link - length_link)*(new_length_link - length_link)  + length_gap / 2.0) / joint_step);
        visual_step = ( new_scale - scale )/iterations;

        count = 0;
        
        ROS_INFO_STREAM("Subscribe: new_length_link: "<<new_length_link << " , joint_offset: "<<joint_offset << " , new_scale: "<<new_scale << " , iterations: "<<iterations << " , visual_step: "<<visual_step);
    }

    void Boxscale::QueueThread()
    {
        static const double timeout = 0.01;
        while (alive_ && rosnode_->ok())
        {
        queue_.callAvailable(ros::WallDuration(timeout));
        }
    }

    void Boxscale::scaleLink(ignition::math::Vector3d scale_, int n_links)
    {
        std::string visual_name_ = name_link_tether+"_visual_"+std::to_string(n_links);
        std::string linkName = name_link_tether+"_"+std::to_string(n_links);
        physics::LinkPtr link_ = this->model_->GetLink(linkName);

        link_->SetScale(scale_);
    }

    // gazebo::msgs::Visual Boxscale::scaleLink(ignition::math::Vector3d scale_, int n_links)
    // {
    //     std::string visual_name_ = name_link_tether+"_visual_"+std::to_string(n_links);
    //     std::string linkName = name_link_tether+"_"+std::to_string(n_links);
    //     physics::LinkPtr link_ = this->model_->GetLink(linkName);
    //     gazebo::msgs::Visual visualMsg = link_->GetVisualMessage(visual_name_);
    //     gazebo::msgs::Vector3d* scale_factor = new gazebo::msgs::Vector3d{gazebo::msgs::Convert(scale_)};
    //     visualMsg.set_name(link_->GetScopedName());
    //     visualMsg.set_parent_name(this->model_->GetScopedName());
    //     visualMsg.set_allocated_scale(scale_factor);
    //     return visualMsg;
    // }

    void Boxscale::moveLinks(double move_link_, int n_links)
    {
        std::string linkName1_ = name_link_tether+"_"+std::to_string(n_links);
        std::string linkName2_ = name_link_tether+"_"+std::to_string(n_links-1);
        physics::LinkPtr link1_ = this->model_->GetLink(linkName1_);
        physics::LinkPtr link2_ = this->model_->GetLink(linkName2_);
        link1_->SetParent(link2_);
        ignition::math::Pose3d plink1_ = link1_->WorldPose(); 
        ignition::math::Pose3d plink2_ = link2_->WorldPose(); 
        // double x_ = plink1_.Pos().X()-plink2_.Pos().X();
        // double y_ = plink1_.Pos().Y()-plink2_.Pos().Y();
        // double z_ = plink1_.Pos().Z()-plink2_.Pos().Z()+move_link_;
        // double R_ = plink1_.Rot().Roll(); 
        // double P_ = plink1_.Rot().Pitch();
        // double Ya_ = plink1_.Rot().Yaw();
        double x_ = 0.0;
        double y_ = 0.0;
        double z_ = plink1_.Pos().Z()+plink2_.Pos().Z()+move_link_;
        double R_ = 0.0;
        double P_ = 0.0;
        double Ya_ = 0.0;
        ignition::math::Pose3d m_ = ignition::math::Pose3d(x_,y_,z_,R_,P_,Ya_);
        std::cerr << "moveLinks["<<n_links <<"]: plink1_.Pos().Z()= "<<plink1_.Pos().Z()<<" , plink2_.Pos().Z()= "<<plink2_.Pos().Z()<< std::endl;
        std::cerr << "moveLinks["<<n_links <<"]: move_link_= "<<move_link_<<" x()= "<<x_<<" , y()= "<<y_<<" , z()="<<z_<<" , R()= "<<R_<<" , P()= "<<P_<<" , Y()="<<Ya_<< std::endl;
       
        link1_->SetRelativePose(m_,true,true);
    }

    void Boxscale::moveJoints(double move_joint_, int n_links)
    {
        std::string jointName_ = name_joint_tether+"_"+std::to_string(n_links)+"_"+std::to_string(n_links+1);
        physics::JointPtr joint_ = this->model_->GetJoint(jointName_);
        joint_->SetPosition(0, move_joint_);
    }

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(Boxscale)
}