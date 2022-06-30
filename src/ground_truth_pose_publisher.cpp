/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); 
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
 
#include <iostream>
#include <string>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>

ignition::transport::v11::Node::Publisher pub;
char *model_name = NULL;
const char *global_frame = "world";
ignition::msgs::Time t;

//////////////////////////////////////////////////
/// \brief Function called each time a topic update is received.
void cb(const ignition::msgs::Pose_V &_msg)
{
  std::cout << "Msg: " << _msg.SerializeAsString() << std::endl << std::endl;
  
  ignition::msgs::Pose_V sending_msg;

  for (auto i = _msg.pose().begin(); i != _msg.pose().end(); i++) {
      if (strcmp (model_name, i->name().c_str()) == 0) {
          std::cout << "Model found\n";
          ignition::msgs::Pose *pose = sending_msg.add_pose();
          ignition::msgs::Header *head = new ignition::msgs::Header;
          auto data = head->add_data();
          data->add_value(global_frame);
          data->set_key("frame_id");
          auto data2 = head->add_data();
          data2->add_value(model_name);
          data2->set_key("child_frame_id");
          pose->set_allocated_header(head);
          ignition::msgs::Vector3d *pos = new ignition::msgs::Vector3d;
          pos->set_x(i->position().x());
          pos->set_y(i->position().y());
          pos->set_z(i->position().z());
          pose->set_allocated_position(pos);

          ignition::msgs::Quaternion *q = new ignition::msgs::Quaternion;
          q->set_w(i->orientation().w());
          q->set_x(i->orientation().x());
          q->set_y(i->orientation().y());
          q->set_z(i->orientation().z());
          pose->set_allocated_orientation(q);

          // Stamp stuff
          ignition::msgs::Time *stamp = new ignition::msgs::Time(t);

          head->set_allocated_stamp(stamp);


          pub.Publish(sending_msg);
      }
  }
}

//////////////////////////////////////////////////
/// \brief Function called each time a topic update is received.
void clock_cb(const ignition::msgs::WorldStatistics &_msg)
{
//   std::cout << "Msg: " << _msg.data() << std::endl << std::endl;
  t = _msg.sim_time();
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  if (argc < 3) {
      std::cout << "Usage: " << argv[0] << " <world_name> <model_name> [<global_frame_id>]";
  }

  ignition::transport::Node node;
  std::ostringstream topic;
  topic << "/world/" << argv[1] << "/pose/info";
  model_name = argv[2];

  if (argc > 3) {
    global_frame = argv[3];
  }

  pub = node.Advertise<ignition::msgs::Pose_V>("/gt");


  // Subscribe to the gt pose
  if (!node.Subscribe(topic.str(), cb))
  {
    std::cerr << "Error subscribing to topic [" << topic.str() << "]" << std::endl;
    return -1;
  }

  // Subscribe to the clock
  if (!node.Subscribe("/stats", clock_cb))
  {
    std::cerr << "Error subscribing to topic [/stats]" << std::endl;
    return -1;
  }

  // Zzzzzz.
  ignition::transport::waitForShutdown();

  return 0;
}
