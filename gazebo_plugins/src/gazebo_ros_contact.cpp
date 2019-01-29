// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gazebo_plugins/gazebo_ros_contact.hpp"

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Entity.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo_msgs/msg/contact_state.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/wrench.hpp>

#include <tinyxml.h>
#include <algorithm>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <iostream>

namespace gazebo_plugins
{

class GazeboRosContactPrivate
{
public:
  /// Callback when a world is created.
  /// \param[in] _world_name The world's name
  void OnWorldCreated(const std::string & _world_name);

  void Callback();

  void PublishContacts(ConstContactsPtr &_msg);

  /// \brief World pointer from Gazebo.
  gazebo::physics::WorldPtr world_;

  /// ROS node for communication, managed by gazebo_ros.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Publishes contacts
  rclcpp::Publisher<gazebo_msgs::msg::ContactState>::SharedPtr contact_pub_;

  /// Gazebo node for communication.
  gazebo::transport::NodePtr gz_node_;

  /// To be notified once the world is created.
  gazebo::event::ConnectionPtr world_created_connection_;

  /// Connection to world update event, called at every iteration
  gazebo::event::ConnectionPtr world_update_event_;

};

GazeboRosContact::GazeboRosContact()
: impl_(std::make_unique<GazeboRosContactPrivate>())
{
}

GazeboRosContact::~GazeboRosContact()
{
}

void GazeboRosContact::Load(int  argc , char **  argv )
{
  // Initialize ROS with arguments
  if (!rclcpp::is_initialized()) {
    rclcpp::init(argc, argv);
    impl_->ros_node_ = gazebo_ros::Node::Get();
  } else {
    impl_->ros_node_ = gazebo_ros::Node::Get();
    RCLCPP_WARN(impl_->ros_node_->get_logger(),
      "gazebo_ros_contact didn't initialize ROS "
      "because it's already initialized with other arguments");
  }

  impl_->contact_pub_ = impl_->ros_node_->create_publisher<gazebo_msgs::msg::ContactState>(
    "/gazebo_contacts");

  impl_->world_created_connection_ = gazebo::event::Events::ConnectWorldCreated(
    std::bind(&GazeboRosContactPrivate::OnWorldCreated, impl_.get(), std::placeholders::_1));

  impl_->world_update_event_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosContactPrivate::Callback, impl_.get()));

}

void GazeboRosContactPrivate::OnWorldCreated(const std::string & _world_name)
{
  // Only support one world
  world_created_connection_.reset();

  world_ = gazebo::physics::get_world();

  // Gazebo transport
  gz_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  gz_node_->Init(_world_name);
}

// Function is called everytime a message is received.
void GazeboRosContactPrivate::Callback()
{
  gz_node_->Subscribe(
    "/gazebo/default/physics/contacts",
    &GazeboRosContactPrivate::PublishContacts, this);
}


// Function is called everytime a message is received.
void GazeboRosContactPrivate::PublishContacts(ConstContactsPtr &_msg)
{
  gazebo_msgs::msg::ContactState contact;
  geometry_msgs::msg::Vector3 position;
  geometry_msgs::msg::Vector3 normals;
  geometry_msgs::msg::Wrench total_wrench;

  if (_msg->contact_size() > 0){
    if (_msg->contact(0).collision1() != "ground_plane::link::collision" && _msg->contact(0).collision2() != "ground_plane::link::collision"){
      contact.collision1_name = _msg->contact(0).collision1();
      contact.collision2_name = _msg->contact(0).collision2();

      for (int j = 0; j <  _msg->contact(0).position_size(); ++j){
        contact.collision1_name = _msg->contact(0).collision1();
        contact.collision2_name = _msg->contact(0).collision2();

        position.x =  _msg->contact(0).position(j).x();
        position.y =  _msg->contact(0).position(j).y();
        position.z =  _msg->contact(0).position(j).z();
        contact.contact_positions.push_back(position);

        normals.x =  _msg->contact(0).normal(j).x();
        normals.y =  _msg->contact(0).normal(j).y();
        normals.z =  _msg->contact(0).normal(j).z();
        contact.contact_normals.push_back(normals);

        contact.depths.push_back(static_cast<float>(_msg->contact(0).depth(0)));
      }
    }
  }
  contact_pub_->publish(contact);
}

GZ_REGISTER_SYSTEM_PLUGIN(GazeboRosContact)

}  // namespace gazebo_ros