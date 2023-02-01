/*
 * Copyright 2013 Open Source Robotics Foundation
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

/*
   Desc: GazeboRosForce plugin for manipulating objects in Gazebo
   Author: John Hsu
   Date: 24 Sept 2008
 */

#include <algorithm>
#include <assert.h>

#include <gazebo_plugins/gazebo_ros_force.h>
#ifdef ENABLE_PROFILER
#include <ignition/common/Profiler.hh>
#endif

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(GazeboRosForce);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosForce::GazeboRosForce()
{
  this->wrench_msg_left.force.x = 0;
  this->wrench_msg_left.force.y = 0;
  this->wrench_msg_left.force.z = 0;
  this->wrench_msg_left.torque.x = 0;
  this->wrench_msg_left.torque.y = 0;
  this->wrench_msg_left.torque.z = 0;

  this->wrench_msg_right.force.x = 0;
  this->wrench_msg_right.force.y = 0;
  this->wrench_msg_right.force.z = 0;
  this->wrench_msg_right.torque.x = 0;
  this->wrench_msg_right.torque.y = 0;
  this->wrench_msg_right.torque.z = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosForce::~GazeboRosForce()
{
  this->update_connection_.reset();

  // Custom Callback Queue
  this->queue_left.clear();
  this->queue_left.disable();
  this->queue_right.clear();
  this->queue_right.disable();
  this->rosnode_left->shutdown();
  this->rosnode_right->shutdown();
  this->callback_queue_thread_left.join();
  this->callback_queue_thread_right.join();

  delete this->rosnode_left;
  delete this->rosnode_right;

}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosForce::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Get the world name.
  this->world_ = _model->GetWorld();

  // load parameters
  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  this->update_rate_ = 100;
  if (_sdf->HasElement("updateRate")) {
    this->update_rate_ = _sdf->GetElement("updateRate")->Get<std::double_t>();
    ROS_INFO_STREAM("gazebo_ros_force: [updateRate] set to [" + std::to_string(update_rate_) + "]");
  }
  else
    ROS_WARN_STREAM("gazebo_ros_force: [updateRate] set to default [" + std::to_string(update_rate_) + "]");

  if (!_sdf->HasElement("bodyNameLeft"))
  {
    ROS_FATAL_NAMED("force", "force plugin missing <bodyNameLeft>, cannot proceed");
    return;
  }
  else
    this->link_name_left_ = _sdf->GetElement("bodyNameLeft")->Get<std::string>();

  this->link_left_ = _model->GetLink(this->link_name_left_);
  if (!this->link_left_)
  {
    ROS_FATAL_NAMED("force", "gazebo_ros_force plugin error: link named: %s does not exist\n",this->link_name_left_.c_str());
    return;
  }

  if (!_sdf->HasElement("bodyNameRight"))
  {
    ROS_FATAL_NAMED("force", "force plugin missing <bodyNameRight>, cannot proceed");
    return;
  }
  else
    this->link_name_right_ = _sdf->GetElement("bodyNameRight")->Get<std::string>();

  this->link_right_ = _model->GetLink(this->link_name_right_);
  if (!this->link_right_)
  {
    ROS_FATAL_NAMED("force", "gazebo_ros_force plugin error: link named: %s does not exist\n",this->link_name_right_.c_str());
    return;
  }

  if (!_sdf->HasElement("topicNameLeft"))
  {
    ROS_FATAL_NAMED("force", "force plugin missing <topicNameLeft>, cannot proceed");
    return;
  }
  else
    this->topic_name_left_ = _sdf->GetElement("topicNameLeft")->Get<std::string>();

  if (!_sdf->HasElement("topicNameRight"))
  {
    ROS_FATAL_NAMED("force", "force plugin missing <topicNameRight>, cannot proceed");
    return;
  }
  else
    this->topic_name_right_ = _sdf->GetElement("topicNameRight")->Get<std::string>();

    // Initialize update rate stuff
    if ( this->update_rate_ > 0.0 ) this->update_period_ = 1.0 / this->update_rate_;
    else this->update_period_ = 0.0;
#if GAZEBO_MAJOR_VERSION >= 8
    last_update_time_ = world_->SimTime();
#else
    last_update_time_ = world_->GetSimTime();
#endif

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("force", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so_left' in the gazebo_ros package)");
    return;
  }

  this->rosnode_left = new ros::NodeHandle(this->robot_namespace_);
  this->rosnode_right = new ros::NodeHandle(this->robot_namespace_);

  // Custom Callback Queue
  ros::SubscribeOptions so_left = ros::SubscribeOptions::create<geometry_msgs::Wrench>(
    this->topic_name_left_,1,
    boost::bind( &GazeboRosForce::UpdateObjectForceLeft,this,_1),
    ros::VoidPtr(), &this->queue_left);
  this->sub_left = this->rosnode_left->subscribe(so_left);

  ros::SubscribeOptions so_right = ros::SubscribeOptions::create<geometry_msgs::Wrench>(
    this->topic_name_right_,1,
    boost::bind( &GazeboRosForce::UpdateObjectForceRight,this,_1),
    ros::VoidPtr(), &this->queue_right);
  this->sub_right = this->rosnode_right->subscribe(so_right);

  // Custom Callback Queue
  this->callback_queue_thread_left = boost::thread( boost::bind( &GazeboRosForce::QueueThreadLeft,this ) );
  this->callback_queue_thread_right = boost::thread( boost::bind( &GazeboRosForce::QueueThreadRight,this ) );

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosForce::UpdateChild, this));
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosForce::UpdateObjectForceLeft(const geometry_msgs::Wrench::ConstPtr& _msg)
{
  this->wrench_msg_left.force.x = _msg->force.x;
  this->wrench_msg_left.force.y = _msg->force.y;
  this->wrench_msg_left.force.z = _msg->force.z;
  this->wrench_msg_left.torque.x = _msg->torque.x;
  this->wrench_msg_left.torque.y = _msg->torque.y;
  this->wrench_msg_left.torque.z = _msg->torque.z;
}

void GazeboRosForce::UpdateObjectForceRight(const geometry_msgs::Wrench::ConstPtr& _msg)
{
  this->wrench_msg_right.force.x = _msg->force.x;
  this->wrench_msg_right.force.y = _msg->force.y;
  this->wrench_msg_right.force.z = _msg->force.z;
  this->wrench_msg_right.torque.x = _msg->torque.x;
  this->wrench_msg_right.torque.y = _msg->torque.y;
  this->wrench_msg_right.torque.z = _msg->torque.z;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosForce::UpdateChild()
{
#ifdef ENABLE_PROFILER
  IGN_PROFILE("GazeboRosForce::OnNewFrame");
  IGN_PROFILE_BEGIN("fill ROS message");
#endif

#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = world_->SimTime();
#else
    common::Time current_time = world_->GetSimTime();
#endif
    double seconds_since_last_update = ( current_time - last_update_time_ ).Double();

    if ( seconds_since_last_update > update_period_ ) {
      //ROS_INFO_STREAM(std::to_string(seconds_since_last_update));
      this->lock_.lock();

      ignition::math::Vector3d force_l(this->wrench_msg_left.force.x,this->wrench_msg_left.force.y,this->wrench_msg_left.force.z);
      ignition::math::Vector3d torque_l(this->wrench_msg_left.torque.x,this->wrench_msg_left.torque.y,this->wrench_msg_left.torque.z);
      this->link_left_->AddRelativeForce(force_l);
      this->link_left_->AddRelativeTorque(torque_l);

      ignition::math::Vector3d force_r(this->wrench_msg_right.force.x,this->wrench_msg_right.force.y,this->wrench_msg_right.force.z);
      ignition::math::Vector3d torque_r(this->wrench_msg_right.torque.x,this->wrench_msg_right.torque.y,this->wrench_msg_right.torque.z);
      this->link_right_->AddRelativeForce(force_r);
      this->link_right_->AddRelativeTorque(torque_r);

      this->lock_.unlock();
      last_update_time_+= common::Time ( update_period_ );
    }

#ifdef ENABLE_PROFILER
  IGN_PROFILE_END();
#endif
}
// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void GazeboRosForce::QueueThreadLeft()
{
  static const double timeout = 0.01;

  while (this->rosnode_left->ok())
  {
    this->queue_left.callAvailable(ros::WallDuration(timeout));
  }
}

void GazeboRosForce::QueueThreadRight()
{
  static const double timeout = 0.01;

  while (this->rosnode_right->ok())
  {
    this->queue_right.callAvailable(ros::WallDuration(timeout));
  }
}
}
