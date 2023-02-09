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

#include <boost/algorithm/string.hpp>
#include <algorithm>
#include <assert.h>

#include <gazebo_plugins/gazebo_ros_force.h>

#ifdef ENABLE_PROFILER
#include <ignition/common/Profiler.hh>
#endif

namespace gazebo {
    GZ_REGISTER_MODEL_PLUGIN(GazeboRosForce);

////////////////////////////////////////////////////////////////////////////////
// Constructor
    GazeboRosForce::GazeboRosForce() {}

////////////////////////////////////////////////////////////////////////////////
// Destructor
    GazeboRosForce::~GazeboRosForce() {
        this->update_connection_.reset();

        // Custom Callback Queue
        this->queue_.clear();
        this->queue_.disable();
        this->rosnode_->shutdown();
        this->callback_queue_thread_.join();

        delete this->rosnode_;
    }

////////////////////////////////////////////////////////////////////////////////
// Load the controller
    void GazeboRosForce::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
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
        } else
            ROS_WARN_STREAM("gazebo_ros_force: [updateRate] set to default [" + std::to_string(update_rate_) + "]");

        if (!_sdf->HasElement("bodyName")) {
            ROS_FATAL_NAMED("force", "force plugin missing <bodyName>, cannot proceed");
            return;
        } else {
            sdf::ElementPtr element = _sdf->GetElement("bodyName");
            auto link_names = element->Get<std::string>();
            boost::erase_all(link_names, " ");
            boost::split(link_names_, link_names, boost::is_any_of(","));
        }

        for (const auto &link_name: link_names_) {
            physics::LinkPtr link = _model->GetLink(link_name);
            if (!link) {
                ROS_FATAL_NAMED("force", "gazebo_ros_force plugin error: link named: %s does not exist!\n",
                                link_name.c_str());
                return;
            }
            links_.push_back(link);
        }

        if (!_sdf->HasElement("topicName")) {
            ROS_FATAL_NAMED("force", "force plugin missing <topicName>, cannot proceed");
            return;
        } else {
            sdf::ElementPtr element = _sdf->GetElement("topicName");
            auto topic_names = element->Get<std::string>();
            boost::erase_all(topic_names, " ");
            boost::split(topic_names_, topic_names, boost::is_any_of(","));
        }

        // Initialize update rate stuff
        if (this->update_rate_ > 0.0) this->update_period_ = 1.0 / this->update_rate_;
        else this->update_period_ = 0.0;
#if GAZEBO_MAJOR_VERSION >= 8
        last_update_time_ = world_->SimTime();
#else
        last_update_time_ = world_->GetSimTime();
#endif

        // Make sure the ROS node for Gazebo has already been initialized
        if (!ros::isInitialized()) {
            ROS_FATAL_STREAM_NAMED("force", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
                    << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }

        this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

        unsigned int n = topic_names_.size();
        for (int i = 0; i < n; i++) {
            this->wrench_msgs_.emplace_back();
            this->wrench_msgs_[i].force.x = 0;
            this->wrench_msgs_[i].force.y = 0;
            this->wrench_msgs_[i].force.z = 0;
            this->wrench_msgs_[i].torque.x = 0;
            this->wrench_msgs_[i].torque.y = 0;
            this->wrench_msgs_[i].torque.z = 0;
        }

        ////////////////////////////////////////////////////////////////////////////////
        // Update the controller
        for (unsigned int i = 0; i < n; i++) {
            UpdateObjectForces.emplace_back([i, this](const geometry_msgs::Wrench::ConstPtr &_msg) {

                this->wrench_msgs_[i].force.x = _msg->force.x;
                this->wrench_msgs_[i].force.y = _msg->force.y;
                this->wrench_msgs_[i].force.z = _msg->force.z;
                this->wrench_msgs_[i].torque.x = _msg->torque.x;
                this->wrench_msgs_[i].torque.y = _msg->torque.y;
                this->wrench_msgs_[i].torque.z = _msg->torque.z;
            });
        }

        ////////////////////////////////////////////////////////////////////////////////

        // Custom Callback Queue
        std::vector<ros::SubscribeOptions> so_vec;
        for (int i = 0; i < n; i++) {
            so_vec.push_back(
                    ros::SubscribeOptions::create<geometry_msgs::Wrench>(
                            topic_names_[i], 1,
                            UpdateObjectForces[i],
                            ros::VoidPtr(), &this->queue_)
            );
            this->subs_.push_back(this->rosnode_->subscribe(so_vec[i]));
        }
        // Custom Callback Queue
        this->callback_queue_thread_ = boost::thread(boost::bind(&GazeboRosForce::QueueThread, this));

        // New Mechanism for Updating every World Cycle
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&GazeboRosForce::UpdateChild, this));
    }

// Update the controller
    void GazeboRosForce::UpdateChild() {
#ifdef ENABLE_PROFILER
        IGN_PROFILE("GazeboRosForce::OnNewFrame");
    IGN_PROFILE_BEGIN("fill ROS message");
#endif

#if GAZEBO_MAJOR_VERSION >= 8
        common::Time current_time = world_->SimTime();
#else
        common::Time current_time = world_->GetSimTime();
#endif
        double seconds_since_last_update = (current_time - last_update_time_).Double();

        if (seconds_since_last_update > update_period_) {
            //ROS_INFO_STREAM(std::to_string(seconds_since_last_update));
            this->lock_.lock();
            unsigned int n = wrench_msgs_.size();
            for (unsigned int i = 0; i < n; i++) {
                ignition::math::Vector3d force(this->wrench_msgs_[i].force.x, this->wrench_msgs_[i].force.y,
                                               this->wrench_msgs_[i].force.z);
                ignition::math::Vector3d torque(this->wrench_msgs_[i].torque.x, this->wrench_msgs_[i].torque.y,
                                                this->wrench_msgs_[i].torque.z);
                this->links_[i]->AddRelativeForce(force);
                this->links_[i]->AddRelativeTorque(torque);
            }
            this->lock_.unlock();

            last_update_time_ += common::Time(update_period_);
        }

#ifdef ENABLE_PROFILER
        IGN_PROFILE_END();
#endif
    }

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
    void GazeboRosForce::QueueThread() {
        static const double timeout = 0.01;

        while (this->rosnode_->ok()) {
            this->queue_.callAvailable(ros::WallDuration(timeout));
        }
    }

}
