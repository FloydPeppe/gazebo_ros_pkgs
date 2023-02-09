/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
 * Desc: A dynamic controller plugin that performs generic force interface.
 * Author: John Hsu
 * Date: 24 Sept 2008
 */

#ifndef GAZEBO_ROS_FORCE_HH
#define GAZEBO_ROS_FORCE_HH

#include <string>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Wrench.h>

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>


namespace gazebo {
/// @addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
/// @{
/** \defgroup GazeboRosForce Plugin XML Reference and Example

  \brief Ros Force Plugin.

  This is a Plugin that collects data from a ROS topic and applies wrench to a body accordingly.

  Example Usage:
  \verbatim
      <gazebo>
        <plugin filename="libgazebo_ros_force.so" name="gazebo_ros_force">
          <bodyName>link_1, link_2</bodyName>
          <topicName>force_1, force_2</topicName>
        </plugin>
      </gazebo>
  \endverbatim

\{
*/

/**
           .

*/

    class GazeboRosForce : public ModelPlugin {

    public:
        /// \brief Constructor
        GazeboRosForce();

        /// \brief Destructor
        virtual ~GazeboRosForce();

    protected:
        // Documentation inherited
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

        // Documentation inherited
        virtual void UpdateChild();

    private:
        /// \brief call back when a Wrench message is published
        /// \param[in] _msg The Incoming ROS message representing the new force to exert.
        std::vector<std::function<void (const geometry_msgs::Wrench::ConstPtr)>> UpdateObjectForces;

        /// \brief The custom callback queue thread function.
        void QueueThread();

        /// \brief A pointer to the gazebo world.
        physics::WorldPtr world_;

        /// \brief A pointer to the Link, where force is applied
        std::vector<physics::LinkPtr> links_;

        /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
        ros::NodeHandle *rosnode_;
        std::vector<ros::Subscriber> subs_;

        /// \brief A mutex to lock access to fields that are used in ROS message callbacks
        boost::mutex lock_;

        /// \brief ROS Wrench topic name inputs
        std::vector<std::string> topic_names_;

        /// \brief The Link this plugin is attached to, and will exert forces on.
        std::vector<std::string> link_names_;

        /// \brief for setting ROS name space
        std::string robot_namespace_;

        // Custom Callback Queue
        ros::CallbackQueue queue_;

        /// \brief Thead object for the running callback Thread.
        boost::thread callback_queue_thread_;

        /// \brief Container for the wrench force that this plugin exerts on the body.
        std::vector<geometry_msgs::Wrench> wrench_msgs_;

        // Pointer to the update event connection
        event::ConnectionPtr update_connection_;

        double update_rate_;
        double update_period_;
        common::Time last_update_time_;
    };
/** \} */
/// @}
}
#endif
