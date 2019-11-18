/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_PLUGINS_ARDUPILOTPLUGIN_HH_
#define GAZEBO_PLUGINS_ARDUPILOTPLUGIN_HH_

#include <sdf/sdf.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

// Modeled after Velodyne example
// http://gazebosim.org/tutorials?tut=guided_i6&cat=
// Having a separate thread to process ROS messages does not seem to be necessary
//#include <thread>
#include "ros/ros.h"
#include "std_msgs/Int32.h"
//#include "ros/callback_queue.h"
//#include "ros/subscribe_options.h"

namespace gazebo
{
  // Forward declare private data class
  class ArduPilotSocketPrivate;
  class ArduPilotPluginPrivate;

  /// \brief Interface ArduPilot from ardupilot stack
  /// modeled after SITL/SIM_*
  ///
  /// The plugin requires the following parameters:
  /// <control>             control description block
  ///    <!-- inputs from Ardupilot -->
  ///    channel            attribute, ardupilot control channel
  ///    multiplier         command multiplier
  ///    <!-- output to Gazebo -->
  ///    type               type of control, VELOCITY, POSITION or EFFORT
  ///    <p_gain>           velocity pid p gain
  ///    <i_gain>           velocity pid i gain
  ///    <d_gain>           velocity pid d gain
  ///    <i_max>            velocity pid max integral correction
  ///    <i_min>            velocity pid min integral correction
  ///    <cmd_max>          velocity pid max command torque
  ///    <cmd_min>          velocity pid min command torque
  ///    <jointName>        motor joint, torque applied here
  ///    <turningDirection> rotor turning direction, 'cw' or 'ccw'
  ///    frequencyCutoff    filter incoming joint state
  ///    samplingRate       sampling rate for filtering incoming joint state
  ///    <rotorVelocitySlowdownSim> for rotor aliasing problem, experimental
  /// <imuName>     scoped name for the imu sensor
  /// <connectionTimeoutMaxCount> timeout before giving up on
  ///                             controller synchronization
  class GAZEBO_VISIBLE ArduPilotPlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: ArduPilotPlugin();

    /// \brief Destructor.
    public: ~ArduPilotPlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Update the control surfaces controllers.
    /// \param[in] _info Update information provided by the server.
    private: void OnUpdate();

    /// \brief Update PID Joint controllers.
    /// \param[in] _dt time step size since last update.
    private: void ApplyMotorForces(const double _dt);

    /// \brief Reset PID Joint controllers.
    private: void ResetPIDs();

    /// \brief Receive motor commands from ArduPilot
    private: void ReceiveMotorCommand();

    /// \brief Send state to ArduPilot
    private: void SendState() const;

    /// \brief Init ardupilot socket
    private: bool InitArduPilotSockets(sdf::ElementPtr _sdf) const;

    /// \brief Private data pointer.
    private: std::unique_ptr<ArduPilotPluginPrivate> dataPtr;

    /// \brief transform from model orientation to x-forward and z-up
    private: ignition::math::Pose3d modelXYZToAirplaneXForwardZDown;

    /// \brief transform from world frame to NED frame
    private: ignition::math::Pose3d gazeboXYZToNED;

    /// \brief This node is for ros communications
    protected: std::unique_ptr<ros::NodeHandle> rosNode;

    private: int rpms[255]; // Same as MAX_MOTORS/255. Values can range from -838 to +838. See iris_with_ardupilot/model.sdf file
    protected: ros::Subscriber r0_sub; //To hold rpm value at index 0 corresponding to channel 0
    protected: ros::Subscriber r1_sub;
    protected: ros::Subscriber r2_sub;
    protected: ros::Subscriber r3_sub;

    public: void r0_channel_cb(const std_msgs::Int32::ConstPtr& msg);
    public: void r1_channel_cb(const std_msgs::Int32::ConstPtr& msg);
    public: void r2_channel_cb(const std_msgs::Int32::ConstPtr& msg);
    public: void r3_channel_cb(const std_msgs::Int32::ConstPtr& msg);
    public: static const int MIN_RPM = -838;
    public: static const int MAX_RPM = 838;
    public: static const int NO_RPM = 1000; // for any value > 838 and < -838, we will use stil provided cmd
  };
}
#endif
