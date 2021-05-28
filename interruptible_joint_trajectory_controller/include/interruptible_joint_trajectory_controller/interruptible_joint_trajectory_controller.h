///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2020, Tormach, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics S.L. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author Robert W. Ellenberg

#pragma once

// C++ standard
#include <cassert>
#include <stdexcept>
#include <string>
#include <memory>

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/dynamic_bitset.hpp>

// ROS
#include <ros/node_handle.h>

// URDF
#include <urdf/model.h>

// ROS messages
#include <control_msgs/JointTrajectoryControllerState.h>
#include <control_msgs/QueryTrajectoryState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <stop_event_msgs/SetNextProbeMove.h>
#include <stop_event_msgs/GetStopEventResult.h>
#include <stop_event_msgs/GetJointTrajectoryErrorContext.h>

// ros_controls
#include <realtime_tools/realtime_server_goal_handle.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/internal/demangle_symbol.h>

#include <machinekit_interfaces/realtime_event_interface.h>
#include <machinekit_interfaces/probe_interface.h>
#include <machinekit_interfaces/joint_event_interface.h>
#include <machinekit_interfaces/generic_interface.h>

// Bring in enums
using machinekit_interfaces::ProbeState;
using machinekit_interfaces::ProbeTransitions;
using stop_event_msgs::GetJointTrajectoryErrorContextResponse;
using stop_event_msgs::SetNextProbeMoveRequest;
using stop_event_msgs::SetNextProbeMoveResponse;

// Project
#include <joint_trajectory_controller/joint_trajectory_controller.h>

namespace interruptible_joint_trajectory_controller
{
static const std::string PROBE_SERVICE_NAME{ "probe" };
static const std::string PROBE_RESULT_SERVICE_NAME{ "probe_result" };
static const std::string ERROR_CONTEXT_SERVICE_NAME{ "error_context" };

struct ProbeSettings
{
  int probe_request_capture_type;
};

/**
 * Controller for executing joint-space trajectories on a group of joints, that
 * can respond to stop events triggered from hardware in the realtime loop.
 *
 * See JointTrajectoryController documentation for details on how the trajectory
 * execution works.
 *
 */
template <class SegmentImpl, class HardwareInterface>
class InterruptibleJointTrajectoryController
  : public joint_trajectory_controller::JointTrajectoryController<
        SegmentImpl, HardwareInterface, ProbeSettings>
{
protected:
  using JointTrajectoryControllerType =
      typename joint_trajectory_controller::JointTrajectoryController<
          SegmentImpl, HardwareInterface, ProbeSettings>;
  using typename JointTrajectoryControllerType::ExtendedTrajectoryPtr;
  using typename JointTrajectoryControllerType::JointTrajectoryConstPtr;
  using typename JointTrajectoryControllerType::RealtimeGoalHandlePtr;
  using typename JointTrajectoryControllerType::Trajectory;

public:
  InterruptibleJointTrajectoryController();

  /** \name Non Real-Time Safe Functions
   *\{*/

  // KLUDGE have to override this method in Controller to be able to initialize
  // multiple hardware interface types
  bool initRequest(hardware_interface::RobotHW* robot_hw,
                   ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh,
                   controller_interface::ControllerBase::ClaimedResources&
                       claimed_resources) override;
  /*\}*/

protected:
  void update(const ros::Time& time, const ros::Duration& period);
  virtual void onTrajectoryError(int error_code);
  /*\}*/

  // Command handling, not real-time safe
  virtual bool updateTrajectoryCommand(const JointTrajectoryConstPtr& msg,
                                       RealtimeGoalHandlePtr gh,
                                       std::string* error_string = nullptr);

  // Service calls, not real-time safe

  /**
   * @brief Service callback to force the controller into the hold position.
   *
   * @param request Dummy for triggering the service
   * @param response True on success.
   *
   * @return False if something went wrong. True otherwise.
   */
  bool handleProbeRequest(stop_event_msgs::SetNextProbeMoveRequest& request,
                          stop_event_msgs::SetNextProbeMoveResponse& response);
  bool handleStopEventResultRequest(
      stop_event_msgs::GetStopEventResultRequest& request,
      stop_event_msgs::GetStopEventResultResponse& response);
  bool handleJointTrajectoryErrorContextRequest(
      stop_event_msgs::GetJointTrajectoryErrorContextRequest& request,
      stop_event_msgs::GetJointTrajectoryErrorContextResponse& response);

  // Real-Time ONLY Functions (must be called within the context of an update)
  virtual void checkReachedTrajectoryGoal(ros::Time const& uptime);
  virtual void checkReachedTrajectoryGoalProbe(int capture_type,
                                               ros::Time const& uptime);

  // Services for controller trajectory behavior
  ros::ServiceServer probe_service_;  //!< Declare success when probe trip
                                      //!< occurs on the next send trajectory
  ros::ServiceServer probe_result_service_;  //!< Request the result of a probe
  ros::ServiceServer error_detail_service_;  //!< Used to query verbose error
                                             //!< data after a motion error has
                                             //!< occurred

  std::vector<machinekit_interfaces::JointEventDataHandle> probe_joint_results_;
  machinekit_interfaces::ProbeHandle probe_handle_;
  machinekit_interfaces::RealtimeEventHandle stop_event_;
  machinekit_interfaces::GenericInt32Handle error_code_;
  int jog_err_threshold_;
  int jog_err_count_;  // Used to count "soft" errors that should only be
                      // reported if many happen at once
};

}  // namespace interruptible_joint_trajectory_controller

#pragma once

namespace interruptible_joint_trajectory_controller
{
template <class SegmentImpl, class HardwareInterface>
InterruptibleJointTrajectoryController<
    SegmentImpl, HardwareInterface>::InterruptibleJointTrajectoryController()
  : JointTrajectoryControllerType()
{
}

template <class SegmentImpl, class HardwareInterface>
bool InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::
    initRequest(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                ros::NodeHandle& controller_nh,
                controller_interface::ControllerBase::ClaimedResources&
                    claimed_resources)
{
  // Initialize auxiliary hardware interfaces required by this controller (probe
  // / stop event interfaces)
  ROS_INFO_STREAM("initRequest for InterruptibleJointTrajectoryController");

  // SO ugly, need to redirect to the base class method, but this is fragile if
  // JointTrajectoryController ever decides to add one... Complete the
  // underlying initialization for the controller (JointTrajectoryController
  // base-level init)
  bool base_init = JointTrajectoryControllerType::initRequest(
      robot_hw, root_nh, controller_nh, claimed_resources);
  if (!base_init)
  {
    return false;
  }

  // This is following the same basic pattern as Controller<>'s initRequest, but
  // we can't just use the basic init function because it's tied to the Joint
  // interface. initRequest itself has to be overridden to handle these
  // auxiliary interfaces.
  {
    ROS_INFO_STREAM_NAMED(this->name_, "Claiming probe resources");
    machinekit_interfaces::ProbeInterface* probe_intf =
        robot_hw->get<machinekit_interfaces::ProbeInterface>();
    auto hw_if_typename = hardware_interface::internal::demangledTypeName<
        machinekit_interfaces::ProbeInterface>();
    if (!probe_intf)
    {
      ROS_ERROR("This controller requires a hardware interface of type '%s'."
                " Make sure this is registered in the "
                "hardware_interface::RobotHW class.",
                hw_if_typename.c_str());
      return false;
    }
    // return which resources are claimed by this controller
    probe_intf->clearClaims();

    try
    {
      probe_handle = probe_intf->getHandle("probe");
    }
    catch (...)
    {
      ROS_ERROR_STREAM_NAMED(this->name_, "Could not find probe in '"
                                              << hw_if_typename << "'.");
      return false;
    }
    hardware_interface::InterfaceResources iface_res(hw_if_typename,
                                                     probe_intf->getClaims());
    claimed_resources.push_back(iface_res);
    probe_intf->clearClaims();
  }
  {
    ROS_INFO_STREAM_NAMED(this->name_, "Claiming joint event data resources");
    machinekit_interfaces::JointEventDataInterface* probe_data_intf =
        robot_hw->get<machinekit_interfaces::JointEventDataInterface>();
    auto hw_if_typename = hardware_interface::internal::demangledTypeName<
        machinekit_interfaces::JointEventDataInterface>();
    if (!probe_data_intf)
    {
      ROS_ERROR("This controller requires a hardware interface of type '%s'."
                " Make sure this is registered in the "
                "hardware_interface::RobotHW class.",
                hw_if_typename.c_str());
      return false;
    }
    // return which resources are claimed by this controller
    probe_data_intf->clearClaims();

    // NOTE: this depends on successful initialization of the controller so we
    // can reuse the found joint names to create probe results per joint
    const unsigned int n_joints = this->joint_names_.size();
    probe_joint_results_.resize(n_joints);
    for (unsigned int i = 0; i < n_joints; ++i)
    {
      std::string const& jname = this->joint_names_[i] + "_probe";
      // Uses a parallel set of handles defined by the joint names (to avoid
      // conflicts with the standard joint handles)
      try
      {
        probe_joint_results_[i] = probe_data_intf->getHandle(jname);
      }
      catch (...)
      {
        ROS_ERROR_STREAM_NAMED(this->name_,
                               "Could not find joint '"
                                   << jname << "' in '"
                                   << this->getHardwareInterfaceType() << "'.");
        return false;
      }
    }

    hardware_interface::InterfaceResources iface_res(
        hw_if_typename, probe_data_intf->getClaims());
    claimed_resources.push_back(iface_res);
    probe_data_intf->clearClaims();
  }
  {
    ROS_INFO_STREAM_NAMED(this->name_, "Claiming HAL pin resources");
    machinekit_interfaces::GenericInt32Interface* generic_int32_intf =
        robot_hw->get<machinekit_interfaces::GenericInt32Interface>();
    auto hw_if_typename = hardware_interface::internal::demangledTypeName<
        machinekit_interfaces::GenericInt32Interface>();
    if (!generic_int32_intf)
    {
      ROS_ERROR("This controller requires a hardware interface of type '%s'."
                " Make sure this is registered in the "
                "hardware_interface::RobotHW class.",
                hw_if_typename.c_str());
      return false;
    }
    generic_int32_intf->clearClaims();

    try
    {
      error_code_ = generic_int32_intf->getHandle("controller_status");
    }
    catch (...)
    {
      ROS_ERROR_STREAM_NAMED(this->name_, "Could not find controller_status in "
                                          "'" << hw_if_typename
                                              << "'.");
      return false;
    }
    hardware_interface::InterfaceResources iface_res(
        hw_if_typename, generic_int32_intf->getClaims());
    claimed_resources.push_back(iface_res);
    generic_int32_intf->clearClaims();
  }

  ROS_INFO_STREAM_NAMED(this->name_, "Claimed " << claimed_resources.size()
                                                << " interfaces");
  for (auto const& s : claimed_resources)
  {
    for (auto const& c : s.resources)
    {
      ROS_INFO_STREAM_NAMED(this->name_, "interface " << s.hardware_interface
                                                      << " claims " << c);
    }
  }
  ROS_INFO_STREAM_NAMED(this->name_, "Starting probe services");
  // Set up services to control probe behavior
  probe_service_ = controller_nh.advertiseService(
      PROBE_SERVICE_NAME,
      &InterruptibleJointTrajectoryController::handleProbeRequest, this);
  probe_result_service_ = controller_nh.advertiseService(
      PROBE_RESULT_SERVICE_NAME,
      &InterruptibleJointTrajectoryController::handleStopEventResultRequest,
      this);
  error_detail_service_ = controller_nh.advertiseService(
      ERROR_CONTEXT_SERVICE_NAME,
      &InterruptibleJointTrajectoryController::
          handleJointTrajectoryErrorContextRequest,
      this);

  // KLUDGE soft error threshold so jogging with probe active doesn't spam the
  // console
  jog_err_threshold_ = 32;
  this->controller_nh_.getParam("jog_error_threshold", jog_err_threshold_);
  jog_err_count_ = -1;
  // success
  this->state_ = controller_interface::Controller<
      HardwareInterface>::ControllerState::INITIALIZED;
  return true;
}

// WARNING do not early abort from this function, it must clean up probe capture
// mode
template <class SegmentImpl, class HardwareInterface>
void InterruptibleJointTrajectoryController<
    SegmentImpl, HardwareInterface>::update(const ros::Time& time,
                                            const ros::Duration& period)
{
  // Acquire the trajectory pointer from the RT box ONCE (here), and pass to
  // various methods as needed
  ExtendedTrajectoryPtr curr_traj_ptr;
  joint_trajectory_controller::TimeData time_data;
  JointTrajectoryControllerType::prepare_for_update(time, period, curr_traj_ptr,
                                                    time_data);
  typename JointTrajectoryControllerType::RealtimeGoalHandlePtr
      current_active_goal(this->rt_active_goal_);

  // First, ensure that a new trajectory's probe settings are applied
  ProbeSettings& settings = curr_traj_ptr->motion_settings;
  if (!curr_traj_ptr->started)
  {
    // Disregard any previously captured probe transitions (up to and including
    // the current timestep) This means that probe transitions are not detected
    // on the very first timestep
    error_code_.set(0);
    probe_handle_.startNewProbeCapture(settings.probe_request_capture_type);
    probe_handle_.acquireProbeTransition();
    // Don't re-apply the settings now that the new trajectory is active
    curr_traj_ptr->started = true;
    // Since we're starting a new trajectory, the previous "stop" event is over
    this->rt_stop_event_triggered_ = false;
  }

  auto probe_transition = probe_handle_.acquireProbeTransition();
  auto const probe_capture_type = probe_handle_.getProbeCapture();

  switch (probe_transition)
  {
    case ProbeTransitions::RISING:
      switch (probe_capture_type)
      {
        case stop_event_msgs::SetNextProbeMoveRequest::
            PROBE_REQUIRE_RISING_EDGE:
        case stop_event_msgs::SetNextProbeMoveRequest::
            PROBE_OPTIONAL_RISING_EDGE:
          this->completeActiveGoal(current_active_goal, time_data.uptime);
          break;
        case stop_event_msgs::SetNextProbeMoveRequest::PROBE_IGNORE_INPUT:
          break;
        default:
          // "RETRACT" is meant to retract off of a surface and continue moving,
          // but should stop if it hits something else
          this->abortActiveGoalWithError(
              current_active_goal, time_data.uptime,
              GetJointTrajectoryErrorContextResponse::
                  PROBE_UNEXPECTED_RISING_EDGE);
          break;
      }
      break;
    case ProbeTransitions::FALLING:
      switch (probe_capture_type)
      {
        case stop_event_msgs::SetNextProbeMoveRequest::
            PROBE_REQUIRE_FALLING_EDGE:
        case stop_event_msgs::SetNextProbeMoveRequest::
            PROBE_OPTIONAL_FALLING_EDGE:
          this->completeActiveGoal(current_active_goal, time_data.uptime);
          break;
        case stop_event_msgs::SetNextProbeMoveRequest::PROBE_RETRACT:
        case stop_event_msgs::SetNextProbeMoveRequest::PROBE_IGNORE_INPUT:
          break;
        default:
          this->abortActiveGoalWithError(
              current_active_goal, time_data.uptime,
              GetJointTrajectoryErrorContextResponse::
                  PROBE_UNEXPECTED_FALLING_EDGE);
          break;
      }
      break;
    case ProbeTransitions::NONE:
      // Probe should not be high for non-probe motions, or if we're looking for
      // a rising edge (and haven't found it yet)
      if (probe_handle_.getProbeState())
      {
        switch (probe_capture_type)
        {
          case stop_event_msgs::SetNextProbeMoveRequest::
              PROBE_REQUIRE_FALLING_EDGE:
          case stop_event_msgs::SetNextProbeMoveRequest::
              PROBE_OPTIONAL_FALLING_EDGE:
          case stop_event_msgs::SetNextProbeMoveRequest::PROBE_RETRACT:
          case stop_event_msgs::SetNextProbeMoveRequest::PROBE_IGNORE_INPUT:
            // We're ok, expect the probe to be active for these motion types
            break;
          case stop_event_msgs::SetNextProbeMoveRequest::
              PROBE_REQUIRE_RISING_EDGE:
          case stop_event_msgs::SetNextProbeMoveRequest::
              PROBE_OPTIONAL_RISING_EDGE:
          default:  // Deliberate fallthrough
            this->abortActiveGoalWithError(
                current_active_goal, time_data.uptime,
                GetJointTrajectoryErrorContextResponse::PROBE_CONTACT_AT_START);
            break;
        }
      }
      break;
    default:
      this->abortActiveGoalWithError(
          current_active_goal, time_data.uptime,
          GetJointTrajectoryErrorContextResponse::PROBE_INVALID_STATE);
      break;
  }

  // Finished reacting to probe events, begin to actually handle a trajectory
  // update
  JointTrajectoryControllerType::update_joint_trajectory(
      curr_traj_ptr->trajectory, time_data, period);
}

template <class SegmentImpl, class HardwareInterface>
void InterruptibleJointTrajectoryController<
    SegmentImpl, HardwareInterface>::onTrajectoryError(int error_code)
{
  // Publish the error code
  error_code_.set(error_code);
}

/**
 * Check if all joints have reached their goal state, and mark the goal handle
 * as succeeded if so. Derived classes can specialize this if they need finer
 * control over goal success (e.g. if there are additional criteria like for
 * probing).
 */
template <class SegmentImpl, class HardwareInterface>
void InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::
    checkReachedTrajectoryGoal(const ros::Time& uptime)
{
  int capture_type = probe_handle_.getProbeCapture();
  if (capture_type)
  {
    checkReachedTrajectoryGoalProbe(capture_type, uptime);
  }
  else
  {
    // Normal moves get forwarded to the stock goal check
    JointTrajectoryControllerType::checkReachedTrajectoryGoal(uptime);
  }
}

template <class SegmentImpl, class HardwareInterface>
void InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::
    checkReachedTrajectoryGoalProbe(int capture_type, const ros::Time& uptime)
{
  // Check if we have reached the end of a
  RealtimeGoalHandlePtr current_active_goal(this->rt_active_goal_);
  if (current_active_goal &&
      this->successful_joint_traj_.count() == this->getNumberOfJoints())
  {
    if (capture_type == stop_event_msgs::SetNextProbeMoveRequest::
                            PROBE_REQUIRE_RISING_EDGE ||
        capture_type == stop_event_msgs::SetNextProbeMoveRequest::
                            PROBE_REQUIRE_FALLING_EDGE)
    {
      this->abortActiveGoalWithError(
          current_active_goal, uptime,
          GetJointTrajectoryErrorContextResponse::PROBE_REACHED_MOTION_END);
    }
    else
    {
      this->completeActiveGoal(current_active_goal, uptime);
    }
    this->rt_active_goal_.reset();
    // TODO pass uptime in here to plane a stop trajectory in case the goal has
    // nonzero velocity?
    this->successful_joint_traj_.reset();
  }
}

template <class SegmentImpl, class HardwareInterface>
bool InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::
    handleProbeRequest(stop_event_msgs::SetNextProbeMoveRequest& request,
                       stop_event_msgs::SetNextProbeMoveResponse& response)
{
  ROS_INFO_STREAM("Probe capture requested, mode " << request.mode);
  this->queued_motion_settings_.probe_request_capture_type =
      (machinekit_interfaces::ProbeCaptureType)request.mode;
  return true;
}

template <class SegmentImpl, class HardwareInterface>
bool InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::
    handleStopEventResultRequest(
        stop_event_msgs::GetStopEventResultRequest& request,
        stop_event_msgs::GetStopEventResultResponse& response)
{
  response.stop_event = (long)probe_handle_.getProbeResultType();
  response.event_time = probe_handle_.getProbeCaptureTime();
  ROS_INFO_STREAM("Handling request for probe results for capture type "
                  << response.stop_event << " at time " << response.event_time);
  if (response.stop_event)
  {
    std::vector<double> probe_positions(this->getNumberOfJoints(), 0.0);
    for (unsigned int joint_index = 0; joint_index < this->getNumberOfJoints();
         ++joint_index)
    {
      probe_positions[joint_index] = 0;
      // Hope that the lock thrashing here doesn't affect RT...
      // response.result.joint_names[joint_index] =
      // this->joint_names_[joint_index];
      probe_positions[joint_index] =
          probe_joint_results_[joint_index].getPosition();
      // probe_state.velocities[joint_index] =
      // probe_joint_results_[joint_index].getVelocity();
      // probe_state.positions[joint_index] = 0;
      // probe_state.velocities[joint_index] = 0;
    }
    response.event_position = probe_positions;
  }
  return true;
}

template <class SegmentImpl, class HardwareInterface>
bool InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::
    handleJointTrajectoryErrorContextRequest(
        stop_event_msgs::GetJointTrajectoryErrorContextRequest& request,
        stop_event_msgs::GetJointTrajectoryErrorContextResponse& response)
{
  response.error_code = error_code_.get();
  ROS_INFO_STREAM("Handling request for error details (current error code is "
                  << response.error_code << ")");
  return true;
}

template <class SegmentImpl, class HardwareInterface>
bool InterruptibleJointTrajectoryController<SegmentImpl, HardwareInterface>::
    updateTrajectoryCommand(const JointTrajectoryConstPtr& msg,
                            RealtimeGoalHandlePtr gh, std::string* error_string)
{
  // Guard against starting a motion if the probe is currently active
  // KLUDGE this should really be done in realtime but this will have to do
  if (probe_handle_.getProbeState() > 0)
  {
    auto requested_capture =
        this->queued_motion_settings_.probe_request_capture_type;
    switch (requested_capture)
    {
      case SetNextProbeMoveRequest::PROBE_NONE:
      case SetNextProbeMoveRequest::PROBE_OPTIONAL_RISING_EDGE:
      case SetNextProbeMoveRequest::PROBE_REQUIRE_RISING_EDGE: {
        if (error_string)
        {
          const std::string err_msg("Can't start a motion or probe move with "
                                    "probe active in probing mode " +
                                    std::to_string(requested_capture));
          *error_string = err_msg;
        }
        else
        {
          // Caller didn't provide a feedback mechanism, so complain directly to
          // the console
          (++jog_err_count_) %= jog_err_threshold_;
          if (!jog_err_count_)
          {
            // KLUDGE avoid console spam by only publishing once we reach the
            // threshold (since these errors usually come from continuous
            // jogging, which tends to be a stream of updates rather than
            // one-off commands)
            ROS_ERROR_STREAM("Can't start a jog motion with probe active. "
                             "Verify probe connection / polarity, click 'Jog "
                             "Ignore Probe' to re-enable jogging, then "
                             "carefully jog the probe to a safe position.");
          }
        }
        this->claimQueuedSettings();
        error_code_.set(
            GetJointTrajectoryErrorContextResponse::PROBE_CONTACT_AT_START);
        return false;
      }
      default:
        break;
    }
  }
  // NOTE: this clears the queued settings once they're accepted
  bool res = JointTrajectoryControllerType::updateTrajectoryCommand(
      msg, gh, error_string);
  if (res)
  {
    // Sends a message immediately when the next jog error occurs, but then
    // silences any additional errors until the threshold is reached.
    jog_err_count_ = -1;
  }
  return res;
}

}  // namespace interruptible_joint_trajectory_controller
