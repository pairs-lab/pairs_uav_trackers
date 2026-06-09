/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>

#include <pairs_uav_managers/tracker.h>

#include <pairs_lib/profiler.h>
#include <pairs_lib/mutex.h>
#include <pairs_lib/attitude_converter.h>
#include <pairs_lib/geometry/cyclic.h>
#include <pairs_lib/geometry/misc.h>

//}

namespace pairs_uav_trackers
{

namespace midair_activation_tracker
{

/* //{ class MidairActivationTracker */

class MidairActivationTracker : public pairs_uav_managers::Tracker {
public:
  bool initialize(const ros::NodeHandle &nh, std::shared_ptr<pairs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                  std::shared_ptr<pairs_uav_managers::control_manager::PrivateHandlers_t> private_handlers);

  std::tuple<bool, std::string> activate(const std::optional<pairs_msgs::TrackerCommand> &last_tracker_cmd);
  void                          deactivate(void);
  bool                          resetStatic(void);

  std::optional<pairs_msgs::TrackerCommand>   update(const pairs_msgs::UavState &uav_state, const pairs_uav_managers::Controller::ControlOutput &last_control_output);
  const pairs_msgs::TrackerStatus             getStatus();
  const std_srvs::SetBoolResponse::ConstPtr enableCallbacks(const std_srvs::SetBoolRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr switchOdometrySource(const pairs_msgs::UavState &new_uav_state);

  const pairs_msgs::ReferenceSrvResponse::ConstPtr           setReference(const pairs_msgs::ReferenceSrvRequest::ConstPtr &cmd);
  const pairs_msgs::VelocityReferenceSrvResponse::ConstPtr   setVelocityReference(const pairs_msgs::VelocityReferenceSrvRequest::ConstPtr &cmd);
  const pairs_msgs::TrajectoryReferenceSrvResponse::ConstPtr setTrajectoryReference(const pairs_msgs::TrajectoryReferenceSrvRequest::ConstPtr &cmd);

  const pairs_msgs::DynamicsConstraintsSrvResponse::ConstPtr setConstraints(const pairs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &cmd);

  const std_srvs::TriggerResponse::ConstPtr hover(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr startTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr stopTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr resumeTrajectoryTracking(const std_srvs::TriggerRequest::ConstPtr &cmd);
  const std_srvs::TriggerResponse::ConstPtr gotoTrajectoryStart(const std_srvs::TriggerRequest::ConstPtr &cmd);

private:
  ros::NodeHandle nh_;

  bool callbacks_enabled_ = true;

  std::string _uav_name_;

  std::shared_ptr<pairs_uav_managers::control_manager::CommonHandlers_t>  common_handlers_;
  std::shared_ptr<pairs_uav_managers::control_manager::PrivateHandlers_t> private_handlers_;

  // | ---------------- the tracker's inner state --------------- |

  bool is_initialized_ = false;
  bool is_active_      = false;

  // | ------------------------ profiler ------------------------ |

  pairs_lib::Profiler profiler_;
  bool              _profiler_enabled_ = false;
};

//}

// | -------------- tracker's interface routines -------------- |

/* //{ initialize() */

bool MidairActivationTracker::initialize(const ros::NodeHandle &nh, std::shared_ptr<pairs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                                         std::shared_ptr<pairs_uav_managers::control_manager::PrivateHandlers_t> private_handlers) {

  this->common_handlers_  = common_handlers;
  this->private_handlers_ = private_handlers;

  _uav_name_ = common_handlers->uav_name;

  nh_ = nh;

  ros::Time::waitForValid();

  // --------------------------------------------------------------
  // |                     loading parameters                     |
  // --------------------------------------------------------------

  // | ---------- loading params using the parent's nh ---------- |

  pairs_lib::ParamLoader param_loader_parent(common_handlers->parent_nh, "ControlManager");

  param_loader_parent.loadParam("enable_profiler", _profiler_enabled_);

  if (!param_loader_parent.loadedSuccessfully()) {
    ROS_ERROR("[MidairActivationTracker]: Could not load all parameters!");
    return false;
  }

  // | ---------------- load plugin's parameters ---------------- |

  private_handlers->param_loader->addYamlFile(ros::package::getPath("pairs_uav_trackers") + "/config/private/midair_activation_tracker.yaml");
  private_handlers->param_loader->addYamlFile(ros::package::getPath("pairs_uav_trackers") + "/config/public/midair_activation_tracker.yaml");

  const std::string yaml_prefix = "pairs_uav_trackers/midair_activation_tracker/";

  if (!private_handlers->param_loader->loadedSuccessfully()) {
    ROS_ERROR("[MidairActivationTracker]: could not load all parameters!");
    return false;
  }

  // | ------------------------ profiler ------------------------ |

  profiler_ = pairs_lib::Profiler(common_handlers->parent_nh, "MidairActivationTracker", _profiler_enabled_);

  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;

  ROS_INFO("[MidairActivationTracker]: initialized");

  return true;
}

//}

/* //{ activate() */

std::tuple<bool, std::string> MidairActivationTracker::activate([[maybe_unused]] const std::optional<pairs_msgs::TrackerCommand> &last_tracker_cmd) {

  std::stringstream ss;

  is_active_ = true;

  ss << "activated";
  ROS_INFO_STREAM("[MidairActivationTracker]: " << ss.str());

  return std::tuple(true, ss.str());
}

//}

/* //{ deactivate() */

void MidairActivationTracker::deactivate(void) {

  is_active_ = false;

  ROS_INFO("[MidairActivationTracker]: deactivated");
}

//}

/* //{ resetStatic() */

bool MidairActivationTracker::resetStatic(void) {

  return false;
}

//}

/* //{ update() */

std::optional<pairs_msgs::TrackerCommand> MidairActivationTracker::update(
    const pairs_msgs::UavState &uav_state, [[maybe_unused]] const pairs_uav_managers::Controller::ControlOutput &last_control_output) {

  // up to this part the update() method is evaluated even when the tracker is not active
  if (!is_active_) {
    return {};
  }

  pairs_lib::Routine    profiler_routine = profiler_.createRoutine("update");
  pairs_lib::ScopeTimer timer =
      pairs_lib::ScopeTimer("MidairActivationTracker::update", common_handlers_->scope_timer.logger, common_handlers_->scope_timer.enabled);

  pairs_msgs::TrackerCommand tracker_cmd;

  tracker_cmd.header.frame_id = uav_state.header.frame_id;
  tracker_cmd.header.stamp    = ros::Time::now();

  tracker_cmd.position.x = uav_state.pose.position.x;
  tracker_cmd.position.y = uav_state.pose.position.y;
  tracker_cmd.position.z = uav_state.pose.position.z;

  tracker_cmd.velocity.x = uav_state.velocity.linear.x;
  tracker_cmd.velocity.y = uav_state.velocity.linear.y;
  tracker_cmd.velocity.z = uav_state.velocity.linear.z;

  try {
    tracker_cmd.heading = pairs_lib::AttitudeConverter(uav_state.pose.orientation).getHeading();
  }
  catch (...) {
    tracker_cmd.heading = pairs_lib::AttitudeConverter(uav_state.pose.orientation).getYaw();
    ROS_WARN_THROTTLE(1.0, "[MidairActivationTracker]: could not get heading");
  }

  tracker_cmd.use_position_vertical   = true;
  tracker_cmd.use_position_horizontal = true;

  tracker_cmd.use_velocity_vertical   = true;
  tracker_cmd.use_velocity_horizontal = true;

  tracker_cmd.use_heading = true;

  ROS_WARN_THROTTLE(0.1, "[MidairActivationTracker]: outputting cmd");

  return {tracker_cmd};
}

//}

/* //{ getStatus() */

const pairs_msgs::TrackerStatus MidairActivationTracker::getStatus() {

  pairs_msgs::TrackerStatus tracker_status;

  tracker_status.active            = is_active_;
  tracker_status.callbacks_enabled = callbacks_enabled_;

  return tracker_status;
}

//}

/* //{ enableCallbacks() */

const std_srvs::SetBoolResponse::ConstPtr MidairActivationTracker::enableCallbacks([[maybe_unused]] const std_srvs::SetBoolRequest::ConstPtr &cmd) {

  std_srvs::SetBoolResponse res;

  res.message = "callbacks are always disabled";
  res.success = true;

  return std_srvs::SetBoolResponse::ConstPtr(new std_srvs::SetBoolResponse(res));
}

//}

/* switchOdometrySource() //{ */

const std_srvs::TriggerResponse::ConstPtr MidairActivationTracker::switchOdometrySource([[maybe_unused]] const pairs_msgs::UavState &new_uav_state) {

  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ hover() */

const std_srvs::TriggerResponse::ConstPtr MidairActivationTracker::hover([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {

  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ startTrajectoryTracking() */

const std_srvs::TriggerResponse::ConstPtr MidairActivationTracker::startTrajectoryTracking([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ stopTrajectoryTracking() */

const std_srvs::TriggerResponse::ConstPtr MidairActivationTracker::stopTrajectoryTracking([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ resumeTrajectoryTracking() */

const std_srvs::TriggerResponse::ConstPtr MidairActivationTracker::resumeTrajectoryTracking([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ gotoTrajectoryStart() */

const std_srvs::TriggerResponse::ConstPtr MidairActivationTracker::gotoTrajectoryStart([[maybe_unused]] const std_srvs::TriggerRequest::ConstPtr &cmd) {
  return std_srvs::TriggerResponse::Ptr();
}

//}

/* //{ setConstraints() */

const pairs_msgs::DynamicsConstraintsSrvResponse::ConstPtr MidairActivationTracker::setConstraints([
    [maybe_unused]] const pairs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &cmd) {

  pairs_msgs::DynamicsConstraintsSrvResponse res;

  res.success = true;
  res.message = "constraints updated";

  return pairs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new pairs_msgs::DynamicsConstraintsSrvResponse(res));
}

//}

/* //{ setReference() */

const pairs_msgs::ReferenceSrvResponse::ConstPtr MidairActivationTracker::setReference([[maybe_unused]] const pairs_msgs::ReferenceSrvRequest::ConstPtr &cmd) {

  return pairs_msgs::ReferenceSrvResponse::Ptr();
}

//}

/* //{ setVelocityReference() */

const pairs_msgs::VelocityReferenceSrvResponse::ConstPtr MidairActivationTracker::setVelocityReference([
    [maybe_unused]] const pairs_msgs::VelocityReferenceSrvRequest::ConstPtr &cmd) {
  return pairs_msgs::VelocityReferenceSrvResponse::Ptr();
}

//}

/* //{ setTrajectoryReference() */

const pairs_msgs::TrajectoryReferenceSrvResponse::ConstPtr MidairActivationTracker::setTrajectoryReference([
    [maybe_unused]] const pairs_msgs::TrajectoryReferenceSrvRequest::ConstPtr &cmd) {
  return pairs_msgs::TrajectoryReferenceSrvResponse::Ptr();
}

//}

}  // namespace midair_activation_tracker

}  // namespace pairs_uav_trackers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pairs_uav_trackers::midair_activation_tracker::MidairActivationTracker, pairs_uav_managers::Tracker)
