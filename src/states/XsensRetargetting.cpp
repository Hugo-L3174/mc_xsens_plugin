#include "XsensRetargetting.h"

#include <mc_control/fsm/Controller.h>
#include <mc_rtc/logging.h>
#include <mc_tasks/TransformTask.h>
#include <mc_tasks/lipm_stabilizer/Contact.h>

#include <SpaceVecAlg/SpaceVecAlg>
#include <iostream>
#include <state-observation/tools/rigid-body-kinematics.hpp>

#include "../XsensPlugin.h"
#include "mc_xsens_plugin/XsensBodyConfiguration.h"

namespace mc_xsens_plugin
{

void XsensRetargetting::start(mc_control::fsm::Controller & ctl)
{
  auto & ds = ctl.datastore();
  if(!ds.has("XsensPlugin"))
  {
    mc_rtc::log::error_and_throw("[{}] This state requires the XsensPlugin to be running", name());
  }
  plugin_ = ds.get<XsensPlugin *>("XsensPlugin");
  config_("fixed_stiffness", fixedStiffness_);
  config_("fixed_weight", fixedWeight_);
  robot_ = config_("robot", ctl.robot().name());
  config_("offset", offset_);
  config_("fixBaseLink", fixBaseLink_);
  config_("unactiveJoints", unactiveJoints_);
  config_("activeBodies", activeBodies_);
  config_("autoTransition", autoTransition_);
  config_("groundingFrames", groundingFrames_);
  if(config_.has("log"))
  {
    if(!ds.has("Replay::SetStartTime"))
    {
      mc_rtc::log::error_and_throw("[{}] The datastore call Replay::SetStartTime does not exist in the Replay plugin",
                                   name());
    }
    if(!ds.has("Replay::SetEndTime"))
    {
      mc_rtc::log::error_and_throw("[{}] The datastore call Replay::SetEndTime does not exist in the Replay plugin",
                                   name());
    }
    if(!ds.has("Replay::SetLog"))
    {
      mc_rtc::log::error_and_throw("[{}] The datastore call Replay::SetLog does not exist in the Replay plugin",
                                   name());
    }
    const std::string & logName = config_("log");
    ds.call<void>("Replay::SetPause", config_("pause", false));
    ds.call<void>("Replay::SkipIter", static_cast<size_t>(config_("skip_iter", 1)));
    ds.call<void>("Replay::SetStartTime", static_cast<double>(config_("start_time", 0.0)));
    if(config_.has("end_time")) { ds.call<void>("Replay::SetEndTime", static_cast<double>(config_("end_time"))); }
    else { ds.call<void>("Replay::SetEndTime", 0.0); }
    ds.call<void>("Replay::SetLog", logName);
  }

  if(robot_.empty()) { mc_rtc::log::error_and_throw<std::runtime_error>("[{}] \"robot\" parameter required", name()); }
  else if(!ctl.hasRobot(robot_))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] No robot named \"{}\"", name(), robot_);
  }
  auto & robot = ctl.robot(robot_);
  const auto & rName = robot.name();
  if(!ctl.config()("Xsens").has(rName))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Robot {} not supported (missing Xsens->{} configuration)",
                                                     rName, name(), rName);
  }

  ctl.gui()->addElement(this,
                        {"Plugin", "Xsens", robot_, "Offset base_link"},
                        mc_rtc::gui::ArrayInput("Offset Translation", offset_.translation()),
                        mc_rtc::gui::RPYInput("Offset RPY", offset_.rotation()));

  // Merge plugin wide configuration with state configuration
  mc_rtc::Configuration fullConfig;
  if(ctl.config().has("Xsens")) { fullConfig.load(ctl.config()("Xsens")); }
  if(config_.has("Xsens")) { fullConfig.load(config_("Xsens")); }

  if(fullConfig.has(rName) && fullConfig(rName).has("bodies")) { bodyConfigurations_ = fullConfig(rName)("bodies"); }
  else
  {
    mc_rtc::log::error_and_throw("[{}] XsensPlugin or this state's configuration should contain an Xsens/{}/bodies "
                                 "entry for each controlled body",
                                 name(), robot.name());
  }
  bool addActive = activeBodies_.empty();

  for(auto & bodyConf : bodyConfigurations_)
  {
    const auto & bodyName = bodyConf.first;
    auto & bodyC = bodyConf.second;
    if(addActive) activeBodies_.push_back(bodyName);

    if(isActiveBody(bodyName))
    {
      ctl.gui()->addElement(
          this, {"Plugin", "Xsens", robot_, "Bodies", bodyName},
          mc_rtc::gui::ArrayInput(
              "Offset translation [m]", [&bodyC]() { return bodyC.offset.translation(); },
              [&bodyC](const Eigen::Vector3d & offset) { bodyC.offset.translation() = offset; }),
          mc_rtc::gui::ArrayInput(
              "Offset rotation [rad]", [&bodyC]() { return mc_rbdyn::rpyFromMat(bodyC.offset.rotation()); },
              [&bodyC](const Eigen::Vector3d & offset) { bodyC.offset.rotation() = mc_rbdyn::rpyToMat(offset); }));
    }
  }

  // ctl.gui()->addElement(
  //     this, {}, mc_rtc::gui::Label("Current motion:", [this]() -> const std::string & { return name(); }),
  //     mc_rtc::gui::Button(fmt::format("Stop in {}s", endInterpolationTime_), [this]() { finishRequested_ = true; }),
  //     mc_rtc::gui::Label("Finished?", finished_), mc_rtc::gui::Input("Transition to NEXT STATE", autoTransition_));

  config_("initialInterpolationTime", initialInterpolationTime_);
  config_("initialStiffnessPercent", initialStiffnessPercent_);
  config_("initialWeightPercent", initialWeightPercent_);
  startStiffnessInterpolator_.values({{0.0, initialStiffnessPercent_}, {initialInterpolationTime_, 1.}});
  startWeightInterpolator_.values({{0.0, initialWeightPercent_}, {initialInterpolationTime_, 1.}});
  config_("endInterpolationTime", endInterpolationTime_);
  config_("endStiffnessPercent", endStiffnessPercent_);
  config_("endWeightPercent", endWeightPercent_);
  endStiffnessInterpolator_.values({{0.0, 1.0}, {endInterpolationTime_, endStiffnessPercent_}});
  endWeightInterpolator_.values({{0.0, 1.0}, {endInterpolationTime_, endWeightPercent_}});

  if(fixBaseLink_)
  {
    /**
     * COMPUTE INITIAL POSE FOR THE BASE LINK
     * This pose is:
     * - horizontal in both pitch and roll, and has the same yaw as the frame in-between both feet center
     * - in translation:
     *   - has the same height as the default robot attitude
     *   - in x/y plane is the point in-between both feet center
     * This should ensure that the Xsens trajectory is always retargetted w.r.t a
     * meaningful base_link frame
     */
    // XXX hardcoded surfaces
    auto leftFootPosW = robot.frame("LeftFoot").position();
    auto rightFootPosW = robot.frame("RightFoot").position();
    double leftFootRatio = config_("leftFootRatio", 0.5);
    auto midFootPosW = sva::interpolate(leftFootPosW, rightFootPosW, leftFootRatio);
    Eigen::Matrix3d R_above_feet_yaw =
        stateObservation::kine::mergeRoll1Pitch1WithYaw2(Eigen::Matrix3d::Identity(), midFootPosW.rotation());
    Eigen::Vector3d t_above_feet = Eigen::Vector3d{midFootPosW.translation().x(), midFootPosW.translation().y(),
                                                   robot.module().default_attitude()[6]};
    initPosW_ = sva::PTransformd{R_above_feet_yaw, t_above_feet};
  }

  // Initialize tasks
  for(auto & bodyName : activeBodies_)
  {
    const auto & body = bodyConfigurations_[bodyName];
    const auto & segmentName = body.segmentName;

    if(robot.hasBody(bodyName))
    {
      auto task = std::make_shared<mc_tasks::TransformTask>(robot.frame(bodyName), body.stiffness, body.weight, false, false);
      task->name(fmt::format("Retargetting_{}_{}", robot.name(), bodyName));
      // task->name(fmt::format("{}", bodyName));
      task->reset();
      task->selectUnactiveJoints(ctl.solver(), unactiveJoints_);
      ctl.solver().addTask(task.get());

      if(debugmode_)
      {
        mc_rtc::log::info("= = = = ctl.solver().addTask = = = =\n bodyName = {}, segmentName = {}\n", bodyName,
                          segmentName);
      }

      tasks_[bodyName] = task;
    }
    else { mc_rtc::log::error("[{}] No body named {}", name(), bodyName); }
  }

  const auto & baseLinkName = robot.mb().body(0).name();
  auto fixedBodies = config_("fixedBodies", std::vector<std::string>{});
  if(fixBaseLink_ && std::find(fixedBodies.begin(), fixedBodies.end(), baseLinkName) == fixedBodies.end())
  {
    fixedBodies.push_back(baseLinkName);
  }
  for(const auto & fixedBody : fixedBodies)
  {
    mc_rtc::log::info("[{}] Body \"{}\" will be fixed", name(), fixedBody);
    fixedTasks_[fixedBody] =
        std::make_shared<mc_tasks::TransformTask>(ctl.robot().frame(fixedBody), fixedStiffness_, fixedWeight_);
    auto & task = fixedTasks_[fixedBody];
    task->reset();
    task->name(fmt::format("fixed_{}", fixedBody));
    task->selectUnactiveJoints(ctl.solver(), unactiveJoints_);
    ctl.solver().addTask(task.get());
    if(fixedBody == baseLinkName) { task->target(initPosW_); }
  }

  output("OK");
  run(ctl);
}

bool XsensRetargetting::run(mc_control::fsm::Controller & ctl)
{
  if(finished_) return autoTransition_;

  auto & ds = ctl.datastore();
  if(ds.has("XsensPlugin::Ready") && !ds.get<bool>("XsensPlugin::Ready")) return false;
  auto & robot = ctl.robot(robot_);
  Eigen::VectorXd dimW = Eigen::VectorXd::Ones(6);

  std::string baseLinkBody = robot.mb().body(0).name();
  std::string baseLinkSegment = "Pelvis";
  auto baseLinkOffset = bodyConfigurations_[baseLinkBody].offset;
  const auto baseLinkSegmentPose = baseLinkOffset * plugin_->data().segment_poses_.at(baseLinkSegment);
  double percentStiffness = startStiffnessInterpolator_.compute(t_);
  double percentWeight = startWeightInterpolator_.compute(t_);

  /**
   * This ensures that the frame in-between the specified grounding frames is
   * on the ground. Ignored if no frame is specified
   */
  if(groundingFrames_.size())
  {
    auto groundingOffset = sva::PTransformd::Identity();
    auto groundingPose = sva::PTransformd::Identity();
    if(groundingFrames_.size() == 1) { groundingPose = robot.frame(groundingFrames_.front()).position(); }
    else if(groundingFrames_.size() == 2)
    {
      groundingPose = sva::interpolate(robot.frame(groundingFrames_.front()).position(),
                                       robot.frame(groundingFrames_.back()).position(), 0.5);
    }
    groundingOffset.translation().z() = -groundingPose.translation().z();
    for(auto & [segmentName, segmentPose] : plugin_->data().segment_poses_)
    {
      segmentPose = segmentPose * groundingOffset;
    }
  }

  for(const auto & bodyName : activeBodies_)
  {
    const auto & body = bodyConfigurations_[bodyName];
    const auto & segmentName = body.segmentName;

    if(robot.hasBody(bodyName))
    {
      try
      {
        auto & bodyTask = *tasks_[bodyName];
        // bodyTask.dimWeight(dimW);
        // bodyTask.stiffness(percentStiffness * body.stiffness);
        // bodyTask.weight(percentWeight * body.weight);

        auto & segmentPose = plugin_->data().segment_poses_[segmentName];
        auto & segmentVel = plugin_->data().segment_vels_[segmentName];
        auto & segmentAcc = plugin_->data().segment_accs_[segmentName];

        if(body.forceHorizontalSegment)
        {
          // Override segment rotation to make it horizontal w.r.t ground plane
          segmentPose.rotation() =
              stateObservation::kine::mergeRoll1Pitch1WithYaw2(Eigen::Matrix3d::Identity(), segmentPose.rotation());
        }

        if(fixBaseLink_)
        {
          // Apply all xsens MVN poses w.r.t a fixed initial robot base link
          auto X_blSP_segmentPose = segmentPose * baseLinkSegmentPose.inv(); // blSP: BaseLink_segmentationPose
          auto X_0_target = body.offset * X_blSP_segmentPose * offset_ * initPosW_;
          bodyTask.target(X_0_target);
          // TODO find equivalent for fixed base link to target vel and acc
        }
        else
        { // Directly apply world segment pose as obtained from w.r.t a fixed initial robot base linkom Xsens MVN
          bodyTask.target(body.offset * segmentPose * offset_); // change the target position
          bodyTask.refVelB(segmentVel);
          bodyTask.refAccel(segmentAcc);
        }
      }
      catch(...)
      {
        mc_rtc::log::error("[{}] No pose for segment {}", name(), segmentName);
      }
    }
    else { mc_rtc::log::error("[{}] No body named {}", name(), bodyName); }
  }

  for(const auto & [fixedBodyName, fixedBodyTask] : fixedTasks_)
  {
    fixedBodyTask->stiffness(percentStiffness * fixedStiffness_);
    fixedBodyTask->weight(percentWeight * fixedWeight_);
  }

  // This section lowers stiffness at the end of the replay to be safe on the real robot, but this stops retargetting in simulation as well

  // if(ds.has("Replay::GetCurrentTime"))
  // {
  //   auto currentTime = ds.call<double>("Replay::GetCurrentTime");
  //   if(finishRequested_ && !finishing_)
  //   { // Requesting finishing early (before the end of the trajectory, start lowering stiffness now)
  //     mc_rtc::log::info("[{}] Requesting finishing after interpolation, stopping in {}s", endInterpolationTime_);
  //     finishing_ = true;
  //     auto endTime = ds.call<double>("Replay::GetEndTime");
  //     endTime_ = std::min(currentTime + endInterpolationTime_, endTime); // end after interpolation
  //   }
  //   else if(!finishing_) { endTime_ = ds.call<double>("Replay::GetEndTime"); }

  //   auto remainingTime = endTime_ - currentTime;
  //   double interpolationDuration = endStiffnessInterpolator_.values().back().first;
  //   // REDUCE STIFFNESS BEFORE STOPPING TO PREVENT DISCONTINUITIES
  //   // Here the trajectory is almost finished
  //   if(remainingTime <= 0)
  //   {
  //     finished_ = true;
  //     return autoTransition_;
  //   }
  //   else if(remainingTime <= interpolationDuration)
  //   {
  //     finishing_ = true;
  //     double endPercentStiffness = endStiffnessInterpolator_.compute(interpolationDuration - (remainingTime));
  //     double endPercentWeight = endWeightInterpolator_.compute(interpolationDuration - (remainingTime));
  //     for(const auto & bodyName : activeBodies_)
  //     {
  //       const auto & body = bodyConfigurations_[bodyName];
  //       tasks_[bodyName]->stiffness(endPercentStiffness * body.stiffness);
  //       tasks_[bodyName]->weight(endPercentWeight * body.weight);
  //     }
  //     for(const auto & [fixedBodyName, fixedBodyTask] : fixedTasks_)
  //     {
  //       fixedBodyTask->stiffness(endPercentStiffness * fixedStiffness_);
  //       fixedBodyTask->weight(endPercentWeight * fixedWeight_);
  //     }
  //   }
  // }

  t_ += ctl.timeStep;
  return false;
}

void XsensRetargetting::teardown(mc_control::fsm::Controller & ctl)
{
  for(const auto & task : tasks_) { ctl.solver().removeTask(task.second.get()); }
  for(const auto & task : fixedTasks_) { ctl.solver().removeTask(task.second.get()); }
  ctl.gui()->removeElements(this);
}

} // namespace mc_xsens_plugin

EXPORT_SINGLE_STATE("XsensRetargetting", mc_xsens_plugin::XsensRetargetting)
