#include "hoap3.h"

#include <mc_rtc/logging.h>

#include "config.h"

namespace mc_robots
{

constexpr auto PI = mc_rtc::constants::PI;

HOAP3RobotModule::HOAP3RobotModule(const std::string& moduleName, Base base, bool canonical)
    : RobotModule(HOAP3_DESCRIPTION_PATH, "hoap3",
                  fmt::format("{}/urdf/hoap3.urdf", HOAP3_DESCRIPTION_PATH)),
      moduleName_(moduleName),
      base_(base),
      canonical_(canonical)
{
  rsdf_dir = fmt::format("{}/rsdf", path);
  calib_dir = fmt::format("{}/calib", path);
  auto convexPath = fmt::format("{}/convex", path);

  // TODO:
  auto virtualLinks = std::vector<std::string>{};

  _canonicalParameters = {moduleName_ + "::Canonical"};

  if (!canonical)
  {
    // TODO: canonical filtered links
    // virtualLinks_.push_back("...");
  }

  // TODO:
  _bodySensors.emplace_back("Accelerometer", "body", sva::PTransformd(Eigen::Vector3d(-0.0325, 0, 0.1095)));
  _bodySensors.emplace_back("FloatingBase", "body", sva::PTransformd::Identity());

  _stance["CHEST_H_PAN_joint"] = {0.0};
  _stance["H_PAN_H_TILT_joint"] = {0.0};
  _stance["H_TILT_H_ROLL_joint"] = {0.0};
  _stance["CHEST_R_SFE_joint"] = {0.872664625997};
  _stance["R_SFE_R_SAA_joint"] = {-0.523598775598};
  _stance["R_SAA_R_SHR_joint"] = {-0.523598775598};
  _stance["R_SHR_R_EB_joint"] = {-1.0471975512};
  _stance["R_EB_R_WR_joint"] = {-0.523598775598};
  _stance["CHEST_L_SFE_joint"] = {0.872664625997};
  _stance["L_SFE_L_SAA_joint"] = {0.523598775598};
  _stance["L_SAA_L_SHR_joint"] = {0.523598775598};
  _stance["L_SHR_L_EB_joint"] = {-1.0471975512};
  _stance["L_EB_L_WR_joint"] = {-0.523598775598};
  _stance["CHEST_WAIST_joint"] = {0.0};
  _stance["WAIST_R_LR_joint"] = {-0.174532925199};
  _stance["R_LR_R_LAA_joint"] = {-0.0872664625997};
  _stance["R_LAA_R_LFE_joint"] = {-0.174532925199};
  _stance["R_LFE_R_KN_joint"] = {0.349065850399};
  _stance["R_KN_R_AFE_joint"] = {-0.174532925199};
  _stance["R_AFE_R_AAA_joint"] = {0.10471975512};
  _stance["WAIST_L_LR_joint"] = {0.174532925199};
  _stance["L_LR_L_LAA_joint"] = {0.0872664625997};
  _stance["L_LAA_L_LFE_joint"] = {-0.174532925199};
  _stance["L_LFE_L_KN_joint"] = {0.349065850399};
  _stance["L_KN_L_AFE_joint"] = {-0.174532925199};
  _stance["L_AFE_L_AAA_joint"] = {-0.10471975512};

  // FIXME
  _forceSensors.push_back(
      mc_rbdyn::ForceSensor("RightFootForceSensor", "r_ankle", sva::PTransformd(Eigen::Vector3d(0, 0, -0.093))));
  _forceSensors.push_back(
      mc_rbdyn::ForceSensor("LeftFootForceSensor", "l_ankle", sva::PTransformd(Eigen::Vector3d(0, 0, -0.093))));

  // FIXME
  _minimalSelfCollisions = {
      // {"body", "L_ELBOW_P_LINK", 0.05, 0.001, 0.},
      // {"body", "R_ELBOW_P_LINK", 0.05, 0.001, 0.},
      // {"torso", "L_SHOULDER_Y_LINK", 0.02, 0.001, 0.},
      // {"torso", "R_SHOULDER_Y_LINK", 0.02, 0.001, 0.},
      // // wrists/hands
      // {"L_WRIST_Y_LINK", "L_HIP_P_LINK", 0.05, 0.02, 0.},
      // {"R_WRIST_Y_LINK", "R_HIP_P_LINK", 0.05, 0.02, 0.},
      // {"L_WRIST_Y_LINK", "body", 0.03, 0.01, 0.},
      // {"R_WRIST_Y_LINK", "body", 0.03, 0.01, 0.},
      // {"r_wrist", "R_HIP_P_LINK", 0.05, 0.02, 0.},
      // {"l_wrist", "L_HIP_P_LINK", 0.05, 0.02, 0.},
      // // legs
      // {"L_FOOT_LINK", "R_FOOT_LINK", 0.03, 0.01, 0.},
      // {"L_FOOT_LINK", "R_KNEE_P_LINK", 0.04, 0.02, 0.},
      // {"R_FOOT_LINK", "L_KNEE_P_LINK", 0.04, 0.02, 0.}
  };

  _commonSelfCollisions = _minimalSelfCollisions;

  // TODO
  // _grippers = {{"l_gripper", {"L_HAND_J0", "L_HAND_J1"}, false}, {"r_gripper", {"R_HAND_J0", "R_HAND_J1"}, true}};

  // FIXME
  _default_attitude = {{1., 0., 0., 0., 0., 0., 0.747187}};

  // Compound Joints
  _compoundJoints = {};

  // FIXME
  // Configure the stabilizer
  _lipmStabilizerConfig.leftFootSurface = "LeftFootCenter";
  _lipmStabilizerConfig.rightFootSurface = "RightFootCenter";
  _lipmStabilizerConfig.torsoBodyName = "torso";
  _lipmStabilizerConfig.comHeight = 0.78;
  _lipmStabilizerConfig.comActiveJoints =
      {
          "Root", "R_HIP_Y", "R_HIP_R", "R_HIP_P", "R_KNEE_P", "R_ANKLE_P", "R_ANKLE_R",
          "L_HIP_Y", "L_HIP_R", "L_HIP_P", "L_KNEE_P", "L_ANKLE_P", "L_ANKLE_R"};
  _lipmStabilizerConfig.torsoPitch = 0;
  _lipmStabilizerConfig.copAdmittance = Eigen::Vector2d{0.01, 0.01};
  _lipmStabilizerConfig.dcmPropGain = 5.0;
  _lipmStabilizerConfig.dcmIntegralGain = 10;
  _lipmStabilizerConfig.dcmDerivGain = 0.0;
  _lipmStabilizerConfig.dcmDerivatorTimeConstant = 1;
  _lipmStabilizerConfig.dcmIntegratorTimeConstant = 10;

  // TODO: check this
  _ref_joint_order = {"CHEST_H_PAN_joint", "H_PAN_H_TILT_joint", "H_TILT_H_ROLL_joint", "CHEST_R_SFE_joint", "R_SFE_R_SAA_joint", "R_SAA_R_SHR_joint", "R_SHR_R_EB_joint", "R_EB_R_WR_joint", "CHEST_L_SFE_joint", "L_SFE_L_SAA_joint", "L_SAA_L_SHR_joint", "L_SHR_L_EB_joint", "L_EB_L_WR_joint", "CHEST_WAIST_joint", "WAIST_R_LR_joint", "R_LR_R_LAA_joint", "R_LAA_R_LFE_joint", "R_LFE_R_KN_joint", "R_KN_R_AFE_joint", "R_AFE_R_AAA_joint", "WAIST_L_LR_joint", "L_LR_L_LAA_joint", "L_LAA_L_LFE_joint", "L_LFE_L_KN_joint", "L_KN_L_AFE_joint", "L_AFE_L_AAA_joint"};

  init(rbd::parsers::from_urdf_file(urdf_path, rbd::parsers::ParserParameters{}
                                                   .fixed(base_ == Base::Fixed)
                                                   .filtered_links(virtualLinks)
                                                   .remove_filtered_links(false)));
}

}  // namespace mc_robots
