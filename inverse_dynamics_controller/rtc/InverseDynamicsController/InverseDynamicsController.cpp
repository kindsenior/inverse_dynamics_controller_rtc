#include "InverseDynamicsController.h"
#include <cnoid/BodyLoader>
#include <cnoid/ForceSensor>
#include <cnoid/RateGyroSensor>
#include <cnoid/ValueTree>
#include <cnoid/EigenUtil>
// #include "MathUtil.h"
#include "CnoidBodyUtil.h"
#include <limits>

static const char* InverseDynamicsController_spec[] = {
  "implementation_id", "InverseDynamicsController",
  "type_name",         "InverseDynamicsController",
  "description",       "InverseDynamicsController component",
  "version",           "0.0",
  "vendor",            "Kunio-Kojima",
  "category",          "example",
  "activity_type",     "DataFlowComponent",
  "max_instance",      "10",
  "language",          "C++",
  "lang_type",         "compile",
  ""
};

InverseDynamicsController::Ports::Ports() :
  // Input ports
  //// Refefence
  m_qRefIn_("qRef", m_qRef_),
  m_dqRefIn_("dqRef", m_dqRef_),
  m_tauRefIn_("tauRef", m_tauRef_),
  m_basePosRefIn_("basePosRef", m_basePosRef_),
  m_baseRpyRefIn_("baseRpyRef", m_baseRpyRef_),
  //// Actual
  m_qActIn_("qAct", m_qAct_),
  m_dqActIn_("dqAct", m_dqAct_),
  m_actImuIn_("actImuIn", m_actImu_),

  // Output ports
  m_tauOut_("tauOut", m_tau_),

  // Service Ports
  m_InverseDynamicsControllerServicePort_("InverseDynamicsControllerService")
  // m_RobotHardwareServicePort_("RobotHardwareService")
  {
}

InverseDynamicsController::InverseDynamicsController(RTC::Manager* manager) : RTC::DataFlowComponentBase(manager),
  ports_(),
  debugLevel_(0)
{
  this->ports_.m_service0_.setComp(this);
}

RTC::ReturnCode_t InverseDynamicsController::onInitialize(){

  // add ports
  //// input ports
  this->addInPort("qRef", this->ports_.m_qRefIn_);
  this->addInPort("dqRef", this->ports_.m_dqRefIn_);
  this->addInPort("tauRef", this->ports_.m_tauRefIn_);
  this->addInPort("basePosRef", this->ports_.m_basePosRefIn_);
  this->addInPort("baseRpyRef", this->ports_.m_baseRpyRefIn_);
  this->addInPort("qAct", this->ports_.m_qActIn_);
  this->addInPort("dqAct", this->ports_.m_dqActIn_);
  this->addInPort("actImuIn", this->ports_.m_actImuIn_);
  //// output ports
  this->addOutPort("tauOut", this->ports_.m_tauOut_);
  //// service ports
  this->ports_.m_InverseDynamicsControllerServicePort_.registerProvider("service0", "InverseDynamicsControllerService", this->ports_.m_service0_);
  this->addPort(this->ports_.m_InverseDynamicsControllerServicePort_);
  // this->ports_.m_RobotHardwareServicePort_.registerConsumer("service0", "RobotHardwareService", this->ports_.m_robotHardwareService0_);
  // this->addPort(this->ports_.m_RobotHardwareServicePort_);
  {
    // load dt
    std::string buf; this->getProperty("dt", buf);
    this->dt_ = std::stod(buf);
    if(this->dt_ <= 0.0){
      this->getProperty("exec_cxt.periodic.rate", buf);
      double rate = std::stod(buf);
      if(rate > 0.0){
        this->dt_ = 1.0/rate;
      }else{
        std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << "dt is invalid" << "\x1b[39m" << std::endl;
        return RTC::RTC_ERROR;
      }
    }
  }

  {
    // load robot model
    cnoid::BodyLoader bodyLoader;
    std::string fileName; this->getProperty("model", fileName);
    if (fileName.find("file://") == 0) fileName.erase(0, strlen("file://"));
    cnoid::BodyPtr robot = bodyLoader.load(fileName);
    if(!robot){
      std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << "failed to load model[" << fileName << "]" << "\x1b[39m" << std::endl;
      return RTC::RTC_ERROR;
    }
    if(!robot->rootLink()->isFreeJoint()){
      std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << "rootLink is not FreeJoint [" << fileName << "]" << "\x1b[39m" << std::endl;
      return RTC::RTC_ERROR;
    }
    actRobot = robot->clone();
    actRobot->calcForwardKinematics(); actRobot->calcCenterOfMass();
    refRobot = robot->clone();
    refRobot->calcForwardKinematics(); refRobot->calcCenterOfMass();

    // generate JointParams
    for(int i=0;i<this->actRobot->numJoints();i++){
      cnoid::LinkPtr joint = this->actRobot->joint(i);
      double climit = 0.0, gearRatio = 0.0, torqueConst = 0.0;
      joint->info()->read("climit",climit); joint->info()->read("gearRatio",gearRatio); joint->info()->read("torqueConst",torqueConst);
    }
  }

  {
    // load end_effector
    std::string endEffectors; this->getProperty("end_effectors", endEffectors);
    std::stringstream ss_endEffectors(endEffectors);
    std::string buf;
    while(std::getline(ss_endEffectors, buf, ',')){
      std::string name;
      std::string parentLink;
      cnoid::Vector3 localp;
      cnoid::Vector3 localaxis;
      double localangle;

      //   name, parentLink, (not used), x, y, z, theta, ax, ay, az
      name = buf;
      if(!std::getline(ss_endEffectors, buf, ',')) break; parentLink = buf;
      if(!std::getline(ss_endEffectors, buf, ',')) break; // not used
      if(!std::getline(ss_endEffectors, buf, ',')) break; localp[0] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localp[1] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localp[2] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localaxis[0] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localaxis[1] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localaxis[2] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localangle = std::stod(buf);

      // check validity
      name.erase(std::remove(name.begin(), name.end(), ' '), name.end()); // remove whitespace
      parentLink.erase(std::remove(parentLink.begin(), parentLink.end(), ' '), parentLink.end()); // remove whitespace
      if(!this->refRobot->link(parentLink)){
        std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << " link [" << parentLink << "]" << " is not found for " << name << "\x1b[39m" << std::endl;
        return RTC::RTC_ERROR;
      }
      cnoid::Matrix3 localR;
      if(localaxis.norm() == 0) localR = cnoid::Matrix3::Identity();
      else localR = Eigen::AngleAxisd(localangle, localaxis.normalized()).toRotationMatrix();
      cnoid::Position localT;
      localT.translation() = localp;
      localT.linear() = localR;

      // TODO: store end-effector parameters to members
    }
  }

  {
    // // 各ForceSensorにつき、act<name>InというInportをつくる
    // cnoid::DeviceList<cnoid::ForceSensor> forceSensors(this->gaitParam_.actRobotRaw->devices());
    // this->ports_.m_actWrenchIn_.resize(forceSensors.size());
    // this->ports_.m_actWrench_.resize(forceSensors.size());
    // for(int i=0;i<forceSensors.size();i++){
    //   std::string name = "act"+forceSensors[i]->name()+"In";
    //   this->ports_.m_actWrenchIn_[i] = std::make_unique<RTC::InPort<RTC::TimedDoubleSeq> >(name.c_str(), this->ports_.m_actWrench_[i]);
    //   this->addInPort(name.c_str(), *(this->ports_.m_actWrenchIn_[i]));
    // }

    // // 各EndEffectorにつき、ref<name>WrenchInというInPortをつくる
    // this->ports_.m_refEEWrenchIn_.resize(this->gaitParam_.eeName.size());
    // this->ports_.m_refEEWrench_.resize(this->gaitParam_.eeName.size());
    // for(int i=0;i<this->gaitParam_.eeName.size();i++){
    //   std::string name = "ref"+this->gaitParam_.eeName[i]+"WrenchIn";
    //   this->ports_.m_refEEWrenchIn_[i] = std::make_unique<RTC::InPort<RTC::TimedDoubleSeq> >(name.c_str(), this->ports_.m_refEEWrench_[i]);
    //   this->addInPort(name.c_str(), *(this->ports_.m_refEEWrenchIn_[i]));
    // }

    // // 各EndEffectorにつき、ref<name>PoseInというInPortをつくる
    // this->ports_.m_refEEPoseIn_.resize(this->gaitParam_.eeName.size());
    // this->ports_.m_refEEPose_.resize(this->gaitParam_.eeName.size());
    // for(int i=0;i<this->gaitParam_.eeName.size();i++){
    //   std::string name = "ref"+this->gaitParam_.eeName[i]+"PoseIn";
    //   this->ports_.m_refEEPoseIn_[i] = std::make_unique<RTC::InPort<RTC::TimedPose3D> >(name.c_str(), this->ports_.m_refEEPose_[i]);
    //   this->addInPort(name.c_str(), *(this->ports_.m_refEEPoseIn_[i]));
    // }

    // // 各EndEffectorにつき、act<name>PoseOutというOutPortをつくる
    // this->ports_.m_actEEPoseOut_.resize(this->gaitParam_.eeName.size());
    // this->ports_.m_actEEPose_.resize(this->gaitParam_.eeName.size());
    // for(int i=0;i<this->gaitParam_.eeName.size();i++){
    //   std::string name = "act"+this->gaitParam_.eeName[i]+"PoseOut";
    //   this->ports_.m_actEEPoseOut_[i] = std::make_unique<RTC::OutPort<RTC::TimedPose3D> >(name.c_str(), this->ports_.m_actEEPose_[i]);
    //   this->addOutPort(name.c_str(), *(this->ports_.m_actEEPoseOut_[i]));
    // }

    // // 各EndEffectorにつき、tgt<name>WrenchOutというOutPortをつくる
    // this->ports_.m_tgtEEWrenchOut_.resize(this->gaitParam_.eeName.size());
    // this->ports_.m_tgtEEWrench_.resize(this->gaitParam_.eeName.size());
    // for(int i=0;i<this->gaitParam_.eeName.size();i++){
    //   std::string name = "tgt"+this->gaitParam_.eeName[i]+"WrenchOut";
    //   this->ports_.m_tgtEEWrenchOut_[i] = std::make_unique<RTC::OutPort<RTC::TimedDoubleSeq> >(name.c_str(), this->ports_.m_tgtEEWrench_[i]);
    //   this->addOutPort(name.c_str(), *(this->ports_.m_tgtEEWrenchOut_[i]));
    // }

    // // 各EndEffectorにつき、act<name>WrenchOutというOutPortをつくる
    // this->ports_.m_actEEWrenchOut_.resize(this->gaitParam_.eeName.size());
    // this->ports_.m_actEEWrench_.resize(this->gaitParam_.eeName.size());
    // for(int i=0;i<this->gaitParam_.eeName.size();i++){
    //   std::string name = "act"+this->gaitParam_.eeName[i]+"WrenchOut";
    //   this->ports_.m_actEEWrenchOut_[i] = std::make_unique<RTC::OutPort<RTC::TimedDoubleSeq> >(name.c_str(), this->ports_.m_actEEWrench_[i]);
    //   this->addOutPort(name.c_str(), *(this->ports_.m_actEEWrenchOut_[i]));
    // }

  }

  // initialize parameters
  this->loop_ = 0;

  return RTC::RTC_OK;
}

// static function
bool InverseDynamicsController::readInPortData(
  const double& dt, const InverseDynamicsController::ControlMode& mode, InverseDynamicsController::Ports& ports,
  cnoid::BodyPtr refRobot, cnoid::BodyPtr actRobot
){
  bool qRef_updated = false;

  // read reference data
  if(ports.m_qRefIn_.isNew()){
    ports.m_qRefIn_.read();
    if(ports.m_qRef_.data.length() == refRobot->numJoints()){
      for(int i=0;i<ports.m_qRef_.data.length();i++){
        if(std::isfinite(ports.m_qRef_.data[i])) refRobot->joint(i)->q() = ports.m_qRef_.data[i];
        else std::cerr << "m_qRef is not finite!" << std::endl;
      }
      qRef_updated = true;
    }
  }
  if(ports.m_tauRefIn_.isNew()){
    ports.m_tauRefIn_.read();
    if(ports.m_tauRef_.data.length() == refRobot->numJoints()){
      for(int i=0;i<ports.m_tauRef_.data.length();i++){
        if(std::isfinite(ports.m_tauRef_.data[i])) refRobot->joint(i)->u() = ports.m_tauRef_.data[i];
        else std::cerr << "m_tauRef is not finite!" << std::endl;
      }
    }
  }
  if(ports.m_basePosRefIn_.isNew()){
    ports.m_basePosRefIn_.read();
    if(std::isfinite(ports.m_basePosRef_.data.x) && std::isfinite(ports.m_basePosRef_.data.y) && std::isfinite(ports.m_basePosRef_.data.z)){
      refRobot->rootLink()->p()[0] = ports.m_basePosRef_.data.x;
      refRobot->rootLink()->p()[1] = ports.m_basePosRef_.data.y;
      refRobot->rootLink()->p()[2] = ports.m_basePosRef_.data.z;
    } else {
      std::cerr << "m_basePosRef is not finite!" << std::endl;
    }
  }
  if(ports.m_baseRpyRefIn_.isNew()){
    ports.m_baseRpyRefIn_.read();
    if(std::isfinite(ports.m_baseRpyRef_.data.r) && std::isfinite(ports.m_baseRpyRef_.data.p) && std::isfinite(ports.m_baseRpyRef_.data.y)){
      refRobot->rootLink()->R() = cnoid::rotFromRpy(ports.m_baseRpyRef_.data.r, ports.m_baseRpyRef_.data.p, ports.m_baseRpyRef_.data.y);
    } else {
      std::cerr << "m_baseRpyRef is not finite!" << std::endl;
    }
  }
  refRobot->calcForwardKinematics();
  refRobot->calcCenterOfMass();

  // for(int i=0;i<ports.m_refEEWrenchIn_.size();i++){
  //   if(ports.m_refEEWrenchIn_[i]->isNew()){
  //     ports.m_refEEWrenchIn_[i]->read();
  //     if(ports.m_refEEWrench_[i].data.length() == 6){
  //       for(int j=0;j<6;j++){
  //         if(std::isfinite(ports.m_refEEWrench_[i].data[j])) refEEWrenchOrigin[i][j] = ports.m_refEEWrench_[i].data[j];
  //         else std::cerr << "m_refEEWrench is not finite!" << std::endl;
  //       }
  //     }
  //   }
  // }

  // for(int i=0;i<ports.m_refEEPoseIn_.size();i++){
  //   if(ports.m_refEEPoseIn_[i]->isNew()){
  //     ports.m_refEEPoseIn_[i]->read();
  //     if(std::isfinite(ports.m_refEEPose_[i].data.position.x) && std::isfinite(ports.m_refEEPose_[i].data.position.y) && std::isfinite(ports.m_refEEPose_[i].data.position.z) &&
  //        std::isfinite(ports.m_refEEPose_[i].data.orientation.r) && std::isfinite(ports.m_refEEPose_[i].data.orientation.p) && std::isfinite(ports.m_refEEPose_[i].data.orientation.y)){
  //       cnoid::Position pose;
  //       pose.translation()[0] = ports.m_refEEPose_[i].data.position.x;
  //       pose.translation()[1] = ports.m_refEEPose_[i].data.position.y;
  //       pose.translation()[2] = ports.m_refEEPose_[i].data.position.z;
  //       pose.linear() = cnoid::rotFromRpy(ports.m_refEEPose_[i].data.orientation.r, ports.m_refEEPose_[i].data.orientation.p, ports.m_refEEPose_[i].data.orientation.y);
  //       refEEPoseRaw[i].setGoal(pose, 0.3); // 0.3秒で補間
  //       ports.refEEPoseLastUpdateTime_ = ports.m_qRef_.tm;
  //     } else {
  //       std::cerr << "m_refEEPose is not finite!" << std::endl;
  //     }
  //   }
  //   refEEPoseRaw[i].interpolate(dt);
  // }

  // read actual data
  if(ports.m_qActIn_.isNew()){
    ports.m_qActIn_.read();
    if(ports.m_qAct_.data.length() == actRobot->numJoints()){
      for(int i=0;i<ports.m_qAct_.data.length();i++){
        if(std::isfinite(ports.m_qAct_.data[i])) actRobot->joint(i)->q() = ports.m_qAct_.data[i];
        else std::cerr << "m_qAct is not finite!" << std::endl;
      }
    }
  }
  if(ports.m_dqActIn_.isNew()){
    ports.m_dqActIn_.read();
    if(ports.m_dqAct_.data.length() == actRobot->numJoints()){
      for(int i=0;i<ports.m_dqAct_.data.length();i++){
        if(std::isfinite(ports.m_dqAct_.data[i])) actRobot->joint(i)->dq() = ports.m_dqAct_.data[i];
        else  std::cerr << "m_dqAct is not finite!" << std::endl;
      }
    }
  }
  if(ports.m_actImuIn_.isNew()){
    ports.m_actImuIn_.read();
    if(std::isfinite(ports.m_actImu_.data.r) && std::isfinite(ports.m_actImu_.data.p) && std::isfinite(ports.m_actImu_.data.y)){
      actRobot->calcForwardKinematics();
      cnoid::RateGyroSensorPtr imu = actRobot->findDevice<cnoid::RateGyroSensor>("gyrometer");
      cnoid::Matrix3 imuR = imu->link()->R() * imu->R_local();
      cnoid::Matrix3 actR = cnoid::rotFromRpy(ports.m_actImu_.data.r, ports.m_actImu_.data.p, ports.m_actImu_.data.y);
      actRobot->rootLink()->R() = Eigen::Matrix3d(Eigen::AngleAxisd(actR) * Eigen::AngleAxisd(imuR.transpose() * actRobot->rootLink()->R())); // 単純に3x3行列の空間でRを積算していると、だんだん数値誤差によって回転行列でなくなってしまう恐れがあるので念の為
    }else{
      std::cerr << "m_actImu is not finite!" << std::endl;
    }
  }
  actRobot->calcForwardKinematics();
  actRobot->calcCenterOfMass();

  // cnoid::DeviceList<cnoid::ForceSensor> forceSensors(actRobot->devices());
  // for(int i=0;i<ports.m_actWrenchIn_.size();i++){
  //   if(ports.m_actWrenchIn_[i]->isNew()){
  //     ports.m_actWrenchIn_[i]->read();
  //     if(ports.m_actWrench_[i].data.length() == 6){
  //       for(int j=0;j<6;j++){
  //         if(std::isfinite(ports.m_actWrench_[i].data[j])) forceSensors[i]->F()[j] = ports.m_actWrench_[i].data[j];
  //         else std::cerr << "m_actWrench is not finite!" << std::endl;
  //       }
  //     }
  //   }
  // }

  return qRef_updated;
}

// static function
bool InverseDynamicsController::execInverseDynamicsController(const InverseDynamicsController::ControlMode& mode, const double dt) {
  if(mode.isSyncToIDCInit()){ // startInverseDynamicsController直後の初回. パラメータのリセット
  }

  // calcTorque

  return true;
}

// static function
bool InverseDynamicsController::writeOutPortData(
  const double& dt, const InverseDynamicsController::ControlMode& mode, InverseDynamicsController::Ports& ports,
  cnoid::BodyPtr refRobot, cnoid::BodyPtr actRobot,
  cpp_filters::TwoPointInterpolator<double>& idleToIdcTransitionInterpolator){
  if(mode.isSyncToIDC()){
    if(mode.isSyncToIDCInit()){
      idleToIdcTransitionInterpolator.reset(0.0);
    }
    idleToIdcTransitionInterpolator.setGoal(1.0,mode.remainTime());
    idleToIdcTransitionInterpolator.interpolate(dt);
  }else if(mode.isSyncToIdle()){
    if(mode.isSyncToIdleInit()){
      idleToIdcTransitionInterpolator.reset(1.0);
    }
    idleToIdcTransitionInterpolator.setGoal(0.0,mode.remainTime());
    idleToIdcTransitionInterpolator.interpolate(dt);
  }

  {
    // tau
    ports.m_tau_.tm = ports.m_qRef_.tm;
    ports.m_tau_.data.length(refRobot->numJoints());
    for(int i=0;i<refRobot->numJoints();i++){
      if(mode.now() == InverseDynamicsController::ControlMode::MODE_IDLE){
        double value = ports.m_tauRef_.data[i];
        if(std::isfinite(value)) ports.m_tau_.data[i] = value;
        else std::cerr << "m_tauRef is not finite in joint(" << i << ")!" << std::endl;
      }else if(mode.isSyncToIDC() || mode.isSyncToIdle()){
        double ratio = idleToIdcTransitionInterpolator.value();
        double value = ports.m_tauRef_.data[i] * (1.0 - ratio) + refRobot->joint(i)->u() * ratio;
        if(std::isfinite(value)) ports.m_tau_.data[i] = value;
        else std::cerr << "m_tauRef or the results of inverse dynamics is not finite in joint(" << i << ")!" << std::endl;
      }else{
        double value = refRobot->joint(i)->u();
        if(std::isfinite(value)) ports.m_tau_.data[i] = value;
        else std::cerr << "the results of inverse dynamics is not finite in joint(" << i << ")!" << std::endl;
      }
    }
    ports.m_tauOut_.write();
  }

  // // Gains
  // if(!CORBA::is_nil(ports.m_robotHardwareService0_._ptr()) && //コンシューマにプロバイダのオブジェクト参照がセットされていない(接続されていない)状態
  //    !ports.m_robotHardwareService0_->_non_existent()){ //プロバイダのオブジェクト参照は割り当てられているが、相手のオブジェクトが非活性化 (RTC は Inactive 状態) になっている状態
  //   for(int i=0;i<gaitParam.genRobot->numJoints();i++){
  //     if(mode.now() == InverseDynamicsController::ControlMode::MODE_IDLE || !gaitParam.jointControllable[i]){
  //       // pass
  //     }else if(mode.isSyncToABC()){
  //       // pass
  //     }else if(mode.isSyncToIdle()){
  //       // pass
  //     }else{
  //       // Stabilizerが動いている間にonDeactivated()->onActivated()が呼ばれると、ゲインがもとに戻らない. onDeactivated()->onActivated()が呼ばれるのはサーボオン直前で、通常、サーボオン時にゲインを指令するので、問題ない.
  //       if(gaitParam.stServoPGainPercentage[i].remain_time() > 0.0 && gaitParam.stServoPGainPercentage[i].current_time() <= dt) { // 補間が始まった初回
  //         if(std::isfinite(gaitParam.stServoPGainPercentage[i].getGoal()) && std::isfinite(gaitParam.stServoPGainPercentage[i].goal_time())){
  //           ports.m_robotHardwareService0_->setServoPGainPercentageWithTime(gaitParam.actRobotTqc->joint(i)->name().c_str(),gaitParam.stServoPGainPercentage[i].getGoal(),gaitParam.stServoPGainPercentage[i].goal_time());
  //         }else{
  //           std::cerr << "setServoPGainPercentageWithTime is not finite!" << std::endl;
  //         }
  //       }
  //       if(gaitParam.stServoDGainPercentage[i].remain_time() > 0.0 && gaitParam.stServoDGainPercentage[i].current_time() <= dt) { // 補間が始まった初回
  //         if(std::isfinite(gaitParam.stServoDGainPercentage[i].getGoal()) && std::isfinite(gaitParam.stServoDGainPercentage[i].goal_time())){
  //           ports.m_robotHardwareService0_->setServoDGainPercentageWithTime(gaitParam.actRobotTqc->joint(i)->name().c_str(),gaitParam.stServoDGainPercentage[i].getGoal(),gaitParam.stServoDGainPercentage[i].goal_time());
  //         }else{
  //           std::cerr << "setServoDGainPercentageWithTime is not finite!" << std::endl;
  //         }
  //       }
  //     }
  //   }
  // }

  // // actEEPose actEEWrench (for wholebodymasterslave)
  // if(mode.isABCRunning()){
  //   for(int i=0;i<gaitParam.eeName.size();i++){
  //     ports.m_actEEPose_[i].tm = ports.m_qRef_.tm;
  //     ports.m_actEEPose_[i].data.position.x = gaitParam.actEEPose[i].translation()[0];
  //     ports.m_actEEPose_[i].data.position.y = gaitParam.actEEPose[i].translation()[1];
  //     ports.m_actEEPose_[i].data.position.z = gaitParam.actEEPose[i].translation()[2];
  //     cnoid::Vector3 rpy = cnoid::rpyFromRot(gaitParam.actEEPose[i].linear());
  //     ports.m_actEEPose_[i].data.orientation.r = rpy[0];
  //     ports.m_actEEPose_[i].data.orientation.p = rpy[1];
  //     ports.m_actEEPose_[i].data.orientation.y = rpy[2];
  //     ports.m_actEEPoseOut_[i]->write();
  //   }
  //   for(int i=0;i<gaitParam.eeName.size();i++){
  //     ports.m_actEEWrench_[i].tm = ports.m_qRef_.tm;
  //     ports.m_actEEWrench_[i].data.length(6);
  //     for(int j=0;j<6;j++) ports.m_actEEWrench_[i].data[j] = gaitParam.actEEWrench[i][j];
  //     ports.m_actEEWrenchOut_[i]->write();
  //   }
  // }

  return true;
}

RTC::ReturnCode_t InverseDynamicsController::onExecute(RTC::UniqueId ec_id){
  std::lock_guard<std::mutex> guard(this->mutex_);

  std::string instance_name = std::string(this->m_profile.instance_name);
  this->loop_++;

  if(!InverseDynamicsController::readInPortData(this->dt_, this->mode_, this->ports_, this->refRobot, this->actRobot)) return RTC::RTC_OK;  // qRef が届かなければ何もしない

  this->mode_.update(this->dt_);

  if(this->mode_.isIDCRunning()) {
    if(this->mode_.isSyncToIDCInit()){ // startInverseDynamicsController直後の初回. 内部パラメータのリセット
    }
    InverseDynamicsController::execInverseDynamicsController(this->mode_, this->dt_);
  }

  InverseDynamicsController::writeOutPortData( this->dt_, this->mode_, this->ports_, this->refRobot, this->actRobot, this->idleToIdcTransitionInterpolator_);

  return RTC::RTC_OK;
}

RTC::ReturnCode_t InverseDynamicsController::onActivated(RTC::UniqueId ec_id){
  std::lock_guard<std::mutex> guard(this->mutex_);
  std::cerr << "[" << m_profile.instance_name << "] "<< "onActivated(" << ec_id << ")" << std::endl;
  this->mode_.reset();
  this->idleToIdcTransitionInterpolator_.reset(0.0);
  return RTC::RTC_OK;
}
RTC::ReturnCode_t InverseDynamicsController::onDeactivated(RTC::UniqueId ec_id){
  std::lock_guard<std::mutex> guard(this->mutex_);
  std::cerr << "[" << m_profile.instance_name << "] "<< "onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}
RTC::ReturnCode_t InverseDynamicsController::onFinalize(){ return RTC::RTC_OK; }

bool InverseDynamicsController::startInverseDynamicsController(){
  if(this->mode_.setNextTransition(ControlMode::START_IDC)){
    std::cerr << "[" << m_profile.instance_name << "] start inverse dynamics controller mode" << std::endl;
    while (this->mode_.now() != ControlMode::MODE_IDC) usleep(1000);
    usleep(1000);
    return true;
  }else{
    std::cerr << "[" << this->m_profile.instance_name << "] inverse dynamics controller is already started" << std::endl;
    return false;
  }
}
bool InverseDynamicsController::stopInverseDynamicsController(){
  if(this->mode_.setNextTransition(ControlMode::STOP_IDC)){
    std::cerr << "[" << m_profile.instance_name << "] stop inverse dynamics controller mode" << std::endl;
    while (this->mode_.now() != ControlMode::MODE_IDLE) usleep(1000);
    usleep(1000);
    return true;
  }else{
    std::cerr << "[" << this->m_profile.instance_name << "] inverse dynamics controller is already stopped or stabilizer is running" << std::endl;
    return false;
  }
}

bool InverseDynamicsController::setInverseDynamicsControllerParam(const OpenHRP::InverseDynamicsControllerService::InverseDynamicsControllerParam& i_param){
  std::lock_guard<std::mutex> guard(this->mutex_);
  return true;
}
bool InverseDynamicsController::getInverseDynamicsControllerParam(OpenHRP::InverseDynamicsControllerService::InverseDynamicsControllerParam& i_param) {
  std::lock_guard<std::mutex> guard(this->mutex_);
  return true;
}

bool InverseDynamicsController::getProperty(const std::string& key, std::string& ret) {
  if (this->getProperties().hasKey(key.c_str())) {
    ret = std::string(this->getProperties()[key.c_str()]);
  } else if (this->m_pManager->getConfig().hasKey(key.c_str())) { // 引数 -o で与えたプロパティを捕捉
    ret = std::string(this->m_pManager->getConfig()[key.c_str()]);
  } else {
    return false;
  }
  std::cerr << "[" << this->m_profile.instance_name << "] " << key << ": " << ret <<std::endl;
  return true;
}

extern "C"{
    void InverseDynamicsControllerInit(RTC::Manager* manager) {
        RTC::Properties profile(InverseDynamicsController_spec);
        manager->registerFactory(profile, RTC::Create<InverseDynamicsController>, RTC::Delete<InverseDynamicsController>);
    }
};
