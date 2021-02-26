/* ------------------------------------------------------------------------------------------------
 * Fraunhofer IML
 * Department Automation and Embedded Systems
 * ------------------------------------------------------------------------------------------------
 * project          : mars_agent_robot
 * ------------------------------------------------------------------------------------------------
 * Author(s)        : Dennis Luensch
 * Contact(s)       : dennis.luensch@iml.fraunhofer.de
 * ------------------------------------------------------------------------------------------------
 * Tabsize          : 2
 * Charset          : UTF-8
 * ---------------------------------------------------------------------------------------------
 */

#include "mars_simulation_ctv_agent/MarsSimulationCtvAgentNode.h"

static const std::string NODE_NAME = "Mars_Simulation_Ctv_Agent_Node";

static const int NODE_RATE_MAX_ROS_PARAM = -1;
static const int NODE_RATE_MAX = 50;

static const std::string LOG_LEVEL_DEBUG = "debug";
static const std::string LOG_LEVEL_INFO = "info";
static const std::string LOG_LEVEL_WARN = "warn";
static const std::string LOG_LEVEL_ERROR = "error";
static const std::string LOG_LEVEL_FATAL = "fatal";

static const std::string STD_NODE_RATE_PARAM_NAME = "node_rate";
static const int STD_NODE_RATE_PARAM = 5;

static const std::string STD_LOG_LEVEL_PARAM_NAME = "log_level";
static const std::string STD_LOG_LEVEL_PARAM = LOG_LEVEL_INFO;

// RobotAgentProperties
static const std::string STD_ROBOT_ID_PARAM_NAME = "robot_id";
static const std::string STD_TYPE_PARAM_NAME = "type";

static const std::string STD_FOOTPRINT_X_PARAM_NAME = "footprint_x";
static const std::string STD_FOOTPRINT_Y_PARAM_NAME = "footprint_y";
static const std::string STD_MIN_HEIGHT_PARAM_NAME = "min_height";
static const std::string STD_MAX_HEIGHT_PARAM_NAME = "max_height";
static const std::string STD_PAYLOAD_PARAM_NAME = "payload";

static const std::string STD_MAX_POS_X_VEL_PARAM_NAME = "max_pos_x_vel";
static const std::string STD_MAX_NEG_X_VEL_PARAM_NAME = "max_neg_x_vel";
static const std::string STD_MAX_POS_X_ACC_PARAM_NAME = "max_pos_x_acc";
static const std::string STD_MAX_NEG_X_ACC_PARAM_NAME = "max_neg_x_acc";
static const std::string STD_MAX_POS_Y_VEL_PARAM_NAME = "max_pos_y_vel";
static const std::string STD_MAX_NEG_Y_VEL_PARAM_NAME = "max_neg_y_vel";
static const std::string STD_MAX_POS_Y_ACC_PARAM_NAME = "max_pos_y_acc";
static const std::string STD_MAX_NEG_Y_ACC_PARAM_NAME = "max_neg_y_acc";

static const std::string STD_MAX_POS_ANG_VEL_PARAM_NAME = "max_pos_ang_vel";
static const std::string STD_MAX_NEG_ANG_VEL_PARAM_NAME = "max_neg_ang_vel";
static const std::string STD_MAX_POS_ANG_ACC_PARAM_NAME = "max_pos_ang_acc";
static const std::string STD_MAX_NEG_ANG_ACC_PARAM_NAME = "max_neg_ang_acc";

static const std::string STD_VELOCITY_CONTROL_SENSITIVITY_PARAM_NAME =
    "velocity_control_sensitivity";
static const std::string STD_MIN_TURNING_RADIUS_PARAM_NAME = "min_turning_radius";

static const std::string STD_BATT_CAPACITY_PARAM_NAME = "batt_capacity";
static const std::string STD_BATT_MAX_VOLTAGE_PARAM_NAME = "batt_max_voltage";
static const std::string STD_VENDOR_PARAM_NAME = "vendor";
static const std::string STD_ACTION_CAPABILITY_PARAM_NAME = "action_capability";

static const std::string STD_USE_INITIAL_POSE_PARAM_NAME = "use_initial_pose";
static const bool STD_USE_INITIAL_POSE_PARAM = false;
// InitialPose
static const std::string STD_INIT_POSE_X_PARAM_NAME = "initial_pose_x";
static const std::string STD_INIT_POSE_Y_PARAM_NAME = "initial_pose_y";
static const std::string STD_INIT_POSE_PHI_PARAM_NAME = "initial_pose_a";

// Durations for the Timer
static const std::string STD_CURRENT_MOTION_MSG_FREQUENCY_PARAM_NAME =
    "current_motion_msg_frequency";
static const std::string STD_BATTERY_STATE_MSG_FREQUENCY_PARAM_NAME = "battery_state_msg_frequency";
static const std::string STD_ROBOT_STATE_MSG_FREQUENCY_PARAM_NAME = "robot_state_msg_frequency";

// Frames
static const std::string STD_GLOBAL_FRAME_ID_PARAM_NAME = "global_frame_id";
static const std::string STD_ROBOT_FRAME_ID_PARAM_NAME = "robot_frame_id";

// Default topics
static const std::string STD_CANCEL_TASK_TOPIC_PARAM_NAME = "cancel_task_topic";
static const std::string STD_ACTION_ASSIGNMENT_TOPIC_PARAM_NAME = "action_assignment_topic";
static const std::string STD_MOTION_ASSIGNMENT_TOPIC_PARAM_NAME = "motion_assignment_topic";
static const std::string STD_ROBOT_DESCRIPTION_TOPIC_PARAM_NAME = "robot_description_topic";
static const std::string STD_ASSIGNMENT_STATE_TOPIC_PARAM_NAME = "assignment_state_topic";
static const std::string STD_ROBOT_STATE_TOPIC_PARAM_NAME = "robot_state_topic";
static const std::string STD_BATTERY_STATE_TOPIC_PARAM_NAME = "battery_state_topic";
static const std::string STD_CURRENT_MOTION_TOPIC_PARAM_NAME = "current_motion_topic";
static const std::string STD_CMD_VEL_TOPIC_PARAM_NAME = "cmd_vel_topic";
static const std::string STD_SCAN_TOPIC_PARAM_NAME = "scan_topic";
static const std::string STD_ACTUAL_STATE_TOPIC_PARAM_NAME = "actual_state_topic";
static const std::string STD_INIT_POSE_TOPIC_PARAM_NAME = "init_pose_topic";

// Threshold Values for the Laserscan
static const std::string STD_USE_OBSTACLE_DETECTION_PARAM_NAME = "use_obstacle_detection";
static const std::string STD_LASERSCAN_THRESHOLD_PARAM_NAME = "laserScan_threshold";
static const std::string STD_LASERSCAN_MINANGLE_PARAM_NAME = "laserScan_minAngle";
static const std::string STD_LASERSCAN_MAXANGLE_PARAM_NAME = "laserScan_maxAngle";
static const std::string STD_LASERSCAN_USE_ANGLE_LIMITATION_PARAM_NAME =
    "laseScan_use_angle_limitation";
static const bool STD_LASERSCAN_USE_ANGLE_LIMITATION_PARAM = false;

// Linear and angular Tolerance
static const std::string STD_LINEAR_TOLERANCE_PARAM_NAME = "linear_tolerance";
static const std::string STD_ANGULAR_TOLERANCE_PARAM_NAME = "angular_tolerance";

// Coefficients for the linear and angular velocity
static const std::string STD_GAIN_KP_PARAM_NAME = "gain_kp";
static const std::string STD_GAIN_KI_PARAM_NAME = "gain_ki";
static const std::string STD_GAIN_KD_PARAM_NAME = "gain_kd";

template <typename T>
T MarsSimulationCtvAgentNode::getParam(ros::NodeHandle& nH, const std::string& paramName)
{
  T value;
  if (nH.getParam(paramName, value))
  {
    ROS_INFO_STREAM("Found parameter: " << paramName << ", value: " << value);
    return value;
  }
  else
  {
    ROS_INFO_STREAM("Cannot find value for parameter: " << paramName);
    return 0;
  }
}

template <typename T>
void MarsSimulationCtvAgentNode::getParam(ros::NodeHandle& nH, const std::string& paramName,
                                          T& paramValue) noexcept(false)
{

  if (nH.getParam(paramName, paramValue))
  {
    ROS_INFO_STREAM("Found parameter: " << paramName);
    //<< ", value: " << paramValue);
  }
  else
  {
    throw mars::common::exception::ReadParamException("Could not read parameter: " + paramName);
  }
}

template <typename T>
void MarsSimulationCtvAgentNode::getParam(ros::NodeHandle& nH, const std::string& paramName,
                                          T& paramValue, const T& defaultParamValue) noexcept(false)
{

  if (nH.getParam(paramName, paramValue))
  {
    ROS_INFO_STREAM("Found parameter: " << paramName);
    //<< ", value: " << paramValue);
  }
  else
  {
    paramValue = defaultParamValue;
  }
}

double deg2rad(double deg) { return (deg * M_PI / 180.0); }

MarsSimulationCtvAgentNode::MarsSimulationCtvAgentNode() { this->mNHPriv = ros::NodeHandle("~"); }

MarsSimulationCtvAgentNode::~MarsSimulationCtvAgentNode() {}

bool MarsSimulationCtvAgentNode::init()
{
  bool initSuccessfully;

  // first read launch parameter
  initSuccessfully = this->readLaunchParams();

  // ... if no error occours initialize the publisher
  if (initSuccessfully)
  {
    initSuccessfully = this->initPublisher();
  }
  else
  {
    ROS_ERROR("[MarsSimulationCtvAgentNode::init] Error while reading launch "
              "file params "
              "-> shutting down node!");
  }

  // ... if no error occours initialize the subscriber
  if (initSuccessfully)
  {
    initSuccessfully = this->initSubscriber();
  }
  else
  {
    ROS_ERROR("[MarsSimulationCtvAgentNode::init] Error while setting up "
              "publisher -> "
              "shutting down node!");
  }

  // if no error occours initialize the dynamic reconfigure
  if (initSuccessfully)
  {
    initSuccessfully = this->initDynamicReconfigure();
  }
  else
  {
    ROS_ERROR("[MarsSimulationCtvAgentNode::init] Error while setting up "
              "subscriber -> "
              "shutting down node!");
  }

  // ... if no error occours initialize the timer
  if (initSuccessfully)
  {
    initSuccessfully = this->initTimer();
  }
  else
  {
    ROS_ERROR("[MarsSimulationCtvAgentNode::init] Error while setting up "
              "dynamic reconfigure -> "
              "shutting down node!");
  }

  if (!initSuccessfully)
  {
    ROS_ERROR("[MarsSimulationCtvAgentNode::init] Error while setting up "
              "timer -> "
              "shutting down node!");
  }

  return initSuccessfully;
}

bool MarsSimulationCtvAgentNode::initPublisher()
{
  bool initSuccessfully = true;

  // Cmd_vel
  this->mCmdVelPub = this->mNH.advertise<geometry_msgs::Twist>(this->mVelocityTopic, 10);
  this->mMoveBase.setVelocityPub(&mCmdVelPub);

  this->mCurrentMotionPub =
      this->mNH.advertise<mars_agent_physical_robot_msgs::Motion>(this->mCurrentMotionTopic, 10);
  this->mMoveBase.setCurrentMotionPub(&mCurrentMotionPub);

  // Battery State
  this->mBatteryStatePub =
      this->mNH.advertise<sensor_msgs::BatteryState>(this->mBatteryStateTopic, 10);
  this->mMoveBase.setBatteryStatePub(&mBatteryStatePub);

  // Robot State
  this->mRobotStatePub =
      this->mNH.advertise<industrial_msgs::RobotStatus>(this->mRobotStateTopic, 10);
  this->mMoveBase.setRobotStatePub(&mRobotStatePub);

  // Assignment State
  this->mAssignmentStatePub = this->mNH.advertise<mars_agent_physical_robot_msgs::AssignmentStatus>(
      this->mAssignmentStateTopic, 10);
  this->mMoveBase.setAssignmentStatePub(&mAssignmentStatePub);

  // Robot Description
  this->mRobotAgentPropertiesPub =
      this->mNH.advertise<mars_agent_physical_robot_msgs::RobotAgentProperties>(
          this->mRobotAgentPropertiesTopic, 10, true);
  this->mMoveBase.setRobotDescriptionPub(&mRobotAgentPropertiesPub);

  // Actual State Publisher
  this->mActualStatePub = this->mNH.advertise<mars_agent_physical_robot_msgs::ActualState>(
      this->mActualStateTopic, 10, true);
  this->mMoveBase.setActualStatePub(&mActualStatePub);

  return initSuccessfully;
}

bool MarsSimulationCtvAgentNode::initSubscriber()
{
  bool initSuccessfully = true;

  this->mMotionAssignmentSub =
      this->mNH.subscribe<mars_agent_physical_robot_msgs::MotionAssignment>(
          this->mMotionAssignmentTopic, 100,
          &mars::simulation::ctv::agent::MoveBaseSimple::addMotionToQueueCallback,
          &this->mMoveBase);

  this->mActionAssignmentSub =
      this->mNH.subscribe<mars_agent_physical_robot_msgs::ActionAssignment>(
          this->mActionAssignmentTopic, 100,
          &mars::simulation::ctv::agent::MoveBaseSimple::addActionToQueueCallback,
          &this->mMoveBase);

  this->mCancelTaskSub = this->mNH.subscribe<mars_agent_physical_robot_msgs::CancelTask>(
      this->mCancelTaskTopic, 100,
      &mars::simulation::ctv::agent::MoveBaseSimple::cancelTaskinQueueCallback, &this->mMoveBase);

  this->mScanSub = this->mNH.subscribe<sensor_msgs::LaserScan>(
      this->mScanTopic, 1, &mars::simulation::ctv::agent::MoveBaseSimple::scanReceivedCallback,
      &this->mMoveBase);

  this->mInitPoseSub = this->mNH.subscribe<geometry_msgs::Pose>(
      this->mInitPoseTopic, 10, &mars::simulation::ctv::agent::MoveBaseSimple::setInitPoseCallback,
      &this->mMoveBase);

  return initSuccessfully;
}

bool MarsSimulationCtvAgentNode::initDynamicReconfigure()
{
  bool initSuccesfully = true;
  dynamic_reconfigure::Server<mars_simulation_ctv_agent::dynamic_pidConfig>::CallbackType
      callbackType;
  callbackType = boost::bind(&MarsSimulationCtvAgentNode::dynamicReconfigureCallback, this, _1, _2);
  this->mServer.setCallback(callbackType);
  return initSuccesfully;
}

void MarsSimulationCtvAgentNode::dynamicReconfigureCallback(
    mars_simulation_ctv_agent::dynamic_pidConfig& config, uint32_t level)
{
  this->mMoveBase.setPIDParameter(config.gain_kp, config.gain_ki, config.gain_kd);
}

bool MarsSimulationCtvAgentNode::initTimer()
{
  bool initSuccesfully = true;

  this->mCurrentMotionTimer = this->mNH.createTimer(
      ros::Duration(1 / this->mCurMotionFrequency),
      &mars::simulation::ctv::agent::MoveBaseSimple::currentMotionTimerCallback, &this->mMoveBase);
  this->mBatteryStateTimer = this->mNH.createTimer(
      ros::Duration(1 / this->mBatteryStateFrequency),
      &mars::simulation::ctv::agent::MoveBaseSimple::batteryStateTimerCallback, &this->mMoveBase);
  this->mRobotStateTimer = this->mNH.createTimer(
      ros::Duration(1 / this->mRobotStateFrequency),
      &mars::simulation::ctv::agent::MoveBaseSimple::robotStateTimerCallback, &this->mMoveBase);

  return initSuccesfully;
}

bool MarsSimulationCtvAgentNode::readLaunchParams()
{
  // IF a needed parameter is not provided, set 'initSuccessfully' to false!
  bool initSuccessfully = true;
  mars::simulation::ctv::agent::Vehicle vehicle;
  geometry_msgs::PoseStamped initPose;
  geometry_msgs::PoseStamped zeroPose;

  // set node log level
  getParam(this->mNHPriv, STD_LOG_LEVEL_PARAM_NAME, this->mNodeLogLevel);
  this->setNodeLogLevel();
  // Node rate
  getParam(this->mNHPriv, STD_NODE_RATE_PARAM_NAME, this->mHz);
  this->mHz = (this->mHz == NODE_RATE_MAX_ROS_PARAM) ? NODE_RATE_MAX : this->mHz;

  // Set the Parameter of the Vehicle
  std::string robot_id;
  mars_agent_physical_robot_msgs::VehicleType vehicleType;
  double min_height;
  double max_height;
  double payload;
  double maxPosXVel;
  double maxNegXVel;
  double maxPosXAcc;
  double maxNegXAcc;
  double maxPosYVel;
  double maxNegYVel;
  double maxPosYAcc;
  double maxNegYAcc;
  double maxPosAngVel;
  double maxNegAngVel;
  double maxPosAngAcc;
  double maxNegAngAcc;
  double velocity_control_sensitivity;
  double turning_radius;
  double batt_capacity;
  double batt_max_voltage;
  std::string vendor;
  mars_agent_physical_robot_msgs::RobotAction action_capability;

  bool useInitPose;

  // TODO: Typs VehicleType and action_capability can not be fetched like this

  getParam(this->mNHPriv, STD_ROBOT_ID_PARAM_NAME, robot_id);
  // getParam(this->mNHPriv, STD_TYPE_PARAM_NAME, vehicleType.vehicle_type);
  getParam(this->mNHPriv, STD_MIN_HEIGHT_PARAM_NAME, min_height);
  getParam(this->mNHPriv, STD_MAX_HEIGHT_PARAM_NAME, max_height);
  getParam(this->mNHPriv, STD_PAYLOAD_PARAM_NAME, payload);
  getParam(this->mNHPriv, STD_MAX_POS_X_VEL_PARAM_NAME, maxPosXVel);
  getParam(this->mNHPriv, STD_MAX_NEG_X_VEL_PARAM_NAME, maxNegXVel);
  getParam(this->mNHPriv, STD_MAX_POS_X_ACC_PARAM_NAME, maxPosXAcc);
  getParam(this->mNHPriv, STD_MAX_NEG_X_ACC_PARAM_NAME, maxNegXAcc);
  getParam(this->mNHPriv, STD_MAX_POS_Y_VEL_PARAM_NAME, maxPosYVel);
  getParam(this->mNHPriv, STD_MAX_NEG_Y_VEL_PARAM_NAME, maxNegYVel);
  getParam(this->mNHPriv, STD_MAX_POS_Y_ACC_PARAM_NAME, maxPosYAcc);
  getParam(this->mNHPriv, STD_MAX_NEG_Y_ACC_PARAM_NAME, maxNegYAcc);
  getParam(this->mNHPriv, STD_MAX_POS_ANG_VEL_PARAM_NAME, maxPosAngVel);
  getParam(this->mNHPriv, STD_MAX_NEG_ANG_VEL_PARAM_NAME, maxNegAngVel);
  getParam(this->mNHPriv, STD_MAX_POS_ANG_ACC_PARAM_NAME, maxPosAngAcc);
  getParam(this->mNHPriv, STD_MAX_NEG_ANG_ACC_PARAM_NAME, maxNegAngAcc);
  getParam(this->mNHPriv, STD_VELOCITY_CONTROL_SENSITIVITY_PARAM_NAME,
           velocity_control_sensitivity);
  getParam(this->mNHPriv, STD_MIN_TURNING_RADIUS_PARAM_NAME, turning_radius);
  getParam(this->mNHPriv, STD_BATT_CAPACITY_PARAM_NAME, batt_capacity);
  getParam(this->mNHPriv, STD_BATT_MAX_VOLTAGE_PARAM_NAME, batt_max_voltage);
  getParam(this->mNHPriv, STD_VENDOR_PARAM_NAME, vendor);
  // getParam(this->mNHPriv, STD_ACTION_CAPABILITY_PARAM_NAME, action_capability);

  vehicle.setRobot_id(robot_id);
  vehicle.setType(vehicleType);
  vehicle.setMaxPosXVel(maxPosXVel);
  vehicle.setMaxNegXVel(maxNegXVel);
  vehicle.setMaxPosXAcc(maxPosXAcc);
  vehicle.setMaxNegXAcc(maxNegXAcc);
  vehicle.setMaxPosYVel(maxPosYVel);
  vehicle.setMaxNegYVel(maxNegYVel);
  vehicle.setMaxPosYAcc(maxPosYAcc);
  vehicle.setMaxNegYAcc(maxNegYAcc);
  vehicle.setMaxPosAngVel(maxPosAngVel);
  vehicle.setMaxNegAngVel(maxNegAngVel);
  vehicle.setMaxPosAngAcc(maxPosAngAcc);
  vehicle.setMaxNegAngAcc(maxNegAngAcc);
  vehicle.setVelocityControlSensitivity(velocity_control_sensitivity);
  vehicle.setTurningRadius(turning_radius);
  vehicle.setBattCapacity(batt_capacity);
  vehicle.setBattMaxVoltage(batt_max_voltage);
  vehicle.setVendor(vendor);
  vehicle.setActionCapability(action_capability);
  vehicle.setMinHeight(min_height);
  vehicle.setMaxHeight(max_height);

  geometry_msgs::PolygonStamped tempFootprint;
  geometry_msgs::Point32 tempPoint;
  std::vector<double> footprint_x;
  getParam(this->mNHPriv, STD_FOOTPRINT_X_PARAM_NAME, footprint_x);
  std::vector<double> footprint_y;
  getParam(this->mNHPriv, STD_FOOTPRINT_Y_PARAM_NAME, footprint_y);
  for (int i = 0; i < footprint_x.size(); i++)
  {
    tempPoint.x = footprint_x[i];
    tempPoint.y = footprint_y[i];
    tempPoint.z = 0;
    tempFootprint.polygon.points.push_back(tempPoint);
  }
  vehicle.setFootprint(tempFootprint);

  this->mMoveBase.setVehicle(vehicle);

  // Set Topics
  getParam(this->mNHPriv, STD_CANCEL_TASK_TOPIC_PARAM_NAME, this->mCancelTaskTopic);
  getParam(this->mNHPriv, STD_ACTION_ASSIGNMENT_TOPIC_PARAM_NAME, this->mActionAssignmentTopic);
  getParam(this->mNHPriv, STD_MOTION_ASSIGNMENT_TOPIC_PARAM_NAME, this->mMotionAssignmentTopic);
  getParam(this->mNHPriv, STD_ROBOT_DESCRIPTION_TOPIC_PARAM_NAME, this->mRobotAgentPropertiesTopic);
  getParam(this->mNHPriv, STD_ASSIGNMENT_STATE_TOPIC_PARAM_NAME, this->mAssignmentStateTopic);
  getParam(this->mNHPriv, STD_ROBOT_STATE_TOPIC_PARAM_NAME, this->mRobotStateTopic);
  getParam(this->mNHPriv, STD_BATTERY_STATE_TOPIC_PARAM_NAME, this->mBatteryStateTopic);
  getParam(this->mNHPriv, STD_CURRENT_MOTION_TOPIC_PARAM_NAME, this->mCurrentMotionTopic);
  getParam(this->mNHPriv, STD_CMD_VEL_TOPIC_PARAM_NAME, this->mVelocityTopic);
  getParam(this->mNHPriv, STD_SCAN_TOPIC_PARAM_NAME, this->mScanTopic);
  getParam(this->mNHPriv, STD_ACTUAL_STATE_TOPIC_PARAM_NAME, this->mActualStateTopic);
  getParam(this->mNHPriv, STD_INIT_POSE_TOPIC_PARAM_NAME, this->mInitPoseTopic);

  // Set Frame IDs
  std::string globalFrameId;
  std::string robotFrameId;
  getParam(this->mNHPriv, STD_GLOBAL_FRAME_ID_PARAM_NAME, globalFrameId);
  getParam(this->mNHPriv, STD_ROBOT_FRAME_ID_PARAM_NAME, robotFrameId);
  this->mMoveBase.setGlobalFrameId(globalFrameId);
  this->mMoveBase.setRobotFrameId(robotFrameId);

  // Set the Threshold Values for the Laserscan
  bool useObstacleDetection;
  float thresholdValue;
  float minAngle;
  float maxAngle;
  bool useAngleLimitation;
  getParam(this->mNHPriv, STD_USE_OBSTACLE_DETECTION_PARAM_NAME, useObstacleDetection);
  getParam(this->mNHPriv, STD_LASERSCAN_THRESHOLD_PARAM_NAME, thresholdValue);
  getParam(this->mNHPriv, STD_LASERSCAN_MINANGLE_PARAM_NAME, minAngle);
  getParam(this->mNHPriv, STD_LASERSCAN_MAXANGLE_PARAM_NAME, maxAngle);
  getParam(this->mNHPriv, STD_LASERSCAN_USE_ANGLE_LIMITATION_PARAM_NAME, useAngleLimitation, STD_LASERSCAN_USE_ANGLE_LIMITATION_PARAM);
  this->mMoveBase.setUseObstacleDetection(useObstacleDetection);
  this->mMoveBase.setLaserScanThresholdValue(thresholdValue);
  this->mMoveBase.setLaserScanMinAngle(minAngle);
  this->mMoveBase.setLaserScanMaxAngle(maxAngle);
  this->mMoveBase.setLaserScanUseAngleLimitation(useAngleLimitation);

  // Set the tolerance Values
  double linearTolerance;
  double angularTolerance;
  getParam(this->mNHPriv, STD_LINEAR_TOLERANCE_PARAM_NAME, linearTolerance);
  getParam(this->mNHPriv, STD_ANGULAR_TOLERANCE_PARAM_NAME, angularTolerance);
  this->mMoveBase.setLinearTolerance(linearTolerance);
  this->mMoveBase.setAngularTolerance(angularTolerance);

  // Set Durations for the Timers
  getParam(this->mNHPriv, STD_CURRENT_MOTION_MSG_FREQUENCY_PARAM_NAME, this->mCurMotionFrequency);
  getParam(this->mNHPriv, STD_BATTERY_STATE_MSG_FREQUENCY_PARAM_NAME, this->mBatteryStateFrequency);
  getParam(this->mNHPriv, STD_ROBOT_STATE_MSG_FREQUENCY_PARAM_NAME, this->mRobotStateFrequency);

  // Set the velocity coefficients
  double Gain_Kp;
  double Gain_Ki;
  double Gain_Kd;
  getParam(this->mNHPriv, STD_GAIN_KP_PARAM_NAME, Gain_Kp);
  getParam(this->mNHPriv, STD_GAIN_KI_PARAM_NAME, Gain_Ki);
  getParam(this->mNHPriv, STD_GAIN_KD_PARAM_NAME, Gain_Kd);
  this->mMoveBase.setGainKp(Gain_Kp);
  this->mMoveBase.setGainKi(Gain_Ki);
  this->mMoveBase.setGainKd(Gain_Kd);

  try
  {
    getParam(this->mNHPriv, STD_USE_INITIAL_POSE_PARAM_NAME, useInitPose);
  }
  catch (mars::common::exception::ReadParamException e)
  {
    useInitPose = STD_USE_INITIAL_POSE_PARAM;
  }

  if (useInitPose)
  {
    // Set initPose position
    getParam(this->mNHPriv, STD_INIT_POSE_X_PARAM_NAME, initPose.pose.position.x);
    getParam(this->mNHPriv, STD_INIT_POSE_Y_PARAM_NAME, initPose.pose.position.y);
    initPose.pose.position.z = 0.0;

    // Set initPose orientation
    initPose.pose.orientation.x = 0.0;
    initPose.pose.orientation.y = 0.0;
    double poseOrientationZdegree;
    double poseOrientationWdegree;
    getParam(this->mNHPriv, STD_INIT_POSE_PHI_PARAM_NAME, poseOrientationZdegree);
    getParam(this->mNHPriv, STD_INIT_POSE_PHI_PARAM_NAME, poseOrientationWdegree);
    initPose.pose.orientation.z = sin(deg2rad(poseOrientationZdegree / 2.0));
    initPose.pose.orientation.w = cos(deg2rad(poseOrientationWdegree / 2.0));

    // set header
    initPose.header.frame_id = this->mMoveBase.getRobotFrameId();
    initPose.header.stamp = ros::Time::now();

    // set initPose
    this->mMoveBase.setInitPose(initPose);
  }
  else
  {
    this->mMoveBase.skipInitialPose();
  }

  // set zero pose
  zeroPose.pose.position.x = 0.0;
  zeroPose.pose.position.y = 0.0;
  zeroPose.pose.position.z = 0.0;
  zeroPose.pose.orientation.x = 0.0;
  zeroPose.pose.orientation.y = 0.0;
  zeroPose.pose.orientation.z = 0.0;
  zeroPose.pose.orientation.w = 1.0;
  zeroPose.header.frame_id = this->mMoveBase.getRobotFrameId();
  zeroPose.header.stamp = ros::Time::now();
  this->mMoveBase.setZeroPose(zeroPose);

  return initSuccessfully;
}

bool MarsSimulationCtvAgentNode::setNodeLogLevel() const
{
  ros::console::Level nodeLogLevel;
  bool setLogLevelSuccessfully = true;

  if (this->mNodeLogLevel == LOG_LEVEL_DEBUG)
  {
    nodeLogLevel = ros::console::levels::Debug;
  }
  else if (this->mNodeLogLevel == LOG_LEVEL_INFO)
  {
    nodeLogLevel = ros::console::levels::Info;
  }
  else if (this->mNodeLogLevel == LOG_LEVEL_WARN)
  {
    nodeLogLevel = ros::console::levels::Warn;
  }
  else if (this->mNodeLogLevel == LOG_LEVEL_ERROR)
  {
    nodeLogLevel = ros::console::levels::Error;
  }
  else if (this->mNodeLogLevel == LOG_LEVEL_FATAL)
  {
    nodeLogLevel = ros::console::levels::Fatal;
  }
  else
  {
    ROS_WARN_STREAM("[MarsSimulationCtvAgentNode::setNodeLogLevel] Wrong log level was "
                    "set in launch file! Level was '"
                    << this->mNodeLogLevel << "' but must be '" << LOG_LEVEL_DEBUG << "', '"
                    << LOG_LEVEL_INFO << "', '" << LOG_LEVEL_WARN << LOG_LEVEL_ERROR << "' or '"
                    << LOG_LEVEL_FATAL);

    setLogLevelSuccessfully = false;
  }

  // set node log level
  if (setLogLevelSuccessfully &&
      ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, nodeLogLevel))
  {
    ros::console::notifyLoggerLevelsChanged();
  }
  else
  {
    ROS_WARN_STREAM("[MarsSimulationCtvAgentNode::setNodeLogLevel] Can not set ros logger "
                    "level with level: "
                    << this->mNodeLogLevel);

    setLogLevelSuccessfully = false;
  }

  return setLogLevelSuccessfully;
}

void MarsSimulationCtvAgentNode::rosMainLoop()
{
  ros::Rate rate(this->mHz);
  ROS_INFO("[MarsSimulationCtvAgentNode::rosMainLoop] Node successfully "
           "initialized. Starting node now!");

  while (ros::ok())
  {
    // State machine
    this->mMoveBase.driveToGoal();
    ros::spinOnce();
    rate.sleep();
  }

  // Stop the node's resources
  ros::shutdown();
}

bool MarsSimulationCtvAgentNode::runROSNode()
{
  bool runNodeSuccessfully = true;

  if (this->init())
  {
    this->rosMainLoop();
  }
  else
  {
    runNodeSuccessfully = false;
  }

  return runNodeSuccessfully;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, ros::this_node::getName());

  MarsSimulationCtvAgentNode node;

  return node.runROSNode() ? EXIT_SUCCESS : EXIT_FAILURE;
}
