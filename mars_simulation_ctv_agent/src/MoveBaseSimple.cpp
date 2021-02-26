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

#include "mars_simulation_ctv_agent/MoveBaseSimple.h"

const double mars::simulation::ctv::agent::MoveBaseSimple::MoveBaseSimple::STD_INVALID_POSE_X =
    -std::numeric_limits<double>::max();
const double mars::simulation::ctv::agent::MoveBaseSimple::MoveBaseSimple::STD_INVALID_POSE_Y =
    -std::numeric_limits<double>::max();
const double mars::simulation::ctv::agent::MoveBaseSimple::MoveBaseSimple::STD_INVALID_POSE_PHI =
    0.0;
static const double STD_OBSTACLE_DETECTED_PERCENT_THRESHOLD = 0.05;

mars::simulation::ctv::agent::MoveBaseSimple::MoveBaseSimple()
    : nh(), private_nh("~"), mState(MBS_NO_POSITION), mGotInitPose(false)
{
  this->mSkipInitialPoseCheck = false;
  this->mGotGlobalPose = false;
}

void mars::simulation::ctv::agent::MoveBaseSimple::setVehicle(const Vehicle& vehicle)
{
  mVehicle = vehicle;
}

void mars::simulation::ctv::agent::MoveBaseSimple::setActualStatePub(ros::Publisher* actualStatePub)
{
  mActualStatePub = actualStatePub;
}

void mars::simulation::ctv::agent::MoveBaseSimple::setVelocityPub(ros::Publisher* velocityPub)
{
  mVelocityPub = velocityPub;
}

void mars::simulation::ctv::agent::MoveBaseSimple::setCurrentMotionPub(ros::Publisher* curMotionPub)
{
  mCurMotionPub = curMotionPub;
}

void mars::simulation::ctv::agent::MoveBaseSimple::setBatteryStatePub(ros::Publisher* batteryState)
{
  mBatteryStatePub = batteryState;
}

void mars::simulation::ctv::agent::MoveBaseSimple::setRobotStatePub(ros::Publisher* robotState)
{
  mRobotStatePub = robotState;
}

void mars::simulation::ctv::agent::MoveBaseSimple::setAssignmentStatePub(
    ros::Publisher* assignmentState)
{
  mAssignmentStatePub = assignmentState;
}

void mars::simulation::ctv::agent::MoveBaseSimple::setRobotDescriptionPub( // robotAgentProperties
                                                                           // message
    ros::Publisher* robotDescription)
{
  mRobotDescriptionPub = robotDescription;
}

void mars::simulation::ctv::agent::MoveBaseSimple::setInitPose(
    const geometry_msgs::PoseStamped& pPose)
{
  if (!this->mGotInitPose)
  {
    if ((pPose.pose.position.x != MoveBaseSimple::STD_INVALID_POSE_X) &&
        (pPose.pose.position.y != MoveBaseSimple::STD_INVALID_POSE_Y))
    {
      this->mInitPose = pPose;
      // Set state to initialized
      this->mState = MBS_WAITING_FOR_INIT;

      // Set state to initialized
      this->setGotInitPose(true);
    }
    else
    {
      ROS_ERROR("[mars::simulation::ctv::agent::MoveBaseSimple::setInitPose] Tryed to set invalied "
                "pose, pose was ignored!");
    }
  }
  else
  {
    ROS_WARN("[mars::simulation::ctv::agent::MoveBaseSimple::setInitPose] Can't set init pose, "
             "robot already initialized.");
  }
}

void mars::simulation::ctv::agent::MoveBaseSimple::skipInitialPose()
{
  this->mSkipInitialPoseCheck = true;
  this->mState = MBS_WAITING_FOR_INIT;

  ROS_DEBUG("Skipping initial pose check!");
}

void mars::simulation::ctv::agent::MoveBaseSimple::setInitPoseCallback(
    const geometry_msgs::PoseConstPtr& msg)
{
  geometry_msgs::PoseStamped lTmpPoseStamped;

  lTmpPoseStamped.pose = *msg;

  this->setInitPose(lTmpPoseStamped);
}

void mars::simulation::ctv::agent::MoveBaseSimple::currentMotionTimerCallback(
    const ros::TimerEvent& event)
{
  mars_agent_physical_robot_msgs::Motion msg;
  msg.current_position = this->mVehicle.getCurrentPose();

  msg.current_velocity.linear.x = this->mVehicle.getCurrentLinearVel();
  msg.current_velocity.linear.y = 0;
  msg.current_velocity.linear.z = 0;
  msg.current_velocity.angular.x = 0;
  msg.current_velocity.angular.y = 0;
  msg.current_velocity.angular.z = this->mVehicle.getCurrentAngularVel();

  this->mCurMotionPub->publish(msg);
}

void mars::simulation::ctv::agent::MoveBaseSimple::batteryStateTimerCallback(
    const ros::TimerEvent& event)
{
  sensor_msgs::BatteryState msg;

  this->mBatteryStatePub->publish(msg);
}

void mars::simulation::ctv::agent::MoveBaseSimple::robotStateTimerCallback(
    const ros::TimerEvent& event)
{
  // TODO: fill the msg
  industrial_msgs::RobotStatus msg;

  this->mRobotStatePub->publish(msg);
}

void mars::simulation::ctv::agent::MoveBaseSimple::addMotionToQueueCallback(
    const mars_agent_physical_robot_msgs::MotionAssignment::ConstPtr& msg)
{
  // save the new goal in our FIFO
  this->mTaskManager.addAssignment(msg);
  ROS_DEBUG("Received new MotionAssignment");
}

void mars::simulation::ctv::agent::MoveBaseSimple::addActionToQueueCallback(
    const mars_agent_physical_robot_msgs::ActionAssignmentConstPtr& msg)
{
  // save the new Action in the TaskManager
  this->mTaskManager.addAssignment(msg);
  ROS_DEBUG("Received new ActionAssignment");
}

void mars::simulation::ctv::agent::MoveBaseSimple::cancelTaskinQueueCallback(
    const mars_agent_physical_robot_msgs::CancelTaskConstPtr& msg)
{
  // Cancel/delete Task in Queue
  this->mTaskManager.deleteTask(msg);
  ROS_DEBUG("Cancel Task in Queue");
}

void mars::simulation::ctv::agent::MoveBaseSimple::setPIDParameter(double gainKp, double gainKi,
                                                                   double gainKd)
{
  this->mGainKp = gainKp;
  this->mGainKi = gainKi;
  this->mGainKd = gainKd;
}

void mars::simulation::ctv::agent::MoveBaseSimple::updateGlobalVehiclePose()
{
  geometry_msgs::PoseStamped robotPoseGlobal;

  try
  {
    this->mTfListener.waitForTransform(this->getGlobalFrameId(), this->getRobotFrameId(),
                                       ros::Time(0), ros::Duration(5.0));
    this->mTfListener.transformPose(this->getGlobalFrameId(), ros::Time(0), mZeroPose,
                                    this->getRobotFrameId(), robotPoseGlobal);

    this->mGotGlobalPose = true;
  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN("Failed to find robot pose (%s) in global frame %s - ROBOT WILL STOPP",
             getRobotFrameId().c_str(), getGlobalFrameId().c_str());

    this->mState = MBS_WAITING_FOR_INIT;
  }
  this->mVehicle.setCurrentPose(robotPoseGlobal);
}

void mars::simulation::ctv::agent::MoveBaseSimple::duringMbsNoPosition()
{
  if (this->mGotInitPose)
  {
    this->mState = MBS_WAITING_FOR_INIT;
    ROS_DEBUG("Received initial pose!");
  }
  else
  {
    ROS_DEBUG("Waiting for initial pose!");
  }
}

void mars::simulation::ctv::agent::MoveBaseSimple::duringMbsWaitForInit()
{
  // At start time there is no actual assignment that can be straight
  this->mAssignmentStraight = false;
  this->mLastMotionAssignment = nullptr;
  // Publish the RobotAgentProperties
  this->robotAgentPropertiesPublisher();

  if (!this->mSkipInitialPoseCheck)
  {
    double distanceToInitpose =
        sqrt(pow(this->mVehicle.getCurrentCordX() - this->mInitPose.pose.position.x, 2) +
             pow(this->mVehicle.getCurrentCordY() - this->mInitPose.pose.position.y, 2));
    if (distanceToInitpose < 0.15)
    {
      ROS_DEBUG("Initial pose of robot successfully set!");
      this->mState = MBS_READY;
    }
    else
    {
      ROS_ERROR("Initial pose is to far away from robot world position in /map!");
    }
  }
  else
  {
    if (this->mGotGlobalPose)
    {
      ROS_DEBUG("Initial pose of robot successfully set!");
      this->mState = MBS_READY;
    }
    else
    {
      ROS_DEBUG("Waiting for global pose!");
    }
  }
}

void mars::simulation::ctv::agent::MoveBaseSimple::duringMbsReady()
{
  // Check if we have a task
  if (this->mTaskManager.isTaskAvailable())
  {
    // Check if the next assignment is ready
    if (this->mTaskManager.isNextAssignmentAvailable())
    {
      this->mState = MBS_FETCH_NEXT_INSTRUCTION;
    }
  }
}

void mars::simulation::ctv::agent::MoveBaseSimple::duringMbsFirstRotation()
{
  double lTargetOrientation =
      atan2(this->mCurrentMotionAssignment->getGoalPose().y - this->mVehicle.getCurrentCordY(),
            this->mCurrentMotionAssignment->getGoalPose().x - this->mVehicle.getCurrentCordX());
  double lAngularDifference =
      angles::shortest_angular_distance(lTargetOrientation, this->mVehicle.getCurrentOrientation());
  double lCurrentAngVel;
  double lDeltaVel;
  double lAngularDeccelerationDifference =
      pow(this->mVehicle.getCurrentAngularVel(), 2) / (2 * this->mVehicle.getMaxNegAngAcc());
  double lLoopDuration = (this->mActualTime - this->mLastTime).toSec();

  if (lAngularDifference > 0)
  {
    // Because the angular difference is >0 the angular velocity have to be <0
    if (lAngularDeccelerationDifference < lAngularDifference)
    {
      lDeltaVel = -this->mMaxPosAngAcc * lLoopDuration;
      lCurrentAngVel = this->mVehicle.getCurrentAngularVel() + lDeltaVel;
      if (lCurrentAngVel < this->mMaxNegAngVel)
      {
        lCurrentAngVel = -this->mMaxNegAngVel;
      }
    }
    else
    {
      lDeltaVel = this->mVehicle.getMaxNegAngAcc() * lLoopDuration;
      lCurrentAngVel = this->mVehicle.getCurrentAngularVel() + lDeltaVel;
      if (lCurrentAngVel > 0)
      {
        lCurrentAngVel = 0.0;
      }
    }
  }
  else
  {
    if (lAngularDeccelerationDifference < fabs(lAngularDifference))
    {
      lDeltaVel = this->mMaxPosAngAcc * lLoopDuration;
      lCurrentAngVel = this->mVehicle.getCurrentAngularVel() + lDeltaVel;
      if (lCurrentAngVel > this->mMaxPosAngVel)
      {
        lCurrentAngVel = this->mMaxPosAngVel;
      }
    }
    else
    {
      lDeltaVel = this->mVehicle.getMaxNegAngAcc() * lLoopDuration;
      lCurrentAngVel = this->mVehicle.getCurrentAngularVel() - lDeltaVel;
      if (lCurrentAngVel < 0)
      {
        lCurrentAngVel = 0.0;
      }
    }
  }
  this->mVehicle.setCurrentLinearVel(0.0);
  this->mVehicle.setCurrentAngularVel(lCurrentAngVel);

  // check if difference is below threshold: if yes, then stop robot and go in state MBS_Moving
  if (fabs(lAngularDifference) < this->mAngularTolerance)
  {
    this->mAssignmentStraight = this->isNextAssignmentStraight();
    if (this->mAssignmentStraight)
    {
      this->setMaxVelocity();
    }
    stopVehicle();
    this->mState = MBS_MOVING;
  }
}

void mars::simulation::ctv::agent::MoveBaseSimple::duringMbsMoving()
{
  double lDistanceToGoal;
  double lCurrentVel;
  double lDeltaVel;
  double lDeccelerationDistance;
  double lLoopDuration;
  double lTargetOrientationX;
  double lTargetOrientationY;
  double lTargetOrientation;
  double lMaxAngularVel;
  double lAngVelP;
  double lAngVelI;
  double lAngVelD;
  double lAngularVel;

  lDistanceToGoal = sqrt(
      pow(this->mVehicle.getCurrentCordX() - this->mCurrentMotionAssignment->getGoalPose().x, 2) +
      pow(this->mVehicle.getCurrentCordY() - this->mCurrentMotionAssignment->getGoalPose().y, 2));
  this->mVehicle.setCurrentDistanceToGoal(lDistanceToGoal);

  // If the current Linear Velocity is higher than the max linear velocity of the next motion
  // Assignment, we have to calculate the decceleration distance to slowdown early
  if (this->mVehicle.getCurrentLinearVel() >= this->mNextMaxPosXVel)
  {
    lDeccelerationDistance =
        pow((this->mVehicle.getCurrentLinearVel() - this->mNextMaxPosXVel), 2) /
        (2 * this->mVehicle.getMaxNegXAcc());
  }
  else
  {
    lDeccelerationDistance = 0;
  }

  lLoopDuration = (this->mActualTime - this->mLastTime).toSec();
  if (lDeccelerationDistance < lDistanceToGoal)
  {
    lDeltaVel = this->mMaxPosXAcc * lLoopDuration;
    lCurrentVel = this->mVehicle.getCurrentLinearVel() + lDeltaVel;
    if (lCurrentVel > this->mMaxPosXVel)
    {
      lCurrentVel = this->mMaxPosXVel;
    }
  }
  else
  {
    lDeltaVel = this->mVehicle.getMaxNegXAcc() * 2 *
                lLoopDuration; // Go one iteration in the future with the velocity planning because
                               // the vehicle drive otherwise a bit too far.
    lCurrentVel = this->mVehicle.getCurrentLinearVel() - lDeltaVel;
    if (lCurrentVel < 0)
    {
      lCurrentVel = 0;
    }
  }
  this->mVehicle.setCurrentLinearVel(lCurrentVel);

  // Calculate angular error to adapt the angular velocity with the lateral control
  if (!(this->mAssignmentStraight))
  {
    lTargetOrientationX =
        this->mCurrentMotionAssignment->getGoalPose().x - this->mVehicle.getCurrentCordX();
    lTargetOrientationY =
        this->mCurrentMotionAssignment->getGoalPose().y - this->mVehicle.getCurrentCordY();
  }
  else
  {
    // If the next Assignment is straight ahead we use the next Goal for the calculation of the
    // angular velocity
    lTargetOrientationY =
        this->mNextMotionAssignment->getGoalPose().y - this->mVehicle.getCurrentCordY();
    lTargetOrientationX =
        this->mNextMotionAssignment->getGoalPose().x - this->mVehicle.getCurrentCordX();
  }

  lTargetOrientation = atan2(lTargetOrientationY, lTargetOrientationX);
  this->mLastsAngularError = this->mAngularError;
  this->mAngularError =
      angles::shortest_angular_distance(lTargetOrientation, this->mVehicle.getCurrentOrientation());
  this->mAngularErrorSum = this->mAngularErrorSum + this->mAngularError;

  // Calculate the Controlparameters for the lateral control
  lAngVelP = mGainKp * this->mAngularError;
  lAngVelI = mGainKi * lLoopDuration * this->mAngularErrorSum;
  lAngVelD = mGainKd * (this->mAngularError - this->mLastsAngularError) / lLoopDuration;
  lAngularVel = lAngVelP + lAngVelI + lAngVelD;

  if (-lAngularVel <= this->mMaxPosAngVel && -lAngularVel >= -this->mMaxPosAngVel)
  {
    this->mVehicle.setCurrentAngularVel(-lAngularVel);
  }
  else
  {
    // If the angular velocity is not inside the constraints the angular velocity is on the max or
    // the minimum.
    if (-lAngularVel > 0)
    {
      this->mVehicle.setCurrentAngularVel(this->mMaxPosAngVel);
    }
    else
    {
      this->mVehicle.setCurrentAngularVel(-this->mMaxPosAngVel);
    }
  }

  // check if the robot is close enough to the goal position
  if (this->mAssignmentStraight)
  { // With Goaldiversity
    if (this->reachedGoalDiversity())
    {
      this->mSkippedAssignments.push_back(this->mCurrentMotionAssignment);
      this->mState = MBS_FINISHED;
    }
  }
  else
  {
    if (this->mVehicle.getCurrentDistanceToGoal() <= this->mLinearTolerance)
    {
      stopVehicle();
      this->mState = MBS_SECOND_ROTATION;
    }
  }
}

void mars::simulation::ctv::agent::MoveBaseSimple::duringMbsSecondRotation()
{
  // compute difference between current and goal orientation
  double lGoalOrientation = this->mCurrentMotionAssignment->getGoalPose().theta;
  double lAngularDifference =
      angles::shortest_angular_distance(lGoalOrientation, this->mVehicle.getCurrentOrientation());
  double lCurrentAngVel;
  double lDeltaVel;
  double lAngularDeccelerationDifference =
      pow(this->mVehicle.getCurrentAngularVel(), 2) / (2 * this->mVehicle.getMaxNegAngAcc());
  double lLoopDuration = (this->mActualTime - this->mLastTime).toSec();

  if (lAngularDifference > 0)
  {
    // Because the angular difference is >0 the angular velocity have to be <0
    if (lAngularDeccelerationDifference < lAngularDifference)
    {
      lDeltaVel = -this->mMaxPosAngAcc * lLoopDuration;
      lCurrentAngVel = this->mVehicle.getCurrentAngularVel() + lDeltaVel;
      if (lCurrentAngVel < this->mMaxNegAngVel)
      {
        lCurrentAngVel = -this->mMaxNegAngVel;
      }
    }
    else
    {
      lDeltaVel = this->mVehicle.getMaxNegAngAcc() * lLoopDuration;
      lCurrentAngVel = this->mVehicle.getCurrentAngularVel() + lDeltaVel;
      if (lCurrentAngVel > 0)
      {
        lCurrentAngVel = 0.0;
      }
    }
  }
  else
  {
    if (lAngularDeccelerationDifference < fabs(lAngularDifference))
    {
      lDeltaVel = this->mMaxPosAngAcc * lLoopDuration;
      lCurrentAngVel = this->mVehicle.getCurrentAngularVel() + lDeltaVel;
      if (lCurrentAngVel > this->mMaxPosAngVel)
      {
        lCurrentAngVel = this->mMaxPosAngVel;
      }
    }
    else
    {
      lDeltaVel = this->mVehicle.getMaxNegAngAcc() * lLoopDuration;
      lCurrentAngVel = this->mVehicle.getCurrentAngularVel() - lDeltaVel;
      if (lCurrentAngVel < 0)
      {
        lCurrentAngVel = 0.0;
      }
    }
  }
  this->mVehicle.setCurrentLinearVel(0.0);
  this->mVehicle.setCurrentAngularVel(lCurrentAngVel);

  // check if difference is below threshold: if yes, then stop robot and state
  // machine is finished
  if (fabs(lAngularDifference) < this->mAngularTolerance)
  {
    ROS_DEBUG("GOAL REACHED WITH (%f) M AND (%f) RAD ERROR",
              fabs(this->mVehicle.getCurrentDistanceToGoal()), fabs(lAngularDifference));
    stopVehicle();
    this->mState = MBS_FINISHED;
    return;
  }

  // If we ignore the orientation at the goal, stop vehicle
  if (!(this->mCurrentMotionAssignment->getUseOrientation()))
  {
    ROS_DEBUG("GOAL REACHED WITH (%f) M ERROR", fabs(this->mVehicle.getCurrentDistanceToGoal()));
    stopVehicle();
    this->mState = MBS_FINISHED;
    return;
  }
}

void mars::simulation::ctv::agent::MoveBaseSimple::duringMbsFinished()
{
  this->mState = MBS_READY;
  // publish last task in the taskmanager when its done

  if (this->mTaskManager.isCurrentTaskDone() || !(this->mTaskManager.isNextAssignmentAvailable()))
  {
    this->mTaskManager.updateLastFinishedAssignmentIds();
    this->assignmentStatusPublisher();
  }
}

void mars::simulation::ctv::agent::MoveBaseSimple::duringMbsFetchNextInstruction()
{
  // Set the error to zero when changing the assignment
  this->mAngularError = 0;
  this->mLastsAngularError = 0;
  this->mAngularErrorSum = 0;

  if (!(this->mAssignmentStraight))
  {
    this->mTaskManager.updateLastFinishedAssignmentIds();
  }
  this->mCurrentAssignment = this->mTaskManager.getNextAssignment();
  this->mLastMotionAssignment = this->mCurrentMotionAssignment;

  // Check if the assignment is an Action or a Motion
  this->mCurrentMotionAssignment =
      std::dynamic_pointer_cast<mars::simulation::ctv::agent::MotionAssignment>(
          this->mCurrentAssignment);
  this->mCurrentActionAssignment =
      std::dynamic_pointer_cast<mars::simulation::ctv::agent::ActionAssignment>(
          this->mCurrentAssignment);

  // Get the next Assignment
  std::shared_ptr<Assignment> nextAssignment = this->mTaskManager.getNextNextAssignment();
  this->mNextMotionAssignment =
      std::dynamic_pointer_cast<mars::simulation::ctv::agent::MotionAssignment>(nextAssignment);

  // publish message with current and last ids
  this->assignmentStatusPublisher();

  this->mNextAssignmentStraight = this->mAssignmentStraight;
  this->mAssignmentStraight = this->isNextAssignmentStraight();
  if (this->mCurrentMotionAssignment != nullptr)
  {
    if (this->mNextAssignmentStraight)
    {
      this->mState = MBS_MOVING;
      this->setMaxVelocity();
      this->setMaxAcceleration();
    }
    else
    {
      this->mState = MBS_FIRST_ROTATION;
      this->setMaxVelocity();
      this->setMaxAcceleration();
    }
  }
  else if (this->mCurrentActionAssignment != nullptr)
  {
    this->mState = MBS_EXECUTE_ACTION;
  }
}

void mars::simulation::ctv::agent::MoveBaseSimple::duringMbsExecuteAction()
{
  ROS_INFO("Execute Action Message");
  // For Simulation wait 5 Seconds
  ros::Duration(5.0).sleep();
  this->mState = MBS_FINISHED;
}

void mars::simulation::ctv::agent::MoveBaseSimple::duringMbsObstacle()
{
  if (this->mObstacleDetected)
  {
    stopVehicle();
  }
  else
  {
    this->mState = this->mHistoryState;
  }
}

void mars::simulation::ctv::agent::MoveBaseSimple::duringMbsError()
{
  if (this->mTaskManager.isNextAssignmentAvailable())
  {
    this->mState = MBS_FETCH_NEXT_INSTRUCTION;
  }
}

void mars::simulation::ctv::agent::MoveBaseSimple::driveToGoal()
{
  this->mLastTime = this->mActualTime;
  this->mActualTime =
      ros::Time(std::chrono::system_clock::now().time_since_epoch().count() / 1000000000.0);
  // Get current position
  updateGlobalVehiclePose();

  // Move Base Simple State Machine
  switch (this->mState)
  {
  case MBS_NO_POSITION:
    duringMbsNoPosition();
    break;

  case MBS_WAITING_FOR_INIT:
    duringMbsWaitForInit();
    break;

  case MBS_READY:
    duringMbsReady();
    break;

  case MBS_FIRST_ROTATION:
    duringMbsFirstRotation();
    break;

  case MBS_MOVING:
    duringMbsMoving();
    break;

  case MBS_SECOND_ROTATION:
    duringMbsSecondRotation();
    break;

  case MBS_FINISHED:
    duringMbsFinished();
    break;

  case MBS_FETCH_NEXT_INSTRUCTION:
    duringMbsFetchNextInstruction();
    break;

  case MBS_EXECUTE_ACTION:
    duringMbsExecuteAction();
    break;

  case MBS_OBSTACLE:
    duringMbsObstacle();
    break;
  }

  // Check if the Robot Skipped one Assignment and if it is finished
  if (!this->mSkippedAssignments.empty())
  {
    double distanceToSkippedGoal =
        sqrt(pow(this->mVehicle.getCurrentCordX() -
                     (*(this->mSkippedAssignments.begin()))->getGoalPose().x,
                 2) +
             pow(this->mVehicle.getCurrentCordY() -
                     (*(this->mSkippedAssignments.begin()))->getGoalPose().y,
                 2));
    if (distanceToSkippedGoal <= 5 * this->mLinearTolerance)
    { // The Tolerance have to be greater because the vehicle do not reach the exact goal point.
      this->mTaskManager.updateLastFinishedMotionAssignmentIds(
          (*(this->mSkippedAssignments.begin()))->getMotionId());
      ROS_DEBUG("GOAL REACHED WITH (%f) M ERROR", distanceToSkippedGoal);
      // publish message with current and last ids
      this->assignmentStatusPublisher();
      this->mSkippedAssignments.pop_front();
    }
  }

  // Publish the actual State
  this->actualStatePublisher();
  // Send the new velocities to the robot
  this->cmd_velPublisher();
}
void mars::simulation::ctv::agent::MoveBaseSimple::setUseObstacleDetection(
    bool useObstacleDetection)
{
  this->mUseObstacleDetection = useObstacleDetection;
}

void mars::simulation::ctv::agent::MoveBaseSimple::setLaserScanThresholdValue(float thresholdValue)
{
  this->mLaserScanThresholdValue = thresholdValue;
}

void mars::simulation::ctv::agent::MoveBaseSimple::setLaserScanMinAngle(float minAngle)
{
  this->mLaserScanMinAngle = minAngle;
}

void mars::simulation::ctv::agent::MoveBaseSimple::setLaserScanMaxAngle(float maxAngle)
{
  this->mLaserScanMaxAngle = maxAngle;
}

void mars::simulation::ctv::agent::MoveBaseSimple::setLaserScanUseAngleLimitation(
    bool useAngleLimitation)
{
  this->mLaserScanUseAngleLimitation = useAngleLimitation;
}

void mars::simulation::ctv::agent::MoveBaseSimple::setAngularTolerance(double angularTolerance)
{
  this->mAngularTolerance = angularTolerance;
}

void mars::simulation::ctv::agent::MoveBaseSimple::setLinearTolerance(double linearTolerance)
{
  mLinearTolerance = linearTolerance;
}

void mars::simulation::ctv::agent::MoveBaseSimple::setGainKp(double value)
{
  this->mGainKp = value;
}

void mars::simulation::ctv::agent::MoveBaseSimple::setGainKi(double value)
{
  this->mGainKi = value;
}

void mars::simulation::ctv::agent::MoveBaseSimple::setGainKd(double value)
{
  this->mGainKd = value;
}

void mars::simulation::ctv::agent::MoveBaseSimple::setGotInitPose(bool gotInitPose)
{
  mGotInitPose = gotInitPose;
}

void mars::simulation::ctv::agent::MoveBaseSimple::setZeroPose(
    const geometry_msgs::PoseStamped& msg)
{
  this->mZeroPose = msg;
}

void mars::simulation::ctv::agent::MoveBaseSimple::scanReceivedCallback(
    const sensor_msgs::LaserScan::ConstPtr& msg)
{
  if (this->mUseObstacleDetection)
  {
    switch (this->mState)
    {
    case MBS_MOVING:
      scanWhileMoving(msg);
      break;
    case MBS_FIRST_ROTATION:
      scanWhileRotation(msg);
      break;
    case MBS_SECOND_ROTATION:
      scanWhileRotation(msg);
      break;
    case MBS_OBSTACLE:
      if (this->mHistoryState == MBS_MOVING)
      {
        scanWhileMoving(msg);
      }
      else
      {
        scanWhileRotation(msg);
      }
      break;
    default:
      break;
    }
  }
}

void mars::simulation::ctv::agent::MoveBaseSimple::scanWhileMoving(
    const sensor_msgs::LaserScan::ConstPtr& msg)
{
  // Calculate Min and Max angle
  int lAngleMinIndex;
  int lAngleMaxIndex;
  // check whether breaking is inside laser scan threshold possible. If not, increase laser scan
  // threshold.
  double lLaserScanThresholdValue =
      (pow(this->mVehicle.getCurrentLinearVel(), 2) / (2 * this->mVehicle.getMaxNegXAcc()) >
       this->mLaserScanThresholdValue)
          ? pow(this->mVehicle.getCurrentLinearVel(), 2) / (2 * this->mVehicle.getMaxNegXAcc())
          : this->mLaserScanThresholdValue;
  int lNumberOfDetections = 0;

  this->determinMinMaxLaserScanArrayIndices(msg, lAngleMinIndex, lAngleMaxIndex);
  lNumberOfDetections =
      this->checkScanForObstacles(msg, lAngleMinIndex, lAngleMaxIndex, lLaserScanThresholdValue);
  this->mObstacleDetected =
      this->checkDetectedObstacleThreshold(lNumberOfDetections, msg->ranges.size());
}

void mars::simulation::ctv::agent::MoveBaseSimple::scanWhileRotation(
    const sensor_msgs::LaserScan::ConstPtr& msg)
{
  // Calculate Min and Max angle
  int lAngleMinIndex;
  int lAngleMaxIndex;
  double lLaserScanThresholdValue =
      (pow(this->mVehicle.getCurrentAngularVel(), 2) / (2 * this->mVehicle.getMaxNegAngAcc()) >
       this->mLaserScanThresholdValue)
          ? pow(this->mVehicle.getCurrentAngularVel(), 2) / (2 * this->mVehicle.getMaxNegAngAcc())
          : this->mLaserScanThresholdValue;
  int lNumberOfDetections = 0;

  this->determinMinMaxLaserScanArrayIndices(msg, lAngleMinIndex, lAngleMaxIndex);
  lNumberOfDetections =
      this->checkScanForObstacles(msg, lAngleMinIndex, lAngleMaxIndex, lLaserScanThresholdValue);
  this->mObstacleDetected =
      this->checkDetectedObstacleThreshold(lNumberOfDetections, msg->ranges.size());
}

void mars::simulation::ctv::agent::MoveBaseSimple::determinMinMaxLaserScanArrayIndices(
    const sensor_msgs::LaserScan::ConstPtr& msg, int& angleMinIndex, int& angleMaxIndex)
{
  if (this->mLaserScanUseAngleLimitation && (this->mLaserScanMinAngle > msg->angle_min))
  {
    angleMinIndex = (int)(fabs(msg->angle_min - this->mLaserScanMinAngle) / msg->angle_increment);
  }
  else
  {
    angleMinIndex = 0;
  }

  if (this->mLaserScanUseAngleLimitation && (this->mLaserScanMaxAngle < msg->angle_max))
  {
    angleMaxIndex = msg->ranges.size() -
                    (int)(fabs(msg->angle_max - this->mLaserScanMaxAngle) / msg->angle_increment);
  }
  else
  {
    angleMaxIndex = msg->ranges.size();
  }
}

int mars::simulation::ctv::agent::MoveBaseSimple::checkScanForObstacles(
    const sensor_msgs::LaserScan::ConstPtr& msg, int& angleMinIndex, int& angleMaxIndex,
    double laserScanThresholdValue)
{
  float laserScanDistance;
  int lNumberOfDetections = 0;

  for (int i = angleMinIndex; i < angleMaxIndex; i++)
  {
    laserScanDistance = msg->ranges[i];

    if (laserScanDistance < msg->range_min || laserScanDistance > msg->range_max)
    {
      continue;
    }
    else if (laserScanDistance < laserScanThresholdValue)
    {
      lNumberOfDetections++;
    }
  }

  return lNumberOfDetections;
}

bool mars::simulation::ctv::agent::MoveBaseSimple::checkDetectedObstacleThreshold(
    int numberOfDections, int amountBeams)
{
  bool lObstacle = false;

  if ((((float)numberOfDections) / amountBeams) > STD_OBSTACLE_DETECTED_PERCENT_THRESHOLD)
  {
    if (this->mState != MBS_OBSTACLE)
    {
      this->mHistoryState = this->mState;
      this->mState = MBS_OBSTACLE;
      ROS_WARN("Obstacle detected");
    }
    lObstacle = true;
  }

  return lObstacle;
}

std::string mars::simulation::ctv::agent::MoveBaseSimple::getRobotFrameId() const
{
  return mRobotFrameId;
}

void mars::simulation::ctv::agent::MoveBaseSimple::setRobotFrameId(const std::string& robotFrameId)
{
  mRobotFrameId = robotFrameId;
}

std::string mars::simulation::ctv::agent::MoveBaseSimple::getGlobalFrameId() const
{
  return mGlobalFrameId;
}

void mars::simulation::ctv::agent::MoveBaseSimple::setGlobalFrameId(
    const std::string& globalFrameId)
{
  mGlobalFrameId = globalFrameId;
}

void mars::simulation::ctv::agent::MoveBaseSimple::stopVehicle()
{
  this->mVehicle.setCurrentLinearVel(0.0);
  this->mVehicle.setCurrentAngularVel(0.0);
}

bool mars::simulation::ctv::agent::MoveBaseSimple::isNextAssignmentStraight()
{
  bool lIsStraight = false;
  std::shared_ptr<MotionAssignment> lNextMotion = this->mNextMotionAssignment;
  std::shared_ptr<MotionAssignment> lLastMotion = this->mLastMotionAssignment;
  std::shared_ptr<MotionAssignment> lCurrentMotion = this->mCurrentMotionAssignment;
  if (lNextMotion != nullptr)
  {
    if (lLastMotion != nullptr)
    {
      double lLastToActualGoalOrientation =
          atan2((lCurrentMotion->getGoalPose().y - lLastMotion->getGoalPose().y),
                (lCurrentMotion->getGoalPose().x - lLastMotion->getGoalPose().x));
      double lLastToNextGoalOrientation =
          atan2((lNextMotion->getGoalPose().y - lLastMotion->getGoalPose().y),
                (lNextMotion->getGoalPose().x - lLastMotion->getGoalPose().x));
      double lAngDifLastToActualGoal = angles::shortest_angular_distance(
          lLastToActualGoalOrientation, lLastToNextGoalOrientation);
      if (lAngDifLastToActualGoal <= this->mAngularTolerance / 10)
      {
        double lAngDifVehToNextTarget = angles::shortest_angular_distance(
            lLastToNextGoalOrientation, this->mVehicle.getCurrentOrientation());
        // if the next goal is straight to the actual goal and straight to the vehicle
        if (fabs(lAngDifVehToNextTarget) < (this->mAngularTolerance))
        {
          lIsStraight = true;
        }
      }
    }
    else
    {
      double lNextTargetOrientation =
          atan2((lNextMotion->getGoalPose().y - this->mVehicle.getCurrentCordY()),
                (lNextMotion->getGoalPose().x - this->mVehicle.getCurrentCordX()));
      double lTargetOrientation =
          atan2((lCurrentMotion->getGoalPose().y - this->mVehicle.getCurrentCordY()),
                (lCurrentMotion->getGoalPose().x - this->mVehicle.getCurrentCordX()));

      double lAngDifTargetToNextTarget =
          angles::shortest_angular_distance(lTargetOrientation, lNextTargetOrientation);
      double lAngDifVehToTarget = angles::shortest_angular_distance(
          lTargetOrientation, this->mVehicle.getCurrentOrientation());

      // if the next goal is straight to the actual goal and straight to the vehicle
      if ((fabs(lAngDifTargetToNextTarget) < (this->mAngularTolerance)) &&
          (fabs(lAngDifVehToTarget) < (this->mAngularTolerance)))
      {
        lIsStraight = true;
      }
    }
  }
  return lIsStraight;
}

void mars::simulation::ctv::agent::MoveBaseSimple::setMaxVelocity()
{
  double lMaxXVel;
  double lNextMaxXVel;
  double lMaxPosAngVel;
  double lMaxNegAngVel;

  // Get the max Velocity for the current linear Motion
  if (this->mVehicle.getMaxPosXVel() <= this->mCurrentMotionAssignment->getMaxXVelocity())
  {
    lMaxXVel = this->mVehicle.getMaxPosXVel();
  }
  else
  {
    lMaxXVel = this->mCurrentMotionAssignment->getMaxXVelocity();
  }

  /* Get the max Velocity of the next MotionAssignment. If the next Assignment is a MotionAssignment
  and it is straight ahead then compute the max Velocity of the next Assignment. Otherwise the max
  Velocity of the next Assignment is zero. If the next Assignment is a MotionAssignment and it is
  straight ahead, we have to calculate the Goaldiversity, too.*/
  if (this->isNextAssignmentStraight())
  {
    if (this->mVehicle.getMaxPosXVel() <= this->mNextMotionAssignment->getMaxXVelocity())
    {
      lNextMaxXVel = this->mVehicle.getMaxPosXVel();
    }
    else
    {
      lNextMaxXVel = this->mNextMotionAssignment->getMaxXVelocity();
    }
    this->calculateGoalDiversity();
  }
  else
  {
    lNextMaxXVel = 0;
  }

  // Get the max Velocity of the current angular motion
  if (this->mVehicle.getMaxPosAngVel() >= this->mCurrentMotionAssignment->getMaxAngVelocity())
  {
    lMaxPosAngVel = this->mCurrentMotionAssignment->getMaxAngVelocity();
  }
  else
  {
    lMaxPosAngVel = this->mVehicle.getMaxPosAngVel();
  }
  if (this->mVehicle.getMaxNegAngVel() >= this->mCurrentMotionAssignment->getMaxAngVelocity())
  {
    lMaxNegAngVel = this->mCurrentMotionAssignment->getMaxAngVelocity();
  }
  else
  {
    lMaxNegAngVel = this->mVehicle.getMaxNegAngVel();
  }

  this->mMaxPosXVel = lMaxXVel;
  this->mNextMaxPosXVel = lNextMaxXVel;
  this->mMaxPosAngVel = lMaxPosAngVel;
  this->mMaxNegAngVel = lMaxNegAngVel;
}

void mars::simulation::ctv::agent::MoveBaseSimple::setMaxAcceleration()
{
  double lMaxPosXAcc;
  double lMaxPosAngAcc;

  // Get the max Acceleration for the current linear Motion
  if (this->mVehicle.getMaxPosXAcc() <= this->mCurrentMotionAssignment->getMaxXAcceleration())
  {
    lMaxPosXAcc = this->mVehicle.getMaxPosXAcc();
  }
  else
  {
    lMaxPosXAcc = this->mCurrentMotionAssignment->getMaxXAcceleration();
  }

  // Get max Acceleration for the current angular motion
  if (this->mVehicle.getMaxPosAngAcc() >= this->mCurrentMotionAssignment->getMaxAngAcceleration())
  {
    lMaxPosAngAcc = this->mCurrentMotionAssignment->getMaxAngAcceleration();
  }
  else
  {
    lMaxPosAngAcc = this->mVehicle.getMaxPosAngAcc();
  }

  this->mMaxPosXAcc = lMaxPosXAcc;
  this->mMaxPosAngAcc = lMaxPosAngAcc;
}

void mars::simulation::ctv::agent::MoveBaseSimple::calculateGoalDiversity()
{
  // If the next Goal is straight ahead we define a Goaldiversity instead of a exact point
  double lTargetOrientation = atan2((this->mCurrentMotionAssignment->getGoalPose().y -
                                     this->mNextMotionAssignment->getGoalPose().y),
                                    (this->mCurrentMotionAssignment->getGoalPose().x -
                                     this->mNextMotionAssignment->getGoalPose().x));
  if (fabs(lTargetOrientation) == M_PI_2) // if the orientation is parallel to the y axis
  {
    this->mGoalDiversitySlope = 0;
    this->mGoalDiversityB = this->mCurrentMotionAssignment->getGoalPose().y;
    if (this->mVehicle.getCurrentCordY() >= this->mGoalDiversityB)
    {
      // Vehicle drives in negative y-direction
      this->mGoalDiversityState = true;
    }
    else
    {
      // Vehicle drives in positive y-direction
      this->mGoalDiversityState = false;
    }
  }
  else
  {
    if (fabs(lTargetOrientation) == 0 || fabs(lTargetOrientation) == M_PI)
    {
      this->mGoalDiversitySlope = INFINITY;
      this->mGoalDiversityB = this->mCurrentMotionAssignment->getGoalPose().x;
      if (this->mVehicle.getCurrentCordX() >= this->mGoalDiversityB)
      {
        // Vehicle drives in negative x-direction
        this->mGoalDiversityState = true;
      }
      else
      {
        // Vehicle drives in positive x-direction
        this->mGoalDiversityState = false;
      }
    }
    else
    {
      this->mGoalDiversitySlope = 1 / tan(lTargetOrientation);
      this->mGoalDiversityB =
          this->mCurrentMotionAssignment->getGoalPose().y -
          this->mGoalDiversitySlope * this->mCurrentMotionAssignment->getGoalPose().x;
      if (this->mVehicle.getCurrentCordY() >=
          this->mGoalDiversitySlope * this->mVehicle.getCurrentCordX() + this->mGoalDiversityB)
      {
        // Vehicle drives with a negative slope in the x/y-plane
        this->mGoalDiversityState = true;
      }
      else
      {
        // vehicle drives with a positive slope in the x/y-plane
        this->mGoalDiversityState = false;
      }
    }
  }
}

bool mars::simulation::ctv::agent::MoveBaseSimple::reachedGoalDiversity()
{
  /*If the next goal is straight ahead, the tolerance have to be larger than the normal linear
  tolerance. Therefore we calculate a dynamic tolerance which is the braking distance depend on the
  actual linear velocity. After that we shift the Goaldiversity by the new dynamic tolerance and
  check if the vehicle has reached the Goaldiversity.*/
  bool lReachedGoalDiversity = false;
  double lTargetOrientation;
  double lGoalOrientation = atan2(this->mCurrentMotionAssignment->getGoalPose().y -
                                      this->mNextMotionAssignment->getGoalPose().y,
                                  this->mCurrentMotionAssignment->getGoalPose().x -
                                      this->mNextMotionAssignment->getGoalPose().x);
  double lDynamicLinearTolerance =
      pow(this->mVehicle.getCurrentLinearVel(), 2) / (2 * this->mVehicle.getMaxPosXAcc());

  if (fabs(lGoalOrientation) == M_PI_2)
  {
    // Goaldiversity is parallel to the x-axis
    if (this->mGoalDiversityState)
    {
      // The vehicle drives in negative y-direction
      if (this->mVehicle.getCurrentCordY() <= this->mGoalDiversityB + lDynamicLinearTolerance)
      {
        lReachedGoalDiversity = true;
      }
    }
    else
    {
      // The vehicle drives in positive y-direction
      if (this->mVehicle.getCurrentCordY() >= this->mGoalDiversityB - lDynamicLinearTolerance)
      {
        lReachedGoalDiversity = true;
      }
    }
  }
  else
  {
    if (fabs(lGoalOrientation) == 0 || fabs(lGoalOrientation) == M_PI)
    {
      // Goaldiversity is parallel to the y-axis
      if (this->mGoalDiversityState)
      {
        // The vehicle drives in negative x-direction
        if (this->mVehicle.getCurrentCordX() <= this->mGoalDiversityB + lDynamicLinearTolerance)
        {
          lReachedGoalDiversity = true;
        }
      }
      else
      {
        // The vehicle drives in positive x-direction
        if (this->mVehicle.getCurrentCordX() >= this->mGoalDiversityB - lDynamicLinearTolerance)
        {
          lReachedGoalDiversity = true;
        }
      }
    }
    else
    {
      // Goal is reached if the vehicle drives over the goal diversity line
      // double lTargetOrientation = atan(1 / this->mGoalDiversitySlope);
      // this->mGoalDiversityB = this->mCurrentMotionAssignment->getGoalPose().y -
      // (lDynamicLinearTolerance * sin(lTargetOrientation)) -
      //                             mGoalDiversitySlope *
      //                             (this->mCurrentMotionAssignment->getGoalPose().x -
      //                             (lDynamicLinearTolerance * cos(lTargetOrientation)));
      // //Goaldiversity is not parallel to an axis
      // if (this->mGoalDiversityState)
      // {
      //   //The vehicle drives with a negative slope in the y/x -plane
      //   if(this->mVehicle.getCurrentCordY() <=
      //   this->mGoalDiversitySlope*this->mVehicle.getCurrentCordX() + this->mGoalDiversityB)
      //   {
      //     lReachedGoalDiversity = true;
      //   }
      // }
      // else
      // {
      //   //The vehicle drives with a positive slope in the y/x -plane
      //   if(this->mVehicle.getCurrentCordY() >=
      //   this->mGoalDiversitySlope*this->mVehicle.getCurrentCordX() + this->mGoalDiversityB)
      //   {
      //     lReachedGoalDiversity = true;
      //   }
      // }
      double distanceToGoal = sqrt(
          pow(this->mVehicle.getCurrentCordX() - this->mCurrentMotionAssignment->getGoalPose().x,
              2) +
          pow(this->mVehicle.getCurrentCordY() - this->mCurrentMotionAssignment->getGoalPose().y,
              2));
      if (lDynamicLinearTolerance <= distanceToGoal)
      {
        lReachedGoalDiversity = true;
      }
    }
  }
  return lReachedGoalDiversity;
}

void mars::simulation::ctv::agent::MoveBaseSimple::actualStatePublisher()
{
  mars_agent_physical_robot_msgs::ActualState msg;
  msg.robot_id = mars::common::Id::convertToMsgId(this->mVehicle.getRobot_id());
  msg.MoveBaseSimpleState = this->mState;
  mActualStatePub->publish(msg);
}

void mars::simulation::ctv::agent::MoveBaseSimple::cmd_velPublisher()
{
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = this->mVehicle.getCurrentLinearVel();
  cmd_vel.angular.z = this->mVehicle.getCurrentAngularVel();
  mVelocityPub->publish(cmd_vel);
}

void mars::simulation::ctv::agent::MoveBaseSimple::assignmentStatusPublisher()
{
  mars_agent_physical_robot_msgs::AssignmentStatus msg;
  msg.current_action_id = mars::common::Id::convertToMsgId(this->mTaskManager.getCurrentActionId());
  msg.current_motion_id = mars::common::Id::convertToMsgId(this->mTaskManager.getCurrentMotionId());
  msg.last_finished_action =
      mars::common::Id::convertToMsgId(this->mTaskManager.getLastFinishedActionId());
  msg.last_finished_motion =
      mars::common::Id::convertToMsgId(this->mTaskManager.getLastFinishedMotionId());
  msg.current_task_id = mars::common::Id::convertToMsgId(this->mTaskManager.getCurrentTaskId());
  msg.footprint = this->mVehicle.getFootprint();
  this->mAssignmentStatePub->publish(msg);
}

void mars::simulation::ctv::agent::MoveBaseSimple::robotAgentPropertiesPublisher()
{
  mars_agent_physical_robot_msgs::VehicleType msg_vehicleType;
  mars_agent_physical_robot_msgs::RobotAction msg_robotAction;
  mars_agent_physical_robot_msgs::RobotAgentProperties msg;
  msg.robot_id = mars::common::Id::convertToMsgId(this->mVehicle.getRobot_id());
  msg_vehicleType.vehicle_type = this->mVehicle.getType().vehicle_type;
  msg.type = msg_vehicleType;
  msg.footprint = this->mVehicle.getFootprint();
  msg.min_height = this->mVehicle.getMinHeight();
  msg.max_height = this->mVehicle.getMaxHeight();
  msg.payload = this->mVehicle.getPayload();
  msg.max_pos_x_vel = this->mVehicle.getMaxPosXVel();
  msg.max_neg_x_vel = this->mVehicle.getMaxNegXVel();
  msg.max_pos_x_acc = this->mVehicle.getMaxPosXAcc();
  msg.max_neg_x_acc = this->mVehicle.getMaxNegXAcc();
  msg.max_pos_y_vel = this->mVehicle.getMaxPosYVel();
  msg.max_neg_y_vel = this->mVehicle.getMaxNegYVel();
  msg.max_pos_y_acc = this->mVehicle.getMaxPosYAcc();
  msg.max_neg_y_acc = this->mVehicle.getMaxNegYAcc();
  msg.max_pos_ang_vel = this->mVehicle.getMaxPosAngVel();
  msg.max_neg_ang_vel = this->mVehicle.getMaxNegAngVel();
  msg.max_pos_ang_acc = this->mVehicle.getMaxPosAngAcc();
  msg.max_neg_ang_acc = this->mVehicle.getMaxNegAngAcc();
  msg.velocity_control_sensitivity = this->mVehicle.getVelocityControlSensitivity();
  msg.min_turning_radius = this->mVehicle.getTurningRadius();
  msg.batt_capacity = this->mVehicle.getBattCapacity();
  msg.batt_max_voltage = this->mVehicle.getBattMaxVoltage();
  msg.vendor = this->mVehicle.getVendor();
  msg_robotAction = this->mVehicle.getActionCapability();
  // TODO: fill msg.action_capability
  // msg.action_capability = msg_robotAction;
  this->mRobotDescriptionPub->publish(msg);
}
