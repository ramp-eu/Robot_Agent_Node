//  Copyright 2020 Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//  https:www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//


#ifndef MOVEBASESIMPLE_H
#define MOVEBASESIMPLE_H

#include "mars_simulation_ctv_agent/TaskManager.h"
#include "mars_simulation_ctv_agent/Vehicle.h"

// include ros and c++ specific stuff
#include <angles/angles.h>
#include <chrono>
#include <mars_common/Id.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// include messages which are published or subscribed to
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <industrial_msgs/RobotStatus.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/LaserScan.h>

#include <mars_agent_physical_robot_msgs/ActionAssignment.h>
#include <mars_agent_physical_robot_msgs/ActualState.h>
#include <mars_agent_physical_robot_msgs/AssignmentStatus.h>
#include <mars_agent_physical_robot_msgs/Motion.h>
#include <mars_agent_physical_robot_msgs/MotionAssignment.h>
#include <mars_agent_physical_robot_msgs/RobotAgentProperties.h>

#include <list>

namespace mars
{
namespace simulation
{
namespace ctv
{
namespace agent
{
// Simple Navigation States
typedef enum _MoveBaseSimpleState
{
  MBS_NO_POSITION = 0,
  MBS_WAITING_FOR_INIT = 1,
  MBS_READY = 2,
  MBS_FIRST_ROTATION = 3,
  MBS_MOVING = 4,
  MBS_SECOND_ROTATION = 5,
  MBS_FINISHED = 6,
  MBS_FETCH_NEXT_INSTRUCTION = 7,
  MBS_EXECUTE_ACTION = 8,
  MBS_OBSTACLE = 9,
  MBS_ERROR = 10
} MoveBaseSimpleState;

class MoveBaseSimple
{
public:
  MoveBaseSimple();

  void setVehicle(const Vehicle& vehicle);

  // Callback method to get a new Motion
  void
  addMotionToQueueCallback(const mars_agent_physical_robot_msgs::MotionAssignmentConstPtr& msg);

  // Callback method to get a new Action
  void
  addActionToQueueCallback(const mars_agent_physical_robot_msgs::ActionAssignmentConstPtr& msg);

  void cancelTaskinQueueCallback(const mars_agent_physical_robot_msgs::CancelTaskConstPtr& msg);

  // Callback method to receive the init position which is send by the GUI or
  // another system
  void setInitPoseCallback(const geometry_msgs::PoseConstPtr& msg);

  // Callback method to receive new laser scan and to identify if the robot has
  // to stop
  void scanReceivedCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

  // Timer Callback Methods
  void currentMotionTimerCallback(const ros::TimerEvent& event);
  void batteryStateTimerCallback(const ros::TimerEvent& event);
  void robotStateTimerCallback(const ros::TimerEvent& event);

  // Dynamic Reconfigure Callback
  void setPIDParameter(double gain_kp, double gain_ki, double gain_kd);

  // Status
  //  mars_agent_physical_robot_msgs::RANState getStatus();

  void setVelocityPub(ros::Publisher* velocityPub);

  void setCurrentMotionPub(ros::Publisher* curMotionPub);

  void setBatteryStatePub(ros::Publisher* batteryState);

  void setRobotStatePub(ros::Publisher* robotState);

  void setAssignmentStatePub(ros::Publisher* assignmentState);

  void setRobotDescriptionPub(ros::Publisher* robotDescription);

  void setActualStatePub(ros::Publisher* actualStatePub);

  // method for the real internal behaviour
  void driveToGoal();

  std::string getGlobalFrameId() const;
  void setGlobalFrameId(const std::string& globalFrameId);

  std::string getRobotFrameId() const;
  void setRobotFrameId(const std::string& robotFrameId);

  // Setter for the Parameter of the laser scan
  void setUseObstacleDetection(bool useObstacleDetection);
  void setLaserScanThresholdValue(float thresholdValue);
  void setLaserScanMinAngle(float minAngle);
  void setLaserScanMaxAngle(float maxAngle);
  void setLaserScanUseAngleLimitation(bool useAngleLimitation);

  // Setter for the linear and angular tolerance
  void setAngularTolerance(double angularTolerance);
  void setLinearTolerance(double linearTolerance);

  // Setter for the linear and angular velocity coefficients
  void setGainKp(double value);
  void setGainKi(double value);
  void setGainKd(double value);

  void setZeroPose(const geometry_msgs::PoseStamped& msg);

  void setInitPose(const geometry_msgs::PoseStamped& pPose);

  void skipInitialPose(void);

  static const double STD_INVALID_POSE_X;
  static const double STD_INVALID_POSE_Y;
  static const double STD_INVALID_POSE_PHI;

private:
  // ros nodehandler
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  mars::simulation::ctv::agent::Vehicle mVehicle;

  // Goal tolerance
  double mLinearTolerance;
  double mAngularTolerance;
  bool mAssignmentStraight;
  bool mNextAssignmentStraight;

  bool mSkipInitialPoseCheck;

  // If the next MotionAssignment ist staight ahead we use a Goaldiversity instead of a exact point
  double mGoalDiversitySlope;
  double mGoalDiversityB;   // Intersection of the Goaldiversity with the y-axis
  bool mGoalDiversityState; // State which Goal Diversity we use

  // Parameter and Errors for the lateral control
  double mGainKp;
  double mGainKi;
  double mGainKd;
  double mLastsAngularError;
  double mAngularError;
  double mAngularErrorSum;

  // Volues of the MotionProfile
  double mMaxPosXVel;
  double mNextMaxPosXVel; // Zero if the next Assignment is not straight ahead
  double mMaxPosXAcc;
  double mLastGoalVel;
  double mMaxPosAngVel;
  double mMaxNegAngVel;
  double mMaxPosAngAcc;
  ros::Time mActualTime;
  ros::Time mLastTime;

  bool mObstacleDetected;

  // Threshold Values for the Laserscan
  bool mUseObstacleDetection;
  float mLaserScanThresholdValue;
  float mLaserScanMinAngle;
  float mLaserScanMaxAngle;
  bool mLaserScanUseAngleLimitation;

  // List for the Assignment that are skipped because the next Assignment is straight ehead
  std::list<std::shared_ptr<mars::simulation::ctv::agent::MotionAssignment>> mSkippedAssignments;

  std::string mGlobalFrameId;
  std::string mRobotFrameId;

  std::shared_ptr<Assignment> mCurrentAssignment;
  std::shared_ptr<MotionAssignment> mCurrentMotionAssignment;
  std::shared_ptr<ActionAssignment> mCurrentActionAssignment;
  std::shared_ptr<MotionAssignment> mLastMotionAssignment;
  std::shared_ptr<MotionAssignment> mNextMotionAssignment;

  // declare internal parameters
  MoveBaseSimpleState mState; // custom state machine
  MoveBaseSimpleState mHistoryState;

  // publisher
  ros::Publisher* mVelocityPub;
  ros::Publisher* mCurMotionPub;
  ros::Publisher* mBatteryStatePub;
  ros::Publisher* mRobotStatePub;
  ros::Publisher* mAssignmentStatePub;
  ros::Publisher* mRobotDescriptionPub;
  ros::Publisher* mActualStatePub;

  TaskManager mTaskManager; // TaskManager

  // initial pose from MAS
  geometry_msgs::PoseStamped mInitPose;
  bool mGotInitPose;
  bool mGotGlobalPose;

  geometry_msgs::PoseStamped mZeroPose; // zero pose as an reference
  tf::TransformListener mTfListener;    // listener to transform pose to correct frame

  void setGotInitPose(bool gotInitPose);

  // Methods for the work with the laserScan
  void scanWhileMoving(const sensor_msgs::LaserScan::ConstPtr& msg);

  void scanWhileRotation(const sensor_msgs::LaserScan::ConstPtr& msg);

  void determinMinMaxLaserScanArrayIndices(const sensor_msgs::LaserScan::ConstPtr& msg,
                                           int& angleMinIndex, int& angleMaxIndex);

  int checkScanForObstacles(const sensor_msgs::LaserScan::ConstPtr& msg, int& angleMinIndex,
                            int& angleMaxIndex, double laserScanThresholdValue);

  bool checkDetectedObstacleThreshold(int numberOfDections, int amountBeams);

  void updateGlobalVehiclePose();

  void duringMbsNoPosition();

  void duringMbsWaitForInit();

  void duringMbsReady();

  void duringMbsFirstRotation();

  void duringMbsMoving();

  void duringMbsSecondRotation();

  void duringMbsFinished();

  void duringMbsFetchNextInstruction();

  void duringMbsExecuteAction();

  void duringMbsObstacle();

  void duringMbsError();

  void stopVehicle();

  /**
   * @brief isNextAssignmentStraight
   * @return true if next MotionAssignment is straight on the current route
   */
  bool isNextAssignmentStraight();

  /**
   *  @brief setMaxVelocity determine if the max velocity of the next motion is lower than the max
   * velocity of the vehicle and sets the member variable to the lower value
   */
  void setMaxVelocity();

  /**
   *  @brief setMaxAcceleration determine if the max acceleration of the next motion is lower than
   * the max acceleration of the vehicle and sets the member variable to the lower value
   */
  void setMaxAcceleration();

  void calculateGoalDiversity();

  bool reachedGoalDiversity();

  void actualStatePublisher();

  void cmd_velPublisher();

  void assignmentStatusPublisher();

  void robotAgentPropertiesPublisher();
};
} // namespace agent
} // namespace ctv
} // namespace simulation
} // namespace mars

#endif // MOVEBASESIMPLE_H
