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


#ifndef MARSSIMULATIONCTVAGENTNODE_H
#define MARSSIMULATIONCTVAGENTNODE_H

// ros node internal includes
#include <mars_common/exception/AdvertiseServiceException.h>
#include <mars_common/exception/ReadParamException.h>

// ros includes
#include <dynamic_reconfigure/server.h>
#include <mars_simulation_ctv_agent/dynamic_pidConfig.h>
#include <ros/ros.h>

// C++ includes
#include <string>

// own includes
#include "MoveBaseSimple.h"
#include "Vehicle.h"

// include messages
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <industrial_msgs/RobotStatus.h>
#include <sensor_msgs/BatteryState.h>

class MarsSimulationCtvAgentNode
{
public:
  /**
   * @brief MarsSimulationCtvAgentNode Creates an object of
   * MarsSimulationCtvAgentNode.
   */
  MarsSimulationCtvAgentNode();

  /**
   * @brief ~MarsSimulationCtvAgentNode Deletes an MarsSimulationCtvAgentNode
   * object.
   */
  ~MarsSimulationCtvAgentNode();

  /**
   * @brief runROSNode Runs the ros node. This is the only method you have to
   * call.
   *
   * @return Returns true if now error occurred during runtime.
   */
  bool runROSNode(void);

  // First member variables, second methods.
protected:
  // First member variables, second methods.
private:
  // ros node handles
  /**
   * @brief mNH Public ros handle.
   */
  ros::NodeHandle mNH;
  /**
   * @brief mNHPriv Private node handle.
   */
  ros::NodeHandle mNHPriv;

  /**
   * @brief mHz Contains the information about the set loop rate of the node.
   */
  int mHz;

  /**
   * @brief mNodeLogLevel Current log level of the node.
   */
  std::string mNodeLogLevel;

  /**
   * @brief mStatusMsgFrequency Frequency of status updates
   */
  double mStatusMsgFrequency;

  /**
   * @brief mMoveBase Calculate path
   */
  mars::simulation::ctv::agent::MoveBaseSimple mMoveBase;

  bool obstacle_detection;

  // Durations for the Timers
  double mCurMotionFrequency;
  double mBatteryStateFrequency;
  double mRobotStateFrequency;

  // Topics for communication
  std::string mVelocityTopic;
  std::string mScanTopic;
  std::string mInitPoseTopic;
  std::string mCurrentMotionTopic;
  std::string mBatteryStateTopic;
  std::string mRobotStateTopic;
  std::string mAssignmentStateTopic;
  std::string mRobotAgentPropertiesTopic;
  std::string mMotionAssignmentTopic;
  std::string mActionAssignmentTopic;
  std::string mCancelTaskTopic;
  std::string mActualStateTopic;

  // declare publish members
  ros::Publisher mCmdVelPub;
  ros::Publisher mCurrentMotionPub;
  ros::Publisher mBatteryStatePub;
  ros::Publisher mRobotStatePub;
  ros::Publisher mAssignmentStatePub;
  ros::Publisher mRobotAgentPropertiesPub;
  ros::Publisher mActualStatePub;

  // declare subscriber members
  ros::Subscriber mScanSub;
  ros::Subscriber mInitPoseSub;
  ros::Subscriber mActionAssignmentSub;
  ros::Subscriber mMotionAssignmentSub;
  ros::Subscriber mCancelTaskSub;

  // dynamic reconfigure members
  dynamic_reconfigure::Server<mars_simulation_ctv_agent::dynamic_pidConfig> mServer;

  // declare Timer members
  ros::Timer mCurrentMotionTimer;
  ros::Timer mBatteryStateTimer;
  ros::Timer mRobotStateTimer;

  // init methods for ros node

  /**
   * @brief init Inits the node. This method automatically calls the methods:
   * initServices(), initPublisher(), initSubsriber(), readLaunchParams() and
   * printNodeInfos() if no error occours during execution.
   *
   * @return Returns true if no error occours.
   */
  bool init(void);

  /**
   * @brief initPublisher Initializes the provided publishers.
   * @return Returns true if no error occurs.
   */
  bool initPublisher(void);

  /**
   * @brief initSubscriber Initializes the subscribers.
   * @return Returns true if no error occurs.
   */
  bool initSubscriber(void);

  /**
   * @brief initDynamicReconfigure Initializes the dynamic reconfigure callback.
   * @return Returns true if no error occurs.
   */
  bool initDynamicReconfigure(void);
  void dynamicReconfigureCallback(mars_simulation_ctv_agent::dynamic_pidConfig& config,
                                  uint32_t level);

  /**
   * @brief initTimer Initializes the Timer.
   * @return Returns true if no error occurs.
   */
  bool initTimer(void);

  /**
   * @brief readLaunchParams Reads the paramters from the launch file.
   * @return Returns true if no error occurs.
   */
  bool readLaunchParams(void);

  /**
   * @brief getParam Reads the named paramter form the parameter server.
   * If the parameter could not be found a default value is assigned.
   */
  template <typename T> T getParam(ros::NodeHandle& nH, const std::string& paramName);

  /**
   *@brief getParam Reads the named parameter from the parameter server.
   * If the parameter could not be found a ReadParamException is thrown.
   * @throw mars::common::exception::ReadParamException
   */
  template <typename T>
  void getParam(ros::NodeHandle& nH, const std::string& paramName, T& paramValue) noexcept(false);

  /**
   *@brief getParam Reads the named parameter from the parameter server.
   * If the parameter could not be found a ReadParamException is thrown.
   * @throw mars::common::exception::ReadParamException
   */
  template <typename T>
  void getParam(ros::NodeHandle& nH, const std::string& paramName, T& paramValue,
                const T& defaultParamValue) noexcept(false);

  /**
   * @brief rosMainLoop Calls rosSpinOnce() with the rate you set with
   * 'node_rate'.
   */
  void rosMainLoop(void);

  /**
   * @brief setNodeLogLevel Sets the log level of the ros console for the node.
   * @return Return true if log level was successfully set.
   */
  bool setNodeLogLevel(void) const;
};

#endif // MARSSIMULATIONCTVAGENTNODE_H
