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


#ifndef MOTIONASSIGNMENT_H
#define MOTIONASSIGNMENT_H

// include base class
#include "mars_simulation_ctv_agent/assignment_type/Assignment.h"

// include ros msgs
#include <mars_agent_physical_robot_msgs/MotionAssignment.h>

namespace mars
{
namespace simulation
{
namespace ctv
{
namespace agent
{

class MotionAssignment : public mars::simulation::ctv::agent::Assignment
{
public:
  /**
   * @brief MotionAssignment Constructs an object of this class.
   * @param msg Incoming msg from the logical agent.
   */
  MotionAssignment(const mars_agent_physical_robot_msgs::MotionAssignmentConstPtr& msg);

  /**
   * @brief getMotionId Gets the Id of this motion.
   * @return Id of this motion.
   */
  mars::common::Id getMotionId() const;

  /**
   * @brief getPointId Gets the Id of the destination point.
   * @return Id of the destination point.
   */
  mars::common::Id getPointId() const;

  /**
   * @brief getIsWaypoint Gets the Flag, if the vehicle should stop at the
   * motion destination.
   * @return Waypoint flag
   */
  bool getIsWaypoint() const;

  /**
   * @brief getUseOrientation Gets the flag, if the orientation of the
   * destination point should be used.
   * @return Orientation flag
   */
  bool getUseOrientation() const;

  /**
   * @brief getMaxVelocity Gets the max velocity, which is permitted for this
   * motion.
   * @return Velocity limit
   */
  float getMaxXVelocity() const;
  float getMaxYVelocity() const;
  float getMaxAngVelocity() const;

  /**
   * @brief getMaxAcceleration Gets the max acceleration, which is permitted for
   * this motion.
   * @return Acceleration limit.
   */
  float getMaxXAcceleration() const;
  float getMaxYAcceleration() const;
  float getMaxAngAcceleration() const;

  geometry_msgs::Pose2D getGoalPose() const;

  geometry_msgs::PolygonStamped getMotionArea() const;

private:
  /**
   * @brief mMotionId Unique id of this specific motion.
   */
  mars::common::Id mMotionId;

  /**
   * @brief mPointId Unique id of the destination point.
   */
  mars::common::Id mPointId;

  /**
   * @brief mIsWaypoint Flag, if the vehicle should stop at the motion
   * destination. TRUE if the point is a waypoint, FALSE if it is a goal
   */
  bool mIsWaypoint;

  /**
   * @brief mUseOrientation Flag, if the orientation of the destination point
   * should be used.
   */
  bool mUseOrientation;

  /**
   * @brief mMaxVelocity Max velocity, which is permitted for this motion.
   */
  float mMaxXVelocity;
  float mMaxYVelocity;
  float mMaxAngVelocity;

  /**
   * @brief mMaxAcceleration Max acceleration, which is permitted for this
   * motion.
   */
  float mMaxXAcceleration;
  float mMaxYAcceleration;
  float mMaxAngAcceleration;

  /**
   * @brief mPolygonStamped
   */
  geometry_msgs::PolygonStamped mMotionArea;

  /**
   * @brief mGoalPose
   */
  geometry_msgs::Pose2D mGoalPose;
};

} // namespace agent
} // namespace ctv
} // namespace simulation
} // namespace mars

#endif // MOTIONASSIGNMENT_H
