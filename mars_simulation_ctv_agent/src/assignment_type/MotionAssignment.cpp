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

#include "mars_simulation_ctv_agent/assignment_type/MotionAssignment.h"


mars::simulation::ctv::agent::MotionAssignment::MotionAssignment(const mars_agent_physical_robot_msgs::MotionAssignmentConstPtr &msg)
{
    this->mPointId = mars::common::Id(msg->point_id);
    this->mTaskId = mars::common::Id(msg->task_id);
    this->mMotionId = mars::common::Id(msg->motion_id);
    this->mGoalPose = msg->point;
    this->mIsWaypoint = msg->is_waypoint;
    this->mUseOrientation = msg->use_orientation;
    this->mMaxXVelocity = msg->max_velocity.linear.x;
    this->mMaxYVelocity = msg->max_velocity.linear.y;
    this->mMaxAngVelocity = msg->max_velocity.angular.z;
    this->mMaxXAcceleration = msg->max_acceleration.linear.x;
    this->mMaxYAcceleration = msg->max_acceleration.linear.y;
    this->mMaxAngAcceleration = msg->max_acceleration.angular.z;
    this->mMotionArea = msg->motion_area;
    this->mSequencePosition = msg->sequence.sequence_number;
}

bool mars::simulation::ctv::agent::MotionAssignment::getUseOrientation() const
{
    return mUseOrientation;
}

geometry_msgs::Pose2D mars::simulation::ctv::agent::MotionAssignment::getGoalPose() const
{
  return mGoalPose;
}

geometry_msgs::PolygonStamped mars::simulation::ctv::agent::MotionAssignment::getMotionArea() const
{
    return mMotionArea;
}

float mars::simulation::ctv::agent::MotionAssignment::getMaxXVelocity() const
{
  return mMaxXVelocity;
}

float mars::simulation::ctv::agent::MotionAssignment::getMaxYVelocity() const
{
    return mMaxYVelocity;
}

float mars::simulation::ctv::agent::MotionAssignment::getMaxAngVelocity() const
{
    return mMaxAngVelocity;
}

float mars::simulation::ctv::agent::MotionAssignment::getMaxXAcceleration() const
{
  return mMaxXAcceleration;
}

float mars::simulation::ctv::agent::MotionAssignment::getMaxYAcceleration() const
{
    return mMaxYAcceleration;
}

float mars::simulation::ctv::agent::MotionAssignment::getMaxAngAcceleration() const
{
    return mMaxAngAcceleration;
}

bool mars::simulation::ctv::agent::MotionAssignment::getIsWaypoint() const
{
    return mIsWaypoint;
}

mars::common::Id mars::simulation::ctv::agent::MotionAssignment::getPointId() const
{
    return mPointId;
}

mars::common::Id mars::simulation::ctv::agent::MotionAssignment::getMotionId() const
{
    return mMotionId;
}
