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

#include "mars_simulation_ctv_agent/assignment_type/ActionAssignment.h"


mars::simulation::ctv::agent::ActionAssignment::ActionAssignment(const mars_agent_physical_robot_msgs::ActionAssignmentConstPtr &msg)
{
  //this->mRobotId = mars::common::Id(msg->action_id);
  this->mActionId = mars::common::Id(msg->action_id);

  this->mActionType = msg->robot_action.action;

  this->mSequencePosition = msg->sequence.sequence_number;
}

//mars::common::Id mars::simulation::ctv::agent::ActionAssignment::getOrderId() const
//{
//  return this->mOrderId;
//}

//mars::common::Id mars::simulation::ctv::agent::ActionAssignment::getRobotId() const
//{
//  return this->mRobotId;
//}

mars::common::Id mars::simulation::ctv::agent::ActionAssignment::getActionId() const
{
  return this->mActionId;
}

