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

