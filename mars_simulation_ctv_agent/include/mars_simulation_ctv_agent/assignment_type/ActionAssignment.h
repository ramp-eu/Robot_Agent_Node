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


#ifndef ACTIONASSIGNMENT_H
#define ACTIONASSIGNMENT_H
//include base class
#include <mars_simulation_ctv_agent/assignment_type/Assignment.h>

//include ros messages
#include <mars_agent_physical_robot_msgs/ActionAssignment.h>

namespace mars
{
namespace simulation
{
namespace ctv
{
namespace agent
{

class ActionAssignment : public mars::simulation::ctv::agent::Assignment
{
public:

  /**
   * @brief ActionAssignment Constructs an object of this class
   * @param msg Incomming msg from the virtual agent
   */
  ActionAssignment(const mars_agent_physical_robot_msgs::ActionAssignmentConstPtr &msg);


  /**
   * @brief getActionId Gets the Id of this action
   * @return Action Id
   */
  mars::common::Id getActionId() const;

  /**
   * @brief getCurrentSequencePosition Gets the position of this action in the sequence
   * @return Sequence position
   */
//  unsigned int getSequencePosition() const;

private:
  /**
   * @brief mActionId Id of this action
   */
  mars::common::Id mActionId;

  /**
   * @brief mActionType Type of this action, based on the enum _ActionType
   */
  int mActionType;

};

} // namespace agent
} // namespace ctv
} // namespace simulation
} // namespace mars

#endif // ACTIONASSIGNMENT_H
