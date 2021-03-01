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


#ifndef TASKMANAGER_H
#define TASKMANAGER_H

// includes
#include <mars_common/Id.h>
#include "mars_simulation_ctv_agent/assignment_type/Assignment.h"
#include "mars_simulation_ctv_agent/Task.h"

// msgs include
#include "mars_agent_physical_robot_msgs/ActionAssignment.h"
#include "mars_agent_physical_robot_msgs/MotionAssignment.h"
#include "mars_agent_physical_robot_msgs/CancelTask.h"

// C++ includes
#include <list>
#include <string>

namespace mars
{
namespace simulation
{
namespace ctv
{
namespace agent
{

class TaskManager
{
public:
  /**
   * @brief TaskManager
   */
  TaskManager();

  /**
   * @brief addAssignment Adds an assignment to the specified task. Constructs
   * a task object, if there is no task with the specified task Id.
   * @param msg ROS msg which carries the information of a ActionAssignment.
   */
  void
  addAssignment(const mars_agent_physical_robot_msgs::ActionAssignmentConstPtr &msg);

  /**
   * @brief addAssignment Adds an assignment to the specified task. Constructs
   * a task object, if there is no task with the specified task Id.
   * @param msg ROS msg which carries the information of a MotionAssignment.
   */
  void
  addAssignment(const mars_agent_physical_robot_msgs::MotionAssignmentConstPtr& msg);

  /**
   * @brief deleteTask Deletes the task specified in the msg completely.
   * @param msg ROS msg which carries the information of the deletion.
   */
  void deleteTask(const mars_agent_physical_robot_msgs::CancelTaskConstPtr &msg);

  /**
   * @brief getNextAssignment Gets the next assignment of the current task. If
   * the current task has non assignments left, it switches to the next task.
   * @return The next assignment
   */
  std::shared_ptr<Assignment> getNextAssignment();

  /**
   * @brief isNextAssignmentAvailable
   * @return
   */
  bool isNextAssignmentAvailable();

  /**
   * @brief getCurrentTaskId Gets the current task Id.
   * @return Current task Id.
   */
  mars::common::Id getCurrentTaskId() const;

  /**
   * @brief getCurrentMotionId Gets the current motion Id.
   * @return Id of the current motion assignment.
   */
  mars::common::Id getCurrentMotionId() const;

  /**
   * @brief getCurrentActionId Gets the current action Id.
   * @return Id of the current action assignment.
   */
  mars::common::Id getCurrentActionId() const;

  /**
   * @brief getLastFinishedMotionId Gets the last finished motion Id.
   * @return Id of the last finished motion assignment.
   */
  mars::common::Id getLastFinishedMotionId() const;

  /**
   * @brief getLastFinishedActionId Gets the last finished action Id.
   * @return Id of the last finished action assignment.
   */
  mars::common::Id getLastFinishedActionId() const;

  /**
   * @brief isCurrentTaskDone
   * @return
   */
  bool isCurrentTaskDone();

  /**
   * @brief updateLastFinishedAssignmentIds
   */
  void updateLastFinishedAssignmentIds();
  /**
   * @brief updateLastFinishedMotionAssignmentIds updates the Last Finished Motion Id. Is used if a MotionAssignment is skipped because the next Assignment is straight ahead.
   * @param finishedMotionId
   */
  void updateLastFinishedMotionAssignmentIds(mars::common::Id finishedMotionId);

  /**
   * @brief isTaskAvailable
   * @return
   */
  bool isTaskAvailable();

  /**
   * @brief getNextNextAssignment
   * @return return the next but one Assignment
   */
  std::shared_ptr<Assignment> getNextNextAssignment();

private:

  std::list<mars::simulation::ctv::agent::Task>::iterator getTask(mars::common::Id taskId);

  /**
   * @brief mTaskQueue Queue in which the tasks are saved and odered.
   */
  std::list<mars::simulation::ctv::agent::Task> mTaskQueue;

  // Meta data

  /**
   * @brief mCurrentTaskId Id of the current processed task.
   */
  mars::common::Id mCurrentTaskId;

  /**
   * @brief mCurrentMotionId Id of the current processed motion.
   */
  mars::common::Id mCurrentMotionId;

  /**
   * @brief mCurrentActionId Id of the current processed action.
   */
  mars::common::Id mCurrentActionId;

  /**
   * @brief mLastFinishedMotionId Id of the last finished motion.
   */
  mars::common::Id mLastFinishedMotionId;

  /**
   * @brief mLastFinishedActionId id of the last finished action.
   */
  mars::common::Id mLastFinishedActionId;

  mars::common::Id mLastFinishedTaskId;


};

} // namespace agent
} // namespace ctv
} // namespace simulation
} // namespace mars

#endif // TASKMANAGER_H
