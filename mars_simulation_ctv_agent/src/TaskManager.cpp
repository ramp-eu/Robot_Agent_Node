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

#include "mars_simulation_ctv_agent/TaskManager.h"

#include <ros/ros.h>

mars::simulation::ctv::agent::TaskManager::TaskManager()
{
  this->mCurrentTaskId = mars::common::Id::createInvalidId();
  this->mCurrentMotionId = mars::common::Id::createInvalidId();
  this->mCurrentActionId = mars::common::Id::createInvalidId();
  this->mLastFinishedActionId = mars::common::Id::createInvalidId();
  this->mLastFinishedMotionId = mars::common::Id::createInvalidId();
  this->mLastFinishedTaskId = mars::common::Id::createInvalidId();
}

void mars::simulation::ctv::agent::TaskManager::addAssignment(
    const mars_agent_physical_robot_msgs::ActionAssignmentConstPtr &msg)
{
  std::list<mars::simulation::ctv::agent::Task>::iterator taskIter;
  mars::common::Id taskId(msg->task_id);

  // search the task
  taskIter = getTask(taskId);

  if (taskIter != this->mTaskQueue.end())
  {
    // append to existing task
    taskIter->addAssignment(msg);
  }
  else
  {
    if (taskId != this->mLastFinishedTaskId)
    {
      //create a task with the given id
      mars::simulation::ctv::agent::Task task(taskId, msg->sequence.length);
      task.addAssignment(msg);
      this->mTaskQueue.push_back(task);
    }
  }
}

void mars::simulation::ctv::agent::TaskManager::addAssignment(
    const mars_agent_physical_robot_msgs::MotionAssignmentConstPtr &msg)
{
  std::list<mars::simulation::ctv::agent::Task>::iterator taskIter;
  mars::common::Id taskId(msg->task_id);

  // search the task
  taskIter = getTask(taskId);

  if (taskIter != this->mTaskQueue.end())
  {
    // append to existing task
    taskIter->addAssignment(msg);
  }
  else
  {
    if (taskId != this->mLastFinishedTaskId)
    {
      //create a task with the given id
      mars::simulation::ctv::agent::Task task(taskId, msg->sequence.length);
      task.addAssignment(msg);
      this->mTaskQueue.push_back(task);
    }
  }
}

void mars::simulation::ctv::agent::TaskManager::deleteTask(
    const mars_agent_physical_robot_msgs::CancelTaskConstPtr &msg)
{

  std::list<mars::simulation::ctv::agent::Task>::iterator iter;
  mars::common::Id cancelTaskId(msg->task_id);

  for (iter = this->mTaskQueue.begin(); iter != this->mTaskQueue.end();
       ++iter)
  {
    if (iter->getTaskId() == msg->task_id)
    {
      this->mTaskQueue.erase(iter);
      break;
    }
  }
}

std::shared_ptr<mars::simulation::ctv::agent::Assignment>
mars::simulation::ctv::agent::TaskManager::getNextAssignment()
{
  std::shared_ptr<mars::simulation::ctv::agent::Assignment> nextAssignment;

  // the current task should always be the first in the queue
  std::list<mars::simulation::ctv::agent::Task>::iterator taskIter;

  taskIter = this->mTaskQueue.begin();

  this->mCurrentTaskId = taskIter->getTaskId();

  nextAssignment = taskIter->getNextAssignment();

  // Check if the assignment is an Action or a Motion
  std::shared_ptr<mars::simulation::ctv::agent::ActionAssignment>
      actionAssignment = nullptr;
  std::shared_ptr<mars::simulation::ctv::agent::MotionAssignment>
      motionAssignment = nullptr;

  actionAssignment =
      std::dynamic_pointer_cast<mars::simulation::ctv::agent::ActionAssignment>(
          nextAssignment);
  motionAssignment =
      std::dynamic_pointer_cast<mars::simulation::ctv::agent::MotionAssignment>(
          nextAssignment);

  // Set current Ids ind consideration of the type of the assignment
  if (motionAssignment != nullptr)
  {
    this->mCurrentMotionId = motionAssignment->getMotionId();
    this->mCurrentActionId = mars::common::Id::createInvalidId();
  }
  else if (actionAssignment != nullptr)
  {
    this->mCurrentActionId = actionAssignment->getActionId();
    this->mCurrentMotionId = mars::common::Id::createInvalidId();
  }

  // Check if the current task is done
  if (taskIter->isDone())
  {
    this->mTaskQueue.pop_front();
  }
  return nextAssignment;
}

bool mars::simulation::ctv::agent::TaskManager::isNextAssignmentAvailable()
{
  bool isAvaiable = false;
  std::list<mars::simulation::ctv::agent::Task>::iterator currentTaskIter =
      this->mTaskQueue.begin();

  if (!(this->mTaskQueue.empty()))
  {
    if (currentTaskIter->isNextAssignmentAvailable())
    {
      isAvaiable = true;
    }
  }
  return isAvaiable;
}

bool mars::simulation::ctv::agent::TaskManager::isCurrentTaskDone()
{
  std::list<mars::simulation::ctv::agent::Task>::iterator currentTaskIter =
      this->mTaskQueue.begin();

  if (currentTaskIter->getLastAssignmentFinished() && this->mTaskQueue.size() < 2)
  {
    this->mLastFinishedTaskId = this->mCurrentTaskId;
    return true;
  }
  return false;
}

void mars::simulation::ctv::agent::TaskManager::
    updateLastFinishedAssignmentIds()
{
  if (this->mCurrentMotionId !=
      mars::common::Id::createInvalidId())
  {
    mLastFinishedMotionId = this->mCurrentMotionId;
  }
  else if (this->mCurrentActionId !=
           mars::common::Id::createInvalidId())
  {
    mLastFinishedActionId = this->mCurrentActionId;
  }
}

void mars::simulation::ctv::agent::TaskManager::
    updateLastFinishedMotionAssignmentIds(mars::common::Id finishedMotionId)
{
  if (finishedMotionId != mars::common::Id::createInvalidId())
  {
    this->mLastFinishedMotionId = finishedMotionId;
  }
}

bool mars::simulation::ctv::agent::TaskManager::isTaskAvailable()
{
  return !(this->mTaskQueue.empty());
}

std::shared_ptr<mars::simulation::ctv::agent::Assignment>
mars::simulation::ctv::agent::TaskManager::getNextNextAssignment()
{
  std::list<mars::simulation::ctv::agent::Task>::iterator taskIter;
  taskIter = this->mTaskQueue.begin();
  if (!(this->isCurrentTaskDone()) && !this->mTaskQueue.empty()) //taskIter->isDone()))
  {
    if (taskIter->isNextAssignmentAvailable())
    {
      unsigned int currentSequenceNumber = taskIter->getCurrentSequencePosition();
      std::shared_ptr<mars::simulation::ctv::agent::Assignment> nextAssignment;
      nextAssignment = taskIter->getAssignment(currentSequenceNumber);

      if (nextAssignment != nullptr)
      {
        std::shared_ptr<mars::simulation::ctv::agent::MotionAssignment> motionAssignment;
        motionAssignment =
            std::dynamic_pointer_cast<mars::simulation::ctv::agent::MotionAssignment>(
                nextAssignment);
        if (motionAssignment != nullptr)
        {
          return nextAssignment;
        }
      }
    }
  }
  return nullptr;
}

std::list<mars::simulation::ctv::agent::Task>::iterator
mars::simulation::ctv::agent::TaskManager::getTask(mars::common::Id taskId)
{
  std::list<mars::simulation::ctv::agent::Task>::iterator iter;

  for (iter = this->mTaskQueue.begin(); iter != this->mTaskQueue.end();
       ++iter)
  {
    if (iter->getTaskId() == taskId)
      return iter;
  }

  return iter;
}

mars::common::Id
mars::simulation::ctv::agent::TaskManager::getLastFinishedActionId() const
{
  return mLastFinishedActionId;
}

mars::common::Id
mars::simulation::ctv::agent::TaskManager::getLastFinishedMotionId() const
{
  return mLastFinishedMotionId;
}

mars::common::Id
mars::simulation::ctv::agent::TaskManager::getCurrentActionId() const
{
  return mCurrentActionId;
}

mars::common::Id
mars::simulation::ctv::agent::TaskManager::getCurrentMotionId() const
{
  return mCurrentMotionId;
}

mars::common::Id
mars::simulation::ctv::agent::TaskManager::getCurrentTaskId() const
{
  return mCurrentTaskId;
}
