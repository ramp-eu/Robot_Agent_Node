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

#include "mars_simulation_ctv_agent/Task.h"
const unsigned int firstSequencePosition = 1; //The first Assignment has this SequenceNumber
mars::simulation::ctv::agent::Task::Task(const mars::common::Id taskId,
                                           const unsigned int taskLength)
    : mTaskId(taskId), mTaskLength(taskLength), mCurrentSequencePosition(firstSequencePosition)
{
}

mars::common::Id mars::simulation::ctv::agent::Task::getTaskId() const
{
  return mTaskId;
}

unsigned int mars::simulation::ctv::agent::Task::getTaskLength() const
{
  return mTaskLength;
}

std::shared_ptr<mars::simulation::ctv::agent::Assignment>
mars::simulation::ctv::agent::Task::getNextAssignment()
{
  std::shared_ptr<mars::simulation::ctv::agent::Assignment> nextAssignment;
  nextAssignment = this->mAssignments.front();
  this->mAssignments.pop_front();

  // increment the current sequence position
  this->mCurrentSequencePosition++;

  return nextAssignment;
}

std::shared_ptr<mars::simulation::ctv::agent::Assignment>
mars::simulation::ctv::agent::Task::getAssignment(unsigned int Position)
{
    std::deque<std::shared_ptr<mars::simulation::ctv::agent::Assignment>>::iterator iter;
    std::shared_ptr<mars::simulation::ctv::agent::Assignment> Assignment = nullptr;

    for(iter = mAssignments.begin(); iter != mAssignments.end(); ++iter)
    {
        if((*iter)->getSequencePosition() == Position)
        {
            Assignment = (*iter);
        }
    }
    return Assignment;
}

bool mars::simulation::ctv::agent::Task::isNextAssignmentAvailable()
{
    // Check if the next assignment is really the next in the sequence
    if(!mAssignments.empty())
    {
        if ((*(mAssignments.begin()))->getSequencePosition() ==
                this->mCurrentSequencePosition)
        {
            //it is the next
            return true;
        }
    }
    else
    {
        //it is not the next, means the next assignment in the task is missing.
        return false;
    }
}

bool mars::simulation::ctv::agent::Task::isEmpty()
{
  return !(this->mAssignments.empty());
}

bool mars::simulation::ctv::agent::Task::isDone()
{
  this->mLastAssignmentFinished = false;
  if(this->mCurrentSequencePosition >= this->mTaskLength+firstSequencePosition)
  {
    this->mLastAssignmentFinished = true;
    return true;
  }
  else
  {
    return false;
  }
}

void mars::simulation::ctv::agent::Task::addAssignment(const mars_agent_physical_robot_msgs::ActionAssignmentConstPtr &msg)
{
  std::deque<std::shared_ptr<mars::simulation::ctv::agent::Assignment>>::
      const_iterator insertPosition;
  std::shared_ptr<mars::simulation::ctv::agent::ActionAssignment> action =
      std::make_shared<mars::simulation::ctv::agent::ActionAssignment>(msg);

  if (this->mReceivedTaskIDs.find(action->getActionId()) == this->mReceivedTaskIDs.end())
  {
    insertPosition = calcInsertPosition(msg->sequence.sequence_number);
    this->mReceivedTaskIDs.insert(action->getActionId());

    this->mAssignments.insert(insertPosition, action);
  }
}

void mars::simulation::ctv::agent::Task::addAssignment(const mars_agent_physical_robot_msgs::MotionAssignmentConstPtr &msg)
{
  std::deque<std::shared_ptr<mars::simulation::ctv::agent::Assignment>>::
      const_iterator insertPosition;
  std::shared_ptr<mars::simulation::ctv::agent::MotionAssignment> motion =
      std::make_shared<mars::simulation::ctv::agent::MotionAssignment>(msg);

  if (this->mReceivedTaskIDs.find(motion->getMotionId()) == this->mReceivedTaskIDs.end())
  {
    insertPosition = calcInsertPosition(msg->sequence.sequence_number);
    this->mReceivedTaskIDs.insert(motion->getMotionId());

    this->mAssignments.insert(insertPosition, motion);
  }
}

bool mars::simulation::ctv::agent::Task::getLastAssignmentFinished() const
{
    return mLastAssignmentFinished;
}

unsigned int mars::simulation::ctv::agent::Task::getCurrentSequencePosition()
{
    return (this->mCurrentSequencePosition);
}

std::deque<
std::shared_ptr<mars::simulation::ctv::agent::Assignment>>::const_iterator
mars::simulation::ctv::agent::Task::calcInsertPosition(
    unsigned int sequencePosition)
{
  std::deque<std::shared_ptr<mars::simulation::ctv::agent::Assignment>>::iterator
      iter;

  for (iter = mAssignments.begin(); iter != mAssignments.end(); ++iter)
  {
    if ((*iter)->getSequencePosition() > sequencePosition)
    {
      break;
    }
  }
  return iter;
}
