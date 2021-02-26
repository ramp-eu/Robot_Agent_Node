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

#ifndef TASK_H
#define TASK_H

// includes
#include <mars_common/Id.h>
#include "mars_simulation_ctv_agent/assignment_type/ActionAssignment.h"
#include "mars_simulation_ctv_agent/assignment_type/MotionAssignment.h"

//msgs include
#include "mars_agent_physical_robot_msgs/ActionAssignment.h"
#include "mars_agent_physical_robot_msgs/MotionAssignment.h"

// C++ includes
#include <list>
#include <deque>
#include <memory>

namespace mars
{
namespace simulation
{
namespace ctv
{
namespace agent
{

class Task
{
public:
  /**
   * @brief Task Constructs an object of this class.
   * @param taskId Id of this task.
   * @param taskLength Number of assignments in this task.
   */
  Task(const mars::common::Id taskId, const unsigned int taskLength);

  /**
   * @brief getTaskId Gets the id of the task.
   * @return Id of this task.
   */
  mars::common::Id getTaskId() const;

  /**
   * @brief getTaskLength Gets the number of assignments of this task.
   * @return Length of this task.
   */
  unsigned int getTaskLength() const;

  /**
   * @brief getNextAssignment Gets next assignment of this task and deletes it from the task.
   * @return The next assignment.
   */
  std::shared_ptr<mars::simulation::ctv::agent::Assignment> getNextAssignment();

  /**
   * @brief getAssignment
   * @param Position
   * @return Assignment
   */
  std::shared_ptr<mars::simulation::ctv::agent::Assignment> getAssignment(unsigned int Position);

  /**
   * @brief isNextAssignmentAvailable Checks if another is available. There is no check, if the assignment is available !
   * @return True if the next assignment is available, False if it is not available.
   */
  bool isNextAssignmentAvailable();

  /**
   * @brief isEmpty Checks if the task is empty.
   * @return True if its empty. False otherwise.
   */
  bool isEmpty();

  /**
   * @brief isDone Checks if the task is done (if the last assignment was requested).
   * @return True if its done. False otherwise.
   */
  bool isDone();

  /**
   * @brief addAssignment Adds a new assignment to the task.
   */
  void addAssignment(const mars_agent_physical_robot_msgs::ActionAssignmentConstPtr &msg);

  /**
   * @brief addAssignment Adds a new assignment to the task.
   */
  void addAssignment(const mars_agent_physical_robot_msgs::MotionAssignmentConstPtr &msg);

  /**
   * @brief getLastAssignmentFinished
   * @return true if Last Assignment in Task is finished
   */
  bool getLastAssignmentFinished() const;

  /**
   * @brief getCurrentSequencePosition
   * @return CurrentSequencePosition
   */
  unsigned int getCurrentSequencePosition();

private:

  /**
   * @brief mTaskId Id of this task.
   */
  const mars::common::Id mTaskId;

  /**
   * @brief mTaskLength Number of assignments in the complete task.
   */
  const unsigned int mTaskLength;

  /**
   * @brief mCurrentSequencePosition Current position in the task assignments.
   */
  unsigned int mCurrentSequencePosition;

  /**
   * @brief mLastAssignmentFinished true if Last Assignment finished
   */
  bool mLastAssignmentFinished;

  /**
   * @brief mAssignments Queue in which the assignments are stored.
   */
  std::deque<std::shared_ptr<mars::simulation::ctv::agent::Assignment>> mAssignments;

  /**
   * @brief calcInsertPosition Calculates the insertion position of a given assignment in the queue.
   * @param sequencePosition Sequencenumber of the given assignment.
   * @return An iterator to the insertion position.
   */
  std::deque<std::shared_ptr<mars::simulation::ctv::agent::Assignment>>::const_iterator calcInsertPosition(unsigned int sequencePosition);

  std::set<mars::common::Id> mReceivedTaskIDs;

};

} // namespace agent
} // namespace ctv
} // namespace simulation
} // namespace mars

#endif // TASK_H
