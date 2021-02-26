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
