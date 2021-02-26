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

#include "mars_simulation_ctv_agent/assignment_type/Assignment.h"

mars::common::Id mars::simulation::ctv::agent::Assignment::getTaskId() const
{
  return this->mTaskId;
}

mars::common::Id mars::simulation::ctv::agent::Assignment::getRobotId() const
{
  return this->mRobotId;
}

unsigned int mars::simulation::ctv::agent::Assignment::getSequencePosition() const
{
  return this->mSequencePosition;
}


