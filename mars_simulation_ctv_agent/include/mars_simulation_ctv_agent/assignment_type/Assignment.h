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

#ifndef ASSIGNMENT_H
#define ASSIGNMENT_H

// includes
#include <mars_common/Id.h>

namespace mars
{
namespace simulation
{
namespace ctv
{
namespace agent
{

class Assignment
{
public:
  virtual mars::common::Id getTaskId() const;

  virtual mars::common::Id getRobotId() const;

  virtual unsigned int getSequencePosition() const;

protected:
  mars::common::Id mTaskId;

  mars::common::Id mRobotId;

  /**
   * @brief mSequencePosition Position in the sequence
   */
  unsigned int mSequencePosition;
};

} // namespace agent
} // namespace ctv
} // namespace simulation
} // namespace mars

#endif // ASSIGNMENT_H
