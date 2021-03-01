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
