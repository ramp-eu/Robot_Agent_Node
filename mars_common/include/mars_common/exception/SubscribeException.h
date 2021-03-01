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


#ifndef MARS_COMMON_SUBSCRIBEEXCEPTION_H
#define MARS_COMMON_SUBSCRIBEEXCEPTION_H

#include <string>

namespace mars
{
namespace common
{
namespace exception
{
/**
 * @brief The SubscribeException class
 */
class SubscribeException : public std::exception
{
public:
  SubscribeException(const std::string& msg) : msg(msg) {}

  ~SubscribeException(void) throw(){}

  virtual const char* what() const throw() { return msg.c_str(); }

private:
  std::string msg;
};
} // namespace exception
} // namespace common
} // namespace mars

#endif // MARS_COMMON_SUBSCRIBEEXCEPTION_H
