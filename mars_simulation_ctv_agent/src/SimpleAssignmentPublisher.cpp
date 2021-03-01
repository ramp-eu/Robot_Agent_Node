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


#include <ros/ros.h>
#include <mars_agent_physical_robot_msgs/ActionAssignment.h>
#include <mars_agent_physical_robot_msgs/MotionAssignment.h>
#include <mars_agent_physical_robot_msgs/CancelTask.h>
#include <mars_agent_physical_robot_msgs/AssignmentStatus.h>
#include <mars_common/Id.h>
#include <string>

static const std::string STD_ASSIGNMENT_PARAM_NAME = "assignment_";

class SimpleAssignmentPublisher{

//methods
public:
  SimpleAssignmentPublisher();
  ~SimpleAssignmentPublisher(){}

  void pubTestStraightdirection(int id);
  void assignmentStateSubscriberCallback(const mars_agent_physical_robot_msgs::AssignmentStatusConstPtr &msg);
  void init();
  void run();

  void fillString(std::string& str);

//members
public:
  ros::Publisher mMotionPub0;
  ros::Publisher mActionPub0;
  ros::Publisher mCancelTaskPub0;
  ros::Publisher mMotionPub1;
  ros::Publisher mActionPub1;
  ros::Publisher mMotionPub2;
  ros::Publisher mActionPub2;
  ros::Publisher mMotionPub3;
  ros::Publisher mActionPub3;
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;
  ros::Subscriber mAssignmentStatusSub;
  int round_counter = 1;
  std::map<int, std::vector<double>> motionAssignmentMap;
  int motionAssignmentCounter = 0;
};


SimpleAssignmentPublisher::SimpleAssignmentPublisher() :  nh(), private_nh("~")
{
  //register publisher and subscriber
  mMotionPub0 = nh.advertise<mars_agent_physical_robot_msgs::MotionAssignment>("/motion_assignment", 10, true);
  mActionPub0 = nh.advertise<mars_agent_physical_robot_msgs::ActionAssignment>("/action_assignment", 10, true);
  mCancelTaskPub0 = nh.advertise<mars_agent_physical_robot_msgs::CancelTask>("/cancel_task", 10, true);

  mMotionPub1 = nh.advertise<mars_agent_physical_robot_msgs::MotionAssignment>("/robot_1/motion_assignment", 10, true);
  mActionPub1 = nh.advertise<mars_agent_physical_robot_msgs::ActionAssignment>("/robot_1/action_assignment", 10, true);

  mMotionPub2 = nh.advertise<mars_agent_physical_robot_msgs::MotionAssignment>("/robot_2/motion_assignment", 10, true);
  mActionPub2 = nh.advertise<mars_agent_physical_robot_msgs::ActionAssignment>("/robot_2/action_assignment", 10, true);

  mMotionPub3 = nh.advertise<mars_agent_physical_robot_msgs::MotionAssignment>("/robot_3/motion_assignment", 10, true);
  mActionPub3 = nh.advertise<mars_agent_physical_robot_msgs::ActionAssignment>("/robot_3/action_assignment", 10, true);

  mAssignmentStatusSub =  this->nh.subscribe<mars_agent_physical_robot_msgs::AssignmentStatus>(
        "/assignment_state", 1, boost::bind(&SimpleAssignmentPublisher::assignmentStateSubscriberCallback, this, _1));
}

void SimpleAssignmentPublisher::assignmentStateSubscriberCallback(const mars_agent_physical_robot_msgs::AssignmentStatusConstPtr &msg){
  mars_common_msgs::Id current_motion_id = msg->current_motion_id;
  mars_common_msgs::Id last_finished_motion = msg->last_finished_motion;
  if(current_motion_id.uuid == last_finished_motion.uuid){
    run();
    std::cout << "Round: " << round_counter++ << std::endl; 
  } else {
    std::cout << "A sequence is executed." << std::endl;
    sleep(1);
  }
}

void SimpleAssignmentPublisher::pubTestStraightdirection(int id)
{
  mars_agent_physical_robot_msgs::MotionAssignment motion1;
  std::string robot = "robot1";
  std::string motion = "motion" + id+1;
  std::string task = "task1";
  std::string point = "point1";

  fillString(robot);
  fillString(motion);
  fillString(task);
  fillString(point);

  mars::common::Id robotId(robot, "bla", mars::common::Id::UUIDCreationType::CREATE_FROM_STRING);
  mars::common::Id motionId(motion, "bla", mars::common::Id::UUIDCreationType::CREATE_FROM_STRING);
  mars::common::Id taskId(task, "bla", mars::common::Id::UUIDCreationType::CREATE_FROM_STRING);
  mars::common::Id pointId(point, "bla", mars::common::Id::UUIDCreationType::CREATE_FROM_STRING);


  motion1.motion_id = mars::common::Id::convertToMsgId(motionId);
  motion1.task_id = mars::common::Id::convertToMsgId(taskId);
  motion1.is_waypoint = false;
  motion1.use_orientation = true;
  motion1.point_id = mars::common::Id::convertToMsgId(pointId);
  motion1.point.x = motionAssignmentMap[id][0];
  motion1.point.y = motionAssignmentMap[id][1];
  motion1.point.theta = motionAssignmentMap[id][2];
  motion1.max_velocity.linear.x = motionAssignmentMap[id][3];
  motion1.max_velocity.angular.z = motionAssignmentMap[id][4];
  motion1.max_acceleration.linear.x = motionAssignmentMap[id][5];
  motion1.max_acceleration.angular.z = motionAssignmentMap[id][6];

  motion1.sequence.length = motionAssignmentMap.size();
  motion1.sequence.sequence_number = id+1;

  motion1.header.stamp = ros::Time::now();

  mMotionPub0.publish(motion1);
}


void SimpleAssignmentPublisher::fillString(std::string& str)
{
  for (int i = str.length(); str.length() < 32; i++)
  {
    str.append("+");
  }
}

void SimpleAssignmentPublisher::run(){
  for(int i = 0; i < motionAssignmentMap.size(); i++){
    this->pubTestStraightdirection(i);
  }
}

void SimpleAssignmentPublisher::init(){
  if(nh.hasParam(STD_ASSIGNMENT_PARAM_NAME + std::to_string(this->motionAssignmentCounter))){
    ROS_INFO_STREAM(STD_ASSIGNMENT_PARAM_NAME + std::to_string(this->motionAssignmentCounter) << " found.");
    nh.getParam(STD_ASSIGNMENT_PARAM_NAME + std::to_string(this->motionAssignmentCounter), this->motionAssignmentMap[this->motionAssignmentCounter]);
    nh.deleteParam(STD_ASSIGNMENT_PARAM_NAME + std::to_string(this->motionAssignmentCounter));
    this->motionAssignmentCounter++;
    init();
  }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_assignment_publisher");

    SimpleAssignmentPublisher assignmentPublisher;
    sleep(1);

    assignmentPublisher.init();
    assignmentPublisher.run();

    // Main Loop
    ros::Rate r(1);
    while(assignmentPublisher.nh.ok())
    {

    ros::spinOnce();
    r.sleep();
    }

    return(0);
}
