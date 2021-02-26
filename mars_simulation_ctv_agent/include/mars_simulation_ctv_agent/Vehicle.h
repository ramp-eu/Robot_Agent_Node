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

#ifndef VEHICLE_H
#define VEHICLE_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <tf/tf.h>
#include <mars_agent_physical_robot_msgs/VehicleType.h>
#include <mars_agent_physical_robot_msgs/RobotAction.h>

namespace mars
{
namespace simulation
{
namespace ctv
{
namespace agent
{
class Vehicle
{
public:
  Vehicle();
  double getMaxPosXVel() const;
  void setMaxPosXVel(double MaxPosXVel);

  double getMaxNegXVel() const;
  void setMaxNegXVel(double MaxNegXVel);

  double getMaxPosXAcc() const;
  void setMaxPosXAcc(double MaxPosXAcc);

  double getMaxNegXAcc() const;
  void setMaxNegXAcc(double MaxNegXAcc);

  double getMaxPosYVel() const;
  void setMaxPosYVel(double MaxPosYVel);

  double getMaxNegYVel() const;
  void setMaxNegYVel(double MaxNegYVel);

  double getMaxPosYAcc() const;
  void setMaxPosYAcc(double MaxPosYAcc);

  double getMaxNegYAcc() const;
  void setMaxNegYAcc(double MaxNegYAcc);

  double getMaxPosAngVel() const;
  void setMaxPosAngVel(double MaxPosAngVel);

  double getMaxNegAngVel() const;
  void setMaxNegAngVel(double MaxNegAngVel);

  double getMaxPosAngAcc() const;
  void setMaxPosAngAcc(double MaxPosAngAcc);

  double getMaxNegAngAcc() const;
  void setMaxNegAngAcc(double MaxNegAngAcc);

  double getBattCapacity() const;
  void setBattCapacity(double battCapacity);

  double getBattMaxVoltage() const;
  void setBattMaxVoltage(double battMaxVoltage);

  double getCurrentOrientation() const;

  double getCurrentCordX() const;

  double getCurrentCordY() const;

  double getCurrentLinearVel() const;
  void setCurrentLinearVel(double currentLinearVel);

  double getCurrentAngularVel() const;
  void setCurrentAngularVel(double currentAngularVel);

  double getCurrentDistanceToGoal() const;
  void setCurrentDistanceToGoal(double currentDistanceToGoal);

  void setCurrentPose(const geometry_msgs::PoseStamped &currentPose);
  geometry_msgs::PoseStamped getCurrentPose() const;

  double getTurningRadius() const;
  void setTurningRadius(double turningRadius);

  double getMinHeight() const;
  void setMinHeight(double minHeight);

  double getMaxHeight() const;
  void setMaxHeight(double maxHeight);

  double getPayload() const;
  void setPayload(double payload);

  geometry_msgs::PolygonStamped getFootprint() const;
  void setFootprint(const geometry_msgs::PolygonStamped &footprint);

  mars_agent_physical_robot_msgs::VehicleType getType() const;
  void setType(const mars_agent_physical_robot_msgs::VehicleType vehicleType);

  std::string getRobot_id() const;
  void setRobot_id(const std::string &value);

  double getVelocityControlSensitivity() const;
  void setVelocityControlSensitivity(double velocityControlSensitivity);

  std::string getVendor() const;
  void setVendor(const std::string &vendor);

  mars_agent_physical_robot_msgs::RobotAction getActionCapability() const;
  void setActionCapability(const mars_agent_physical_robot_msgs::RobotAction &actionCapability);

private:
  // RobotAgentProperties
  std::string robot_id;
  mars_agent_physical_robot_msgs::VehicleType mType;
  geometry_msgs::PolygonStamped mFootprint;

  double mMinHeight;
  double mMaxHeight;
  double mPayload;

  double mMaxPosXVel;
  double mMaxNegXVel;
  double mMaxPosXAcc;
  double mMaxNegXAcc;
  double mMaxPosYVel;
  double mMaxNegYVel;
  double mMaxPosYAcc;
  double mMaxNegYAcc;
  double mMaxPosAngVel;
  double mMaxNegAngVel;
  double mMaxPosAngAcc;
  double mMaxNegAngAcc;

  double mVelocityControlSensitivity;
  double mTurningRadius;
  double mBattCapacity;
  double mBattMaxVoltage;

  std::string mVendor;
  mars_agent_physical_robot_msgs::RobotAction mActionCapability;


  //current vehicle velocities and vehicle pose
  double mCurrentLinearVel;
  double mCurrentAngularVel;
  geometry_msgs::PoseStamped mCurrentPose;

  //Meta Info
  double mCurrentDistanceToGoal;
};
}
}
}
}
#endif // VEHICLE_H
