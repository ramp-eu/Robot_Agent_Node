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

#include "mars_simulation_ctv_agent/Vehicle.h"

mars::simulation::ctv::agent::Vehicle::Vehicle()
{

}

double mars::simulation::ctv::agent::Vehicle::getMaxPosXVel() const
{
    return mMaxPosXVel;
}

void mars::simulation::ctv::agent::Vehicle::setMaxPosXVel(double MaxPosXVel)
{
    this->mMaxPosXVel = MaxPosXVel;
}

double mars::simulation::ctv::agent::Vehicle::getMaxNegXVel() const
{
    return mMaxNegXVel;
}

void mars::simulation::ctv::agent::Vehicle::setMaxNegXVel(double MaxNegXVel)
{
    this->mMaxNegXVel = MaxNegXVel;
}

double mars::simulation::ctv::agent::Vehicle::getMaxPosYVel() const
{
    return mMaxPosYVel;
}

void mars::simulation::ctv::agent::Vehicle::setMaxPosYVel(double MaxPosYVel)
{
    this->mMaxPosYVel = MaxPosYVel;
}

double mars::simulation::ctv::agent::Vehicle::getMaxNegYVel() const
{
    return mMaxNegYVel;
}

void mars::simulation::ctv::agent::Vehicle::setMaxNegYVel(double MaxNegYVel)
{
    this->mMaxNegYVel = MaxNegYVel;
}

double mars::simulation::ctv::agent::Vehicle::getMaxPosXAcc() const
{
    return mMaxPosXAcc;
}

void mars::simulation::ctv::agent::Vehicle::setMaxPosXAcc(double MaxPosXAcc)
{
    this->mMaxPosXAcc = MaxPosXAcc;
}

double mars::simulation::ctv::agent::Vehicle::getMaxNegXAcc() const
{
    return mMaxNegXAcc;
}

void mars::simulation::ctv::agent::Vehicle::setMaxNegXAcc(double MaxNegXAcc)
{
    this->mMaxNegXAcc = MaxNegXAcc;
}

double mars::simulation::ctv::agent::Vehicle::getMaxPosYAcc() const
{
    return mMaxPosYAcc;
}

void mars::simulation::ctv::agent::Vehicle::setMaxPosYAcc(double MaxPosYAcc)
{
    this->mMaxPosYAcc = MaxPosYAcc;
}

double mars::simulation::ctv::agent::Vehicle::getMaxNegYAcc() const
{
    return mMaxNegYAcc;
}

void mars::simulation::ctv::agent::Vehicle::setMaxNegYAcc(double MaxNegYAcc)
{
    this->mMaxNegYAcc = MaxNegYAcc;
}

double mars::simulation::ctv::agent::Vehicle::getMaxPosAngVel() const
{
    return mMaxPosAngVel;
}

void mars::simulation::ctv::agent::Vehicle::setMaxPosAngVel(double MaxPosAngVel)
{
    this->mMaxPosAngVel = MaxPosAngVel;
}

double mars::simulation::ctv::agent::Vehicle::getMaxNegAngVel() const
{
    return mMaxNegAngVel;
}

void mars::simulation::ctv::agent::Vehicle::setMaxNegAngVel(double MaxNegAngVel)
{
    this->mMaxNegAngVel = MaxNegAngVel;
}

double mars::simulation::ctv::agent::Vehicle::getMaxPosAngAcc() const
{
    return this->mMaxPosAngAcc;
}

void mars::simulation::ctv::agent::Vehicle::setMaxPosAngAcc(double MaxPosAngAcc)
{
    this->mMaxPosAngAcc = MaxPosAngAcc;
}

double mars::simulation::ctv::agent::Vehicle::getMaxNegAngAcc() const
{
    return this->mMaxNegAngAcc;
}

void mars::simulation::ctv::agent::Vehicle::setMaxNegAngAcc(double MaxNegAngAcc)
{
    this->mMaxNegAngAcc = MaxNegAngAcc;
}

double mars::simulation::ctv::agent::Vehicle::getBattCapacity() const
{
  return mBattCapacity;
}

void mars::simulation::ctv::agent::Vehicle::setBattCapacity(double battCapacity)
{
  this->mBattCapacity = battCapacity;
}

double mars::simulation::ctv::agent::Vehicle::getBattMaxVoltage() const
{
    return mBattMaxVoltage;
}

void mars::simulation::ctv::agent::Vehicle::setBattMaxVoltage(double battMaxVoltage)
{
    this->mBattMaxVoltage = battMaxVoltage;
}

double mars::simulation::ctv::agent::Vehicle::getCurrentOrientation() const
{
  return (tf::getYaw(mCurrentPose.pose.orientation));
}


double mars::simulation::ctv::agent::Vehicle::getCurrentCordX() const
{
  return mCurrentPose.pose.position.x;
}

double mars::simulation::ctv::agent::Vehicle::getCurrentCordY() const
{
  return mCurrentPose.pose.position.y;
}

double mars::simulation::ctv::agent::Vehicle::getCurrentAngularVel() const
{
  return mCurrentAngularVel;
}

void mars::simulation::ctv::agent::Vehicle::setCurrentAngularVel(double currentAngularVel)
{
  mCurrentAngularVel = currentAngularVel;
}

double mars::simulation::ctv::agent::Vehicle::Vehicle::getCurrentDistanceToGoal() const
{
  return mCurrentDistanceToGoal;
}

void mars::simulation::ctv::agent::Vehicle::Vehicle::setCurrentDistanceToGoal(double currentDistanceToGoal)
{
  mCurrentDistanceToGoal = currentDistanceToGoal;
}

void mars::simulation::ctv::agent::Vehicle::setCurrentPose(const geometry_msgs::PoseStamped &currentPose)
{
  this->mCurrentPose = currentPose;
}

geometry_msgs::PoseStamped mars::simulation::ctv::agent::Vehicle::getCurrentPose() const
{
    return mCurrentPose;
}

double mars::simulation::ctv::agent::Vehicle::getPayload() const
{
    return mPayload;
}

void mars::simulation::ctv::agent::Vehicle::setPayload(double payload)
{
    mPayload = payload;
}

geometry_msgs::PolygonStamped mars::simulation::ctv::agent::Vehicle::getFootprint() const
{
    return mFootprint;
}

void mars::simulation::ctv::agent::Vehicle::setFootprint(const geometry_msgs::PolygonStamped &footprint)
{
    mFootprint = footprint;
}

std::string mars::simulation::ctv::agent::Vehicle::getRobot_id() const
{
    return robot_id;
}

void mars::simulation::ctv::agent::Vehicle::setRobot_id(const std::string &value)
{
    robot_id = value;
}

double mars::simulation::ctv::agent::Vehicle::getVelocityControlSensitivity() const
{
    return mVelocityControlSensitivity;
}

void mars::simulation::ctv::agent::Vehicle::setVelocityControlSensitivity(double velocityControlSensitivity)
{
    mVelocityControlSensitivity = velocityControlSensitivity;
}

mars_agent_physical_robot_msgs::VehicleType mars::simulation::ctv::agent::Vehicle::getType() const
{
    return mType;
}

void mars::simulation::ctv::agent::Vehicle::setType(const mars_agent_physical_robot_msgs::VehicleType vehicleType)
{
    mType.vehicle_type = vehicleType.vehicle_type;
}

std::string mars::simulation::ctv::agent::Vehicle::getVendor() const
{
    return mVendor;
}

void mars::simulation::ctv::agent::Vehicle::setVendor(const std::string &vendor)
{
    mVendor = vendor;
}

mars_agent_physical_robot_msgs::RobotAction mars::simulation::ctv::agent::Vehicle::getActionCapability() const
{
    return mActionCapability;
}

void mars::simulation::ctv::agent::Vehicle::setActionCapability(const mars_agent_physical_robot_msgs::RobotAction &actionCapability)
{
    mActionCapability = actionCapability;
}

double mars::simulation::ctv::agent::Vehicle::getMaxHeight() const
{
    return mMaxHeight;
}

void mars::simulation::ctv::agent::Vehicle::setMaxHeight(double maxHeight)
{
    mMaxHeight = maxHeight;
}

double mars::simulation::ctv::agent::Vehicle::getMinHeight() const
{
    return mMinHeight;
}

void mars::simulation::ctv::agent::Vehicle::setMinHeight(double minHeight)
{
    mMinHeight = minHeight;
}

double mars::simulation::ctv::agent::Vehicle::getTurningRadius() const
{
    return mTurningRadius;
}

void mars::simulation::ctv::agent::Vehicle::setTurningRadius(double turningRadius)
{
    mTurningRadius = turningRadius;
}

double mars::simulation::ctv::agent::Vehicle::getCurrentLinearVel() const
{
    return mCurrentLinearVel;
}

void mars::simulation::ctv::agent::Vehicle::setCurrentLinearVel(double currentLinearVel)
{
  mCurrentLinearVel = currentLinearVel;
}
