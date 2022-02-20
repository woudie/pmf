#include "pmf_simulation/DcMotorSim.h"

namespace pmf_simulation
{

DcMotorSim::DcMotorSim(ros::NodeHandle n,
                       const gazebo::physics::JointPtr &joint,
                       const gazebo::physics::LinkPtr &link)
{
  joint_ = joint;
  ros::param::get("/Motor/inductance", props_.inductance);
  ros::param::get("/Motor/resistance", props_.resistance);
  ros::param::get("/Motor/torque_constant", props_.torque_constant);
  ros::param::get("/Motor/backemf_constant", props_.backemf_constant);
  ros::param::get("/Motor/max_current", props_.max_current);
  ros::param::get("/Motor/max_voltage", props_.max_voltage);

  current_ = 0.0;
}

// TODO: Improve motor simulation w/ additional constants and saturation effects
void DcMotorSim::step(double ts, double voltage_in, double load_torque)
{
  current_ += ts / props_.inductance * (voltage_in - props_.resistance * current_ - props_.torque_constant * joint_->GetVelocity(0));
  if (current_ > props_.max_current){
    current_ = props_.max_current;
  }else if (current_ < -props_.max_current){
    current_ = -props_.max_current;
  }
  joint_->SetForce(0, current_ * props_.torque_constant);
}

}