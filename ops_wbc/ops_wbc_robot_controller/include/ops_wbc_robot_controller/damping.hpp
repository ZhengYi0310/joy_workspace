#ifndef __OPS_WBC_ROBOT_CONTROLLER_DAMPING_HPP
#define __OPS_WBC_ROBOT_CONTROLLER_DAMPING_HPP

#include <ops_wbc_robot/ops_wbc_robot.hpp>
#include "ops_wbc_robot_controller/task.hpp"

namespace ops_wbc_robot_controller
{

  class Damping : public Task
  {
  public:
    explicit Damping(const ops_wbc_robot::RobotPtr& robot);
    virtual const std::string& getName() const override { return task::DAMPING; }
    virtual void computeGeneralizedForce(Eigen::VectorXd& tau) override;

  private:
    ops_wbc_robot::RobotPtr robot_;
  };

} // namespace ahl_ctrl

#endif // __AHL_ROBOT_CONTROLLER_DAMPING_HPP
