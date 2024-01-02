
#include <QPInverseKinematics.h>

bool QPInverseKinematics::_setCostAndConstraints(Robot& robot)
{
    _n_var = robot.getNrOfDegreesOfFreedom() + 6;
    _list_costs.emplace_back(std::make_unique<CostConfigurationVelocity>(_n_var));
    _list_costs.emplace_back(std::make_unique<CostErrorDesiredConfigurationVelocity>(_n_var, _frameName_ee, _w_p_ee_des));
    _list_constraints.emplace_back(std::make_unique<ConstraintBaseVel>(_n_var));
    _list_constraints.emplace_back(std::make_unique<ConstraintJointVel>(_n_var, robot.getNrOfDegreesOfFreedom()));
    return true;
};

bool QPInverseKinematics::setDesiredFramePosition(iDynTree::Position w_p_ee_des, std::string frameName_ee)
{
    yInfo() << "QPInverseKinematics::setDesiredFramePosition: setting desired position";
    _w_p_ee_des = w_p_ee_des;
    _frameName_ee = frameName_ee;
    return true;
}

// Constraints and Costs classes used in the QP

bool ConstraintBaseVel::compute(Robot& robot, Eigen::Ref<Eigen::MatrixXd> linearMatrix, Eigen::Ref<Eigen::VectorXd> lowerBound, Eigen::Ref<Eigen::VectorXd> upperBound, unsigned int& count_constraints){
    linearMatrix.block(count_constraints, 0, _n_constraints, _n_var) = iDynTree::toEigen(robot.getJacobian("base_link"));
    return true;
};

bool ConstraintJointVel::compute(Robot& robot, Eigen::Ref<Eigen::MatrixXd> linearMatrix, Eigen::Ref<Eigen::VectorXd> lowerBound, Eigen::Ref<Eigen::VectorXd> upperBound, unsigned int& count_constraints){
    linearMatrix.block(count_constraints, 6, _n_constraints, _n_constraints) = Eigen::MatrixXd::Identity(_n_constraints, _n_constraints);
    lowerBound.segment(count_constraints, _n_constraints) = - speed_limit * Eigen::VectorXd::Ones(_n_constraints);
    upperBound.segment(count_constraints, _n_constraints) = speed_limit * Eigen::VectorXd::Ones(_n_constraints); 
    return true;
};

bool CostConfigurationVelocity::compute(Robot& robot, Eigen::Ref<Eigen::MatrixXd> hessian, Eigen::Ref<Eigen::VectorXd> gradient){
    hessian += _gain * Eigen::MatrixXd::Identity(_n_var,_n_var);
    gradient += _gain * Eigen::VectorXd::Zero(_n_var);
    return true;
};

bool CostErrorDesiredConfigurationVelocity::configure(Robot& robot){
    _J_ee_pos.resize(3, robot.getNrOfDegreesOfFreedom() + 6);
    return true;
};

bool CostErrorDesiredConfigurationVelocity::compute(Robot& robot, Eigen::Ref<Eigen::MatrixXd> hessian, Eigen::Ref<Eigen::VectorXd> gradient){
    _J_ee_pos = iDynTree::toEigen(robot.getJacobian(_frameName_ee)).block(0, 0, 3, robot.getNrOfDegreesOfFreedom() + 6);
    iDynTree::Position _w_p_ee = robot.getWorldTransform(_frameName_ee).getPosition();
    hessian += (iDynTree::toEigen(_J_ee_pos)).transpose() * iDynTree::toEigen(_J_ee_pos);
    gradient += iDynTree::toEigen(_J_ee_pos).transpose() * iDynTree::toEigen(_w_p_ee - _w_p_ee_des);
    return true;
};
