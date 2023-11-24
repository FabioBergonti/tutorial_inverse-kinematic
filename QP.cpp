#include <QP.h>

bool QPControlProblem::configure(Robot& robot)
{
    _n_var = robot.getNrOfDegreesOfFreedom() + 6;
    _hessian.resize(_n_var, _n_var);
    _gradient.resize(_n_var);
    _linearMatrix.resize(0, _n_var);
    _lowerBound.resize(0);
    _upperBound.resize(0);
    _count_constraints = 0;

    _minimiseConfigurationVelocity(robot, 1, true);
    _minimiseErrorDesiredConfigurationVelocity(robot, 1, true);
    _constraintBaseVel(robot, true);
    _boundJointVel(robot, 0.10, true);

    _n_constraints = _count_constraints;

    _solver.settings()->setWarmStart(true);
    _solver.settings()->setVerbosity(false);
    _solver.data()->setNumberOfVariables(_n_var);
    _solver.data()->setNumberOfConstraints(_n_constraints);
    _outputQP = Eigen::VectorXd::Zero(_n_var);
    return true;
}

bool QPControlProblem::update(Robot& robot)
{
    _hessian.setZero();
    _gradient.setZero();
    _count_constraints = 0;

    _minimiseConfigurationVelocity(robot, 1);
    _minimiseErrorDesiredConfigurationVelocity(robot, 1);
    _constraintBaseVel(robot);
    _boundJointVel(robot, 0.10);

    return true;
}

bool QPControlProblem::solve()
{
    _hessianSparse = _hessian.sparseView();
    _linearMatrixSparse = _linearMatrix.sparseView();

    if (!_solver.isInitialized()){
        yInfo() << "QPControlProblem::solve: initialising solver";
        if (!_solver.data()->setHessianMatrix(_hessianSparse))
            return false;
        if (!_solver.data()->setGradient(_gradient))
            return false;
        if (!_solver.data()->setLinearConstraintsMatrix(_linearMatrixSparse))
            return false;
        if (!_solver.data()->setLowerBound(_lowerBound))
            return false;
        if (!_solver.data()->setUpperBound(_upperBound))
            return false;
        if (!_solver.initSolver())
            return false;
    }
    else {
        // yInfo() << "QPControlProblem::solve: updating solver";
        if (!_solver.updateHessianMatrix(_hessianSparse))
            return false;
        if (!_solver.updateGradient(_gradient))
            return false;
        if (!_solver.updateLinearConstraintsMatrix(_linearMatrixSparse))
            return false;
        if (!_solver.updateBounds(_lowerBound, _upperBound))
            return false;
        if (_solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
            return false;
    }
    _outputQP = _solver.getSolution();
    return true;
}

Eigen::VectorXd QPControlProblem::getSolution()
{
    return _outputQP;
}

bool QPControlProblem::setDesiredFramePosition(iDynTree::Position w_p_ee_des, std::string frameName_ee)
{
    yInfo() << "QPControlProblem::setDesiredFramePosition: setting desired position";
    _w_p_ee_des = w_p_ee_des;
    _frameName_ee = frameName_ee;
    return true;
}

bool QPControlProblem::_minimiseConfigurationVelocity(Robot& robot, const double gain, const bool configure)
{
    if (configure){
        yInfo() << "QPControlProblem::_minimiseConfigurationVelocity: configuring";
    }
    _hessian += gain * Eigen::MatrixXd::Identity(_n_var,_n_var);
    _gradient += gain * Eigen::VectorXd::Zero(_n_var);
    return true;
}

bool QPControlProblem::_minimiseErrorDesiredConfigurationVelocity(Robot& robot, const double gain, const bool configure)
{
    if (configure){
    yInfo() << "QPControlProblem::_minimiseConfigurationVelocity: configuring";
    _J_ee_pos.resize(3, robot.getNrOfDegreesOfFreedom() + 6);
    }
    _J_ee_pos = iDynTree::toEigen(robot.getJacobian(_frameName_ee)).block(0, 0, 3, robot.getNrOfDegreesOfFreedom() + 6);
    _w_p_ee = robot.getWorldTransform(_frameName_ee).getPosition();
    _hessian += (iDynTree::toEigen(_J_ee_pos)).transpose() * iDynTree::toEigen(_J_ee_pos);
    _gradient += iDynTree::toEigen(_J_ee_pos).transpose() * iDynTree::toEigen(_w_p_ee - _w_p_ee_des);
    return true;
}

bool QPControlProblem::_constraintBaseVel(Robot& robot, const bool configure)
{
    if (configure){
        yInfo() << "QPControlProblem::_constraintBaseVel: configuring";
        _linearMatrix.conservativeResize(_count_constraints + 6, Eigen::NoChange);
        _lowerBound.conservativeResize(_count_constraints + 6);
        _upperBound.conservativeResize(_count_constraints + 6);
        _linearMatrix.block(_count_constraints, 0, 6, _n_var) = Eigen::MatrixXd::Zero(6, _n_var);
        _lowerBound.segment(_count_constraints, 6) = Eigen::VectorXd::Zero(6);
        _upperBound.segment(_count_constraints, 6) = Eigen::VectorXd::Zero(6);
    }
    _linearMatrix.block(_count_constraints, 0, 6, _n_var) = iDynTree::toEigen(robot.getJacobian(_frameName_base));
    _count_constraints += 6;
    return true;
}

bool QPControlProblem::_boundJointVel(Robot& robot, const double speed_limit, const bool configure)
{
    if (configure){
        yInfo() << "QPControlProblem::_boundJointVel: configuring";
        _linearMatrix.conservativeResize(_count_constraints + robot.getNrOfDegreesOfFreedom(), Eigen::NoChange);
        _lowerBound.conservativeResize(_count_constraints + robot.getNrOfDegreesOfFreedom());
        _upperBound.conservativeResize(_count_constraints + robot.getNrOfDegreesOfFreedom());
        _linearMatrix.block(_count_constraints, 0, robot.getNrOfDegreesOfFreedom(), _n_var) = Eigen::MatrixXd::Zero(robot.getNrOfDegreesOfFreedom(), _n_var);
        _lowerBound.segment(_count_constraints, robot.getNrOfDegreesOfFreedom()) = Eigen::VectorXd::Zero(robot.getNrOfDegreesOfFreedom());
        _upperBound.segment(_count_constraints, robot.getNrOfDegreesOfFreedom()) = Eigen::VectorXd::Zero(robot.getNrOfDegreesOfFreedom());
    }
    _linearMatrix.block(_count_constraints, 6, robot.getNrOfDegreesOfFreedom(), robot.getNrOfDegreesOfFreedom()) = Eigen::MatrixXd::Identity(robot.getNrOfDegreesOfFreedom(), robot.getNrOfDegreesOfFreedom());
    _lowerBound.segment(_count_constraints, robot.getNrOfDegreesOfFreedom()) = - speed_limit * Eigen::VectorXd::Ones(robot.getNrOfDegreesOfFreedom());
    _upperBound.segment(_count_constraints, robot.getNrOfDegreesOfFreedom()) = speed_limit * Eigen::VectorXd::Ones(robot.getNrOfDegreesOfFreedom()); 
    _count_constraints += robot.getNrOfDegreesOfFreedom();  
    return true;
}
