#include <QP.h>

bool QPControlProblem::configure(Robot& robot)
{
    _n_var = robot.getNrOfDegreesOfFreedom() + 6;
    _list_costs.emplace_back(std::make_unique<CostConfigurationVelocity>(_n_var));
    _list_costs.emplace_back(std::make_unique<CostErrorDesiredConfigurationVelocity>(_n_var, _frameName_ee, _w_p_ee_des));
    _list_constraints.emplace_back(std::make_unique<ConstraintBaseVel>(_n_var));
    _list_constraints.emplace_back(std::make_unique<ConstraintJointVel>(_n_var, robot.getNrOfDegreesOfFreedom()));

    _hessian.resize(_n_var, _n_var);
    _gradient.resize(_n_var);
    _linearMatrix.resize(0, _n_var);
    _lowerBound.resize(0);
    _upperBound.resize(0);

    for (auto& cost : _list_costs){
        cost->init(robot);
    }
    for (auto& cost : _list_costs){
        cost->evaluate(robot, _hessian, _gradient);
    }

    _count_constraints = 0;
    for (auto& constraint : _list_constraints){
        constraint->init(robot, _linearMatrix, _lowerBound, _upperBound, _count_constraints);
    }
    _count_constraints = 0;
    for (auto& constraint : _list_constraints){
        constraint->evaluate(robot, _linearMatrix, _lowerBound, _upperBound, _count_constraints);
    }

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
    for (auto& cost : _list_costs){
        cost->evaluate(robot, _hessian, _gradient);
    }
    _count_constraints = 0;
    for (auto& constraint : _list_constraints){
        constraint->evaluate(robot, _linearMatrix, _lowerBound, _upperBound, _count_constraints);
    }
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

bool ConstraintBaseVel::evaluate(Robot& robot, Eigen::Ref<Eigen::MatrixXd> linearMatrix, Eigen::Ref<Eigen::VectorXd> lowerBound, Eigen::Ref<Eigen::VectorXd> upperBound, unsigned int& count_constraints){
    linearMatrix.block(count_constraints, 0, _n_constraints, _n_var) = iDynTree::toEigen(robot.getJacobian("base_link"));
    count_constraints += _n_constraints;
    return true;
};

bool ConstraintJointVel::evaluate(Robot& robot, Eigen::Ref<Eigen::MatrixXd> linearMatrix, Eigen::Ref<Eigen::VectorXd> lowerBound, Eigen::Ref<Eigen::VectorXd> upperBound, unsigned int& count_constraints){
    linearMatrix.block(count_constraints, 6, _n_constraints, _n_constraints) = Eigen::MatrixXd::Identity(_n_constraints, _n_constraints);
    lowerBound.segment(count_constraints, _n_constraints) = - speed_limit * Eigen::VectorXd::Ones(_n_constraints);
    upperBound.segment(count_constraints, _n_constraints) = speed_limit * Eigen::VectorXd::Ones(_n_constraints); 
    count_constraints += _n_constraints;
    return true;
};

bool CostConfigurationVelocity::evaluate(Robot& robot, Eigen::Ref<Eigen::MatrixXd> hessian, Eigen::Ref<Eigen::VectorXd> gradient){
    hessian += _gain * Eigen::MatrixXd::Identity(_n_var,_n_var);
    gradient += _gain * Eigen::VectorXd::Zero(_n_var);
    return true;
};

bool CostErrorDesiredConfigurationVelocity::init(Robot& robot){
    _J_ee_pos.resize(3, robot.getNrOfDegreesOfFreedom() + 6);
    return true;
};

bool CostErrorDesiredConfigurationVelocity::evaluate(Robot& robot, Eigen::Ref<Eigen::MatrixXd> hessian, Eigen::Ref<Eigen::VectorXd> gradient){
    _J_ee_pos = iDynTree::toEigen(robot.getJacobian(_frameName_ee)).block(0, 0, 3, robot.getNrOfDegreesOfFreedom() + 6);
    iDynTree::Position _w_p_ee = robot.getWorldTransform(_frameName_ee).getPosition();
    hessian += (iDynTree::toEigen(_J_ee_pos)).transpose() * iDynTree::toEigen(_J_ee_pos);
    gradient += iDynTree::toEigen(_J_ee_pos).transpose() * iDynTree::toEigen(_w_p_ee - _w_p_ee_des);
    return true;
};
