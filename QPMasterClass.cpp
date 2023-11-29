#include <QPMasterClass.h>

bool QPMasterClass::_setCostAndConstraints(Robot& robot){
    _n_var = 0;
    _list_constraints.clear();
    _list_costs.clear();
    return true;
};

bool QPMasterClass::configure(Robot& robot)
{

    _setCostAndConstraints(robot);

    _hessian.resize(_n_var, _n_var);
    _gradient.resize(_n_var);
    _linearMatrix.resize(0, _n_var);
    _lowerBound.resize(0);
    _upperBound.resize(0);

    for (auto& cost : _list_costs){
        cost->configure(robot);
    }
    for (auto& cost : _list_costs){
        cost->compute(robot, _hessian, _gradient);
    }

    _count_constraints = 0;
    for (auto& constraint : _list_constraints){
        constraint->configure(robot, _linearMatrix, _lowerBound, _upperBound, _count_constraints);
    }
    _count_constraints = 0;
    for (auto& constraint : _list_constraints){
        constraint->compute(robot, _linearMatrix, _lowerBound, _upperBound, _count_constraints);
    }

    _n_constraints = _count_constraints;
    _solver.settings()->setWarmStart(true);
    _solver.settings()->setVerbosity(false);
    _solver.data()->setNumberOfVariables(_n_var);
    _solver.data()->setNumberOfConstraints(_n_constraints);
    _outputQP = Eigen::VectorXd::Zero(_n_var);
    return true;
}

bool QPMasterClass::update(Robot& robot)
{
    _hessian.setZero();
    _gradient.setZero();
    for (auto& cost : _list_costs){
        cost->compute(robot, _hessian, _gradient);
    }
    _count_constraints = 0;
    for (auto& constraint : _list_constraints){
        constraint->compute(robot, _linearMatrix, _lowerBound, _upperBound, _count_constraints);
    }
    return true;
}

bool QPMasterClass::solve()
{
    _hessianSparse = _hessian.sparseView();
    _linearMatrixSparse = _linearMatrix.sparseView();

    if (!_solver.isInitialized()){
        yInfo() << "QPMasterClass::solve: initialising solver";
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
        // yInfo() << "QPMasterClass::solve: updating solver";
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

Eigen::VectorXd QPMasterClass::getSolution()
{
    return _outputQP;
}
