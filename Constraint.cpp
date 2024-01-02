#include "Constraint.h"

Constraint::Constraint(unsigned int n_var, unsigned int n_constraints){
    _n_constraints = n_constraints;
    _n_var = n_var;
};

unsigned int Constraint::getNConstraints(){
    return _n_constraints;
};

bool Constraint::configure(Robot& robot, Eigen::MatrixXd& linearMatrix, Eigen::VectorXd& lowerBound, Eigen::VectorXd& upperBound, unsigned int& count_constraints){
    linearMatrix.conservativeResize(count_constraints + _n_constraints, Eigen::NoChange);
    lowerBound.conservativeResize(count_constraints + _n_constraints);
    upperBound.conservativeResize(count_constraints + _n_constraints);
    linearMatrix.block(count_constraints, 0, _n_constraints, _n_var) = Eigen::MatrixXd::Zero(_n_constraints, _n_var);
    lowerBound.segment(count_constraints, _n_constraints) = Eigen::VectorXd::Zero(_n_constraints);
    upperBound.segment(count_constraints, _n_constraints) = Eigen::VectorXd::Zero(_n_constraints);
    return true;
};

bool Constraint::compute(Robot& robot, Eigen::Ref<Eigen::MatrixXd> linearMatrix, Eigen::Ref<Eigen::VectorXd> lowerBound, Eigen::Ref<Eigen::VectorXd> upperBound, unsigned int& count_constraints){
    return true;
};
