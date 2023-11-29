#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <Eigen/Dense>
#include <Robot.h>
#include <iostream>

class Constraint
{
    public:
        Constraint(unsigned int n_var, unsigned int n_constraints=0);
        bool configure(Robot& robot, Eigen::MatrixXd& linearMatrix, Eigen::VectorXd& lowerBound, Eigen::VectorXd& upperBound, unsigned int& count_constraints);
        virtual bool compute(Robot& robot, Eigen::Ref<Eigen::MatrixXd> linearMatrix, Eigen::Ref<Eigen::VectorXd> lowerBound, Eigen::Ref<Eigen::VectorXd> upperBound, unsigned int& count_constraints);
        unsigned int _n_constraints;
        unsigned int _n_var;
};

#endif
