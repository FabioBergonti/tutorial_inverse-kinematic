#ifndef COST_H
#define COST_H

#include <Eigen/Dense>
#include <Robot.h>
#include <iostream>

class Cost
{
    public:
        Cost(unsigned int n_var);
        virtual bool init(Robot& robot);
        virtual bool evaluate(Robot& robot, Eigen::Ref<Eigen::MatrixXd> hessian, Eigen::Ref<Eigen::VectorXd> gradient);
        unsigned int _n_var;
};

#endif
