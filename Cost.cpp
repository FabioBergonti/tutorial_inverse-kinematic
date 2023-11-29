#include <Cost.h>

Cost::Cost(unsigned int n_var){
    _n_var = n_var;
};

bool Cost::init(Robot& robot){
    return true;
};

bool Cost::evaluate(Robot& robot, Eigen::Ref<Eigen::MatrixXd> hessian, Eigen::Ref<Eigen::VectorXd> gradient){
    return true;
};
