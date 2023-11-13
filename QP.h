#ifndef QP_MODULE_H
#define QP_MODULE_H

#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Core/EigenHelpers.h>



class QPControlProblem
{
    public:
        QPControlProblem(){};
        void computeHessian(Eigen::Ref<Eigen::MatrixXd> hessianDense, iDynTree::MatrixDynSize J_ee_pos, double gain_min_speed);
        void computeGradient(Eigen::Ref<Eigen::VectorXd> gradient, iDynTree::MatrixDynSize J_ee_pos, iDynTree::Position w_deltap_ee);
        void computeLinearConstraintsMatrix(Eigen::Ref<Eigen::MatrixXd> linearMatrixDense, iDynTree::MatrixDynSize J_base);
        void computeBounds(Eigen::Ref<Eigen::VectorXd> lowerBound, Eigen::Ref<Eigen::VectorXd> upperBound, double speed_limit);    
};



#endif /* end of include guard QP_MODULE_H */
