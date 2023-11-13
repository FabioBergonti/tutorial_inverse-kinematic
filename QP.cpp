#include <QP.h>

void QPControlProblem::computeHessian(Eigen::Ref<Eigen::MatrixXd> hessianDense, iDynTree::MatrixDynSize J_ee_pos, double gain_min_speed)
{
    int n_var = J_ee_pos.cols();
    hessianDense = (iDynTree::toEigen(J_ee_pos)).transpose() * iDynTree::toEigen(J_ee_pos);
    hessianDense += gain_min_speed * Eigen::MatrixXd::Identity(n_var,n_var);
}

void QPControlProblem::computeGradient(Eigen::Ref<Eigen::VectorXd> gradient, iDynTree::MatrixDynSize J_ee_pos, iDynTree::Position w_deltap_ee)
{
    gradient =  iDynTree::toEigen(J_ee_pos).transpose() * iDynTree::toEigen(w_deltap_ee);
}

void QPControlProblem::computeLinearConstraintsMatrix(Eigen::Ref<Eigen::MatrixXd> linearMatrixDense, iDynTree::MatrixDynSize J_base)
{
    int n_rows = 0;
    int n_joints = J_base.cols() - 6;
    linearMatrixDense.block(n_rows, 0, J_base.rows(), J_base.cols()) = iDynTree::toEigen(J_base);
    n_rows = J_base.rows();
    linearMatrixDense.block(n_rows, 6, n_joints, n_joints) = Eigen::MatrixXd::Identity(n_joints, n_joints);
}

void QPControlProblem::computeBounds(Eigen::Ref<Eigen::VectorXd> lowerBound, Eigen::Ref<Eigen::VectorXd> upperBound, double speed_limit)
{
    int n_rows = 0;
    int n_joints = 23;
    lowerBound.segment(n_rows, 6) = Eigen::VectorXd::Zero(6);
    upperBound.segment(n_rows, 6) = Eigen::VectorXd::Zero(6);
    n_rows = 6;
    lowerBound.segment(n_rows, n_joints) = - speed_limit * Eigen::VectorXd::Ones(n_joints);
    upperBound.segment(n_rows, n_joints) = speed_limit * Eigen::VectorXd::Ones(n_joints);
}
