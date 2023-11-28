#ifndef QP_MODULE_H
#define QP_MODULE_H

#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <Robot.h>
#include <string>
#include <yarp/os/LogStream.h>
#include <vector>
#include <Constraint.h>

class QPControlProblem
{
    public:
        QPControlProblem(){};
        bool update(Robot& robot);
        bool configure(Robot& robot);
        bool solve();
        Eigen::VectorXd getSolution();

        bool setDesiredFramePosition(iDynTree::Position w_p_ee_des, std::string frameName_ee);

    private:
        // Robot _robot;
        bool _minimiseConfigurationVelocity(Robot& robot, const double gain=1);
        bool _minimiseErrorDesiredConfigurationVelocity(Robot& robot, const double gain=1);
        bool _constraintBaseVel(Robot& robot);
        bool _boundJointVel(Robot& robot, const double speed_limit);

        std::string _frameName_base {"base_link"};
        std::string _frameName_ee;

        bool _configure_qp_problem {true};

        unsigned int _n_var;
        unsigned int _n_constraints;
        unsigned int _count_constraints;
        iDynTree::MatrixDynSize _J_ee_pos;
        iDynTree::Position _w_p_ee;
        iDynTree::Position _w_p_ee_des;

        std::vector<std::unique_ptr<Constraint>> _list_constraints;

        Eigen::MatrixXd _hessian;
        Eigen::VectorXd _gradient;
        Eigen::MatrixXd _linearMatrix;
        Eigen::VectorXd _lowerBound;
        Eigen::VectorXd _upperBound;
        Eigen::SparseMatrix<double> _hessianSparse;
        Eigen::SparseMatrix<double> _linearMatrixSparse;

        OsqpEigen::Solver _solver;
        Eigen::VectorXd _outputQP;

};

class ConstraintBaseVel : public Constraint{
    public:
        ConstraintBaseVel(unsigned int n_var) : Constraint(n_var, 6) {
            // Additional initialization for ConstraintBaseVel if needed
        }
        bool evaluate(Robot& robot, Eigen::Ref<Eigen::MatrixXd> linearMatrix, Eigen::Ref<Eigen::VectorXd> lowerBound, Eigen::Ref<Eigen::VectorXd> upperBound, unsigned int& count_constraints) override;
};

class ConstraintJointVel : public Constraint
{
    public:
        ConstraintJointVel(unsigned int n_var, unsigned int n_dof) : Constraint(n_var, n_dof) {
            // Additional initialization for ConstraintJointVel if needed
        }
        bool evaluate(Robot& robot, Eigen::Ref<Eigen::MatrixXd> linearMatrix, Eigen::Ref<Eigen::VectorXd> lowerBound, Eigen::Ref<Eigen::VectorXd> upperBound, unsigned int& count_constraints) override;
        double speed_limit {0.10};
};

#endif /* end of include guard QP_MODULE_H */
