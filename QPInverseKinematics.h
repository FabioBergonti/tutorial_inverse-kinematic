#ifndef QPIK_MODULE_H
#define QPIK_MODULE_H

#include <QPMasterClass.h>
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <Robot.h>
#include <string>
#include <yarp/os/LogStream.h>
#include <vector>
#include <Constraint.h>
#include <Cost.h>


class QPInverseKinematics : public QPMasterClass
{
    public:
        QPInverseKinematics(){};
        bool setDesiredFramePosition(iDynTree::Position w_p_ee_des, std::string frameName_ee);

    private:
        bool _setCostAndConstraints(Robot& robot) override;
        std::string _frameName_base {"base_link"};
        std::string _frameName_ee;
        iDynTree::MatrixDynSize _J_ee_pos;
        iDynTree::Position _w_p_ee;
        iDynTree::Position _w_p_ee_des;
};

class ConstraintBaseVel : public Constraint{
    public:
        ConstraintBaseVel(unsigned int n_var) : Constraint(n_var, 6) {
            // Additional initialization for ConstraintBaseVel if needed
        }
        bool compute(Robot& robot, Eigen::Ref<Eigen::MatrixXd> linearMatrix, Eigen::Ref<Eigen::VectorXd> lowerBound, Eigen::Ref<Eigen::VectorXd> upperBound, unsigned int& count_constraints) override;
};

class ConstraintJointVel : public Constraint
{
    public:
        ConstraintJointVel(unsigned int n_var, unsigned int n_dof) : Constraint(n_var, n_dof) {
            // Additional initialization for ConstraintJointVel if needed
        }
        bool compute(Robot& robot, Eigen::Ref<Eigen::MatrixXd> linearMatrix, Eigen::Ref<Eigen::VectorXd> lowerBound, Eigen::Ref<Eigen::VectorXd> upperBound, unsigned int& count_constraints) override;
        double speed_limit {0.10};
};

class CostConfigurationVelocity : public Cost
{
    public:
        CostConfigurationVelocity(unsigned int n_var) : Cost(n_var) {
            // Additional initialization for CostConfigurationVelocity if needed
        }
        bool compute(Robot& robot, Eigen::Ref<Eigen::MatrixXd> hessian, Eigen::Ref<Eigen::VectorXd> gradient) override;
        double _gain {1};
};

class CostErrorDesiredConfigurationVelocity : public Cost
{
    public:
        CostErrorDesiredConfigurationVelocity(unsigned int n_var, std::string frameName_ee, iDynTree::Position w_p_ee_des) : Cost(n_var) {
            _frameName_ee = frameName_ee;
            _w_p_ee_des = w_p_ee_des;
        }
        bool configure(Robot& robot) override;
        bool compute(Robot& robot, Eigen::Ref<Eigen::MatrixXd> hessian, Eigen::Ref<Eigen::VectorXd> gradient) override;
        double _gain {1};
        iDynTree::MatrixDynSize _J_ee_pos;
        iDynTree::Position _w_p_ee_des;
        std::string _frameName_ee;
};

#endif /* end of include guard QPIK_MODULE_H */
