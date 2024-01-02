#ifndef QP_MODULE_H
#define QP_MODULE_H

#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/EigenHelpers.h>
#include <Robot.h>
#include <string>
#include <yarp/os/LogStream.h>
#include <vector>
#include <Constraint.h>
#include <Cost.h>

class QPMasterClass
{
    public:
        QPMasterClass(){};
        bool update(Robot& robot);
        bool configure(Robot& robot);
        bool solve();
        Eigen::VectorXd getSolution();

    protected:
        virtual bool _setCostAndConstraints(Robot& robot);
        unsigned int _n_var;
        std::vector<std::unique_ptr<Constraint>> _list_constraints;
        std::vector<std::unique_ptr<Cost>> _list_costs;

    private:
        unsigned int _n_constraints;
        unsigned int _count_constraints;
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

#endif /* end of include guard QP_MODULE_H */
