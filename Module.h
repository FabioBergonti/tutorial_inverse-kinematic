#ifndef MODULE_H
#define MODULE_H

#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/Position.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>

#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/PolyDriver.h>

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/KinDynComputations.h>

#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>

#include <QP.h>

class Module : public yarp::os::RFModule
{
    // Class to compute model quantities
    iDynTree::KinDynComputations kinDynModel;

    // Attributes to communicate with the robot YARP-based interface
    yarp::dev::PolyDriver robotDevice;

    // YARP Interfaces exposed by the remotecontrolboardremapper
    yarp::dev::IControlLimits    *ilim{nullptr};
    yarp::dev::IEncoders         *ienc{nullptr};
    yarp::dev::IControlMode      *imod{nullptr};
    yarp::dev::IPositionDirect   *ipos{nullptr};

    // Quantities used by the control
    yarp::sig::Vector positionsInRad;
    yarp::sig::Vector velocitiesInRadS;
    yarp::sig::Vector positionsInDeg;
    yarp::sig::Vector velocitiesInDegS;

    yarp::sig::Vector outputQP_yarp;


    //write
    yarp::sig::Vector kp; // Nm/rad
    yarp::sig::Vector referenceJointPositions; // deg
    yarp::sig::Vector referenceJointVelocities; // deg/s
    yarp::sig::Vector zeroJointPositions; // deg
    yarp::sig::Vector old_referenceJointPositions; // deg
    yarp::sig::Vector grav;
    double time_zero;
    double delta_time;

    iDynTree::VectorDynSize jointPos;
    iDynTree::VectorDynSize jointVel;
    iDynTree::Position w_p_com;
    iDynTree::MatrixDynSize J_com;
    iDynTree::MatrixDynSize J_ee;
    iDynTree::MatrixDynSize J_ee_pos;
    iDynTree::MatrixDynSize J_base;
    iDynTree::Position w_p_ee;
    iDynTree::Position w_p_base;
    iDynTree::Position w_p_ee_des;

    std::string frameName_base;
    std::string frameName_ee;

    Eigen::SparseMatrix<double> hessian;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::MatrixXd hessianDense;
    Eigen::MatrixXd linearMatrixDense;
    Eigen::VectorXd gradient;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    // Instantiate OSQP solver
    OsqpEigen::Solver solver;
    QPControlProblem qp_problem;

public:
    virtual double getPeriod ();
    virtual bool updateModule ();
    virtual bool configure (yarp::os::ResourceFinder &rf);
    virtual bool close ();
};


#endif /* end of include guard: MODULE_H */
