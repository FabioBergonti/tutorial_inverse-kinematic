#ifndef MODULE_H
#define MODULE_H

#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/KinDynComputations.h>

#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>

#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/PolyDriver.h>

#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>

#include <QP.h>
#include <Robot.h>

class Module : public yarp::os::RFModule
{
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
    yarp::sig::Vector referenceJointPositions; // deg
    yarp::sig::Vector referenceJointVelocities; // deg/s
    yarp::sig::Vector old_referenceJointPositions; // deg

    iDynTree::MatrixDynSize J_ee_pos;
    iDynTree::Position w_p_ee_des;

    const std::string frameName_base {"base_link"};
    const std::string frameName_ee {"r_arm_jet_turbine"};

    QPControlProblem qp_problem;

    Robot robot;

public:
    virtual double getPeriod ();
    virtual bool updateModule ();
    virtual bool configure (yarp::os::ResourceFinder &rf);
    virtual bool close ();
};


#endif /* end of include guard: MODULE_H */
