#ifndef MODULE_H
#define MODULE_H

#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>

#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/PolyDriver.h>

#include <iDynTree/KinDynComputations.h>

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
    yarp::dev::ITorqueControl    *itrq{nullptr};

    // Quantities used by the control
    yarp::sig::Vector positionsInRad;
    yarp::sig::Vector velocitiesInRadS;
    yarp::sig::Vector positionsInDeg;
    yarp::sig::Vector velocitiesInDegS;
    yarp::sig::Vector gravityCompensation;
    yarp::sig::Vector referencePositionsInRad;

    //write
    yarp::sig::Vector errorInRad;
    yarp::sig::Vector kp; // Nm/rad
    yarp::sig::Vector kd; // Nm/rad
    yarp::sig::Vector torquesInNm;
    yarp::sig::Vector zeroDofs;
    yarp::sig::Vector baseZeroDofs;
    yarp::sig::Vector grav;

public:
    virtual double getPeriod ();
    virtual bool updateModule ();
    virtual bool configure (yarp::os::ResourceFinder &rf);
    virtual bool close ();
};


#endif /* end of include guard: MODULE_H */
