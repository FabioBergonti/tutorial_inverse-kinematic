#include "Robot.h"
#include <Module.h>

#include <cmath>

#include <cstdlib>
#include <iDynTree/Core/MatrixFixSize.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Twist.h>
#include <iostream>
#include <yarp/dev/IControlMode.h>
#include <yarp/os/Log.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/LogStream.h>

// iDynTree headers
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/yarp/YARPEigenConversions.h>

void addVectorOfStringToProperty(yarp::os::Property& prop, std::string key, std::vector<std::string> & list)
{
    prop.addGroup(key);
    yarp::os::Bottle & bot = prop.findGroup(key).addList();
    for(size_t i=0; i < list.size(); i++)
    {
        bot.addString(list[i].c_str());
    }
    return;
}

void convertDegToRad(const yarp::sig::Vector& vecDeg, yarp::sig::Vector& vecRad)
{
    if (vecDeg.size() != vecRad.size()) {
        yError() << "convertDegToRad: wrong vector size";
        return;
    }

    for (size_t i=0; i < vecDeg.size(); i++) {
        vecRad[i] = (M_PI/180.0)*vecDeg[i];
    }
}

void convertRadToDeg(const yarp::sig::Vector& vecRad, yarp::sig::Vector& vecDeg)
{
    if (vecDeg.size() != vecRad.size()) {
        yError() << "convertDegToRad: wrong vector size";
        return;
    }

    for (size_t i=0; i < vecRad.size(); i++) {
        vecDeg[i] = (180.0/M_PI)*vecRad[i];
    }
}

void copyYarpVector(const yarp::sig::Vector &vecOriginal, yarp::sig::Vector &vecCopy)
{
    // Copies the elements of a YARP vector into another one
    if (vecCopy.size() != vecOriginal.size())
    {
        yError() << "copyYarpVector: wrong vector size";
        // exit on error
        exit(1);
    }

    for (size_t i = 0; i < vecOriginal.size(); i++)
    {
        vecCopy[i] = vecOriginal[i];
    }
}

double Module::getPeriod () { return 0.01; }

bool Module::updateModule ()
{
    //read state
    ienc->getEncoders(positionsInDeg.data());
    convertDegToRad(positionsInDeg, positionsInRad);

    ienc->getEncoderSpeeds(velocitiesInDegS.data());
    convertDegToRad(velocitiesInDegS, velocitiesInRadS);

    // Compute the bias term of the inverse dynamics, passing data to iDynTree
    // Note: for the sake of simplicity we are allocate dynamically this iDynTree
    // quantities here, that in general is not real time safe.
    // We are considering the "fixed base" case, i.e. the base is always fixed to the ground
    iDynTree::Transform w_H_b = iDynTree::Transform::Identity(); //identity + zero vector
    iDynTree::Twist baseVel = iDynTree::Twist::Zero();

    // Set the robot state
    robot.setState(w_H_b,baseVel, positionsInRad, velocitiesInRadS);
    // solve QP
    qp_problem.update(robot);
    qp_problem.solve();

    // Link yarp and eigen QP output vectors using the Eigen Map
    Eigen::Map<Eigen::VectorXd> outputQP = iDynTree::toEigen(outputQP_yarp);
    outputQP = qp_problem.getSolution();

    //evaluate reference joint positions
    referenceJointVelocities = outputQP_yarp.subVector(6, robot.getNrOfDegreesOfFreedom() + 6 - 1) * 180.0 / M_PI;
    for (size_t i = 0; i < positionsInRad.size(); i++) {
        referenceJointPositions(i) = old_referenceJointPositions(i) + referenceJointVelocities(i) *  Module::getPeriod();
    }

    ipos->setPositions(referenceJointPositions.data());
    copyYarpVector(referenceJointPositions, old_referenceJointPositions);

    iDynTree::Position w_deltaP_ee = robot.getWorldTransform(frameName_ee).getPosition() - w_p_ee_des;
    yInfo() << "error pos" << w_deltaP_ee.toString() << " norm: " << iDynTree::toEigen(w_deltaP_ee).norm();

    return true;
}

bool Module::configure (yarp::os::ResourceFinder &rf)
{
    using namespace yarp::os;
    using namespace yarp::sig;

    ///////////////////////////////////////////////////////
    //// Open the remotecontrolboardremapper YARP device
    ///////////////////////////////////////////////////////

    Property options;
    options.put("device","remotecontrolboardremapper");


    // Note: this joint list are tipically loaded from configuration
    // file and they are not harcoded in the code. However, to reduce
    // the complexity of the example we are hardcoding the lists.
    // Torso Joints
    std::vector<std::string> axesList;
    axesList.push_back("torso_pitch");
    axesList.push_back("torso_roll");
    axesList.push_back("torso_yaw");

    // Left arm
    axesList.push_back("l_shoulder_pitch");
    axesList.push_back("l_shoulder_roll");
    axesList.push_back("l_shoulder_yaw");
    axesList.push_back("l_elbow");

    // Right arm
    axesList.push_back("r_shoulder_pitch");
    axesList.push_back("r_shoulder_roll");
    axesList.push_back("r_shoulder_yaw");
    axesList.push_back("r_elbow");

    // Left leg
    axesList.push_back("l_hip_pitch");
    axesList.push_back("l_hip_roll");
    axesList.push_back("l_hip_yaw");
    axesList.push_back("l_knee");
    axesList.push_back("l_ankle_pitch");
    axesList.push_back("l_ankle_roll");

    // Right leg
    axesList.push_back("r_hip_pitch");
    axesList.push_back("r_hip_roll");
    axesList.push_back("r_hip_yaw");
    axesList.push_back("r_knee");
    axesList.push_back("r_ankle_pitch");
    axesList.push_back("r_ankle_roll");

    addVectorOfStringToProperty(options, "axesNames", axesList);

    Bottle remoteControlBoards;
    Bottle & remoteControlBoardsList = remoteControlBoards.addList();

    std::string robotPortPrefix = rf.check("robot", yarp::os::Value("icubSim"), "Port prefix used for the controlboards").asString();
    remoteControlBoardsList.addString("/"+robotPortPrefix+"/torso");
    remoteControlBoardsList.addString("/"+robotPortPrefix+"/left_arm");
    remoteControlBoardsList.addString("/"+robotPortPrefix+"/right_arm");
    remoteControlBoardsList.addString("/"+robotPortPrefix+"/left_leg");
    remoteControlBoardsList.addString("/"+robotPortPrefix+"/right_leg");
    
    options.put("remoteControlBoards",remoteControlBoards.get(0));
    options.put("localPortPrefix","/test");
    Property & remoteControlBoardsOpts = options.addGroup("REMOTE_CONTROLBOARD_OPTIONS");
    remoteControlBoardsOpts.put("writeStrict","on");

    // Actually open the device
    bool ok = robotDevice.open(options);
    if (!ok) {
        yError() << "Could not open remotecontrolboardremapper object.\n";
        return false;
    }

    // Try to obtain the interfaces
    ok=ok && robotDevice.view(ilim);
    ok=ok && robotDevice.view(ienc);
    ok=ok && robotDevice.view(imod);
    ok=ok && robotDevice.view(ipos);

    if (!ok) {
        yError()<<"Unable to open interfaces";
        return false;
    }

    ///////////////////////////////////////////////////////////////
    //// Load the model in the iDynTree::KinDynComputations
    ///////////////////////////////////////////////////////////////

    // We assume that the model.urdf can be found by the ResourceFinder:
    // this means either that the file is in the current working directory,
    // or it is found using the ResourceFinder search hierarchy, documented in
    // http://www.yarp.it/yarp_data_dirs.html

    std::string modelFullPath = rf.findFileByName("model.urdf");

    iDynTree::Vector3 gravity;
    gravity.zero();
    gravity(2) = -9.81;
    robot.configure(modelFullPath, axesList, gravity);

    ///////////////////////////////////////////////////////////////
    //// Resize buffers
    ///////////////////////////////////////////////////////////////

    yInfo() << "Number of DOFs: " << robot.getNrOfDegreesOfFreedom();

    referenceJointPositions.resize(robot.getNrOfDegreesOfFreedom(), 0.0); // deg
    referenceJointVelocities.resize(robot.getNrOfDegreesOfFreedom(), 0.0); // deg/s
    old_referenceJointPositions.resize(robot.getNrOfDegreesOfFreedom(), 0.0); // deg
    positionsInDeg.resize(robot.getNrOfDegreesOfFreedom(), 0.0);
    positionsInRad.resize(robot.getNrOfDegreesOfFreedom(), 0.0);
    velocitiesInDegS.resize(robot.getNrOfDegreesOfFreedom(), 0.0);
    velocitiesInRadS.resize(robot.getNrOfDegreesOfFreedom(), 0.0);

    // Make sure that we are reading data from the robot before proceeding
    bool readEncoderSuccess = false;
    for (int i=0; i < 10 && !readEncoderSuccess; i++) {
        readEncoderSuccess = ienc->getEncoders(positionsInDeg.data());
        if (!readEncoderSuccess) {
            yarp::os::Time::delay(0.1);
        }
    }

    if (!readEncoderSuccess) {
        yError()<<"Unable to read encoders, exiting.";
        return false;
    }

    //read state
    convertDegToRad(positionsInDeg, positionsInRad);
    convertDegToRad(velocitiesInDegS, velocitiesInRadS);
    iDynTree::Transform w_H_b = iDynTree::Transform::Identity(); //identity + zero vector
    iDynTree::Twist baseVel = iDynTree::Twist::Zero();
    // Set the robot state
    robot.setState(w_H_b, baseVel, positionsInRad, velocitiesInRadS);

    //compute J_ee_pos
    J_ee_pos.resize(3, robot.getNrOfDegreesOfFreedom() + 6);
    J_ee_pos = iDynTree::toEigen(robot.getJacobian(frameName_ee)).block(0, 0, 3, robot.getNrOfDegreesOfFreedom() + 6);
    //compute w_p_ee
    iDynTree::Position w_p_ee = robot.getWorldTransform(frameName_ee).getPosition();
    w_p_ee_des = iDynTree::Position(w_p_ee(0) + 0.2, 0.0, w_p_ee(2) + 0.2);

    yInfo() << "error pos" << (w_p_ee - w_p_ee_des).toString() << " norm: " << iDynTree::toEigen(w_p_ee - w_p_ee_des).norm();

    // configure QP
    qp_problem.setDesiredFramePosition(w_p_ee_des, frameName_ee);
    qp_problem.configure(robot);
    outputQP_yarp.resize(robot.getNrOfDegreesOfFreedom() + 6);

    // init old_referenceJointPositions
    copyYarpVector(positionsInDeg, old_referenceJointPositions);

    // Setting the control mode of all the controlled joints to torque control mode
    // See http://wiki.icub.org/wiki/Control_Modes for more info about the control modes
    std::vector<int> ctrlModes(robot.getNrOfDegreesOfFreedom(), VOCAB_CM_POSITION_DIRECT);
    imod->setControlModes(ctrlModes.data());

    return true;
}

bool Module::close ()
{
    std::vector<int> ctrlModes(positionsInDeg.size(), VOCAB_CM_POSITION);
    imod->setControlModes(ctrlModes.data());

    //cleanup stuff
    robotDevice.close();
    return true;
}
