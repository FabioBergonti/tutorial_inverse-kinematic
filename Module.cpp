#include <Module.h>

#include <cmath>

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
    iDynTree::VectorDynSize jointPos(positionsInRad.size()), jointVelSetToZero(velocitiesInRadS.size());
    jointVelSetToZero.zero();

    // Convert measured values to iDynTree quantities
    iDynTree::toiDynTree(positionsInRad, jointPos);

    // Set all other input quantities of the inverse dynamics to zero

    iDynTree::Vector3 gravity;
    gravity.zero();
    gravity(2) = -9.81;

    kinDynModel.setRobotState(w_H_b, jointPos, baseVel, jointVelSetToZero, gravity);

    // The computeGeneralizedGravityForces method is computing the g(q) term, for both the base and the joint
    iDynTree::FreeFloatingGeneralizedTorques g_q(kinDynModel.model());
    kinDynModel.generalizedGravityForces(g_q);

    // We extract the joint part to a YARP vector
    iDynTree::toYarp(g_q.jointTorques(), gravityCompensation);

    //compute control
    for (size_t i = 0; i < positionsInRad.size(); i++) {
        errorInRad(i) = referencePositionsInRad(i) - positionsInRad(i);
        torquesInNm(i) = gravityCompensation(i) + kp(i) * errorInRad(i) - kd(i) * velocitiesInRadS(i);
    }

    itrq->setRefTorques(torquesInNm.data());

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

    size_t actuatedDOFs = axesList.size();

    // Actually open the device
    bool ok = robotDevice.open(options);
    if (!ok) {
        std::cout << "Could not open remotecontrolboardremapper object.\n";
        return false;
    }

    // Try to obtain the interfaces
    ok=ok && robotDevice.view(ilim);
    ok=ok && robotDevice.view(ienc);
    ok=ok && robotDevice.view(imod);
    ok=ok && robotDevice.view(itrq);

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

    // We use the iDynTree::ModelLoader class to extract from the URDF file
    // a model containing only the joint we are interested in controlling, and
    // in the same order with which we configured the remotecontrolboardremapper
    // device, to avoid complicated remapping between the vectors used in the YARP
    // devices and the one used by the iDynTree model .
    iDynTree::ModelLoader mdlLoader;
    ok = mdlLoader.loadReducedModelFromFile(modelFullPath, axesList);

    // Once we loaded the model, we pass it to the KinDynComputations class to
    // compute dynamics quantities such as the vector of gravity torques
    ok = ok && kinDynModel.loadRobotModel(mdlLoader.model());

    if (!ok) {
        yError()<<"Unable to open model " << modelFullPath;
        return false;
    }

    const iDynTree::Model& model = kinDynModel.model();

    ///////////////////////////////////////////////////////////////
    //// Resize buffers
    ///////////////////////////////////////////////////////////////

    std::cout << "Number of DOFs: " << actuatedDOFs << "\n";

    positionsInDeg.resize(actuatedDOFs, 0.0);
    positionsInRad.resize(actuatedDOFs, 0.0);
    velocitiesInDegS.resize(actuatedDOFs, 0.0);
    velocitiesInRadS.resize(actuatedDOFs, 0.0);
    gravityCompensation.resize(actuatedDOFs, 0.0);
    referencePositionsInRad.resize(actuatedDOFs, 0.0);

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

    convertDegToRad(positionsInDeg, referencePositionsInRad);

    iDynTree::IJointConstPtr joint = model.getJoint(model.getJointIndex("l_shoulder_pitch"));
    if (!joint) {
        std::cout << "Could not retrieve index of l_shoulder_pitch.\n";
        return false;
    }
    referencePositionsInRad(joint->getDOFsOffset()) += 1.5;

    joint = model.getJoint(model.getJointIndex("l_elbow"));
    if (!joint) {
        std::cout << "Could not retrieve index of l_elbow.\n";
        return false;
    }
    referencePositionsInRad(joint->getDOFsOffset()) += 1.5;

    joint = model.getJoint(model.getJointIndex("r_shoulder_pitch"));
    if (!joint) {
        std::cout << "Could not retrieve index of r_shoulder_pitch.\n";
        return false;
    }
    referencePositionsInRad(joint->getDOFsOffset()) += 1.5;

    joint = model.getJoint(model.getJointIndex("r_elbow"));
    if (!joint) {
        std::cout << "Could not retrieve index of r_elbow.\n";
        return false;
    }
    referencePositionsInRad(joint->getDOFsOffset()) += 1.5;

    //write
    errorInRad.resize(actuatedDOFs, 0.0);

    kp.resize(actuatedDOFs, 1.5);
    kd.resize(actuatedDOFs, 0.5);

    torquesInNm.resize(actuatedDOFs, 0.0);

    zeroDofs.resize(actuatedDOFs, 0.0);
    baseZeroDofs.resize(6, 0.0);

    grav.resize(3, 0.0);
    grav(2) = -9.81;

    // Setting the control mode of all the controlled joints to torque control mode
    // See http://wiki.icub.org/wiki/Control_Modes for more info about the control modes
    std::vector<int> ctrlModes(actuatedDOFs, VOCAB_CM_TORQUE);
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
