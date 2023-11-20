#include <Robot.h>
#include <cstdlib>
#include <iostream>

bool Robot::configure(std::string modelFullPath, std::vector<std::string> axesList, iDynTree::Vector3 gravity)
{
    _gravity = gravity;
    // We use the iDynTree::ModelLoader class to extract from the URDF file
    // a model containing only the joint we are interested in controlling, and
    // in the same order with which we configured the remotecontrolboardremapper
    // device, to avoid complicated remapping between the vectors used in the YARP
    // devices and the one used by the iDynTree model .
    iDynTree::ModelLoader mdlLoader;
    bool ok = mdlLoader.loadReducedModelFromFile(modelFullPath, axesList);

    // Once we loaded the model, we pass it to the KinDynComputations class to
    // compute dynamics quantities such as the vector of gravity torques
    ok = ok && _kinDynModel.loadRobotModel(mdlLoader.model());

    if (!ok) {
        yError()<<"Unable to open model " << modelFullPath;
        return false;
    }

    const iDynTree::Model& model = _kinDynModel.model();
    std::cout << "Robot configure" << std::endl;

    n_dof = axesList.size();
    if (n_dof != model.getNrOfDOFs()) {
        yError() << "The number of axes in the model (" << model.getNrOfDOFs() << ") does not match the number of axes in the configuration file (" << n_dof << ")";
        return false;
    }

    _jointPos.resize(n_dof);
    _jointVel.resize(n_dof);

    _initJacobian();
    _initTransform();

    return true;
}

bool Robot::setState(iDynTree::Transform w_H_b, iDynTree::Twist baseVel, yarp::sig::Vector positionsInRad, yarp::sig::Vector velocitiesInRadS)
{
    std::cout << "Robot setState" << std::endl;
    iDynTree::toiDynTree(positionsInRad, _jointPos);
    iDynTree::toiDynTree(velocitiesInRadS, _jointVel);
    _w_H_b = w_H_b;
    _baseVel = baseVel;
    _kinDynModel.setRobotState(_w_H_b, _jointPos, _baseVel, _jointVel, _gravity);
    _computeJacobian();
    _computeTransform();
    return true;
}

// Jacobian

bool Robot::_initJacobian()
{
    for (auto frame : _list_jacobian_frames){
        _jacobian[frame].resize(6, n_dof + 6);
    }
    _jacobian["com"].resize(3, n_dof + 6);
    return true;
}

bool Robot::_computeJacobian()
{
    for (auto frame : _list_jacobian_frames){
        _kinDynModel.getFrameFreeFloatingJacobian(
            _kinDynModel.getFrameIndex(frame), 
            _jacobian[frame]
            );
    }
    _kinDynModel.getCenterOfMassJacobian(_jacobian["com"]);
    return true;
}

iDynTree::MatrixDynSize Robot::getJacobian(std::string frameName)
{
    if (_jacobian.find(frameName) == _jacobian.end())
    {
        yError() << "getJacobian: Frame " << frameName <<" is not recognized";
        exit(1);
    }
    return _jacobian[frameName];
}

// Transform

bool Robot::_initTransform()
{
    for (auto frame : _list_world_transform_frames){
        _world_transform[frame] = iDynTree::Transform::Identity();
    }
    return true;
}

bool Robot::_computeTransform()
{
    for (auto frame : _list_world_transform_frames){
        _world_transform[frame] = _kinDynModel.getWorldTransform(
            _kinDynModel.getFrameIndex(frame));
    }
    return true;
}

iDynTree::Transform Robot::getWorldTransform(std::string frameName)
{
    if (_world_transform.find(frameName) == _world_transform.end())
    {
        yError() << "getTransform: Frame " << frameName <<" is not recognized";
        exit(1);
    }
    return _world_transform[frameName];
}
