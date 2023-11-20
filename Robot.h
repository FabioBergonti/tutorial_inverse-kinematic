#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <iDynTree/Core/Transform.h>
#include <unordered_map>
#include <yarp/sig/Vector.h>

#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/KinDynComputations.h>

#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/yarp/YARPConversions.h>
#include <yarp/os/LogStream.h>

#include <cmath>
#include <string>

class Robot
{
    public:
        Robot(){
            std::cout << "Robot constructor" << std::endl;
        }
        bool configure(std::string modelFullPath, std::vector<std::string> axesList, iDynTree::Vector3 gravity);
        bool setState(iDynTree::Transform w_H_b, iDynTree::Twist baseVel, yarp::sig::Vector positionsInRad, yarp::sig::Vector velocitiesInRadS);

        // yarp::sig::Vector positionsInRad;
        // yarp::sig::Vector velocitiesInRadS;
        // yarp::sig::Vector positionsInDeg;
        // yarp::sig::Vector velocitiesInDegS;
        // yarp::sig::Vector grav;

        size_t n_dof;

        iDynTree::VectorDynSize getJointPos(){return _jointPos;}
        iDynTree::VectorDynSize getJointVel(){return _jointVel;}
        iDynTree::Transform getBasePose(){return _w_H_b;}
        iDynTree::Twist getBaseVel(){return _baseVel;}
        iDynTree::MatrixDynSize getJacobian(std::string frameName);
        iDynTree::Transform getWorldTransform(std::string frameName);


    private:
        iDynTree::KinDynComputations _kinDynModel;;
        iDynTree::VectorDynSize _jointPos;
        iDynTree::VectorDynSize _jointVel;
        iDynTree::Transform _w_H_b;
        iDynTree::Twist _baseVel;
        iDynTree::Vector3 _gravity;



        bool _initJacobian();
        bool _computeJacobian();
        std::unordered_map<std::string, iDynTree::MatrixDynSize> _jacobian;
        std::vector<std::string> _list_jacobian_frames {"l_arm_jet_turbine", "r_arm_jet_turbine", "base_link"};

        bool _initTransform();
        bool _computeTransform();
        std::unordered_map<std::string, iDynTree::Transform> _world_transform;
        std::vector<std::string> _list_world_transform_frames {"l_arm_jet_turbine", "r_arm_jet_turbine", "base_link"};

};


#endif
