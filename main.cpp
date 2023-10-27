#include "Module.h"
#include <yarp/os/Network.h>
#include <cstdlib>

int main(int argc, char **argv)
{
    using namespace yarp::os;
    using namespace yarp::sig;

    yarp::os::Network yarp;

    if (!yarp::os::Network::checkNetwork(5.0)) {
        std::cout << "Yarp network not found\n";
        return EXIT_FAILURE;
    }

    ResourceFinder rf = ResourceFinder::getResourceFinderSingleton();
    rf.configure(argc, argv);
    Module module;
    
    return module.runModule(rf);
}
