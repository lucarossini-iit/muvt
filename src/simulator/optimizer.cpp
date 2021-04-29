#include <simulator/optimizer.h>

using namespace XBot::HyperGraph;

Optimizer::Optimizer():
_nhpr("~")
{
    init_load_simulator();
    init_load_optimizer();
}

void Optimizer::init_load_simulator()
{
    int n;
    double distance;
    if (!_nhpr.getParam("n", n))
        std::runtime_error("Missing mandatory parameter 'n'");
    if (!_nhpr.getParam("distance", distance))
        std::runtime_error("Missing mandatory parameter 'distance'");

    _simulator = std::make_shared<Simulator>(n, distance);
}
