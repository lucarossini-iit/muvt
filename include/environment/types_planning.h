#include <g2o/core/factory.h>
#include <environment/edge_xyz.h>

namespace g2o {
    G2O_REGISTER_TYPE_GROUP(planning);
    
    G2O_REGISTER_TYPE(EDGE_SCALAR_XYZ, EdgeScalarXYZ);
}