#ifndef VERTEX_ROBOT_POS_H
#define VERTEX_ROBOT_POS_H

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_dynamic_vertex.h>

#include <environment/joint/robot_pos.h>

#include <iostream>

using namespace g2o;

namespace Muvt { namespace HyperGraph {

// The template argument defines the number of DoFs of the robot
class VertexRobotPos : public BaseDynamicVertex<Eigen::VectorXd>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    /**
     * @brief The VertexRobotPos Constructor 
     */ 
    VertexRobotPos();
    
    /**
     * @brief update the number of DoFs
     * @param n is the number of DoFs
     */
    virtual bool setDimensionImpl(int newDimension);
    
    /**
     * @brief Overrides the g2o function that resets the estimate value of the VERTEX_ROBOT_POS_H
     */ 
    virtual void setToOriginImpl() override;
    
    /**
     * @brief Overrides the g2o function that defines how the perturbation is applied to estimate
     * @param update is the perturbation
     */
    virtual void oplusImpl(const double* update) override;
    
    virtual bool setEstimateDataImpl(const number_t* est) override;
    
    virtual bool getEstimateData(number_t* est) const override;
    
    virtual int estimateDimension() const override;
    
    virtual bool setMinimalEstimateDataImpl(const number_t* est) override;

    virtual bool getMinimalEstimateData(number_t* est) const override;

    virtual int minimalEstimateDimension() const override; 
    
    virtual bool read(std::istream& is) override
    {
        // Read the dimension
        int dimension;
        is >> dimension;
        if (is.good() == false) {
        return false;
        }

        // Set the dimension; we call the method here to ensure stuff like
        // cache and the workspace is setup
        setDimension(dimension);

        // Read the state
        return g2o::internal::readVector(is, _estimate);
    }
    
    virtual bool write(std::ostream& os) const override
    {
        os << _estimate.size() << " ";
        return g2o::internal::writeVector(os, _estimate);
    }
    
private:
    int _n_dof;
};


    
} }


#endif
