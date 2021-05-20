#ifndef VERTEX_ROBOT_POS_H
#define VERTEX_ROBOT_POS_H

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_dynamic_vertex.h>

#include <environment/robot_pos.h>

#include <iostream>

using namespace g2o;

namespace XBot { namespace HyperGraph {

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
    void setNDoFs(int n);
    
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
        Eigen::VectorXd p;
        bool state = internal::readVector(is, p);
        setEstimate(p);
        return state;
    }
    
    virtual bool write(std::ostream& os) const override
    {
        return internal::writeVector(os, estimate());
    }
    
private:
    int _n_dof;
};


    
} }


#endif