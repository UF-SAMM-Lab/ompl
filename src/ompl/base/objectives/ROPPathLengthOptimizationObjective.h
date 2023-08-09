/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Luis G. Torres, Jonathan Gammell (allocInformedStateSampler) */

#ifndef OMPL_BASE_OBJECTIVES_ROP_PATH_LENGTH_OPTIMIZATION_OBJECTIVE_
#define OMPL_BASE_OBJECTIVES_ROP_PATH_LENGTH_OPTIMIZATION_OBJECTIVE_

#include "ompl/base/OptimizationObjective.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include <Eigen/Core>
#include <ros/ros.h>
#include <unsupported/Eigen/CXX11/Tensor>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud.h>
#include <rosdyn_core/primitives.h>
#include <rop_msgs/opv_array_msg.h>
#include <rop_msgs/opv_msg.h>

namespace ompl
{
    namespace base
    {
        /** \brief An optimization objective which corresponds to optimizing path length. */
        class ROPPathLengthOptimizationObjective : public OptimizationObjective
        {
        public:
            ROPPathLengthOptimizationObjective(const SpaceInformationPtr &si);

            /** \brief Returns identity cost. */
            Cost stateCost(const State *s) const override;

            /** \brief Motion cost for this objective is defined as
                the configuration space distance between \e s1 and \e
                s2, using the method SpaceInformation::distance(). */
            Cost motionCost(const State *s1, const State *s2) const override;

            /** \brief the motion cost heuristic for this objective is
                simply the configuration space distance between \e s1
                and \e s2, since this is the optimal cost between any
                two states assuming no obstacles. */
            Cost motionCostHeuristic(const State *s1, const State *s2) const override;

            /** \brief Allocate a state sampler for the path-length objective (i.e., direct ellipsoidal sampling). */
            InformedSamplerPtr allocInformedStateSampler(const ProblemDefinitionPtr &probDefn,
                                                         unsigned int maxNumberCalls) const override;

                
            mutable Eigen::Tensor<double,3> m_occupancy;
            mutable Eigen::Vector3d workspace_lb={-1,-1,0.5};
            mutable Eigen::Vector3d workspace_ub={1,1,2.5};
            mutable std::vector<int> m_npnt;
            rosdyn::ChainPtr chain;
            std::vector<std::string> link_names;
            mutable std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d>> m_transformations;
            mutable std::map<std::string,std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >> m_test_points;
            mutable std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > m_points;
            std::vector<std::string> links_to_check;
            double step_;
            double m_resolution=0.05;
            ros::Subscriber sub_opvs;
            ros::NodeHandle nh_;
            double nu_;
            void WorkcellGrid(void);
            void opvs_callback(const std::vector<double> &opv);
            double totalROP(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& points) const;
            double occupiedROPMultiplier(Eigen::VectorXd q) const;
            double rop_cost(Eigen::VectorXd parent, Eigen::VectorXd new_node) const;
            double m_inv_resolution;
            int dimension_=0;
            StateSpacePtr ss;
            std::vector<double> opvs_array;
        };
    }
}

#endif
