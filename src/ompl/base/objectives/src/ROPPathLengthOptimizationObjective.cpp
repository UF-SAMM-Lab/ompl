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

/* Author: Jared Flowers */

#include "ompl/base/objectives/ROPPathLengthOptimizationObjective.h"
#include <memory>
#include "ompl/base/samplers/informed/PathLengthDirectInfSampler.h"

// ompl::base::ROPPathLengthOptimizationObjective::ROPPathLengthOptimizationObjective(const SpaceInformationPtr &si)
//   : ompl::base::OptimizationObjective(si)
// {
//     description_ = "Path Length";

//     // Setup a default cost-to-go heuristics:
//     setCostToGoHeuristic(base::goalRegionCostToGo);
// }

ompl::base::Cost ompl::base::ROPPathLengthOptimizationObjective::stateCost(const State *) const
{
    return identityCost();
}

// ompl::base::Cost ompl::base::ROPPathLengthOptimizationObjective::motionCost(const State *s1, const State *s2) const
// {
//     return Cost(si_->distance(s1, s2));
// }

ompl::base::Cost ompl::base::ROPPathLengthOptimizationObjective::motionCostHeuristic(const State *s1,
                                                                                  const State *s2) const
{
    return motionCost(s1, s2);
}

ompl::base::InformedSamplerPtr ompl::base::ROPPathLengthOptimizationObjective::allocInformedStateSampler(
    const ProblemDefinitionPtr &probDefn, unsigned int maxNumberCalls) const
{
// Make the direct path-length informed sampler and return. If OMPL was compiled with Eigen, a direct version is
// available, if not a rejection-based technique can be used
    return std::make_shared<PathLengthDirectInfSampler>(probDefn, maxNumberCalls);
}

ompl::base::ROPPathLengthOptimizationObjective::ROPPathLengthOptimizationObjective(const SpaceInformationPtr &si)
  : ompl::base::OptimizationObjective(si)
{
    description_ = "ROPE Path Length";

    // Setup a default cost-to-go heuristics:
    setCostToGoHeuristic(base::goalRegionCostToGo);
    dimension_ = si_->getStateDimension();

    // name="rop metrics";

    pt_cloud_pub = nh_.advertise<sensor_msgs::PointCloud>("/rope_cloud",1);
    pts_pub = nh_.advertise<sensor_msgs::PointCloud>("/rope_points",1);
    urdf::Model robo_model;
    robo_model.initParam("robot_description");
    std::string base_frame_ = "world";
    std::string tool_frame = "tip";
    if (!nh_.getParam("/ROPE/base_frame", base_frame_))
    {
        ROS_ERROR("/ROPE/base_frame not defined");
        throw std::invalid_argument("base_frame is not defined");
    }
    if (!nh_.getParam("/ROPE/tool_frame", tool_frame))
    {
        ROS_ERROR("/ROPE/tool_frame not defined");
        throw std::invalid_argument("base_frame is not defined");
    }

    // if (!nh_.getParam("avoid_prob_threshold", prob_threshold))
    // {
    //   ROS_WARN_STREAM("/avoid_prob_threshold not defined, using "<<prob_threshold);
    // }
    
    if (!nh_.getParam("/ROPE/links_for_rop", links_to_check))
    {
        ROS_ERROR("/ROPE/links_for_rop not defined");
        throw std::invalid_argument("links_for_rop is not defined");
    }  
    if (!nh_.getParam("/ROPE/rop_resolution", m_resolution))
    {
        ROS_WARN_STREAM("/ROPE/rop_resolution not defined, using "<<m_resolution);
    }
    // std::cout<<"m_resolution:"<<m_resolution<<std::endl;
    m_inv_resolution=1/m_resolution;
    // std::cout<<"m_inv_resolution:"<<m_inv_resolution<<std::endl;

    Eigen::Vector3d grav;
    grav << 0, 0, -9.806;
    chain = rosdyn::createChain(robo_model, base_frame_, tool_frame, grav);
    
    link_names=chain->getLinksName();
    for (int i=0;i<link_names.size();i++) std::cout<<link_names[i]<<", ";
    std::cout<<std::endl;
    Eigen::VectorXd q;
    q.resize(dimension_);
    q.setZero();
    m_transformations=chain->getTransformations(q);
    // std::cout<<m_transformations.size()<<std::endl;
    int n_pts = 0;
    for (std::string link_name:links_to_check) {
        int it = std::find(link_names.begin(),link_names.end(),link_name) - link_names.begin();
        Eigen::Affine3d t_start = m_transformations.at(it);
        std::cout<<"t_start:"<<t_start.translation().transpose()<<std::endl;
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> link_pts;
        // link_pts.emplace_back(t_start.translation());
        link_pts.emplace_back(Eigen::Vector3d(0,0,0));
        n_pts++;
        if (it<m_transformations.size()-1) {
          Eigen::Affine3d t_tip = m_transformations.at(it+1);
          std::cout<<"t_tip:"<<t_tip.translation().transpose()<<std::endl;
          Eigen::Vector3d diff = t_tip.translation()-t_start.translation();
          // std::cout<<"diff.norm():"<<diff.norm()<<std::endl;
          int npts = std::ceil(diff.norm()*m_inv_resolution);
          // std::cout<<"npts:"<<npts<<std::endl;
          for (int i=1;i<npts;i++) {
              Eigen::Vector3d p = double(i)/double(npts)*diff;//t_start.translation()+
              std::cout<<p.transpose()<<std::endl;
              link_pts.emplace_back(p);
              n_pts++;
          }
        } else {
          Eigen::Vector3d t_tip = Eigen::Vector3d(0,0,0.2);//t_start.translation()+
          std::cout<<"t_tip:"<<t_tip.transpose()<<std::endl;
          Eigen::Vector3d diff = t_tip;//-t_start.translation();
          // std::cout<<"diff.norm():"<<diff.norm()<<std::endl;
          int npts = std::ceil(diff.norm()*m_inv_resolution);
          // std::cout<<"npts:"<<npts<<std::endl;
          for (int i=1;i<=npts;i++) {
              Eigen::Vector3d p = double(i)/double(npts)*diff;//t_start.translation()+
              std::cout<<p.transpose()<<std::endl;
              link_pts.emplace_back(p);
              n_pts++;
          }
        }
        m_test_points.insert({link_name,link_pts});
    }
    // std::cout<<"n_pts:"<<n_pts<<std::endl;
    m_points.resize(n_pts);

    if (!nh_.getParam("/ROPE/computation_step", step_))
    {
        ROS_ERROR("/ROPE/computation_step not defined");
    }
    // std::string opv_topic = "rop_opvs";
    // if (!nh_.getParam("/ROPE/rop_opv_topic", opv_topic))
    // {
    //     ROS_WARN_STREAM("/ROPE/rop_opv_topic not defined, using "<<opv_topic);
    // }

    WorkcellGrid();
    std::cout<<"reading opvs"<<std::endl;
    if (!nh_.hasParam("/ROPE/opvs"))
    {
        ROS_WARN_STREAM("no opvs in parameter /ROPE/opvs");
    } else 
    {
      nh_.getParam("/ROPE/opvs", opvs_array);
      std::cout<<opvs_array.size()<<std::endl;
      if (opvs_array.size()>1) {
          opvs_callback(opvs_array);
      } else {
        ROS_WARN_STREAM("empty opvs list in parameter /ROPE/opvs");
        sensor_msgs::PointCloud pt_cloud;  
        pt_cloud.header.frame_id="world";
        pt_cloud.header.stamp=ros::Time::now();
        pt_cloud.channels.resize(1);
        pt_cloud.channels.at(0).name="prob_successful_passage";
        pt_cloud_pub.publish(pt_cloud);
      }
    }
    

    // sub_opvs = nh_.subscribe<rop_msgs::opv_array_msg>(opv_topic,1,&ompl::base::ROPPathLengthOptimizationObjective::opvs_callback,this);

    // ss = si_->getStateSpace();
}

void ompl::base::ROPPathLengthOptimizationObjective::WorkcellGrid(void)
{
  
  std::vector<double> ws_lb_param;

  if (nh_.getParam("/ROPE/workspace_lower_bounds_xyz",ws_lb_param))
  {
    for (int i=0;i<3;i++) workspace_lb[i] = ws_lb_param[i];
  } else {
    ROS_DEBUG("/ROPE/workspace_lower_bounds_xyz is not set, default={-1,-1,0.5}");
  }  

  std::vector<double> ws_ub_param;

  if (nh_.getParam("/ROPE/workspace_upper_bounds_xyz",ws_ub_param))
  {
    for (int i=0;i<3;i++) workspace_ub[i] = ws_ub_param[i];
  } else {
    ROS_DEBUG("/ROPE/workspace_lower_bounds_xyz is not set, default={1,1,2.5}");
  }
  std::vector<int> npnt;
  for (int i=0;i<3;i++) {
    npnt.push_back(std::ceil((workspace_ub[i]-workspace_lb[i])*m_inv_resolution)+1);
    assert(npnt.back()>0);
  }
  // m_voxel_volume=m_resolution * m_resolution * m_resolution;
  m_occupancy.resize(npnt[0],npnt[1],npnt[2]);
  m_occupancy.setConstant(1.0f);
  m_npnt=npnt;
}

void ompl::base::ROPPathLengthOptimizationObjective::opvs_callback(const std::vector<double> &opv)
{
  m_occupancy.setConstant(1.0f);
  int num_opvs = int(opv.size()/13);
  ROS_WARN_STREAM("num_opvs:"<<num_opvs);
  sensor_msgs::PointCloud pts;
  for (int o=0;o<num_opvs;o++) {
    double prob_successful_passage=opv[o*13+12];
    //descritize OPV
    Eigen::Tensor<double,3> occup(m_npnt[0],m_npnt[1],m_npnt[2]);
    occup.setConstant(1.0f);
    Eigen::Vector3d near_pt_1;
    for (int i=0;i<3;i++) near_pt_1[i]=opv[i+o*13];
    Eigen::Vector3d far_pt_1;
    for (int i=0;i<3;i++) far_pt_1[i]=opv[i+o*13+3];
    Eigen::Vector3d near_pt_2;
    for (int i=0;i<3;i++) near_pt_2[i]=opv[i+o*13+6];
    Eigen::Vector3d far_pt_2;
    for (int i=0;i<3;i++) far_pt_2[i]=opv[i+o*13+9];

    Eigen::Vector3d pts1_delta = far_pt_1-near_pt_1;
    Eigen::Vector3d pts2_delta = far_pt_2-near_pt_2;
    int npts1 = std::ceil(pts1_delta.norm()*m_inv_resolution);
    int npts2 = std::ceil(pts2_delta.norm()*m_inv_resolution);
    int n_lines = std::max(npts1,npts2);
    for (int i=0;i<=n_lines;i++) {
      Eigen::Vector3d p1 = near_pt_1+double(i)/double(n_lines)*pts1_delta;
      Eigen::Vector3d p2 = near_pt_2+double(i)/double(n_lines)*pts2_delta;
      Eigen::Vector3d pts3_delta = p2-p1;
      int npts3 = std::ceil(pts3_delta.norm()*m_inv_resolution);
      for (int k=0;k<=npts3;k++)
      {
        Eigen::Vector3d p = p1+double(k)/double(npts3)*pts3_delta;
        geometry_msgs::Point32 pt;
        pt.x = p[0];
        pt.y = p[1];
        pt.z = p[2];
        pts.points.push_back(pt);
        double ix=(p[0]-workspace_lb[0])*m_inv_resolution;
        double iy=(p[1]-workspace_lb[1])*m_inv_resolution;
        double iz=(p[2]-workspace_lb[2])*m_inv_resolution;

        if ( (ix<0) || (ix>=m_npnt[0]))
          continue;
        if ( (iy<0) || (iy>=m_npnt[1]))
          continue;
        if ( (iz<0) || (iz>=m_npnt[2]))
          continue;
        occup(int(std::floor(ix)),
            int(std::floor(iy)),
            int(std::floor(iz)))=prob_successful_passage;
        occup(int(std::floor(ix)),
            int(std::floor(iy)),
            int(std::ceil(iz)))=prob_successful_passage;
        occup(int(std::floor(ix)),
            int(std::ceil(iy)),
            int(std::floor(iz)))=prob_successful_passage;
        occup(int(std::floor(ix)),
            int(std::ceil(iy)),
            int(std::ceil(iz)))=prob_successful_passage;
        occup(int(std::ceil(ix)),
            int(std::floor(iy)),
            int(std::floor(iz)))=prob_successful_passage;
        occup(int(std::ceil(ix)),
            int(std::floor(iy)),
            int(std::ceil(iz)))=prob_successful_passage;
        occup(int(std::ceil(ix)),
            int(std::ceil(iy)),
            int(std::floor(iz)))=prob_successful_passage;
        occup(int(std::ceil(ix)),
            int(std::ceil(iy)),
            int(std::ceil(iz)))=prob_successful_passage;
      }
    }

    // for (int i=0;i<=npts1;i++) {
    //   Eigen::Vector3d p1 = near_pt_1+double(i)/double(npts1)*pts1_delta;
    //   for (int j=0;j<=npts2;j++) {
    //     Eigen::Vector3d p2 = near_pt_2+double(j)/double(npts2)*pts2_delta;
    //     //descritize p1->p2 line
    //     Eigen::Vector3d pts3_delta = p2-p1;
    //     int npts3 = std::ceil(pts3_delta.norm()*0.5*m_inv_resolution);
    //     for (int k=0;k<=npts3;k++)
    //     {
    //       Eigen::Vector3d p = p1+double(k)/double(npts3)*pts3_delta;
    //       geometry_msgs::Point32 pt;
    //       pt.x = p[0];
    //       pt.y = p[1];
    //       pt.z = p[2];
    //       pts.points.push_back(pt);
    //       double ix=(p[0]-workspace_lb[0])*m_inv_resolution;
    //       double iy=(p[1]-workspace_lb[1])*m_inv_resolution;
    //       double iz=(p[2]-workspace_lb[2])*m_inv_resolution;

    //       if ( (ix<0) || (ix>=m_npnt[0]))
    //         continue;
    //       if ( (iy<0) || (iy>=m_npnt[1]))
    //         continue;
    //       if ( (iz<0) || (iz>=m_npnt[2]))
    //         continue;
    //       occup(int(std::floor(ix)),
    //           int(std::floor(iy)),
    //           int(std::floor(iz)))=prob_successful_passage;
    //       occup(int(std::floor(ix)),
    //           int(std::floor(iy)),
    //           int(std::ceil(iz)))=prob_successful_passage;
    //       occup(int(std::floor(ix)),
    //           int(std::ceil(iy)),
    //           int(std::floor(iz)))=prob_successful_passage;
    //       occup(int(std::floor(ix)),
    //           int(std::ceil(iy)),
    //           int(std::ceil(iz)))=prob_successful_passage;
    //       occup(int(std::ceil(ix)),
    //           int(std::floor(iy)),
    //           int(std::floor(iz)))=prob_successful_passage;
    //       occup(int(std::ceil(ix)),
    //           int(std::floor(iy)),
    //           int(std::ceil(iz)))=prob_successful_passage;
    //       occup(int(std::ceil(ix)),
    //           int(std::ceil(iy)),
    //           int(std::floor(iz)))=prob_successful_passage;
    //       occup(int(std::ceil(ix)),
    //           int(std::ceil(iy)),
    //           int(std::ceil(iz)))=prob_successful_passage;
    //     }
    //   }
    // }
    m_occupancy=(m_occupancy*occup);
  }
  if (!pts.points.empty()) {
    pts.header.frame_id="world";
    pts.header.stamp=ros::Time::now();
    pts_pub.publish(pts);
  }

  sensor_msgs::PointCloud pt_cloud;  
  pt_cloud.header.frame_id="world";
  pt_cloud.header.stamp=ros::Time::now();
  pt_cloud.channels.resize(1);
  pt_cloud.channels.at(0).name="prob_successful_passage";
  for (int i=0;i<m_npnt[0];i++) {
    for (int j=0;j<m_npnt[1];j++) {
      for (int k=0;k<m_npnt[2];k++) {
        if (m_occupancy(i,j,k)<1.0) {
          geometry_msgs::Point32 pt;
          pt.x = workspace_lb(0)+(double(i)+0.5)*m_resolution;
          pt.y = workspace_lb(1)+(double(j)+0.5)*m_resolution;
          pt.z = workspace_lb(2)+(double(k)+0.5)*m_resolution;
          pt_cloud.points.push_back(pt);
          pt_cloud.channels.at(0).values.push_back(1-m_occupancy(i,j,k));
        }
      }
    }
  }
  pt_cloud_pub.publish(pt_cloud);
  // m_occupancy = 1/m_occupancy;

}

double ompl::base::ROPPathLengthOptimizationObjective::totalROP(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& points) const
{
  double total_rop=1.0;
  double ix;
  double iy;
  double iz;


  for (const Eigen::Vector3d& p: points)
  {
    ix=(p(0)-workspace_lb(0))*m_inv_resolution;
    if ( (ix<0) || (int(std::ceil(ix))>=m_npnt[0]))
      continue;

    iy=(p(1)-workspace_lb(1))*m_inv_resolution;
    if ( (iy<0) || (int(std::ceil(iy))>=m_npnt[1]))
      continue;

    iz=(p(2)-workspace_lb(2))*m_inv_resolution;
    if ( (iz<0) || (int(std::ceil(iz))>=m_npnt[2]))
      continue;
    // std::cout<<"ix:"<<ix<<",iy:"<<iy<<",iz:"<<iz<<std::endl;
    double prob_success=            m_occupancy(int(std::floor(ix)),int(std::floor(iy)),int(std::floor(iz) ));
    prob_success=std::min(prob_success,m_occupancy(int(std::floor(ix)),int(std::floor(iy)),int(std::ceil(iz)  )));
    prob_success=std::min(prob_success,m_occupancy(int(std::floor(ix)),int(std::ceil(iy) ),int(std::floor(iz) )));
    prob_success=std::min(prob_success,m_occupancy(int(std::floor(ix)),int(std::ceil(iy) ),int(std::ceil(iz)  )));
    prob_success=std::min(prob_success,m_occupancy(int(std::ceil(ix) ),int(std::floor(iy)),int(std::floor(iz) )));
    prob_success=std::min(prob_success,m_occupancy(int(std::ceil(ix) ),int(std::floor(iy)),int(std::ceil(iz)  )));
    prob_success=std::min(prob_success,m_occupancy(int(std::ceil(ix) ),int(std::ceil(iy) ),int(std::floor(iz) )));
    prob_success=std::min(prob_success,m_occupancy(int(std::ceil(ix) ),int(std::ceil(iy) ),int(std::ceil(iz)  )));
    total_rop*=prob_success;
  }
  // total_occupancy*=m_voxel_volume;
  return 1/total_rop;
}


double ompl::base::ROPPathLengthOptimizationObjective::occupiedROPMultiplier(Eigen::VectorXd q) const
{
  unsigned ipnt=0;
  m_transformations=chain->getTransformations(q);
  for (std::string link_name:links_to_check) {
    auto il = std::find(link_names.begin(),link_names.end(),link_name);
    if ( il != link_names.end() )
    {
      int it = il-link_names.begin();
      const Eigen::Affine3d& t= m_transformations.at(it);
      const auto& pnts= m_test_points.at(link_name);
      // std::cout<<"pnts:"<<pnts.size()<<std::endl;
      for (unsigned int ip=0;ip<pnts.size();ip++)
      {
        // std::cout<<"ipnt:"<<ipnt<<","<<m_points.size()<<std::endl;
        m_points.at(ipnt++)=t*pnts.at(ip);
      }
    }
  }
  return totalROP(m_points);
}

ompl::base::Cost ompl::base::ROPPathLengthOptimizationObjective::motionCost(const State *s1, const State *s2) const
{

    const double *s1i = static_cast<const ompl::base::RealVectorStateSpace::StateType *>(s1)->values;
    const double *s2i = static_cast<const ompl::base::RealVectorStateSpace::StateType *>(s2)->values;

    Eigen::VectorXd p(dimension_);
    Eigen::VectorXd c(dimension_);
    for (unsigned int i = 0; i < dimension_; ++i)
    {   
        p[i] = (*s1i++);
        c[i] = (*s2i++);
    }
    double cost = rop_cost(p,c);
    return Cost(cost);
}

double ompl::base::ROPPathLengthOptimizationObjective::rop_cost(Eigen::VectorXd parent, Eigen::VectorXd new_node) const
{    
  double length = (new_node - parent).norm();
  if (length < 1e-6)
    return length;

  double max_multiplier = 0.0;
  unsigned int nsteps = std::ceil(length / step_);
  double inv_nsteps = 1.0 / nsteps;
  double distance = length / nsteps;
  Eigen::VectorXd diff = (new_node - parent)* inv_nsteps;
  for (unsigned int istep = 0; istep <= nsteps; istep++)
  {
    Eigen::VectorXd q = parent + diff * istep;
    //check for intersection of voxels
    max_multiplier = std::max(occupiedROPMultiplier(q),max_multiplier);
  }

  return length*max_multiplier;
}


