/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Giulia Vezzani
 * email:  giulia.vezzani@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef __SUPERQUADRIC_H__
#define __SUPERQUADRIC_H__

#include <string>
#include <deque>
#include <map>

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>
#include <IpReturnCodes.hpp>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

/**
  * This class solves the optimization problem with the Ipopt software package
  * and returns the estiamted superquadric, better fitting a given point cloud.
  */
class  SuperQuadric_NLP : public Ipopt::TNLP
{

protected:
    bool bounds_automatic;
    yarp::sig::Vector x_v;
    yarp::sig::Vector x0;
    yarp::sig::Matrix bounds;
    double aux_objvalue;
    std::string obj_class;

    yarp::os::ResourceFinder *rf;

    /* Get info for the nonlinear problem to be solved with ipopt */
    /****************************************************************/
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m,Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag, Ipopt::TNLP::IndexStyleEnum &index_style);

    /* Compute bounds variable from the point cloud for speeding up optimization */
    /****************************************************************/
    void computeBounds();

    /* Get the variable bound for the nonlinear problem to be solved with ipopt */
    /****************************************************************/
    bool get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                         Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u);


    /* Get the starting point for the nonlinear problem to be solved with ipopt */
    /****************************************************************/
     bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                                bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
                                Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda);

     /* Cost function of the nonlinear problem to be solved with ipopt */
     /****************************************************************/
     bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                    Ipopt::Number &obj_value);

     /* Auxiliary function for computing the cost function of the nonlinear problem */
     /****************************************************************/
     void F(const Ipopt::Number *x, std::deque<yarp::sig::Vector> &points, bool &new_x);

     /* Auxiliary function for computing the cost function of the nonlinear problem */
     /****************************************************************/
     double f(const Ipopt::Number *x, yarp::sig::Vector &point_cloud);

     /* Auxiliary function for computing the gradient of the cost function of the nonlinear problem */
     /****************************************************************/
     double F_v(const yarp::sig::Vector &x, const std::deque<yarp::sig::Vector> &points);

     /* Auxiliary function for computing the gradient of the cost function of the nonlinear problem */
     /****************************************************************/
     double f_v(const yarp::sig::Vector &x, const yarp::sig::Vector &point_cloud);

     /* Gradient of the cost function of the nonlinear problem */
     /****************************************************************/
     bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                      Ipopt::Number *grad_f);

     /* Constraints of the nonlinear problem */
     /****************************************************************/
     bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                 Ipopt::Index m, Ipopt::Number *g);

     /* Jacobian of the constraints of the nonlinear problem */
     /****************************************************************/
     bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                     Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
                     Ipopt::Index *jCol, Ipopt::Number *values);

     /* Compute a good starting point for the nonlinear problem */
    /****************************************************************/
    void computeX0(yarp::sig::Vector &x0, std::deque<yarp::sig::Vector> &point_cloud);

    /* Compute initial superquadric orientation from the point cloud */
    /****************************************************************/
    void computeInitialOrientation(yarp::sig::Vector &x0, std::deque<yarp::sig::Vector> &point_cloud);

    /* Compute bounding box from the point cloud */
    /****************************************************************/
    yarp::sig::Matrix computeBoundingBox(std::deque<yarp::sig::Vector> &points, const yarp::sig::Vector &x0);

    /* Finalize the solution obtained from the optimization problem */
   /****************************************************************/
   void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n,
                          const Ipopt::Number *x, const Ipopt::Number *z_L,
                          const Ipopt::Number *z_U, Ipopt::Index m,
                          const Ipopt::Number *g, const Ipopt::Number *lambda,
                          Ipopt::Number obj_value, const Ipopt::IpoptData *ip_data,
                          Ipopt::IpoptCalculatedQuantities *ip_cq);


public:
    yarp::sig::Vector solution;
    std::deque<yarp::sig::Vector> points_downsampled;

    /* Init function */
    /****************************************************************/
    void init();

    /* Set point to be used for superquadric estimation */
    /****************************************************************/
    void setPoints(const std::deque<yarp::sig::Vector> &point_cloud, const int &optimizer_points);

    /* Configure function */
    /****************************************************************/
    void configure(yarp::os::ResourceFinder *rf, bool bounds_aut, const std::string &object_class);

    /* Return the estimated superquadric */
    /****************************************************************/
    yarp::sig::Vector get_result() const;

    /* Read matrix containing parameters from a txt file */
    /****************************************************************/
    bool readMatrix(const std::string &tag, yarp::sig::Matrix &matrix, const int &dimension, yarp::os::ResourceFinder *rf);

};

#endif

