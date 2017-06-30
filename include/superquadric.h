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

    /****************************************************************/
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m,Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag, Ipopt::TNLP::IndexStyleEnum &index_style);

    /****************************************************************/
    void computeBounds();

    /****************************************************************/
    bool get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                         Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u);


    /****************************************************************/
     bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                                bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
                                Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda);

     /****************************************************************/
     bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                    Ipopt::Number &obj_value);

     /****************************************************************/
     void F(const Ipopt::Number *x, std::deque<yarp::sig::Vector> &points, bool &new_x);

      /****************************************************************/
     double f(const Ipopt::Number *x, yarp::sig::Vector &point_cloud);

     /****************************************************************/
     double F_v(const yarp::sig::Vector &x, const std::deque<yarp::sig::Vector> &points);

     /****************************************************************/
     double f_v(const yarp::sig::Vector &x, const yarp::sig::Vector &point_cloud);

     /****************************************************************/
     bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                      Ipopt::Number *grad_f);

     /****************************************************************/
     bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                 Ipopt::Index m, Ipopt::Number *g);

     /****************************************************************/
     bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                     Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
                     Ipopt::Index *jCol, Ipopt::Number *values);

    /****************************************************************/
    void computeX0(yarp::sig::Vector &x0, std::deque<yarp::sig::Vector> &point_cloud);

    /****************************************************************/
    void computeInitialOrientation(yarp::sig::Vector &x0, std::deque<yarp::sig::Vector> &point_cloud);

    /****************************************************************/
    yarp::sig::Matrix computeBoundingBox(std::deque<yarp::sig::Vector> &points, const yarp::sig::Vector &x0);

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

    /****************************************************************/
    void init();

    /****************************************************************/
    void setPoints(const std::deque<yarp::sig::Vector> &point_cloud, const int &optimizer_points);

    /****************************************************************/
    void configure(yarp::os::ResourceFinder *rf, bool bounds_aut, const std::string &object_class);

    /****************************************************************/
    yarp::sig::Vector get_result() const;

    /****************************************************************/
    bool readMatrix(const std::string &tag, yarp::sig::Matrix &matrix, const int &dimension, yarp::os::ResourceFinder *rf);

};

#endif

