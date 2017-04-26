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

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;


class  SuperQuadric_NLP : public Ipopt::TNLP
{

protected:
    bool bounds_automatic;
    Vector x_v;
    Vector x0;
    Matrix bounds;
    double aux_objvalue;

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
     void F(const Ipopt::Number *x, deque<Vector> &points, bool &new_x);

      /****************************************************************/
     double f(const Ipopt::Number *x, Vector &point_cloud);

     /****************************************************************/
     double F_v(const Vector &x, const deque<Vector> &points);

     /****************************************************************/
     double f_v(const Vector &x, const Vector &point_cloud);

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
     //bool readMatrix(const string &tag, Matrix &matrix, const int &dimension, ResourceFinder *rf);

    /****************************************************************/
    void computeX0(Vector &x0, deque<Vector> &point_cloud);

    /****************************************************************/
    void computeInitialOrientation(Vector &x0,deque<Vector> &point_cloud);

    /****************************************************************/
    Matrix computeBoundingBox(deque<Vector> &points, const Vector &x0);

   /****************************************************************/
   void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n,
                          const Ipopt::Number *x, const Ipopt::Number *z_L,
                          const Ipopt::Number *z_U, Ipopt::Index m,
                          const Ipopt::Number *g, const Ipopt::Number *lambda,
                          Ipopt::Number obj_value, const Ipopt::IpoptData *ip_data,
                          Ipopt::IpoptCalculatedQuantities *ip_cq);


public:
    Vector solution;
    deque<Vector> points_downsampled;

    /****************************************************************/
    void init();

    /****************************************************************/
    void setPoints(const deque<Vector> &point_cloud, const int &optimizer_points);

    /****************************************************************/
    void configure(bool bounds_aut);

    /****************************************************************/
    Vector get_result() const;

};

#endif

