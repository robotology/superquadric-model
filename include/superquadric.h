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

    /** Get info for the nonlinear problem to be solved with ipopt
    * @param n is the dimension of the variable
    * @param m is the number of constraints
    * @param nnz_jac_g is the dimensions of the jacobian
    * @param nnz_h_lag is an ipopt variable
    * @param index_styl is an ipopt variable
    * @return true
    */
    /****************************************************************/
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m,Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag, Ipopt::TNLP::IndexStyleEnum &index_style);

    /** Compute bounds variable from the point cloud for speeding up optimization */
    /****************************************************************/
    void computeBounds();

    /** Get variable bounds for the nonlinear problem to be solved with ipopt
   * @param n is the dimension of the variable
   * @param m is the number of constraints
   * @param x_l is the lower bound of the variable
   * @param x_u is the upper bound of the variable
   * @param g_l is the lower bound of the constraints
   * @param g_u is the upper bound of the constraints
   * @return true
   */
    /****************************************************************/
    bool get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                         Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u);


    /** Get the starting point for the nonlinear problem to be solved with ipopt
    * @param n is the dimension of the variable
    * @param init_x is the starting point of the optimization problem
    * @param x is the variable
    * @param init_z is an ipopt variable
    * @param z_L is an ipopt variable
    * @param z_U is an ipopt variable
    * @param m is the number of constraints
    * @param init_lambda is an ipopt variable
    * @param lambda is an ipopt variable
    * @return true
    */
    /****************************************************************/
     bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                                bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
                                Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda);

     /** Cost function of the nonlinear problem to be solved with ipopt
     * @param n is the dimension of the variable
     * @param x is the variable
     * @param new_x takes into account is the variable has been updated or not
     * @param obj_value is the value of the cost function
     * @retun true
     */
     /****************************************************************/
     bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                    Ipopt::Number &obj_value);

     /** Auxiliary function for computing cost function of the nonlinear problem to be solved with ipopt
     * @param x is the variable
     * @param points_on is object point cloud
     * @param new_x takes into account is the variable has been updated or not
     * @retun the cost function value
     */
     /****************************************************************/
     void F(const Ipopt::Number *x, std::deque<yarp::sig::Vector> &points, bool &new_x);

     /** Auxiliary function for computing cost function of the nonlinear problem to be solved with ipopt
    * @param obj is the Vector of the object
    * @param x is the variable
    * @param point is one point of the point cloud
    * @return a part of the cost function value
    */
     /****************************************************************/
     double f(const Ipopt::Number *x, yarp::sig::Vector &point_cloud);

     /** Auxiliary function for computing the gradient of cost function of the nonlinear problem
     * @param x is the variable
     * @param points_on is one point of object point cloud
     * @return cost function value
     */
     /****************************************************************/
     double F_v(const yarp::sig::Vector &x, const std::deque<yarp::sig::Vector> &points);

     /** Auxiliary function for computing the gradient cost function of the nonlinear problem
     * @param obj is the Vector of the object
     * @param x is the variable
     * @param point is one point of the point cloud
     * @return a part of the cost function value
     */
     /****************************************************************/
     double f_v(const yarp::sig::Vector &x, const yarp::sig::Vector &point_cloud);

     /** Gradient of the cost function of the nonlinear problem
      * @param x is the variable
      * @param n is the dimension of the variable
      * @param new_x takes into account is the variable has been updated or not
      * @param grad_f is the gradient of the cost function
      */
     /****************************************************************/
     bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                      Ipopt::Number *grad_f);

     /** Constraints of the nonlinear problem
      * @param n is the dimension of the variable
      * @param x is the variable
      * @param m is the number of constraints
      * @param new_x takes into account is the variable has been updated or not
      * @param g is the values of the constraints
      * @return true
      */
     /****************************************************************/
     bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                 Ipopt::Index m, Ipopt::Number *g);

     /**  Jacobian of the constraints of the nonlinear problem
     * @param n is the dimension of the variable
     * @param x is the variable
     * @param m is the number of constraints
     * @param new_x takes into account is the variable has been updated or not
     * @param iRow contains the jacobian raws
     * @param iCol contains the jacobian columns
     * @param values contains the jacobian values
     * @return true
     */
     /****************************************************************/
     bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                     Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
                     Ipopt::Index *jCol, Ipopt::Number *values);

     /** Compute a good starting point for the nonlinear problem
    * @param x0 is the initial value for the superquadric to be estimated
    * @param point_cloud is the object point cloud
    */
    /****************************************************************/
    void computeX0(yarp::sig::Vector &x0, std::deque<yarp::sig::Vector> &point_cloud);

    /** Compute initial superquadric orientation from the point cloud
    * @param x0 is the initial value for the superquadric to be estimated
    * @param point_cloud is the object point cloud
    */
    /****************************************************************/
    void computeInitialOrientation(yarp::sig::Vector &x0, std::deque<yarp::sig::Vector> &point_cloud);

    /** Compute bounding box from the point cloud
    * @param points is the point cloud
    * @param x0 is the initial value for the superquadric to be estimated
    * @return a matrix with the variable boundss
    */
    /****************************************************************/
    yarp::sig::Matrix computeBoundingBox(std::deque<yarp::sig::Vector> &points, const yarp::sig::Vector &x0);

    /** Finalize the solution
   * @param n is the dimension of the variable
   * @param x is the variable
   * @param m is the number of constraints
   * @param init_z is an ipopt variable
   * @param z_L is an ipopt variable
   * @param z_U is an ipopt variable
   * @param status says if the problem has been solved or not
   * @param obj_value is the final cost function values
   */
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

    /** Init function */
    /****************************************************************/
    void init();

    /** Set point to be used for superquadric estimation
    * @param point_cloud is the object point cloud
    * @param optimizer_points is the maximum number of points to be used for the optimization problem
    */
    /****************************************************************/
    void setPoints(const std::deque<yarp::sig::Vector> &point_cloud, const int &optimizer_points);

    /** Configure function
    * @param rf is the resource finder
    * @param bounds_aut is to set or not the automatic computation of the variable bound
    * @param object_class is the object class according to its shape
    */
    /****************************************************************/
    void configure(yarp::os::ResourceFinder *rf, bool bounds_aut, const std::string &object_class);

    /** Extract the solution
    * @return the superquadric as a Vector
    */
    /****************************************************************/
    yarp::sig::Vector get_result() const;

    /** Function for reading matrices from config files
    * @param tag is the name of the quantity to be read from text
    * @param matrix is the matrix to be filled
    * @param dimensions is the matrix dimensions
    * @param rf is the resource finder
    * @return true/false on success/failure
    */
    /****************************************************************/
    bool readMatrix(const std::string &tag, yarp::sig::Matrix &matrix, const int &dimension, yarp::os::ResourceFinder *rf);

};

#endif

