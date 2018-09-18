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

#ifndef __COMPUTATION_H__
#define __COMPUTATION_H__

#include <yarp/dev/all.h>

#include <opencv2/opencv.hpp>

#include <iCub/ctrl/filters.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>

#include "superquadric.h"
#include "tree.h"

/**
  * This class filters the point cloud according to its density
  */
/*******************************************************************************/
class SpatialDensityFilter
{
public:

    /*******************************************************************************/
    static std::vector<int>  filter(const cv::Mat &data,const double radius, const int maxResults, std::deque<yarp::sig::Vector> &points);
};

/**
  * This class provides a thread for computing in real time the superquadric,
  * filtering the point cloud and the superquadric.
  * The optimization problem for estimating the superquadric is solved with Ipopt software package.
  */
/*******************************************************************************/
class SuperqComputation : public yarp::os::Thread
{
protected:

    int count;
    bool save_points;
    std::string tag_file;
    std::string homeContextPath;
    std::string pointCloudFileName;
    std::vector<cv::Point> contour;

    yarp::os::BufferedPort<yarp::os::Bottle> pointPort;

    // Filters parameters
    double radius;
    int nnThreshold;
    int numVertices;
    int median_order;
    int min_median_order;
    int new_median_order;
    bool filter_points;
    bool fixed_window;
    bool filter_superq;
    double threshold_median;
    double min_norm_vel;

    // On/off parameters
    bool go_on;
    bool one_shot;
    bool merge_model;

    // Options for optimization with ipopt
    double tol, sum;
    double max_cpu_time;
    int acceptable_iter,max_iter;
    int optimizer_points;
    bool bounds_automatic;
    std::string mu_strategy,nlp_scaling_method;
    yarp::sig::Vector elem_x;

    int n_nodes;
    int h_tree;

    double t_superq;
    int count_file;
    std::string ob_class;

    yarp::os::ResourceFinder *rf;
    double t,t0;
    yarp::os::Mutex mutex;
    yarp::os::Mutex &mutex_shared;

    // Classes uses for filtering the superquadric
    iCub::ctrl::MedianFilter *mFilter;
    iCub::ctrl::AWPolyEstimator *PolyEst;

    // Properties with all the options
    yarp::os::Property filter_points_par;
    yarp::os::Property filter_superq_par;
    yarp::os::Property ipopt_par;



public:

    bool single_superq;
    bool superq_computed;

    superqTree *superq_tree;

    bool merge;
    bool debug;

    int std_median_order;
    /** max median order width*/
    int max_median_order;

    /** Object superquadric*/
    yarp::sig::Vector &x;
    /** Filtered object superquadric*/
    yarp::sig::Vector &x_filtered;
    std::deque<yarp::sig::Vector> &points, points_splitted1, points_splitted2;
    std::deque<cv::Point> blob_points;

    /** Input image*/
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *imgIn;

    /***********************************************************************/
    SuperqComputation(yarp::os::Mutex &mutex_shared, int _rate, bool _filter_points, bool single_superq, bool _filter_superq, bool _fixed_window, std::deque<yarp::sig::Vector> &_points, yarp::sig::ImageOf<yarp::sig::PixelRgb> *imgIn,
                      std::string _tag_file, double _threshold_median, const yarp::os::Property &filters_points_par, yarp::sig::Vector &_x, yarp::sig::Vector &_x_filtered,
                      const yarp::os::Property &filters_superq_par, const yarp::os::Property &optimizer_par, const std::string &_homeContextPath, bool _save_points, yarp::os::ResourceFinder *rf, superqTree *superq_tree, bool _merge_model, int _h_tree);

    /** Set options for filtering the point cloud
    * @param newOptions is a Property with the new options to set
    * @param first_time takes into account if the options have already been set or not
    */
    /***********************************************************************/
    void setPointsFilterPar(const yarp::os::Property &newOptions, bool first_time);

    /** Set options for filtering the superquadric
    * @param newOptions is a Property with the new options to set
    * @param first_time takes into account if the options have already been set or not
    */
    /***********************************************************************/
    void setSuperqFilterPar(const yarp::os::Property &newOptions, bool first_time);

    /** Set options for the optimization problem solved by Ipopt
    * @param newOptions is a Property with the new options to set
    * @param first_time takes into account if the options have already been set or not
    */
    /***********************************************************************/
    void setIpoptPar(const yarp::os::Property &newOptions, bool first_time);

    /** Get options used for filtering the point cloud
    * @return a property with the options on the points filtering
    */
    /***********************************************************************/
    yarp::os::Property getPointsFilterPar();

    /** Get options used for filtering the superquadric
    * @return a property with the options on superquadric filtering
    * /
    /***********************************************************************/
    yarp::os::Property getSuperqFilterPar();

    /** Get options used for solving the optimization problem with Ipopt
    * @return a property with the options on ipopt
    * /
    /***********************************************************************/
    yarp::os::Property getIpoptPar();

    /** Set a parameter equal to a value
    * @param par_name is the name of the variable we want to change
    * @param value is the new balue
    */
    /***********************************************************************/
    void setPar(const std::string &par_name, const std::string &value);

    /** Init function of Thread class */
    /***********************************************************************/
    virtual bool threadInit();

    /** Run function of Thread class */
    /***********************************************************************/
    virtual void run();

    /** Release function of Thread class */
    /***********************************************************************/
    virtual void threadRelease();

    /** Save point cloud used for reconstructing the superquadric
    * @param namefile is the name of the file where to save the superquadric
    * @param colors is a Vector with the color to be used for saving the point cloud
    */
    /***********************************************************************/
    void savePoints(const std::string &namefile, const yarp::sig::Vector &colors);

    /** Read the object point cloud from txt for offline tests */
    /***********************************************************************/
    bool readPointCloud();

    /** Filter the received point cloud */
    /***********************************************************************/
    void filter();

    /** Compute the superquadric modeling the object
    * @return true/false on success/failure
    */
    /***********************************************************************/
    bool computeSuperq();

    /** Filter the superquadri with the median filter */
    /***********************************************************************/
    yarp::sig::Vector computeMultipleSuperq(const std::deque<yarp::sig::Vector> &points);

    /***********************************************************************/
    void filterSuperq();

    /** Reset median filter */
    /***********************************************************************/
    void resetMedianFilter();

    /** Compute window size according to the object velocity */
    /***********************************************************************/
    int adaptWindComputation();

    /** Configure superquadric filter */
    /***********************************************************************/
    bool configFilterSuperq();

    /** Configure point cloud filter */
    /***********************************************************************/
    bool config3Dpoints();

    /** Return the computed superquadric
    * @param filtered_or_not can be "on" or "off"
    * @return the estimated superquadric filtered or not
    */
    /***********************************************************************/
    yarp::sig::Vector getSolution(bool filtered_or_not);

    /** Get the point cloud for computing the superquadric
    * @param p is the object point cloud
    */
    /***********************************************************************/
    void sendPoints(const std::deque<yarp::sig::Vector> &p);

    /** Get the point cloud for computing the superquadric */
    /***********************************************************************/
    void getPoints3D();

    /** Return the computation time for estimating the superquadric
    * @return the period value
    */
    /***********************************************************************/
    double getTime();

    /***********************************************************************/
    void iterativeModeling();

    /***********************************************************************/
    bool mergeModeling(node *node);

    /****************************************************************/
    void superqUsingPlane(node *node, yarp::sig::Vector &plane);

    /****************************************************************/
    void computeSuperqAxis(node *node);

    /****************************************************************/
    bool axisParallel(node *node1, node *node2, yarp::sig::Matrix &relations);

    /****************************************************************/
    bool sectionEqual(node *node1, node *node2, yarp::sig::Matrix &relations);

    /***********************************************************************/
    void splitPoints(bool merging, node *leaf);

   /***********************************************************************/
   void computeNestedSuperq(node *newnode);

   /***********************************************************************/
   bool splitMore(node * node);

   /****************************************************************/
   double evaluateLoss(yarp::sig::Vector &superq, std::deque<yarp::sig::Vector> &points_splitted);

   /****************************************************************/
   double f(yarp::sig::Vector &x, yarp::sig::Vector &point_cloud);

   /***********************************************************************/
   yarp::sig::Vector computeOneShot(const std::deque<yarp::sig::Vector> &p);

   /***********************************************************************/
   void computeOneShotMultiple(const std::deque<yarp::sig::Vector> &p);

   /****************************************************************/
   bool computeEdges(node *node, std::deque<yarp::sig::Vector> &edges, yarp::sig::Matrix &relations);

   /****************************************************************/
   bool edgesClose(node *node1, node *node2, yarp::sig::Matrix &relations);


};

#endif
