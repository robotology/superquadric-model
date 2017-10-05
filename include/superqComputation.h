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

/*******************************************************************************/
class SpatialDensityFilter
{
public:

    /*******************************************************************************/
    static std::vector<int>  filter(const cv::Mat &data,const double radius, const int maxResults, std::deque<yarp::sig::Vector> &points);
};

/*******************************************************************************/
class SuperqComputation : public yarp::os::RateThread
{
protected:

    int count;
    bool save_points;
    std::string tag_file;
    std::string homeContextPath;
    yarp::os::ConstString pointCloudFileName;
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

    // Options for optimization with ipopt
    double tol, sum;
    double max_cpu_time;
    int acceptable_iter,max_iter;
    int optimizer_points;
    bool bounds_automatic;
    std::string mu_strategy,nlp_scaling_method;
    yarp::sig::Vector elem_x;

    double t_superq;
    int count_file;
    std::string ob_class;
    
    yarp::os::ResourceFinder *rf;
    double t,t0;
    yarp::os::Mutex mutex;

    // Classes uses for filtering the superquadric
    iCub::ctrl::MedianFilter *mFilter;
    iCub::ctrl::AWPolyEstimator *PolyEst;

    // Properties with all the options
    yarp::os::Property filter_points_par;
    yarp::os::Property filter_superq_par;
    yarp::os::Property ipopt_par;
public:

    // Median order parameters
    int std_median_order;
    int max_median_order;

    // Shared variables
    yarp::sig::Vector &x;
    yarp::sig::Vector &x_filtered;
    std::deque<yarp::sig::Vector> &points;
    std::deque<cv::Point> blob_points;

    // Input image
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *imgIn;

    /***********************************************************************/
    SuperqComputation(int _rate, bool _filter_points, bool _filter_superq, bool _fixed_window, std::deque<yarp::sig::Vector> &_points, yarp::sig::ImageOf<yarp::sig::PixelRgb> *imgIn,
                      std::string _tag_file, double _threshold_median, const yarp::os::Property &filters_points_par, yarp::sig::Vector &_x, yarp::sig::Vector &_x_filtered,
                      const yarp::os::Property &filters_superq_par, const yarp::os::Property &optimizer_par, const std::string &_homeContextPath, bool _save_points, yarp::os::ResourceFinder *rf);

    /* Set options for filtering the point cloud */
    /***********************************************************************/
    void setPointsFilterPar(const yarp::os::Property &newOptions, bool first_time);

    /* Set options for filtering the superquadric */
    /***********************************************************************/
    void setSuperqFilterPar(const yarp::os::Property &newOptions, bool first_time);

    /* Set options for the optimization problem solved by Ipopt */
    /***********************************************************************/
    void setIpoptPar(const yarp::os::Property &newOptions, bool first_time);

    /* Get options used for filtering the point cloud */
    /***********************************************************************/
    yarp::os::Property getPointsFilterPar();

    /* Get options used for filtering the superquadric */
    /***********************************************************************/
    yarp::os::Property getSuperqFilterPar();

    /* Get options used for solving the optimization problem with Ipopt */
    /***********************************************************************/
    yarp::os::Property getIpoptPar();

    /* Set a parameter equal to a value */
    /***********************************************************************/
    void setPar(const std::string &par_name, const std::string &value);

    /* Init function of RateThread class */
    /***********************************************************************/
    virtual bool threadInit();

    /* Run function of RateThread class */
    /***********************************************************************/
    virtual void run();

    /* Release function of RateThread class */
    /***********************************************************************/
    virtual void threadRelease();

    /* Save point cloud used for reconstructing the superquadric */
    /***********************************************************************/
    void savePoints(const std::string &namefile, const yarp::sig::Vector &colors);

    /* Read the object point cloud from txt for offline tests */
    /***********************************************************************/
    bool readPointCloud();

    /* Filter the received point cloud */
    /***********************************************************************/
    void filter();

    /* Compute the superquadric modeling the object */
    /***********************************************************************/
    bool computeSuperq();

    /* Filter the superquadri with the median filter */
    /***********************************************************************/
    void filterSuperq();

    /* Reset median filter */
    /***********************************************************************/
    void resetMedianFilter();

    /* Compute window size according to the object velocity */
    /***********************************************************************/
    int adaptWindComputation();

    /* Configure superquadric filter */
    /***********************************************************************/
    bool configFilterSuperq();

    /* Configure point cloud filter */
    /***********************************************************************/
    bool config3Dpoints();

    /* Return the computed superquadric */
    /***********************************************************************/
    yarp::sig::Vector getSolution(bool filtered_or_not);

    /* Get the point cloud for computing the superquadric */
    /***********************************************************************/
    void sendPoints(const std::deque<yarp::sig::Vector> &p);

    /* Get the point cloud for computing the superquadric */
    /***********************************************************************/
    void getPoints3D();

    /* Return the computation time for estimating the superquadric*/
    /***********************************************************************/
    double getTime();     
};

#endif


