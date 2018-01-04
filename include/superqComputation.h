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

	/** Count variable**/
    int count;
	/** Boolean variable for enabling point cloud saving**/
    bool save_points;
	/** Tag name of files for saving 3D points**/
    std::string tag_file;
	/** Path where code context is located **/
    std::string homeContextPath;
	/** Pointcloud name file in case the module runs offline**/
    yarp::os::ConstString pointCloudFileName;
	/** OpenCV variable for blob extraction**/
    std::vector<cv::Point> contour;

	/** Port for receiving point cloud **/
    yarp::os::BufferedPort<yarp::os::Bottle> pointPort;

    // Filters parameters
	/** Radius for spatial density filter**/
    double radius;
	/** Density threshold for spatial density filter**/
    int nnThreshold;
    int numVertices;
	/** Median filder order**/
    int median_order;  
	/** Minimum median filder order allowed**/ 
    int min_median_order;   
	/** New median filder order estimated**/ 
    int new_median_order;
	/** Boolean variable for enabling point cloud filtering**/
    bool filter_points;
	/** Boolean variable for enabling the use of a fixed window during the median filter**/
    bool fixed_window;	
	/** Boolean variable for enabling superquadric filtering**/
    bool filter_superq;
	/** Threshold for velocity estimation for median order**/
    double threshold_median;
	/** Minimum norm of velocity for considering the object moving**/
    double min_norm_vel;

    // On/off parameters
	/** Boolean variable for going to the next step of the state machine**/
    bool go_on;
    bool one_shot;

    // Options for optimization with ipopt
	/** Tolerance of the optimization problem **/
    double tol;
	double sum;
	/** Max cpu time allowed for solving the optimization problem**/
    double max_cpu_time;
	/** Acceptable iter of Ipopt algorithm**/
    int acceptable_iter;
	/** Maximum iteration allowed of Ipopt algorithm**/
	int max_iter;
	/** Number of 3D points used for optimization**/
    int optimizer_points;
	/** Boolean varibal for enabling automatic computation of variables bounds**/
    bool bounds_automatic;
	/** Mu strategy of the Ipopt algorithm**/
    std::string mu_strategy;
	/** NLP scaling method of the Ipopt algorithm**/
	std::string nlp_scaling_method;
    yarp::sig::Vector elem_x;

	/** Time required for computing superquadric**/
    double t_superq;
    int count_file;
	/** Objec class: cylinder, sphere of box**/
    std::string ob_class;
    
    yarp::os::ResourceFinder *rf;
    double t,t0;
    yarp::os::Mutex mutex;
    yarp::os::Mutex &mutex_shared;

    // Classes uses for filtering the superquadric
	/** Median filter for discarding wrong superquadrics**/
    iCub::ctrl::MedianFilter *mFilter;
	/** Time Polyestimator for estimating object velocity*/
    iCub::ctrl::AWPolyEstimator *PolyEst;

    // Properties with all the options
	/** Parameters of point cloud filter**/
    yarp::os::Property filter_points_par;
	/** Parameters of superquadric filter**/
    yarp::os::Property filter_superq_par;
	/** Parameters of the Ipopt optimization problem**/
    yarp::os::Property ipopt_par;
public:

    /** median order width*/
    int std_median_order;
    /** max median order width*/
    int max_median_order;

    /** Object superquadric*/
    yarp::sig::Vector &x;
    /** Filtered object superquadric*/
    yarp::sig::Vector &x_filtered;
    /** Object point cloud*/
    std::deque<yarp::sig::Vector> &points;
    /** Object  2D blob*/
    std::deque<cv::Point> blob_points;

    /** Input image*/
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *imgIn;

    /***********************************************************************/
    SuperqComputation(yarp::os::Mutex &mutex_shared, int _rate, bool _filter_points, bool _filter_superq, bool _fixed_window, std::deque<yarp::sig::Vector> &_points, yarp::sig::ImageOf<yarp::sig::PixelRgb> *imgIn,
                      std::string _tag_file, double _threshold_median, const yarp::os::Property &filters_points_par, yarp::sig::Vector &_x, yarp::sig::Vector &_x_filtered,
                      const yarp::os::Property &filters_superq_par, const yarp::os::Property &optimizer_par, const std::string &_homeContextPath, bool _save_points, yarp::os::ResourceFinder *rf);

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
};

#endif


