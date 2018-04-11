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

#ifndef __MODULE_H__
#define __MODULE_H__

#include <yarp/dev/all.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <opencv2/opencv.hpp>

#include "superqComputation.h"
#include "superqVisualization.h"

#include "src/superquadricModel_IDL.h"

/**
  * The SuperqModule class handle the superquadric computation and
  * visualization, the point cloud and superquadric filtering and the
  * interaction with the user.
  * It is used to set all the parameters (offline and online) and to launch
  * all the thread for superquadric computation and visualization.
  */
/*******************************************************************************/
class SuperqModule : public yarp::os::RFModule,
                     public superquadricModel_IDL
{
protected:

    /** Red value for visualization**/
    int r;
    /** Green value for visualization **/
    int g;
    /** Blue value for visualization **/
    int b;
    /** Count variable**/
    int count;
    /** Computation thread rate**/
    int rate;
    /** Visualization thread rate**/
    int rate_vis;
    /** Tag name of files for saving 3D points**/
    std::string tag_file;
    /** Path where code context is located **/
    std::string homeContextPath;
    /** Pointcloud name file in case the module runs offline**/
    yarp::os::ConstString pointCloudFileName;
    /** Output file name saving the estimated superquadric**/
    std::string outputFileName;
    /** OpenCV variable for blob extraction**/
    std::vector<cv::Point> contour;
    /** 3D points used for reconstructing the superquadric **/
    std::deque<yarp::sig::Vector> points;
    /** 3D points auxiliary used for reconstructing the superquadric **/
    std::deque<yarp::sig::Vector> points_aux;
    /** 2D points of the segmented object **/
    std::deque<cv::Point> blob_points;

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
    /** Maximum median filder order allowed**/
    int max_median_order;
    /** New median filder order estimated**/
    int new_median_order;
    /** Boolean variable for enabling point cloud filtering**/
    bool filter_points;
    /** Boolean variable for enabling the use of a fixed window during the median filter**/
    bool fixed_window;
    /** Boolean variable for enabling superquadric filtering**/
    bool filter_superq;
    /** String used for deciding what to plot: "points" or "superq"**/
    std::string what_to_plot;
    /** Threshold for velocity estimation for median order**/
    double threshold_median;
    /** Minimum norm of velocity for considering the object moving**/
    double min_norm_vel;

    // On/off options
    /** Boolean variable for enabling online or offline mode**/
    bool mode_online;
    /** Boolean variable for enabling visualization**/
    bool visualization_on;
    /** Boolean variable for going to the next step of the state machine**/
    bool go_on;
    /** Boolean variable for resetting the median filter**/
    bool reset;
    /** Boolean variable for enabling point cloud saving**/
    bool save_points;

    // Optimization parameters
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
    /** Mu strategy of the Ipopt algorithm**/
    std::string mu_strategy;
    /** NLP scaling method of the Ipopt algorithm**/
    std::string nlp_scaling_method;

    /** Estimated superquadric**/
    yarp::sig::Vector x;
    /** Filtered superquadric**/
    yarp::sig::Vector x_filtered;

    // Time variables
    /** Time required for computing superquadric**/
    double t_superq;
    /** Times used for computing several superquadrics**/
    std::deque<double> times_superq;
    /** Time for visualization**/
    double t_vis;
    /** Collections of times required for visualization**/
    std::deque<double> times_vis;

    /** Port for streaming the computed superquadric**/
    yarp::os::BufferedPort<yarp::os::Property> portSuperq;
    /** Rpc port for interaction**/
    yarp::os::RpcServer portRpc;

    // Variables for visualization and gaze
    /** Number of points used for visualization**/
    int vis_points;
    /** Number of visualization step**/
    int vis_step;
    /** Eye camera selected **/
    std::string eye;
    yarp::sig::Matrix R,H,K;
    yarp::sig::Vector point,point1;
    yarp::sig:: Vector point2D;
    /** Color used for visualization**/
    std::deque<int> Color;
    /** Gaze Control driver for visualization**/
    yarp::dev::PolyDriver GazeCtrl;
    /** Gaze Control interface **/
    yarp::dev::IGazeControl *igaze;

    yarp::os::ResourceFinder *rf;
    double t,t0;
    std::deque<std::string> advanced_params;
    yarp::os::Mutex mutex;
    yarp::os::Mutex mutex_shared;

    std::string object_class;

    // Classes of the threads
    /** SuperqComputation class actually computes the superquadric**/
    SuperqComputation *superqCom;
    /** SuperqVisualization class shows the estimated superquadric**/
    SuperqVisualization *superqVis;

    // Property with all the parameters
    /** Parameters of point cloud filter**/
    yarp::os::Property filter_points_par;
    /** Parameters of superquadric filter**/
    yarp::os::Property filter_superq_par;
    /** Parameters of the Ipopt optimization problem**/
    yarp::os::Property ipopt_par;

    /************************************************************************/
    bool attach(yarp::os::RpcServer &source);

    /** Set a tag name for saving the superquadric
    * @param tag_file is the name of the file where to save the superquadric
    * @return true
    */
    /************************************************************************/
    bool set_tag_file(const std::string &tag_file);

     /** Get the tag name used for saving the superquadric
     * @return the currect name of the file used for saving
     */
    /************************************************************************/
    std::string get_tag_file();

     /** Return if visualization is on or off
    * @return  "on" or "off"
    */
    /**********************************************************************/
    std::string get_visualization();

    /** Set if visualization is on or off
    * @param e can be "on" or "off"
    * @return true/false on success/failure
    */
    /**********************************************************************/
    bool set_visualization(const std::string &e);

    /** Return the computed superquadric, given the  2D blob of the object
    * @param  blob is the 2D blob of the object
    * @return  a property with the estimated superquadric
    */
    /**********************************************************************/
    yarp::os::Property get_superq();

    /** Get the point cloud for computing the superquadric
    * @param p is the point cloud to be acquired
    * @return true
    */
    /**********************************************************************/
    bool send_point_clouds(const std::vector<yarp::sig::Vector> &p);

    /** Reset median filter for improving superquadric estimation
    * @return true
    */
    /**********************************************************************/
    bool reset_filter();

    /** Return the filtered superquadric
    * @return a property with the filtered superquadric
    */
    /**********************************************************************/
    yarp::os::Property get_superq_filtered();

    /** Property fill the property with the superquadric solution
    * @param sol is a Vector of the computed superquadric
    * @return a Property with the solution
    */
    /**********************************************************************/
    yarp::os::Property fillProperty(const yarp::sig::Vector &sol);

    /** Set if to filter or not the point cloud
    * @param entre can be "on" or "off"
    * @return true/false on success/failure
    */
    /**********************************************************************/
    bool set_points_filtering(const std::string &entry);

    /** Get if the point cloud is filtered or not
    * @return "on" or "off"
    */
    /**********************************************************************/
    std::string get_points_filtering();

    /** Set if to filter or not the superquadric
    * @param entry can be "on" or "off"
    * @return true/false on success/failure
    */
    /**********************************************************************/
    bool set_superq_filtering(const std::string &entry);

    /** Get if the superquadric is filtered or not
    * @return "on" or "off"
    */
    /**********************************************************************/
    std::string get_superq_filtering();

    /** Get options of a given field: visualization, optimization, filtering
    * @param field is one of the field of options
    * @return property with the options of interested
    */
    /**********************************************************************/
    yarp::os::Property get_options(const std::string &field);

    /** Set options of specified field: visualization, optimization, filtering
    * @param newOptions is a property with the options to be set
    * @param field is the field of the options to be set
    * @return true/false on success/failure
    */
    /**********************************************************************/
    bool set_options(const yarp::os::Property &newOptions, const std::string &field);

    /** Set object class for improving superquadric estimation
    * @param objclass is the object class (according to the shape)
    * @return true/false on success/failure
    */
    /**********************************************************************/
    bool set_object_class(const std::string &objclass);

public:
    /** Get period function of RF module
    * @return the period
    */
    /***********************************************************************/
    double getPeriod();

    /** updateModule function of RF module */
    /***********************************************************************/
    bool updateModule();

    /** configure function of RF module */
    /***********************************************************************/
    bool configure(yarp::os::ResourceFinder &rf);

    /** interrupt module function of RF module */
    /***********************************************************************/
    bool interruptModule();

    /** close function of RF module */
    /***********************************************************************/
    bool close();

    /** Configure all on/off options */
    /***********************************************************************/
    bool configOnOff(yarp::os::ResourceFinder &rf);

    /** Configure point cloud filter options */
    /***********************************************************************/
    bool configFilter(yarp::os::ResourceFinder &rf);

    /** Configure superquadric filter options */
    /***********************************************************************/
    bool configFilterSuperq(yarp::os::ResourceFinder &rf);

    /** Open ports for communication */
    /***********************************************************************/
    bool configServices(yarp::os::ResourceFinder &rf);

    /** Configure superquadric computation otpions */
    /***********************************************************************/
    bool configSuperq(yarp::os::ResourceFinder &rf);

    /** Configure visualization options */
    /***********************************************************************/
    bool configViewer(yarp::os::ResourceFinder &rf);

    /** Save computed superquadric */
    /***********************************************************************/
    void saveSuperq();

    /** Set if to save or not the used point cloud
    * @param entry can be "on" or "off"
    */
    /***********************************************************************/
    bool set_save_points(const std::string &entry);

     /** Get if the used point cloud is saved or not
    * @return "on" or "off"
    */
    /***********************************************************************/
    std::string get_save_points();

     /** In offline mode, read the point cloud from a txt file */
    /***********************************************************************/
    bool readPointCloud();
};

#endif


