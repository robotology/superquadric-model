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

#include "tree.h"
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

    int r,g,b;
    int count;
    int rate, rate_vis;
    std::string tag_file;
    std::string homeContextPath;
    std::string pointCloudFileName;
    std::string outputFileName;
    std::vector<cv::Point> contour;
    std::deque<yarp::sig::Vector> points;
    std::deque<yarp::sig::Vector> points_aux;
    std::deque<cv::Point> blob_points;

    // Filters parameters
    double radius;
    double f_thres;
    int tree_splitting;
    int nnThreshold;
    int numVertices;
    int median_order;
    int min_median_order;
    int max_median_order;
    int new_median_order;
    bool filter_points;
    bool fixed_window;
    bool filter_superq;
    bool single_superq;
    std::string what_to_plot;
    double threshold_median;
    double min_norm_vel;

    // On/off options
    bool mode_online;
    bool visualization_on;
    bool go_on;
    bool reset;
    bool save_points;
    bool merge_model;

    // Optimization parameters
    double tol, sum;
    double max_cpu_time;
    int acceptable_iter,max_iter;
    int optimizer_points;
    std::string mu_strategy,nlp_scaling_method;
    int h_tree;

    yarp::sig::Vector x;
    yarp::sig::Vector x_filtered;

    // Time variables
    double t_superq;
    std::deque<double> times_superq;
    double t_vis;
    std::deque<double> times_vis;

    // Input image port
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > portImgIn;
    // Port for streaming the computed superquadric
    yarp::os::BufferedPort<yarp::os::Property> portSuperq;
    // Rpc port for interaction
    yarp::os::RpcServer portRpc;

    // Variables for visualization and gaze
    int vis_points;
    int vis_step;
    std::string eye;
    yarp::sig::Matrix R,H,K;
    yarp::sig::Vector point,point1;
    yarp::sig:: Vector point2D;
    std::deque<int> Color;
    yarp::dev::PolyDriver GazeCtrl;
    yarp::dev::IGazeControl *igaze;

    yarp::os::ResourceFinder *rf;
    double t,t0, t_mult;
    std::deque<std::string> advanced_params;
    yarp::os::Mutex mutex;
    yarp::os::Mutex mutex_shared;

    std::string object_class;

    // Classes of the threads
    SuperqComputation *superqCom;
    SuperqVisualization *superqVis;

    // Property with all the parameters
    yarp::os::Property filter_points_par;
    yarp::os::Property filter_superq_par;
    yarp::os::Property ipopt_par;

    superqTree *superq_tree;

    yarp::sig::ImageOf<yarp::sig::PixelRgb> *imgIn;

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
     //yarp::os::Property get_superq(const std::vector<yarp::sig::Vector> &blob);

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
    yarp::os::Property fillMultipleSolutions(node *leaf);

    /**********************************************************************/
    void addSuperqInProp(node *leaf, int &count, yarp::os::Property &superq_pr);

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

    /**********************************************************************/
    bool set_single_superq(const std::string &s);
};

#endif
