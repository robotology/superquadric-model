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
    yarp::os::ConstString pointCloudFileName;
    std::string outputFileName;
    std::vector<cv::Point> contour;
    std::deque<yarp::sig::Vector> points;
    std::deque<cv::Point> blob_points;

    double radius;
    int nnThreshold;
    int numVertices;
    int median_order;
    int min_median_order;
    int max_median_order;
    int new_median_order;
    bool filter_points;
    bool fixed_window;
    bool filter_superq;
    std::string what_to_plot;
    double threshold_median;
    double min_norm_vel;

    bool mode_online;
    bool visualization_on;
    bool go_on;
    bool save_points;
    double tol, sum;
    double max_cpu_time;
    int acceptable_iter,max_iter;
    int optimizer_points;
    std::string mu_strategy,nlp_scaling_method;
    yarp::sig::Vector x;
    yarp::sig::Vector x_filtered;

    double t_superq;
    std::deque<double> times_superq;
    double t_vis;
    std::deque<double> times_vis;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > portImgIn;
    yarp::os::BufferedPort<yarp::os::Property> portSuperq;
    yarp::os::RpcServer portRpc;

    yarp::dev::PolyDriver GazeCtrl;
    yarp::dev::IGazeControl *igaze;

    int vis_points;
    int vis_step;
    std::string eye;
    yarp::sig::Matrix R,H,K;
    yarp::sig::Vector point,point1;
    yarp::sig:: Vector point2D;
    std::deque<int> Color;

    yarp::os::ResourceFinder *rf;
    double t,t0;
    std::deque<std::string> advanced_params;
    yarp::os::Mutex mutex;

    SuperqComputation *superqCom;
    SuperqVisualization *superqVis;

    yarp::os::Property filter_points_par;
    yarp::os::Property filter_superq_par;
    yarp::os::Property ipopt_par;

    yarp::sig::ImageOf<yarp::sig::PixelRgb> *imgIn;

    /************************************************************************/
    bool attach(yarp::os::RpcServer &source);

    /************************************************************************/
    bool set_tag_file(const std::string &tag_file);

    /************************************************************************/
    std::string get_tag_file();

    /**********************************************************************/
    std::string get_visualization();

    /**********************************************************************/
    bool set_visualization(const std::string &e);

    /**********************************************************************/
    yarp::os::Property get_superq(const std::vector<yarp::sig::Vector> &blob, bool filtered_superq);

    /**********************************************************************/
    yarp::os::Property fillProperty(const yarp::sig::Vector &sol);

    /**********************************************************************/
    bool set_points_filtering(const std::string &entry);

    /**********************************************************************/
    std::string get_points_filtering();

    /**********************************************************************/
    bool set_superq_filtering(const std::string &entry);

    /**********************************************************************/
    std::string get_superq_filtering();

    /**********************************************************************/
    yarp::os::Property get_options(const std::string &field);

    /**********************************************************************/
    bool set_options(const yarp::os::Property &newOptions, const std::string &field);

public:
    /***********************************************************************/
    double getPeriod();

    /***********************************************************************/
    bool updateModule();

    /***********************************************************************/
    bool configure(yarp::os::ResourceFinder &rf);

    /***********************************************************************/
    bool interruptModule();

    /***********************************************************************/
    bool close();

    /***********************************************************************/
    bool configOnOff(yarp::os::ResourceFinder &rf);

    /***********************************************************************/
    bool configFilter(yarp::os::ResourceFinder &rf);

    /***********************************************************************/
    bool configFilterSuperq(yarp::os::ResourceFinder &rf);

    /***********************************************************************/
    bool configServices(yarp::os::ResourceFinder &rf);

    /***********************************************************************/
    bool configSuperq(yarp::os::ResourceFinder &rf);

    /***********************************************************************/
    bool configViewer(yarp::os::ResourceFinder &rf);

    /***********************************************************************/
    void saveSuperq();

    /***********************************************************************/
    bool set_save_points(const std::string &entry);

    /***********************************************************************/
    std::string get_save_points();

    /***********************************************************************/
    bool readPointCloud();
};

#endif


