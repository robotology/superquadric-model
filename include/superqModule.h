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

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

/*******************************************************************************/
class SuperqModule : public RFModule,
                     public superquadricModel_IDL
{
protected:

    int r,g,b;
    int count;
    int rate, rate_vis;
    string objname;
    string method;
    string homeContextPath;
    ConstString pointCloudFileName;
    string outputFileName;
    vector<cv::Point> contour;
    deque<Vector> points;
    deque<cv::Point> blob_points;

    RpcClient portBlobRpc;
    RpcClient portSFMrpc;
    RpcClient portOPCrpc;
    RpcServer portRpc;

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
    string what_to_plot;
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
    string mu_strategy,nlp_scaling_method;
    Vector x;
    Vector elem_x;
    Vector x_filtered;
    deque<Vector> x_window;

    double t_superq;
    double t_shows1, t_shows2;

    BufferedPort<ImageOf<PixelRgb> > portImgIn;
    BufferedPort<ImageOf<PixelRgb> > portImgOut;
    BufferedPort<Vector> portSuperq;

    PolyDriver GazeCtrl;
    IGazeControl *igaze;

    int vis_points;
    int vis_step;
    string eye;
    Matrix R,H,K;
    Vector point,point1;
    Vector point2D;
    deque<int> Color;

    ResourceFinder *rf;
    double t,t0;
    deque<string> advanced_params;
    Mutex mutex;

    SuperqComputation *superqCom;
    SuperqVisualization *superqVis;

    Property filter_points_par;
    Property filter_superq_par;
    Property ipopt_par;

    ImageOf<PixelRgb> *imgIn;

    /************************************************************************/
    bool attach(RpcServer &source);
    /************************************************************************/
    bool set_object_name(const string &object_name);

    /************************************************************************/
    bool set_seed_point(const int x, const int y);

    /************************************************************************/
    string get_object_name();

    /************************************************************************/
    string get_method();

    /************************************************************************/
    vector<int> get_color();

    /**********************************************************************/
    bool set_color(const int red, const int green, const int blue);

    /**********************************************************************/
    string get_visualization_on();

    /**********************************************************************/
    bool set_visualization_on(const string &e);

    /**********************************************************************/
    string get_eye();

    /**********************************************************************/
    bool set_eye(const string &e);

    /**********************************************************************/
    int get_visualized_points();

    /**********************************************************************/
    bool set_visualized_points(const int v);

    /**********************************************************************/
    Property get_superq(const string &name, bool filtered_or_not);

    /**********************************************************************/
    bool set_filtering(const string &entry);

    /**********************************************************************/
    string get_filtering();

    /**********************************************************************/
    bool set_filtering_superq(const string &entry);

    /**********************************************************************/
    string get_filtering_superq();

    /**********************************************************************/
    Property get_advanced_options(const string &field);

    /**********************************************************************/
    bool set_advanced_options(const Property &newOptions, const string &field);

    /**********************************************************************/
    bool set_plot(const string &plot);

    /**********************************************************************/
    string get_plot();

    /**********************************************************************/
    bool set_visualized_points_step(const int step);

    /**********************************************************************/
    int get_visualized_points_step();

    /**********************************************************************/
    bool set_fixed_window(const string &entry);

    /**********************************************************************/
    bool get_fixed_window();

public:
    /***********************************************************************/
    double getPeriod();

    /***********************************************************************/
    bool updateModule();

    /***********************************************************************/
    bool configure(ResourceFinder &rf);

    /***********************************************************************/
    bool interruptModule();

    /***********************************************************************/
    bool close();

    /***********************************************************************/
    bool configOnOff(ResourceFinder &rf);

    /***********************************************************************/
    bool configFilter(ResourceFinder &rf);

    /***********************************************************************/
    bool configFilterSuperq(ResourceFinder &rf);

    /***********************************************************************/
    bool configServices(ResourceFinder &rf);

    /***********************************************************************/
    bool configSuperq(ResourceFinder &rf);

    /***********************************************************************/
    bool configViewer(ResourceFinder &rf);

    /***********************************************************************/
    void saveSuperq();

    /***********************************************************************/
    bool set_save_points(const string &entry);

    /***********************************************************************/
    string get_save_points();

    /***********************************************************************/
    bool readPointCloud();
};

#endif


