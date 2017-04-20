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

#include <opencv2/opencv.hpp>

#include <iCub/ctrl/filters.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>

#include "superquadric.h"

#include "src/superquadricModel_IDL.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace iCub::ctrl;

/*******************************************************************************/
class SpatialDensityFilter
{
public:

    /*******************************************************************************/
    static vector<int>  filter(const cv::Mat &data,const double radius, const int maxResults, deque<Vector> &points);
};

/*******************************************************************************/
class SuperqModule : public RFModule,
                     public superquadricModel_IDL
{
protected:

    int r,g,b;
    int count;
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
    bool filter_on;
    bool fixed_window;
    bool filter_superq;
    string what_to_plot;
    double threshold_median;
    double min_norm_vel;

    bool mode_online;
    bool go_on;
    double tol, sum;
    double max_cpu_time;
    int acceptable_iter,max_iter;
    unsigned int optimizer_points;
    string mu_strategy,nlp_scaling_method;
    Vector x;
    Vector elem_x;
    Vector x_filtered;
    deque<Vector> x_window;

    double t_superq;
    double t_shows1, t_shows2;

    BufferedPort<ImageOf<PixelRgb> > portImgIn;
    BufferedPort<ImageOf<PixelRgb> > portImgOut;
    BufferedPort<Bottle> portSuperqIn;

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

    MedianFilter *mFilter;
    AWPolyEstimator *PolyEst;

    /************************************************************************/
    bool attach(RpcServer &source);
    /************************************************************************/
    bool set_object_name(const string &object_name);

    /************************************************************************/
    bool set_seed_point(const int &x, const int &y);

    /************************************************************************/
    string get_object_name();

    /************************************************************************/
    string get_method();

    /************************************************************************/
    vector<int> get_color();

    /**********************************************************************/
    bool set_color(const int red, const int green, const int blue);

    /**********************************************************************/
    string get_eye();

    /**********************************************************************/
    bool set_eye(const string &e);

    /**********************************************************************/
    int get_optimizer_points();

    /**********************************************************************/
    bool set_optimizer_points(const int max);

    /**********************************************************************/
    int get_visualized_points();

    /**********************************************************************/
    bool set_visualized_points(const int v);

    /**********************************************************************/
    vector<double> get_superq(const string &name, const string &filtered_or_not);

    /**********************************************************************/
    bool set_filtering(const string &entry);

    /**********************************************************************/
    string get_filtering();

    /**********************************************************************/
    bool set_filtering_superq(const string &entry);

    /**********************************************************************/
    string get_filtering_superq();

    /**********************************************************************/
    bool set_fixed_median_order(const int m);

    /**********************************************************************/
    double get_fixed_median_order();

    /**********************************************************************/
    bool set_max_median_order(const int m);

    /**********************************************************************/
    double get_max_median_order();

    /**********************************************************************/
    bool set_min_median_order(const int m);

    /**********************************************************************/
    double get_min_median_order();

    /**********************************************************************/
    bool set_tol(const double t);

    /**********************************************************************/
    double get_tol();

    /**********************************************************************/
    bool set_max_time(const double max_t);

    /**********************************************************************/
    double get_max_time();

    /**********************************************************************/
    Property get_advanced_options();

    /**********************************************************************/
    bool set_advanced_options(const Property &newOptions);

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
    bool config3Dpoints(ResourceFinder &rf);

    /***********************************************************************/
    bool configSuperq(ResourceFinder &rf);

    /***********************************************************************/
    bool configViewer(ResourceFinder &rf);

    /***********************************************************************/
    void acquirePointsFromBlob();

    /***********************************************************************/
    void getBlob( const PixelRgb &color);

    /***********************************************************************/
    void get3Dpoints( const PixelRgb &color);

    /***********************************************************************/
    void pointFromName();

    /***********************************************************************/
    void savePoints(const string &namefile, const Vector &colors);

    /***********************************************************************/
    void saveSuperq();

    /***********************************************************************/
    bool readPointCloud();

    /***********************************************************************/
    void filter();

    /***********************************************************************/
    bool computeSuperq();

    /***********************************************************************/
    void filterSuperq();

    /***********************************************************************/
    int adaptWindComputation();

    /***********************************************************************/
    bool showPoints();

    /***********************************************************************/
    bool showSuperq(Vector &x_toshow);

    /*******************************************************************************/
    Vector from3Dto2D(const Vector &point3D);
};

#endif


