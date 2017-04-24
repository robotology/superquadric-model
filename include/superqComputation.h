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

//#include "src/superquadricModel_IDL.h"

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
class SuperqComputation : public RateThread
{
protected:

    int count;
    bool save_points;
    string objname;
    string homeContextPath;
    ConstString pointCloudFileName;
    vector<cv::Point> contour;
    deque<Vector> points;
    deque<cv::Point> blob_points;

    RpcClient portBlobRpc;
    RpcClient portSFMrpc;
    RpcClient portOPCrpc;

    double radius;
    int nnThreshold;
    int numVertices;
    int median_order;
    int std_median_order;
    int min_median_order;
    int max_median_order;
    int new_median_order;
    bool filter_points;
    bool fixed_window;
    bool filter_superq;
    double threshold_median;
    double min_norm_vel;

    bool go_on;
    double tol, sum;
    double max_cpu_time;
    int acceptable_iter,max_iter;
    int optimizer_points;
    bool bounds_automatic;
    string mu_strategy,nlp_scaling_method;
    Vector x;
    Vector elem_x;
    Vector x_filtered;
    deque<Vector> x_window;

    double t_superq;
    
    ResourceFinder *rf;
    double t,t0;
    deque<string> advanced_params;
    Mutex mutex;

    MedianFilter *mFilter;
    AWPolyEstimator *PolyEst;

    Property filter_points_par;
    Property filter_superq_par;
    Property ipopt_par;
public:

    ImageOf<PixelRgb> *imgIn;
    /***********************************************************************/
    SuperqComputation(int _rate, bool _filter_points, bool _filter_superq, bool _fixed_window,string _objname, double _threshold_median,
                      const Property &filters_points_par, const Property &filters_superq_par, const Property &optimizer_par, const string &_homeContextPath, bool _save_points);
    /***********************************************************************/
    void setPointsFilterPar(const Property &newOptions);

    /***********************************************************************/
    void setSuperqFilterPar(const Property &newOptions);

    /***********************************************************************/
    void setIpoptPar(const Property &newOptions);

    /***********************************************************************/
    Property getPointsFilterPar();

    /***********************************************************************/
    Property getSuperqFilterPar();

    /***********************************************************************/
    Property getIpoptPar();

    /***********************************************************************/
    void setPar(const string &par_name, const string &value);

    /***********************************************************************/
    virtual bool threadInit();

    /***********************************************************************/
    virtual void run();

    /***********************************************************************/
    virtual void threadRelease();

    /***********************************************************************/
    void acquirePointsFromBlob(ImageOf<PixelRgb> *imgIn);

    /***********************************************************************/
    void getBlob();

    /***********************************************************************/
    void get3Dpoints(ImageOf<PixelRgb> *imgIn);

    /***********************************************************************/
    void pointFromName();

    /***********************************************************************/
    void savePoints(const string &namefile, const Vector &colors);

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
    bool configFilterSuperq();

    /***********************************************************************/
    bool config3Dpoints();

    /***********************************************************************/
    void sendImg(ImageOf<PixelRgb> *Img);

    /***********************************************************************/
    Vector getSolution(const string &name, bool filtered_or_not);

    /***********************************************************************/
    void setContour(cv::Point p);

    /***********************************************************************/
    void getPoints(deque<Vector> &p);

    /***********************************************************************/
    void sendPoints(deque<Vector> &p);
};

#endif


