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

    bool go_on;
    bool one_shot;
    double tol, sum;
    double max_cpu_time;
    int acceptable_iter,max_iter;
    int optimizer_points;
    bool bounds_automatic;
    std::string mu_strategy,nlp_scaling_method;
    yarp::sig::Vector elem_x;

    bool single_superq;
    int num_superq;
    double f_thresh;

    double t_superq;
    int count_file;
    std::string ob_class;
    
    yarp::os::ResourceFinder *rf;
    double t,t0;
    yarp::os::Mutex mutex;

    iCub::ctrl::MedianFilter *mFilter;
    iCub::ctrl::AWPolyEstimator *PolyEst;

    yarp::os::Property filter_points_par;
    yarp::os::Property filter_superq_par;
    yarp::os::Property ipopt_par;
public:

    int std_median_order;
    int max_median_order;

    yarp::sig::Vector &x;
    yarp::sig::Vector &x_filtered;
    std::deque<yarp::sig::Vector> &points;
    std::deque<cv::Point> blob_points;

    std::deque<yarp::sig::Vector> points_splitted1, points_splitted2;
    std::deque<yarp::sig::Vector> good_superq;

    std::deque<yarp::sig::Vector> planes;

    yarp::sig::ImageOf<yarp::sig::PixelRgb> *imgIn;

    /***********************************************************************/
    SuperqComputation(int _rate, bool _filter_points, bool _filter_superq, bool single_superq, bool _fixed_window, std::deque<yarp::sig::Vector> &_points, yarp::sig::ImageOf<yarp::sig::PixelRgb> *imgIn,
                      std::string _tag_file, double _threshold_median, const yarp::os::Property &filters_points_par, yarp::sig::Vector &_x, yarp::sig::Vector &_x_filtered,
                      const yarp::os::Property &filters_superq_par, const yarp::os::Property &optimizer_par, const std::string &_homeContextPath, bool _save_points, yarp::os::ResourceFinder *rf);

    /***********************************************************************/
    void setPointsFilterPar(const yarp::os::Property &newOptions, bool first_time);

    /***********************************************************************/
    void setSuperqFilterPar(const yarp::os::Property &newOptions, bool first_time);

    /***********************************************************************/
    void setIpoptPar(const yarp::os::Property &newOptions, bool first_time);

    /***********************************************************************/
    yarp::os::Property getPointsFilterPar();

    /***********************************************************************/
    yarp::os::Property getSuperqFilterPar();

    /***********************************************************************/
    yarp::os::Property getIpoptPar();

    /***********************************************************************/
    void setPar(const std::string &par_name, const std::string &value);

    /***********************************************************************/
    virtual bool threadInit();

    /***********************************************************************/
    virtual void run();

    /***********************************************************************/
    virtual void threadRelease();

    /***********************************************************************/
    void getBlob();

    /***********************************************************************/
    void get3Dpoints(yarp::sig::ImageOf<yarp::sig::PixelRgb> *imgIn);

    /***********************************************************************/
    void pointFromName();

    /***********************************************************************/
    void savePoints(const std::string &namefile, const yarp::sig::Vector &colors);

    /***********************************************************************/
    bool readPointCloud();

    /***********************************************************************/
    void filter();

    /***********************************************************************/
    bool computeSuperq();

    /***********************************************************************/
    yarp::sig::Vector computeMultipleSuperq(const std::deque<yarp::sig::Vector> &points);

    /***********************************************************************/
    void filterSuperq();

    /***********************************************************************/
    void resetMedianFilter();

    /***********************************************************************/
    int adaptWindComputation();

    /***********************************************************************/
    bool configFilterSuperq();

    /***********************************************************************/
    bool config3Dpoints();

    /***********************************************************************/
    void sendImg(yarp::sig::ImageOf<yarp::sig::PixelRgb> *Img);

    /***********************************************************************/
    yarp::sig::Vector getSolution(bool filtered_or_not);

    /***********************************************************************/
    void setContour(cv::Point p);

    /***********************************************************************/
    void getPoints(std::deque<yarp::sig::Vector> &p);

    /***********************************************************************/
    void sendPoints(const std::deque<yarp::sig::Vector> &p);

    /***********************************************************************/
    void getPoints3D();

    /***********************************************************************/
    double getTime();

    /***********************************************************************/
    void iterativeModeling();

    /***********************************************************************/
    void mergeModeling();

    /***********************************************************************/
    void splitPoints(const int &iter, std::deque<yarp::sig::Vector> &points_splitted, bool merging);

    /****************************************************************/
    std::deque<double> evaluateLoss(std::deque<yarp::sig::Vector> &superq, int &count);

    /****************************************************************/
   double f(yarp::sig::Vector &x, yarp::sig::Vector &point_cloud);
};

#endif


