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

#ifndef __VISUALIZATION_H__
#define __VISUALIZATION_H__

#include <string>
#include <deque>
#include <map>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

#include <opencv2/opencv.hpp>

/*******************************************************************************/
class SuperqVisualization : public yarp::os::RateThread
{
protected:
    // Output image
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > portImgOut;

    // Parameters for visualization
    int r,g,b;
    double t_vis;
    int vis_step;
    int vis_points;   
    std::string what_to_plot;

    yarp::sig::Vector point,point1;
    yarp::sig::Vector point2D;
    std::deque<int> Color;

    // Variables for gaze
    std::string eye;
    yarp::sig::Matrix R,H,K;
    yarp::dev::IGazeControl *igaze;

    yarp::os::Mutex mutex;

public:

    // Shared variables
    yarp::sig::Vector &superq;
    yarp::sig::Vector &superq_filtered;
    std::deque<yarp::sig::Vector> &points;

    //Input image
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *&imgIn;

    /***********************************************************************/
    SuperqVisualization(int rate, const std::string &_eye, const std::string &_what_to_plot, yarp::sig::Vector &x, yarp::sig::Vector &x_filtered,
                        std::deque<int> &_Color, yarp::dev::IGazeControl *_igaze, const yarp::sig::Matrix _K, std::deque<yarp::sig::Vector> &_points,
                        const int &_vis_points, const int &_vis_step, yarp::sig::ImageOf<yarp::sig::PixelRgb> *&imgIn);

    /* Show point cloud on the image */
    /***********************************************************************/
    bool showPoints();

    /* Show reconstructed superquadric on the image */
    /***********************************************************************/
    bool showSuperq(yarp::sig::Vector &x_to_show);

    /* Compute 2D pixels from 3D points */
    /***********************************************************************/
    yarp::sig::Vector from3Dto2D(const yarp::sig::Vector &point3D);

    /* Init function of RateThread */
    /***********************************************************************/
    virtual bool threadInit();

    /* Run function of RateThread */
    /***********************************************************************/
    virtual void run();

    /* Release function of RateThread */
    /***********************************************************************/
    virtual void threadRelease();

    /* Set a given parameter equal to a string */
    /***********************************************************************/
    void setPar(const std::string &par_name, const std::string &value);

    /* Set a given parameter equal to a desired value */
    /***********************************************************************/
    void setPar(const std::string &par_name, const int &value);

    /* Set color for visualization  */
    /***********************************************************************/
    void setColor (const int &r, const int &g, const int &b);

    /* Set parameters for visualization */
    /***********************************************************************/    
    void setPar(const yarp::os::Property &newOptions, bool first_time);

    /* Get parameters for visualization */
    /***********************************************************************/
    yarp::os::Property getPar();

    /* Get time required for visualization */
    /***********************************************************************/
    double getTime();
};

#endif
