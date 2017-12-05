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

#include <tree.h>

#include <opencv2/opencv.hpp>

/**
  * This class shows the point cloud used for modeling or the estimated
  * superquadric overlapped on the camera image and in real time.
  */
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

    superqTree *&superq_tree;

public:

    bool &single_superq;

    yarp::sig::Vector &superq;
    /** Filtered superquadric */
    yarp::sig::Vector &superq_filtered;
    /** Object point cloud */
    std::deque<yarp::sig::Vector> &points;

    /** Input image */
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *&imgIn;

    /***********************************************************************/
    SuperqVisualization(int rate, const std::string &_eye, const std::string &_what_to_plot, yarp::sig::Vector &x, yarp::sig::Vector &x_filtered,
                        std::deque<int> &_Color, yarp::dev::IGazeControl *_igaze, const yarp::sig::Matrix _K, std::deque<yarp::sig::Vector> &_points,
                        const int &_vis_points, const int &_vis_step, yarp::sig::ImageOf<yarp::sig::PixelRgb> *&imgIn, superqTree *&superq_tree, bool &single_superq);

    /** Show point cloud on the image
    * @return true
    */
    /***********************************************************************/
    bool showPoints();

    /** Show reconstructed superquadric on the image
    * @param x_to_show is the superquadric to be shown
    * @return true/false on success/failure
    */
    /***********************************************************************/
    bool showSuperq(yarp::sig::Vector &x_to_show);

    /** Compute 2D pixels from 3D points
    * @param point3D is the 3D point to be converted
    * @return a 2D vector representing the corresponding pixel
    */
    /***********************************************************************/
    bool showMultipleSuperqs(superqTree *&superq_tree);

    /***********************************************************************/
    bool showTree(node *leaf);

    /***********************************************************************/
    yarp::sig::Vector from3Dto2D(const yarp::sig::Vector &point3D);

    /** Init function of RateThread */
    /***********************************************************************/
    virtual bool threadInit();

    /** Run function of RateThread */
    /***********************************************************************/
    virtual void run();

    /** Release function of RateThread */
    /***********************************************************************/
    virtual void threadRelease();

    /** Set a given parameter equal to a string
    * @param par_name is the name of the parameter to be changed
    * @param value is the new value
    */
    /***********************************************************************/
    void setPar(const std::string &par_name, const std::string &value);

    /** Set a given parameter equal to a desired value
    * @param par_name is the name of the parameter to be changed
    * @param value is the new value
    */
    /***********************************************************************/
    void setPar(const std::string &par_name, const int &value);

    /** Set color for visualization
    * @param r is the red component
    * @param g is the green component
    * @param b is the blue component
    */
    /***********************************************************************/
    void setColor (const int &r, const int &g, const int &b);

    /** Set parameters for visualization
    * @param newOptions is a Property with the new options to set
    * @param first_time takes into account if the options have already been set or not
    */
    /***********************************************************************/
    void setPar(const yarp::os::Property &newOptions, bool first_time);

    /** Get parameters for visualization
    * @return a property with all the visualization options
    */
    /***********************************************************************/
    yarp::os::Property getPar();

    /** Get time required for visualization
    * @return the visualization time
    */
    /***********************************************************************/
    double getTime();
};

#endif
