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
    int r,g,b;
    double t_vis;
    std::string what_to_plot;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > portImgOut;

    int vis_points;
    int vis_step;
    std::string eye;
    yarp::sig::Matrix R,H,K;
    yarp::sig::Vector point,point1;
    yarp::sig::Vector point2D;
    std::deque<int> Color;

    //yarp::dev::IGazeControl *igaze;

    yarp::os::Mutex mutex;

public:

    yarp::sig::Vector &superq;
    yarp::sig::Vector &superq_filtered;

    std::deque<yarp::sig::Vector> &points;

    yarp::sig::ImageOf<yarp::sig::PixelRgb> *&imgIn;

//    /***********************************************************************/
//    SuperqVisualization(int rate, const std::string &_eye, const std::string &_what_to_plot, yarp::sig::Vector &x, yarp::sig::Vector &x_filtered,
//                        std::deque<int> &_Color, yarp::dev::IGazeControl *_igaze, const yarp::sig::Matrix _K, std::deque<yarp::sig::Vector> &_points,
//                        const int &_vis_points, const int &_vis_step, yarp::sig::ImageOf<yarp::sig::PixelRgb> *&imgIn);
    /***********************************************************************/
    SuperqVisualization(int rate, const std::string &_eye, const std::string &_what_to_plot, yarp::sig::Vector &x, yarp::sig::Vector &x_filtered,
                        std::deque<int> &_Color, const yarp::sig::Matrix _K, std::deque<yarp::sig::Vector> &_points,
                        const int &_vis_points, const int &_vis_step, yarp::sig::ImageOf<yarp::sig::PixelRgb> *&imgIn);

    /***********************************************************************/
    bool readPointCloud();

    /***********************************************************************/
    bool showPoints();

    /***********************************************************************/
    bool showSuperq(yarp::sig::Vector &x_to_show);

    /***********************************************************************/
    yarp::sig::Vector from3Dto2D(const yarp::sig::Vector &point3D);

    /***********************************************************************/
    virtual bool threadInit();

    /***********************************************************************/
    virtual void run();

    /***********************************************************************/
    virtual void threadRelease();

    /***********************************************************************/
    void sendImg(yarp::sig::ImageOf<yarp::sig::PixelRgb> *Img);

    /***********************************************************************/
    void sendSuperq(yarp::sig::Vector &x);

    /***********************************************************************/
    void sendPoints(std::deque<yarp::sig::Vector> &points);

    /***********************************************************************/
    void setPar(const std::string &par_name, const std::string &value);

    /***********************************************************************/
    void setPar(const std::string &par_name, const int &value);

    /***********************************************************************/
    void setColor (const int &r, const int &g, const int &b);

    /***********************************************************************/    
    void setPar(const yarp::os::Property &newOptions, bool first_time);

    /***********************************************************************/
    yarp::os::Property getPar();

    /***********************************************************************/
    double getTime();
};

#endif
