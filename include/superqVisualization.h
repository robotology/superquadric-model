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

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

/*******************************************************************************/
class SuperqVisualization : public RateThread
{
protected:
    int r,g,b;
    string what_to_plot;
    BufferedPort<ImageOf<PixelRgb> > portImgOut;

    int vis_points;
    int vis_step;
    string eye;
    Matrix R,H,K;
    Vector point,point1;
    Vector point2D;
    deque<int> Color;

    deque<Vector> points;
    Vector superq;

    IGazeControl *igaze;

    Mutex mutex;

public:

    ImageOf<PixelRgb> *imgIn;

    /***********************************************************************/
    SuperqVisualization(int rate, const string &_eye, const string &_what_to_plot,
                        deque<int> &_Color,IGazeControl *_igaze, const Matrix _K,
                        const int &_vis_points, const int &_vis_step);

    /***********************************************************************/
    bool readPointCloud();

    /***********************************************************************/
    bool showPoints();

    /***********************************************************************/
    bool showSuperq(Vector &x_to_show);

    /***********************************************************************/
    Vector from3Dto2D(const Vector &point3D);

    /***********************************************************************/
    virtual bool threadInit();

    /***********************************************************************/
    virtual void run();

    /***********************************************************************/
    virtual void threadRelease();

    /***********************************************************************/
    void sendImg(ImageOf<PixelRgb> *Img);

    /***********************************************************************/
    void sendSuperq(Vector &x);

    /***********************************************************************/
    void sendPoints(deque<Vector> &points);

    /***********************************************************************/
    void setPar(const string &par_name, const string &value);

    /***********************************************************************/
    void setPar(const string &par_name, const int &value);

    /***********************************************************************/
    void setColor (const int &r, const int &g, const int &b);

};

#endif
