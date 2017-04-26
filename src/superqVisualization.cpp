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

#include <yarp/math/Math.h>

#include "superqVisualization.h"

using namespace yarp::math;

/***********************************************************************/
SuperqVisualization::SuperqVisualization(int _rate,const string &_eye, const string &_what_to_plot,
                                         deque<int> &_Color,IGazeControl *_igaze, const Matrix _K,
                                         const int &_vis_points, const int &_vis_step):
                                          RateThread(_rate), eye(_eye), what_to_plot(_what_to_plot), Color(_Color), igaze(_igaze), K(_K),
                                         vis_points(_vis_points), vis_step(_vis_step)
{

}

/***********************************************************************/
bool SuperqVisualization::showSuperq(Vector &x_toshow)
{
    LockGuard lg(mutex);

    PixelRgb color(Color[0],Color[1],Color[2]);
    Vector pos, orient;
    double co,so,ce,se;
    Stamp *stamp=NULL;

    ImageOf<PixelRgb> &imgOut=portImgOut.prepare();
    imgOut=*imgIn;

    R=euler2dcm(x_toshow.subVector(8,10));
    R=R.transposed();

    if ((norm(x_toshow)>0.0))
    {
        if (eye=="left")
        {
            if (igaze->getLeftEyePose(pos,orient,stamp))
            {
                H=axis2dcm(orient);
                H.setSubcol(pos,0,3);
                H=SE3inv(H);
            }
        }
        else
        {
            if (igaze->getRightEyePose(pos,orient,stamp))
            {
                H=axis2dcm(orient);
                H.setSubcol(pos,0,3);
                H=SE3inv(H);
            }
        }

        double step=2*M_PI/vis_points;

        for (double eta=-M_PI; eta<M_PI; eta+=step)
        {
             for (double omega=-M_PI; omega<M_PI;omega+=step)
             {
                 co=cos(omega); so=sin(omega);
                 ce=cos(eta); se=sin(eta);

                 point[0]=x_toshow[0] * sign(ce)*(pow(abs(ce),x_toshow[3])) * sign(co)*(pow(abs(co),x_toshow[4])) * R(0,0) +
                            x_toshow[1] * sign(ce)*(pow(abs(ce),x_toshow[3]))* sign(so)*(pow(abs(so),x_toshow[4])) * R(0,1)+
                                x_toshow[2] * sign(se)*(pow(abs(se),x_toshow[3])) * R(0,2) + x_toshow[5];

                 point[1]=x_toshow[0] * sign(ce)*(pow(abs(ce),x_toshow[3])) * sign(co)*(pow(abs(co),x_toshow[4])) * R(1,0) +
                            x_toshow[1] * sign(ce)*(pow(abs(ce),x_toshow[3])) * sign(so)*(pow(abs(so),x_toshow[4])) * R(1,1)+
                                x_toshow[2] * sign(se)*(pow(abs(se),x_toshow[3])) * R(1,2) + x_toshow[6];

                 point[2]=x_toshow[0] * sign(ce)*(pow(abs(ce),x_toshow[3])) * sign(co)*(pow(abs(co),x_toshow[4])) * R(2,0) +
                            x_toshow[1] * sign(ce)*(pow(abs(ce),x_toshow[3])) * sign(so)*(pow(abs(so),x_toshow[4])) * R(2,1)+
                                x_toshow[2] * sign(se)*(pow(abs(se),x_toshow[3])) * R(2,2) + x_toshow[7];

                 point2D=from3Dto2D(point);

                 cv::Point target_point((int)point2D[0],(int)point2D[1]);

                 if ((target_point.x<0) || (target_point.y<0) || (target_point.x>=320) || (target_point.y>=240))
                 {
                     yError("[SuperqVisualization]: Not acceptable pixels!");
                 }
                 else
                    imgOut.pixel(target_point.x, target_point.y)=color;

             }
         }
    }

    portImgOut.write();

    return true;
}

/***********************************************************************/
bool SuperqVisualization::showPoints()
{
    PixelRgb color(Color[0],Color[1],Color[2]);
    Stamp *stamp=NULL;
    Vector pos, orient;

    ImageOf<PixelRgb> &imgOut=portImgOut.prepare();
    imgOut=*imgIn;

    if (eye=="left")
    {
        if (igaze->getLeftEyePose(pos,orient,stamp))
        {
            H=axis2dcm(orient);
            H.setSubcol(pos,0,3);
            H=SE3inv(H);
        }
    }
    else
    {
        if (igaze->getRightEyePose(pos,orient,stamp))
        {
            H=axis2dcm(orient);
            H.setSubcol(pos,0,3);
            H=SE3inv(H);
        }
    }

    Vector point(3,0.0);

     for (size_t i=0; i<points.size(); i+=vis_step)
     {
         point=points[i].subVector(0,2);
         point2D=from3Dto2D(point);

         cv::Point target_point((int)point2D[0],(int)point2D[1]);

         if ((target_point.x<0) || (target_point.y<0) || (target_point.x>=320) || (target_point.y>=240))
         {
             yError("[SuperqVisualization]:  Not acceptable pixels!");
         }
         else
            imgOut.pixel(target_point.x, target_point.y)=color;
     }

    portImgOut.write();

    return true;
}

/*******************************************************************************/
Vector SuperqVisualization::from3Dto2D(const Vector &point3D)
{
    Vector point2D(3,0.0);
    Vector point_aux(4,1.0);
    point_aux.setSubvector(0,point3D);
    point2D=K*H*point_aux;
    return point2D.subVector(0,1)/point2D[2];
}

/***********************************************************************/
bool SuperqVisualization::threadInit()
{
    cout<<endl<<"[SuperqVisualization]: Thread initing ... "<<endl<<endl;
    
    portImgOut.open("/superquadric-model/img:o");

    R.resize(4,4);
    H.resize(4,4);
    point2D.resize(2,0.0);
    point.resize(3,0.0);
    point1.resize(3,0.0);
    superq.resize(11,0.0);

    return true;
}

/***********************************************************************/
void SuperqVisualization::run()
{
    double t0=Time::now();
    if (what_to_plot=="superq" && imgIn!=NULL)
        showSuperq(superq);
    else if (what_to_plot=="points" && points.size()>0)
        showPoints();
    t_vis=Time::now()-t0;
}

/***********************************************************************/
void SuperqVisualization::sendImg(ImageOf<PixelRgb> *Img)
{
    LockGuard lg(mutex);
    imgIn=Img;
}

/***********************************************************************/
void SuperqVisualization::sendSuperq(Vector &x)
{
    LockGuard lg(mutex);
    superq=x;
}

/***********************************************************************/
void SuperqVisualization::sendPoints(deque<Vector> &p)
{
    LockGuard lg(mutex);
    points.clear();
    for (size_t i=0; i<p.size(); i++)
    {
        points.push_back(p[i]);
    }
}

/**********************************************************************/
void SuperqVisualization:: threadRelease()
{
    cout<<endl<<"[SuperVisualization]: Thread releasing ... "<<endl<<endl;

    if (!portImgOut.isClosed())
        portImgOut.close();
}

/***********************************************************************/
void SuperqVisualization::setPar(const Property &newOptions)
{
    Bottle &groupBottle=newOptions.findGroup("visualized_points");
    LockGuard lg(mutex);

    if (!groupBottle.isNull())
    {
        int v_points=groupBottle.get(1).asInt();
        if ((v_points)>=1 && (v_points<=300))
                vis_points=v_points;
        else
            vis_points=3;
    }

    Bottle &groupBottle2=newOptions.findGroup("what_to_plot");
    if (!groupBottle2.isNull())
    {
        string plot=groupBottle2.get(1).asString();
        if ((plot=="superq") || (plot=="points"))
                what_to_plot=plot;
        else
            what_to_plot="superq";
    }

    Bottle &groupBottle3=newOptions.findGroup("visualized_points_step");
    if (!groupBottle3.isNull())
    {
        int vpoint=groupBottle3.get(1).asInt();
        if ((vpoint>=1) && (vpoint<=100))
                vis_step=vpoint;
        else
            vis_step=10;
    }

    Bottle &groupBottle5=newOptions.findGroup("camera");
    if (!groupBottle5.isNull())
    {
        string cam=groupBottle5.get(1).asString();
        if ((cam=="left") || (cam=="right"))
                eye=cam;
        else
            eye="left";
    }

    Bottle &groupBottle6=newOptions.findGroup("color");
    if (!groupBottle6.isNull())
    {
        string col=groupBottle6.get(1).asString();
        if (col=="red")
        {
            Color[0]=255; Color[1]=0; Color[2]=0;
        }
        else if (col=="green")
        {
            Color[0]=0; Color[1]=255; Color[2]=0;
        }
        else if (col=="blue")
        {
            Color[0]=0; Color[1]=0; Color[2]=255;
        }                
        else
        {
            Color[0]=255; Color[1]=0; Color[2]=0;
        }
    }
}

/***********************************************************************/
Property SuperqVisualization::getPar()
{
    LockGuard lg(mutex);

    Property advOptions;
    advOptions.put("visualized_points",vis_points);
    if (Color[0]==255 && Color[1]==0 && Color[2]==0)
        advOptions.put("color","red");
    else if  (Color[0]==0 && Color[1]==255 && Color[2]==0)
        advOptions.put("color","green");
    else if  (Color[0]==0 && Color[1]==0 &&Color[2]==255)
        advOptions.put("color","blue");
    advOptions.put("camera",eye);
    advOptions.put("visualized_points_step",vis_step);
    advOptions.put("what_to_plot",what_to_plot);
    return advOptions;
}


/***********************************************************************/
double SuperqVisualization::getTime()
{   
    LockGuard lg(mutex);
    return t_vis;
}

