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

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;

/***********************************************************************/
SuperqVisualization::SuperqVisualization(int _rate, const string &_what_to_plot, Vector &_x, Vector &_x_filtered,
                                         deque<int> &_Color, const Matrix _K, deque<Vector> &_points,
                                         const int &_vis_points, const int &_vis_step, ImageOf<PixelRgb> *&_imgIn):
                                         RateThread(_rate), what_to_plot(_what_to_plot), Color(_Color), K(_K),
                                         vis_points(_vis_points), vis_step(_vis_step), superq(_x), superq_filtered(_x_filtered), points(_points), imgIn(_imgIn)
{

}

/***********************************************************************/
bool SuperqVisualization::showSuperq(Vector &x_toshow)
{
    LockGuard lg(mutex);

    PixelRgb color(Color[0],Color[1],Color[2]);
    double co,so,ce,se;

    ImageOf<PixelRgb> &imgOut=portImgOut.prepare();

    imgOut=*imgIn;

    R=euler2dcm(x_toshow.subVector(8,10));
    R=R.transposed();

    if ((norm(x_toshow)>0.0))
    {
        Property *frame_info=portFrameIn.read(false);
        Vector x(3);
        Vector o(4);

        if (frame_info!=NULL)
        {
            Bottle &pose_b=frame_info->findGroup("depth");
            Bottle *pose=pose_b.get(1).asList();
            x[0]=pose->get(0).asDouble();
            x[1]=pose->get(1).asDouble();
            x[2]=pose->get(2).asDouble();

            o[0]=pose->get(3).asDouble();
            o[1]=pose->get(4).asDouble();
            o[2]=pose->get(5).asDouble();
            o[3]=pose->get(6).asDouble();

            H=axis2dcm(o);
            H.setSubcol(x,0,3);
            H(3,3)=1;
            H=SE3inv(H);
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

                 point2D=from3Dto2D(point, H);

                 cv::Point target_point((int)point2D[0],(int)point2D[1]);

                 if ((target_point.x<0) || (target_point.y<0) || (target_point.x>=320) || (target_point.y>=240))
                 {
                     yWarning("[SuperqVisualization]: Not acceptable pixels!");
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

    ImageOf<PixelRgb> &imgOut=portImgOut.prepare();
    imgOut=*imgIn;

    Property *frame_info=portFrameIn.read(false);
    Vector x(3);
    Vector o(4);

    if (frame_info!=NULL)
    {
        Bottle &pose_b=frame_info->findGroup("depth");
        Bottle *pose=pose_b.get(1).asList();
        x[0]=pose->get(0).asDouble();
        x[1]=pose->get(1).asDouble();
        x[2]=pose->get(2).asDouble();

        o[0]=pose->get(3).asDouble();
        o[1]=pose->get(4).asDouble();
        o[2]=pose->get(5).asDouble();
        o[3]=pose->get(6).asDouble();

        
        H.resize(4,4);
        H=axis2dcm(o);
        H.setSubcol(x,0,3);
        H(3,3)=1;
        H=SE3inv(H);
    }

    Vector point(3,0.0);

     for (size_t i=0; i<points.size(); i+=vis_step)
     {
         point=points[i].subVector(0,2);
         point2D=from3Dto2D(point, H);

         cv::Point target_point((int)point2D[0],(int)point2D[1]);

         if ((target_point.x<0) || (target_point.y<0) || (target_point.x>=320) || (target_point.y>=240))
         {
             yWarning("[SuperqVisualization]:  Not acceptable pixels!");
         }
         else
            imgOut.pixel(target_point.x, target_point.y)=color;
     }

    portImgOut.write();

    return true;
}

/*******************************************************************************/
Vector SuperqVisualization::from3Dto2D(const Vector &point3D, Matrix &H)
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
    yInfo()<<"[SuperqVisualization]: Thread initing ... ";
    
    portImgOut.open("/superquadric-model/img:o");
    portFrameIn.open("/superquadric-model/frame:i");

    R.resize(4,4);
    point2D.resize(2,0.0);
    point.resize(3,0.0);
    point1.resize(3,0.0);
    superq.resize(11,0.0);
    H.resize(4,4);
    H.eye();

    return true;
}

/***********************************************************************/
void SuperqVisualization::run()
{
    double t0=Time::now();
    if (what_to_plot=="superq" && imgIn!=NULL)
        showSuperq(superq_filtered);
    else if (what_to_plot=="points" && points.size()>0)
        showPoints();    
    t_vis=Time::now()-t0;
}

/**********************************************************************/
void SuperqVisualization:: threadRelease()
{
    yInfo()<<"[SuperVisualization]: Thread releasing ... ";

    if (!portImgOut.isClosed())
        portImgOut.close();
    if (!portFrameIn.isClosed())
        portFrameIn.close();
}

/***********************************************************************/
void SuperqVisualization::setPar(const Property &newOptions, bool first_time)
{
    LockGuard lg(mutex);
    int v_points=newOptions.find("visualized_points").asInt();
    if (newOptions.find("visualized_points").isNull() && (first_time==true))
    {
        vis_points=3;
    }
    else if (!newOptions.find("visualized_points").isNull())
    {
        if ((v_points>=1) && (v_points<=300))
        {
            vis_points=v_points;
        }
        else
        {
            vis_points=3;
        }
    }

    string plot=newOptions.find("what_to_plot").asString();
    if (newOptions.find("what_to_plot").isNull() && (first_time==true))
    {
        what_to_plot="superq";
    }
    else if (!newOptions.find("what_to_plot").isNull())
    {
        if ((plot=="superq") || (plot=="points"))
        {
            what_to_plot=plot;
        }
        else
        {
            what_to_plot="superq";
        }
    }

    int vpoint=newOptions.find("visualized_points_step").asInt();
    if (newOptions.find("visualized_points_step").isNull() && (first_time==true))
    {
        vis_step=10;
    }
    else if (!newOptions.find("visualized_points_step").isNull())
    {
        if ((vpoint>=10) && (vpoint<=100))
        {
            vis_step=vpoint;
        }
        else if (vpoint<10)
        {
            vis_step=10;
        }
        else if (vpoint>100)
        {
            vis_step=100;
        }
    }

    string col=newOptions.find("color").asString();
    if (newOptions.find("color").isNull() && (first_time==true))
    {
        Color[0]=255; Color[1]=0; Color[2]=0;
    }
    else if (!newOptions.find("color").isNull())
    {
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

