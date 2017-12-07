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
SuperqVisualization::SuperqVisualization(int _rate,const string &_eye, const string &_what_to_plot, Vector &_x, Vector &_x_filtered,
                                         deque<int> &_Color,IGazeControl *_igaze, const Matrix _K, deque<Vector> &_points,
                                         const int &_vis_points, const int &_vis_step, ImageOf<PixelRgb> *&_imgIn, superqTree *&_superq_tree, bool &_single_superq):
                                         RateThread(_rate), eye(_eye), what_to_plot(_what_to_plot), Color(_Color), igaze(_igaze), K(_K), single_superq(_single_superq),
                                         vis_points(_vis_points), vis_step(_vis_step), superq(_x), superq_filtered(_x_filtered), points(_points), imgIn(_imgIn), superq_tree(_superq_tree)
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
bool SuperqVisualization::addSuperq(Vector &x_toshow, ImageOf<PixelRgb> &imgOut)
{
    LockGuard lg(mutex);

    PixelRgb color(Color[0],Color[1],Color[2]);
    Vector pos, orient;
    double co,so,ce,se;
    Stamp *stamp=NULL;

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
                     yWarning("[SuperqVisualization]: Not acceptable pixels!");
                 }
                 else
                    imgOut.pixel(target_point.x, target_point.y)=color;
             }
         }
    }

    return true;

}

/***********************************************************************/
bool SuperqVisualization::showMultipleSuperqs(superqTree *&superq_tree)
{
    ImageOf<PixelRgb> &imgOut=portImgOut.prepare();
    imgOut=*imgIn;
    if (superq_tree!=NULL)
        showTree(superq_tree->root, imgOut);

    portImgOut.write();
    return true;
}

/***********************************************************************/
bool SuperqVisualization::showTree(node *leaf, ImageOf<PixelRgb> &imgOut)
{    
    if(leaf!=NULL)
    {
        addSuperq(leaf->superq, imgOut);
        if (leaf->right!=NULL)
            showTree(leaf->right, imgOut);

        if (leaf->left!=NULL)
            showTree(leaf->left, imgOut);
    }
    else
    {

        yDebug()<<"Finished";
    }
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
             yWarning("[SuperqVisualization]:  Not acceptable pixels!");
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
    yInfo()<<"[SuperqVisualization]: Thread initing ... ";
    
    portImgOut.open("/superquadric-model/img:o");

    R.resize(4,4);
    H.resize(4,4);
    point2D.resize(2,0.0);
    point.resize(3,0.0);
    point1.resize(3,0.0);

    return true;
}

/***********************************************************************/
void SuperqVisualization::run()
{
    //LockGuard lg(mutex);
    double t0=Time::now();
    if (what_to_plot=="superq" && imgIn!=NULL && single_superq==true)
        showSuperq(superq_filtered);
    if (what_to_plot=="superq" && imgIn!=NULL && single_superq==false)
        showMultipleSuperqs(superq_tree);
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

    string cam=newOptions.find("camera").asString();
    if (newOptions.find("camera").isNull() && (first_time==true))
    {
        eye="left";
    }
    else if (!newOptions.find("camera").isNull())
    {
        if ((cam=="left") || (cam=="right"))
        {
             eye=cam;
        }
        else
        {
            eye="left";
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

