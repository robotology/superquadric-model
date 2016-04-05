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

#include <cmath>
#include <limits>
#include <algorithm>
#include <string>
#include <sstream>
#include <deque>

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>
#include <IpReturnCodes.hpp>

#include <opencv2/opencv.hpp>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include "superquadric.cpp"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

class SuperqModule : public RFModule, public PortReader
{
protected:
    bool go;
    int r,g,b;
    int color_distance;
    int downsampling;
    double spatial_distance;
    string homeContextPath;
    vector<cv::Point> contour;
    deque<Vector> points;
    deque<Vector> points_toshow;

    Mutex mutex;

    BufferedPort<ImageOf<PixelMono> > portDispIn;
    BufferedPort<ImageOf<PixelRgb> >  portDispOut;
    BufferedPort<ImageOf<PixelRgb> >  portRgbIn;
    Port portContour;
    RpcClient portSFMrpc;
    RpcServer portRpc;

    bool go_on;
    double tol, sum;    
    int acceptable_iter,max_iter;    
    string mu_strategy,nlp_scaling_method;
    Vector x;

    BufferedPort<ImageOf<PixelRgb> > portImgIn;
    BufferedPort<ImageOf<PixelRgb> > portImgOut;
    BufferedPort<Bottle> portSuperqIn;
    int eye;

    PolyDriver GazeCtrl;
    IGazeControl *igaze;

    Matrix R;
    Vector point,point1;
    Vector point2D;
    deque<int> Color;

    ResourceFinder *rf;

public:

    /***********************************************************************/
    double getPeriod()
    {
        return 0.1;
    }

    /***********************************************************************/
    bool updateModule()
    {
        go_on=acquirePoints();

        if ((go_on==false) && (!isStopping()))
        {
            yError("No image available! ");
            return false;
        }

        if (points.size()>0)
        {
            yInfo()<<"number of points acquired:"<< points.size();
            go_on=computeSuperq();
        }

        if (go_on==false)
        {
            yError("Not found a suitable superquadric! ");
            return false;
        }

        else
            go_on=showSuperq();


        if ((go_on==false) && (!isStopping()))
        {
            yError("No image available! ");
            return false;
        }

        return true;
    }

    /***********************************************************************/
    bool configure(ResourceFinder &rf)
    {
        bool config_ok;

        config_ok=config3dpoints(rf);

        if (config_ok)
            config_ok=configSuperq(rf);

        if (config_ok)
            config_ok=configViewer(rf);

        return config_ok;
    }

    /***********************************************************************/
    bool interruptModule()
    {
        //interrupt for 3d-points
        portDispIn.interrupt();
        portDispOut.interrupt();
        portRgbIn.interrupt();
        portContour.interrupt();
        portSFMrpc.interrupt();
        portRpc.interrupt();

        //interrupt for viewer
        portImgIn.interrupt();
        portImgOut.interrupt();

        return true;
    }

    /***********************************************************************/
    bool close()
    {
        if (!portDispIn.isClosed())
            portDispIn.close();

        if (!portDispOut.isClosed())
            portDispOut.close();

        if (!portRgbIn.isClosed())
            portRgbIn.close();

        if (portContour.isOpen())
            portContour.close();

        if (portSFMrpc.asPort().isOpen())
            portSFMrpc.close();

        if (portRpc.asPort().isOpen())
            portRpc.close();

        if (!portImgIn.isClosed())
            portImgIn.close();

        if (!portImgOut.isClosed())
            portImgOut.close();

        GazeCtrl.close();

        return true;
    }

    /***********************************************************************/
    bool config3dpoints(ResourceFinder &rf)
    {
        portDispIn.open("/superquadric-detection/disp:i");
        portDispOut.open("/superquadric-detection/disp:o");
        portRgbIn.open("/superquadric-detection/rgb:i");
        portContour.open("/superquadric-detection/contour:i");
        portSFMrpc.open("/superquadric-detection/SFM:rpc");
        portRpc.open("/superquadric-detection/rpc");

        portContour.setReader(*this);
        attach(portRpc);

        homeContextPath=rf.getHomeContextPath().c_str();
        downsampling=std::max(1,rf.check("downsampling",Value(3)).asInt());
        spatial_distance=rf.check("spatial_distance",Value(0.004)).asDouble();
        color_distance=rf.check("color_distance",Value(6)).asInt();
        go=false;

        return true;
    }

    /***********************************************************************/
    bool configSuperq(ResourceFinder &rf)
    {
        this->rf=&rf;

        tol=rf.check("tol",Value(1e-2)).asDouble();
        acceptable_iter=rf.check("acceptable_iter",Value(0)).asInt();
        max_iter=rf.check("max_iter",Value(numeric_limits<int>::max())).asInt();

        mu_strategy=rf.find("mu_strategy").asString().c_str();
        if (rf.find("mu_strategy").isNull())
            mu_strategy="adaptive";
        nlp_scaling_method=rf.find("nlp_scaling_method").asString().c_str();
        if (rf.find("nlp_scaling_method").isNull())
            nlp_scaling_method="none";

        return true;
    }

    /***********************************************************************/
    bool configViewer(ResourceFinder &rf)
    {
        x.resize(11,0.0);
        portImgIn.open("/superquadric-detection/img:i");
        portImgOut.open("/superquadric-detection/img:o");

        eye=rf.check("eye",Value(0)).asInt();

        if (Bottle *B=rf.find("color").asList())
        {
            if (B->size()>=3)
            {
                for(size_t i=0; i<B->size();i++)
                    Color.push_back(B->get(i).asInt());
            }

            r=Color[0]; g=Color[1]; b=Color[2];
        }
        else
        {
            r=255; g=255; b=0;
        }

        Property optionG;
        optionG.put("device","gazecontrollerclient");
        optionG.put("remote","/iKinGazeCtrl");
        optionG.put("local","/superquadric-detection/gaze");

        GazeCtrl.open(optionG);
        igaze=NULL;

        if (GazeCtrl.isValid())
            GazeCtrl.view(igaze);
        else
            return false;

        R.resize(4,4);
        point2D.resize(2,0.0);
        point.resize(3,0.0);
        point1.resize(3,0.0);

        return true;
    }

    /***********************************************************************/
    bool acquirePoints()
    {
        PixelRgb color(r,g,b);

        ImageOf<PixelMono> *imgDispIn=portDispIn.read();
        if (imgDispIn==NULL)
            return false;

        ImageOf<PixelRgb> *imgIn=portImgIn.read();
        if (imgIn==NULL)
            return false;

        ImageOf<PixelRgb> &imgDispOut=portDispOut.prepare();
        imgDispOut.resize(imgDispIn->width(),imgDispIn->height());

        cv::Mat imgDispInMat=cv::cvarrToMat((IplImage*)imgDispIn->getIplImage());
        cv::Mat imgDispOutMat=cv::cvarrToMat((IplImage*)imgDispOut.getIplImage());
        cv::cvtColor(imgDispInMat,imgDispOutMat,CV_GRAY2RGB);

        if (contour.size()>0)
        {
            vector<vector<cv::Point> > contours;
            contours.push_back(contour);
            cv::drawContours(imgDispOutMat,contours,0,cv::Scalar(255,255,0));

            cv::Rect rect=cv::boundingRect(contour);
            cv::rectangle(imgDispOutMat,rect,cv::Scalar(255,50,0));

            if (go)
            {
                Bottle cmd,reply;

                points_toshow.clear();

                cmd.addString("Rect");
                cmd.addInt(rect.x);     cmd.addInt(rect.y);
                cmd.addInt(rect.width); cmd.addInt(rect.height);
                cmd.addInt(downsampling);
                if (portSFMrpc.write(cmd,reply))
                {
                    int idx=0;
                    for (int x=rect.x; x<rect.x+rect.width; x+=downsampling)
                    {
                        for (int y=rect.y; y<rect.y+rect.height; y+=downsampling)
                        {
                            if (cv::pointPolygonTest(contour,cv::Point2f((float)x,(float)y),false)>0.0)
                            {
                                Vector point(3,0.0);
                                point[0]=reply.get(idx+0).asDouble();
                                point[1]=reply.get(idx+1).asDouble();
                                point[2]=reply.get(idx+2).asDouble();

                                points.push_back(point);
                                points_toshow.push_back(point);
                            }

                            idx+=3;
                        }                       
                    }
                }

                go=false;
            }
        }

        for(size_t i=0;i<points_toshow.size();i+=3)
        {
            Vector point=points_toshow[i];
            igaze->get2DPixel(eye, point, point2D);
            cv::Point target_point(point2D[0],point2D[1]);
            imgDispOut.pixel(target_point.x, target_point.y)=color;
        }

        portDispOut.write();

        return true;
    }

    /***********************************************************************/
    bool computeSuperq()
    {
        Ipopt::SmartPtr<Ipopt::IpoptApplication> app=new Ipopt::IpoptApplication;
        app->Options()->SetNumericValue("tol",tol);
        app->Options()->SetIntegerValue("acceptable_iter",acceptable_iter);
        app->Options()->SetStringValue("mu_strategy",mu_strategy);
        app->Options()->SetIntegerValue("max_iter",max_iter);
        app->Options()->SetStringValue("nlp_scaling_method",nlp_scaling_method);
        app->Options()->SetStringValue("hessian_approximation","limited-memory");
        app->Options()->SetIntegerValue("print_level",0);
        app->Initialize();

        Ipopt::SmartPtr<SuperQuadric_NLP> superQ_nlp= new SuperQuadric_NLP;

        superQ_nlp->init();
        superQ_nlp->configure(this->rf);
        superQ_nlp->usePoints(points);

        double t,t0;
        t0=Time::now();

        points.clear();

        Ipopt::ApplicationReturnStatus status=app->OptimizeTNLP(GetRawPtr(superQ_nlp));
        t=Time::now()-t0;

        if (status==Ipopt::Solve_Succeeded)
        {
            x=superQ_nlp->get_result();
            yInfo("Solution of the optimization problem: %s", x.toString(3,3).c_str());
            yInfo("Computed in: %f [s]", t);
            return true;
        }
        else
            return false;
    }

    /***********************************************************************/
    bool showSuperq()
    {
        PixelRgb color(r,g,b);

        ImageOf<PixelRgb> *imgIn=portImgIn.read();
        if (imgIn==NULL)
            return false;
        
        ImageOf<PixelRgb> &imgOut=portImgOut.prepare();
        imgOut=*imgIn;

        R=euler2dcm(x.subVector(8,10));
        
        if ((norm(x)!=0.0) && (go_on==true))
        {
            for (double eta=-M_PI; eta<M_PI; eta+=M_PI/8)
            {
                 for (double omega=-M_PI; omega<M_PI;omega+=M_PI/8)
                 {
                     //point1[0]=point[0]; point1[1]=point[1]; point1[2]=point[2];

                     point[0]=x[0] * sign(cos(eta))*(pow(abs(cos(eta)),x[3])) * sign(cos(omega))*(pow(abs(cos(omega)),x[4])) * R(0,0) +
                                x[1] * sign(cos(eta))*(pow(abs(cos(eta)),x[3]))* sign(sin(omega))*(pow(abs(sin(omega)),x[4])) * R(0,1)+
                                    x[2] * sign(sin(eta))*(pow(abs(sin(eta)),x[3])) * R(0,2) + x[5];

                     point[1]=x[0] * sign(cos(eta))*(pow(abs(cos(eta)),x[3])) * sign(cos(omega))*(pow(abs(cos(omega)),x[4])) * R(1,0) +
                                x[1] * sign(cos(eta))*(pow(abs(cos(eta)),x[3])) * sign(sin(omega))*(pow(abs(sin(omega)),x[4])) * R(1,1)+
                                    x[2] * sign(sin(eta))*(pow(abs(sin(eta)),x[3])) * R(1,2) + x[6];

                     point[2]=x[0] * sign(cos(eta))*(pow(abs(cos(eta)),x[3])) * sign(cos(omega))*(pow(abs(cos(omega)),x[4])) * R(2,0) +
                                x[1] * sign(cos(eta))*(pow(abs(cos(eta)),x[3])) * sign(sin(omega))*(pow(abs(sin(omega)),x[4])) * R(2,1)+
                                    x[2] * sign(sin(eta))*(pow(abs(sin(eta)),x[3])) * R(2,2) + x[7];
                    
                    igaze->get2DPixel(eye, point, point2D);
                    cv::Point target_point(point2D[0],point2D[1]);
                    //igaze->get2DPixel(0, point1, point2D);
                    //cv::Point target_point1(point2D[0],point2D[1]);
                    imgOut.pixel(target_point.x, target_point.y)=color;
                    //cv::line(imgOutMat,target_point,target_point1,cv::Scalar(255,0,0));
                 }

            }
        }
        
        portImgOut.write();

        return true;
    }

    /*******************************************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
        string cmd=command.get(0).asString().c_str();
        int ack=Vocab::encode("ack");
        int nack=Vocab::encode("nack");

        if (cmd=="clear")
        {
            LockGuard lg(mutex);
            contour.clear();
            go=false;
            reply.addVocab(ack);
        }
        else if (cmd=="go")
        {
            if (portSFMrpc.getOutputCount()==0)
                reply.addVocab(nack);
            else
            {
                LockGuard lg(mutex);

                if (contour.size()>2)
                {
                    go=true;
                    reply.addVocab(ack);
                }
                else
                    reply.addVocab(nack);
            }
        }
        else
            RFModule::respond(command,reply);

        return true;
   }

   /*******************************************************************************/
   bool read(ConnectionReader &connection)
   {
       Bottle data; data.read(connection);
       if (data.size()>=2)
       {
           LockGuard lg(mutex);
           cv::Point point(data.get(0).asInt(),data.get(1).asInt());
           contour.push_back(point);
       }

       return true;
   }
};

/**********************************************************************/
int main(int argc,char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("unable to find YARP server!");
        return 1;
    }

    SuperqModule mod;
    ResourceFinder rf;
    rf.setDefaultContext("superquadric");
    rf.configure(argc,argv);
    return mod.runModule(rf);
}


