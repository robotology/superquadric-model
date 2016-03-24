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
    vector<cv::Point> contour;
    vector<cv::Point> floodPoints;
    string homeContextPath;
    int downsampling;
    double spatial_distance;
    int color_distance;
    Mutex mutex;
    bool go,flood3d,flood;

    BufferedPort<ImageOf<PixelMono> > portDispIn;
    BufferedPort<ImageOf<PixelRgb> >  portDispOut;
    BufferedPort<ImageOf<PixelRgb> >  portRgbIn;
    BufferedPort<Bottle>              portOutPoints;
    Port portContour;
    RpcClient portSFM;
    RpcServer portRpc;

    double tol, sum;
    int acceptable_iter,max_iter;
    bool go_on;

    string mu_strategy,nlp_scaling_method;
    Ipopt::ApplicationReturnStatus status;
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app;
    Ipopt::SmartPtr<SuperQuadric_NLP>  superQ_nlp;

    deque<Vector> points;
    Vector x;

    BufferedPort<ImageOf<PixelRgb> > portImgIn;
    BufferedPort<ImageOf<PixelRgb> > portImgOut;
    BufferedPort<Bottle> portSuperqIn;

    PolyDriver clientGazeCtrl;
    IGazeControl *igaze;

    Matrix R;
    Vector point,point1;
    Vector point2D;

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

        if(go_on==false)
        {
            yError("Not image available! ");    
            return false;
        }

        if(points.size()>0)
        {
            yInfo("Number of acquired points not null ");
            go_on=computeSuperq();
        }

        if(go_on==false)
        {
            yError("Not found a suitable superquadric! ");
            return false;
        }

        yInfo("Object superquadric found! ");
        go_on=showSuperq();

        if(go_on==false)
        {
            yError("Not image available! ");
            return false;
        }

        return true;
    }

    /***********************************************************************/
    bool configure(ResourceFinder &rf)
    {
        bool config_ok;

        config_ok=config3dpoints(rf);

        if(config_ok)
            config_ok=configSuperq(rf);

        if(config_ok)
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
        portSFM.interrupt();
        portRpc.interrupt();

        //interrupt for viewer
        portImgIn.interrupt();
        portImgOut.interrupt();

        return true;
    }

    /***********************************************************************/
    bool close()
    {
        //close for 3d-points
        portDispIn.close();
        portDispOut.close();
        portRgbIn.close();
        portContour.close();
        portSFM.close();
        portRpc.close();

        //close for viewer
        portImgIn.close();
        portImgOut.close();
        clientGazeCtrl.close();
        return true;
    }

    /***********************************************************************/
    bool config3dpoints(ResourceFinder &rf)
    {
        portDispIn.open("/superquadric-detection/disp:i");
        portDispOut.open("/superquadric-detection/disp:o");
        portRgbIn.open("/superquadric-detection/rgb:i");
        portContour.open("/superquadric-detection/contour:i");
        portSFM.open("/superquadric-detection/SFM:rpc");
        portRpc.open("/superquadric-detection/rpc");

        portContour.setReader(*this);
        attach(portRpc);

        homeContextPath=rf.getHomeContextPath().c_str();
        downsampling=std::max(1,rf.check("downsampling",Value(1)).asInt());
        spatial_distance=rf.check("spatial_distance",Value(0.004)).asDouble();
        color_distance=rf.check("color_distance",Value(6)).asInt();
        go=flood3d=flood=false;

        return true;
    }

    /***********************************************************************/
    bool configSuperq(ResourceFinder &rf)
    {
        tol=rf.check("tol",Value(1e-5)).asDouble();
        acceptable_iter=rf.check("acceptable_iter",Value(0)).asInt();
        max_iter=rf.check("max_iter",Value(numeric_limits<int>::max())).asInt();

        mu_strategy=rf.find("mu_strategy").asString().c_str();
        if(rf.find("mu_strategy").isNull())
            mu_strategy="adaptive";
        nlp_scaling_method=rf.find("nlp_scaling_method").asString().c_str();
        if(rf.find("nlp_scaling_method").isNull())
            nlp_scaling_method="none";

        app=new Ipopt::IpoptApplication;
        app->Options()->SetNumericValue("tol",tol);
        app->Options()->SetIntegerValue("acceptable_iter",acceptable_iter);
        app->Options()->SetStringValue("mu_strategy",mu_strategy);
        app->Options()->SetIntegerValue("max_iter",max_iter);
        app->Options()->SetStringValue("nlp_scaling_method",nlp_scaling_method);
        app->Options()->SetStringValue("hessian_approximation","limited-memory");
        app->Options()->SetIntegerValue("print_level",0);
        app->Initialize();

        superQ_nlp= new SuperQuadric_NLP;

        superQ_nlp->init();
        superQ_nlp->configure(rf);

        return true;

    }

    /***********************************************************************/
    bool configViewer(ResourceFinder &rf)
    {
        x.resize(11,0.0);
        portImgIn.open("/superquadric-detection/img:i");
        portImgOut.open("/superquadric-detection/img:o");

        Property optionG;
        optionG.put("device","gazecontrollerclient");
        optionG.put("remote","/iKinGazeCtrl");
        optionG.put("local","/superquadric-detection/gaze");

        clientGazeCtrl.open(optionG);
        igaze=NULL;

        if (!clientGazeCtrl.open(optionG))
            clientGazeCtrl.view(igaze);
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

        PixelRgb color(255,255,0);
        for (size_t i=0; i<floodPoints.size(); i++)
            imgDispOut.pixel(floodPoints[i].x,floodPoints[i].y)=color;
        //vector<Vector> points;

        if (contour.size()>0)
        {

            vector<vector<cv::Point> > contours;
            contours.push_back(contour);
            cv::drawContours(imgDispOutMat,contours,0,cv::Scalar(255,255,0));

            cv::Rect rect=cv::boundingRect(contour);
            cv::rectangle(imgDispOutMat,rect,cv::Scalar(255,50,0));


            if (go||flood3d||flood)
            {
                //points.clear();

                Bottle cmd,reply;

                if (go)
                {

                    cmd.addString("Rect");
                    cmd.addInt(rect.x);     cmd.addInt(rect.y);
                    cmd.addInt(rect.width); cmd.addInt(rect.height);
                    cmd.addInt(downsampling);
                    if (portSFM.write(cmd,reply))
                    {

                        int idx=0;
                        for (int x=rect.x; x<rect.x+rect.width; x+=downsampling)
                        {
                            for (int y=rect.y; y<rect.y+rect.height; y+=downsampling)
                            {
                                if (cv::pointPolygonTest(contour,cv::Point2f((float)x,(float)y),false)>0.0)
                                {
                                    Vector point(6,0.0);
                                    point[0]=reply.get(idx+0).asDouble();
                                    point[1]=reply.get(idx+1).asDouble();
                                    point[2]=reply.get(idx+2).asDouble();
                                    if (norm(point)>0.0)
                                    {
                                        PixelRgb px=imgIn->pixel(x,y);
                                        point[3]=px.r;
                                        point[4]=px.g;
                                        point[5]=px.b;

                                        points.push_back(point);
                                    }
                                }

                                idx+=3;
                            }
                        }
                    }
                }
                if (flood3d)
                {
                    cmd.addString("Flood3D");
                    cmd.addInt(contour.back().x);
                    cmd.addInt(contour.back().y);
                    cmd.addDouble(spatial_distance);
                    if (portSFM.write(cmd,reply))
                    {
                        for (int i=0; i<reply.size(); i+=5)
                        {
                            int x=reply.get(i+0).asInt();
                            int y=reply.get(i+1).asInt();
                            PixelRgb px=imgIn->pixel(x,y);

                            Vector point(6,0.0);
                            point[0]=reply.get(i+2).asDouble();
                            point[1]=reply.get(i+3).asDouble();
                            point[2]=reply.get(i+4).asDouble();
                            point[3]=px.r;
                            point[4]=px.g;
                            point[5]=px.b;

                            points.push_back(point);
                            floodPoints.push_back(cv::Point(x,y));
                        }
                    }
                }
                else if (flood)
                {
                    cv::Point seed(contour.back().x,contour.back().y);
                    PixelMono c=imgDispIn->pixel(seed.x,seed.y);
                    cv::Scalar delta(color_distance);
                    cv::floodFill(imgDispInMat,seed,cv::Scalar(255),NULL,delta,delta,4|cv::FLOODFILL_FIXED_RANGE);
                    cv::cvtColor(imgDispInMat,imgDispOutMat,CV_GRAY2RGB);
                }

                go=flood3d=false;

            }



        }
        cout<<"points size "<<points.size()<<endl;
        portDispOut.write();
        if(points.size()>0)
            return true;

    }

    /***********************************************************************/
    bool computeSuperq()
    {
        superQ_nlp->usePoints(points);
        double t,t0;
        t0=Time::now();

        status=app->OptimizeTNLP(GetRawPtr(superQ_nlp));
        t=Time::now()-t0;
        cout<<"t "<<t<<endl;

        points.clear();

        if(status==Ipopt::Solve_Succeeded)
        {
            x=superQ_nlp->get_result();
            cout<<"solution "<<x.toString()<<endl;
            yInfo("Solution of the optimization problem: %s", x.toString().c_str());
            return true;
        }
        else
            return false;
    }

    /***********************************************************************/
    bool showSuperq()
    {
        PixelRgb color(255,255,0);

        ImageOf<PixelRgb> *imgIn=portImgIn.read(false);
        if (imgIn==NULL)
            return false;
        
        ImageOf<PixelRgb> &imgOut=portImgOut.prepare();
        imgOut=imgIn;

        R=euler2dcm(x.subVector(8,10));
        
        if(points.size()>0 && go_on==true)
        {       
            for(double eta=-M_PI; eta<M_PI; eta+=0.4)
            {
                 for(double omega=-M_PI; omega<M_PI;omega+=0.4)
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
                    
                    igaze->get2DPixel(0, point, point2D);
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
            floodPoints.clear();
            go=flood3d=flood=false;
            reply.addVocab(ack);
        }
        else if ((cmd=="go") || (cmd=="flood3d"))
        {
            if (portSFM.getOutputCount()==0)
                reply.addVocab(nack);
            else
            {
                LockGuard lg(mutex);
                if (cmd=="go")
                {
                    if (contour.size()>2)
                    {
                        flood=false;
                        go=true;
                        reply.addVocab(ack);
                    }
                    else
                        reply.addVocab(nack);
                }
                else if (cmd=="flood3d")
                {
                    if (command.size()>=2)
                        spatial_distance=command.get(1).asDouble();

                    contour.clear();
                    floodPoints.clear();
                    flood=false;
                    flood3d=true;
                    reply.addVocab(ack);
                }
            }
        }
        else if (cmd=="flood")
        {
            if (command.size()>=2)
                color_distance=command.get(1).asInt();

            contour.clear();
            floodPoints.clear();
            flood=true;
            reply.addVocab(ack);
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


