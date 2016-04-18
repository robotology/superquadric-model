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
#include <algorithm>
#include <string>
#include <sstream>
#include <deque>
#include <map>
#include <set>
#include <fstream>
#include <iomanip>

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>
#include <IpReturnCodes.hpp>

#include <opencv2/opencv.hpp>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include "superquadric.cpp"

#include "../idl_dox/superquadricDetection_IDL.h"


using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

class SuperqModule : public RFModule,
                     public superquadricDetection_IDL
{
protected:
    bool go;
    int r,g,b;
    int downsampling;
    string objname;
    string method;
    vector<cv::Point> contour;
    deque<Vector> points;
    deque<cv::Point> blob_points;

    Mutex mutex;

    BufferedPort<ImageOf<PixelMono> > portDispIn;
    BufferedPort<ImageOf<PixelRgb> >  portDispOut;
    BufferedPort<ImageOf<PixelRgb> >  portRgbIn;
    RpcClient portBlobRpc;
    Port portContour;
    RpcClient portSFMrpc;
    RpcClient portOPCrpc;
    RpcServer portRpc;

    bool go_on;
    double tol, sum;    
    int acceptable_iter,max_iter;    
    string mu_strategy,nlp_scaling_method;
    Vector x;

    BufferedPort<ImageOf<PixelRgb> > portImgIn;
    BufferedPort<ImageOf<PixelRgb> > portImgOut;
    BufferedPort<Bottle> portSuperqIn;

    PolyDriver GazeCtrl;
    IGazeControl *igaze;

    int vis_points;
    string eye;
    Matrix R,H,K;
    Vector point,point1;
    Vector point2D;
    deque<int> Color;

    ResourceFinder *rf;

    /************************************************************************/
    bool attach(RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

    /************************************************************************/
    bool object_name(const string &object_name)
    {
        objname=object_name;
        method="name";
        return true;
    }

    /************************************************************************/
    bool seed_point()
    {
        method="point";
        return true;
    }

    /************************************************************************/
    string which_object()
    {
        return objname;
    }

    /************************************************************************/
    string name_or_not()
    {
        return method;
    }

    /************************************************************************/
    int get_downsampling()
    {
        return downsampling;
    }

    /************************************************************************/
    bool set_downsampling(const int32_t d)
    {
        if((d>0) && (d<100))
        {
            downsampling=d;
            return true;
        }
        else
            return false;
    }

    /************************************************************************/
    vector<int> get_rgb()
    {
        vector<int> rgb;
        rgb.clear();
        rgb.push_back(r);
        rgb.push_back(g);
        rgb.push_back(b);
        return rgb;
    }

    /**********************************************************************/
    bool set_rgb(const int32_t &red, const int32_t &green, const int32_t &blue)
    {
        if ((r<255) && (g<255) && (b<255))
        {
            r=red;
            g=green;
            b=blue;
            return true;
        }
        else
            return false;
    }

    /**********************************************************************/
    string get_eye()
    {
        return eye;
    }

    /**********************************************************************/
    bool set_eye(const string &e)
    {
        if ((e=="left") || (e=="right"))
        {
            eye=e;
            return true;
        }
        else
            return false;
    }

    /**********************************************************************/
    int get_visualized_points()
    {
        return vis_points;
    }

    /**********************************************************************/
    bool set_visualized_points(const int32_t v)
    {
        if ((v>10) && (v<1000))
        {
            vis_points=v;
            return true;
        }
        else
            return false;
    }

public:
    /***********************************************************************/
    double getPeriod()
    {
        return 0.1;
    }

    /***********************************************************************/
    bool updateModule()
    {
        go_on=acquirePointsFromBlob();

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

        if ((go_on==false) && (!isStopping()))
        {
            yError("Not found a suitable superquadric! ");
            //return false;
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

        config_ok=config3Dpoints(rf);

        if (config_ok)
            config_ok=configSuperq(rf);

        if (config_ok)
            config_ok=configViewer(rf);

        return config_ok;
    }

    /***********************************************************************/
    bool interruptModule()
    {
        portDispIn.interrupt();
        portDispOut.interrupt();
        portRgbIn.interrupt();
        portBlobRpc.interrupt();
        portContour.interrupt();
        portSFMrpc.interrupt();
        portRpc.interrupt();

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

        if (!portBlobRpc.asPort().isOpen())
            portBlobRpc.close();

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
    bool config3Dpoints(ResourceFinder &rf)
    {
        portDispIn.open("/superquadric-detection/disp:i");
        portDispOut.open("/superquadric-detection/disp:o");
        portRgbIn.open("/superquadric-detection/rgb:i");
        portBlobRpc.open("/superquadric-detection/blob:rpc");
        portContour.open("/superquadric-detection/contour:i");
        portSFMrpc.open("/superquadric-detection/SFM:rpc");
        portOPCrpc.open("/superquadric-detection/OPC:rpc");
        portRpc.open("/superquadric-detection/rpc");

        portContour.setReader(*this);

        attach(portRpc);

        downsampling=std::max(1,rf.check("downsampling",Value(3)).asInt());
        go=false;
        vis_points=16;

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

        eye=rf.find("eye").asString().c_str();
        if (rf.find("eye").isNull())
            eye="left";

        if (Bottle *B=rf.find("color").asList())
        {
            if (B->size()>=3)
            {
                for (int i=0; i<B->size();i++)
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

        Bottle info;
        igaze->getInfo(info);
        K.resize(3,4);
        K.zero();

        Bottle *intr_par;

        if (eye=="left")
            intr_par=info.find("camera_intrinsics_left").asList();
        else
            intr_par=info.find("camera_intrinsics_right").asList();

        K(0,0)=intr_par->get(0).asDouble();
        K(0,1)=intr_par->get(1).asDouble();
        K(0,2)=intr_par->get(2).asDouble();
        K(1,1)=intr_par->get(5).asDouble();
        K(1,2)=intr_par->get(6).asDouble();
        K(2,2)=1;

        R.resize(4,4);
        H.resize(4,4);
        point2D.resize(2,0.0);
        point.resize(3,0.0);
        point1.resize(3,0.0);

        return true;
    }

    /***********************************************************************/
    bool acquirePointsFromBlob()
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

        if (method=="seed_point")
        {
            if (contour.size()>0)
            {
                getBlob(imgDispOut,color);
            }
            else if (blob_points.size()>0)
            {
                get3Dpoints(imgDispOut, color);
            }
        }
        else if (method=="name")
        {
            //if ( objname.empty())
                pointFromName();

            if (contour.size()>0)
            {
                getBlob(imgDispOut,color);
            }
            else if (blob_points.size()>0)
            {
                get3Dpoints(imgDispOut, color);
            }
        }

        portDispOut.write();

        return true;
    }

    /***********************************************************************/
    void getBlob(ImageOf<PixelRgb> &imgDispOut,PixelRgb &color)
    {
        Bottle cmd,reply;
        blob_points.clear();
        points.clear();
        cmd.addString("get_component_around");
        cmd.addInt(contour[0].x); cmd.addInt(contour[0].y);

        if (portBlobRpc.write(cmd,reply))
        {
            get3Dpoints(imgDispOut,color);
        }
        else
        {
            contour.clear();
            points.clear();
            yError("lbpExtract reply is fail!");
        }
    }

    /***********************************************************************/
    void get3Dpoints(ImageOf<PixelRgb> &imgDispOut, PixelRgb &color)
    {
        Bottle cmd,reply;
        cmd.addString("Points");

        for (size_t i=0; i<blob_points.size(); i++)
        {
            cv::Point single_point=blob_points[i];
            cmd.addInt(single_point.x);
            cmd.addInt(single_point.y);
            imgDispOut.pixel(single_point.x, single_point.y)=color;
        }

        if (portSFMrpc.write(cmd,reply))
        {
            for (int idx=0;idx<reply.size();idx+=3)
            {
                Vector point(3,0.0);
                point[0]=reply.get(idx+0).asDouble();
                point[1]=reply.get(idx+1).asDouble();
                point[2]=reply.get(idx+2).asDouble();

                points.push_back(point);
            }

            if (points.size()<=0)
            {
                yError("Some problems in point acquisition!");
            }
        }
        else
        {
            yError("SFM reply is fail!");
            points.clear();
        }
    }

    /***********************************************************************/
    void pointFromName()
    {
        Bottle cmd,reply;
        blob_points.clear();
        points.clear();
        contour.clear();
        cmd.addVocab(Vocab::encode("ask"));
        Bottle &content=cmd.addList();
        Bottle &cond_1=content.addList();
        cond_1.addString("entity");
        cond_1.addString("==");
        cond_1.addString("object");
        content.addString("&&");
        Bottle &cond_2=content.addList();
        cond_2.addString("name");
        cond_2.addString("==");
        cond_2.addString(objname);

        cout<<"cmd 1 "<<cmd.toString()<<endl;
        portOPCrpc.write(cmd,reply);
        if(reply.size()>1)
        {
            if(reply.get(0).asVocab()==Vocab::encode("ack"))
            {
                if (Bottle *b=reply.get(1).asList())
                {
                    if (Bottle *b1=b->get(1).asList())
                    {
                        int id=b1->get(0).asInt();
                        Bottle cmd;
                        cmd.addVocab(Vocab::encode("get"));
                        Bottle &info=cmd.addList();
                        info.addString("id");
                        Bottle &info2=info.addList();
                        info2.addInt(id);
                        Bottle &info3=cmd.addList();
                        info3.addString("proSet");
                        Bottle &info4=info3.addList();
                        info4.addList().addString("position_2D");
                        cout<<"cmd 2 "<<cmd.toString()<<endl;
                    }
                    else
                        yInfo("no object id provided by OPC!");
                }
                else
                    yInfo("uncorrect reply from OPC!");

                portOPCrpc.write(cmd,reply);
                if(reply.size()>1)
                {
                    if(reply.get(0).asVocab()==Vocab::encode("ack"))
                    {
                        if (Bottle *b=reply.get(1).asList())
                        {
                            if (Bottle *b1=b->get(0).asList())
                            {
                                if (Bottle *b2=b1->find("position_2D").asList())
                                {
                                    cv::Point p;
                                    p.x=b2->get(0).asInt();
                                    p.y=b2->get(1).asInt();
                                    cout<<"px and py "<<p.x<<" "<<p.y<<endl;
                                    contour.push_back(p);
                                }
                                else
                                    yInfo("position_2D field not found in the OPC reply!");
                            }
                            else
                                yInfo("uncorrect reply structure received!");
                        }
                        else
                            yInfo("uncorrect reply structure received!");
                    }
                    else
                        yInfo("Failure in reply for object 2D point!");
                }
                else
                    yInfo("reply size for 2D point less than 1!");
            }
            else
                yInfo("Failure in reply for object id!");
        }
        else
            yInfo("reply size for object id less than 1!");
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
        Vector pos, orient;
        double co,so,ce,se;
        Stamp* stamp=NULL;

        ImageOf<PixelRgb> *imgIn=portImgIn.read();
        if (imgIn==NULL)
            return false;
        
        ImageOf<PixelRgb> &imgOut=portImgOut.prepare();
        imgOut=*imgIn;

        R=euler2dcm(x.subVector(8,10));
        
        if ((norm(x)!=0.0) && (go_on==true))
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

            for (double eta=-M_PI; eta<M_PI; eta+=M_PI/step)
            {
                 for (double omega=-M_PI; omega<M_PI;omega+=M_PI/step)
                 {
                     co=cos(omega); so=sin(omega);
                     ce=cos(eta); se=sin(eta);

                     point[0]=x[0] * sign(ce)*(pow(abs(ce),x[3])) * sign(co)*(pow(abs(co),x[4])) * R(0,0) +
                                x[1] * sign(ce)*(pow(abs(ce),x[3]))* sign(so)*(pow(abs(so),x[4])) * R(0,1)+
                                    x[2] * sign(se)*(pow(abs(se),x[3])) * R(0,2) + x[5];

                     point[1]=x[0] * sign(ce)*(pow(abs(ce),x[3])) * sign(co)*(pow(abs(co),x[4])) * R(1,0) +
                                x[1] * sign(ce)*(pow(abs(ce),x[3])) * sign(so)*(pow(abs(so),x[4])) * R(1,1)+
                                    x[2] * sign(se)*(pow(abs(se),x[3])) * R(1,2) + x[6];

                     point[2]=x[0] * sign(ce)*(pow(abs(ce),x[3])) * sign(co)*(pow(abs(co),x[4])) * R(2,0) +
                                x[1] * sign(ce)*(pow(abs(ce),x[3])) * sign(so)*(pow(abs(so),x[4])) * R(2,1)+
                                    x[2] * sign(se)*(pow(abs(se),x[3])) * R(2,2) + x[7];

                    point2D=from3Dto2D(point);
                    cv::Point target_point(point2D[0],point2D[1]);

                    if ((target_point.x<0) || (target_point.y<0))
                    {
                        yError("Negative pixels!");
                    }

                    imgOut.pixel(target_point.x, target_point.y)=color;

                 }
             }
        }
        
        portImgOut.write();

        return true;
    }

    /*******************************************************************************/
    Vector from3Dto2D(const Vector &point3D)
    {
        Vector point2D(3,0.0);
        Vector point_aux(4,1.0);
        point_aux.setSubvector(0,point3D);

        point2D=K*H*point_aux;
        return point2D.subVector(0,1)/point2D[2];
    }

    /*******************************************************************************/
  /**  bool read(ConnectionReader &connection)
    {
        Bottle data; data.read(connection);
        if (data.size()>=2)
        {
            LockGuard lg(mutex);
            cv::Point point(data.get(0).asInt(),data.get(1).asInt());
            contour.push_back(point);
       }

        return true;
    }*/
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


