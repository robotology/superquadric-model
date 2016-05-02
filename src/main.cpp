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

#include "src/superquadricDetection_IDL.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

/*******************************************************************************/
class SpatialDensityFilter
{
public:

    /*******************************************************************************/
    static vector<int>  filter(const cv::Mat &data,const double radius, const int maxResults, deque<Vector> &points)
    {
        cv::flann::KDTreeIndexParams indexParams;
        cv::flann::Index kdtree(data,indexParams);

        cv::Mat query(1,data.cols,CV_32F);
        cv::Mat indices,dists;

        vector<int> res(data.rows);
        for (size_t i=0; i<res.size(); i++)
        {            
            for (int c=0; c<query.cols; c++)
                query.at<float>(0,c)=data.at<float>(i,c);

            res[i]=kdtree.radiusSearch(query,indices,dists,radius,maxResults,cv::flann::SearchParams(128));

            Vector point(3,0.0);
            if (res[i]>=maxResults)
            {
                point[0]=data.at<float>(i,0);
                point[1]=data.at<float>(i,1);
                point[2]=data.at<float>(i,2);
                points.push_back(point);
            }
        }

        return res;
    }
};

/*******************************************************************************/
class SuperqModule : public RFModule,
                     public superquadricDetection_IDL
{
protected:

    int r,g,b;
    int count;
    int downsampling;
    string objname;
    string method;
    string homeContextPath;
    ConstString pointCloudFileName;
    string outputFileName;
    vector<cv::Point> contour;
    deque<Vector> points;
    deque<cv::Point> blob_points;

    BufferedPort<ImageOf<PixelMono> > portDispIn;
    BufferedPort<ImageOf<PixelRgb> >  portDispOut;
    BufferedPort<ImageOf<PixelRgb> >  portRgbIn;
    RpcClient portBlobRpc;
    Port portContour;
    RpcClient portSFMrpc;
    RpcClient portOPCrpc;
    RpcServer portRpc;

    double radius;
    int nnThreshold;
    int numVertices;
    bool filter_on;
    bool file_on;

    bool mode_online;
    bool go_on;
    double tol, sum;
    double max_cpu_time;
    int acceptable_iter,max_iter;
    unsigned int optimizer_points;
    string mu_strategy,nlp_scaling_method;    
    Vector x;
    double t_superq;

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
    double t,t0;

    /************************************************************************/
    bool attach(RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

    /************************************************************************/
    bool set_object_name(const string &object_name)
    {
        objname=object_name;
        method="name";
        outputFileName=homeContextPath+"/"+objname+".off";
        yDebug()<<"file output "<<outputFileName;
        return true;
    }

    /************************************************************************/
    bool set_seed_point(const int &x, const int &y)
    {
        if ((x>0) && (y>0))
        {
            cv::Point p;
            p.x=x;
            p.y=y;yDebug()<<"file output "<<outputFileName;
            contour.push_back(p);
        }
        method="point";
        return true;
    }

    /************************************************************************/
    string get_object_name()
    {
        return objname;
    }

    /************************************************************************/
    string get_method()
    {
        return method;
    }

    /************************************************************************/
    int get_downsampling_acquisition()
    {
        return downsampling;
    }

    /************************************************************************/
    bool set_downsampling_acquisition(const int d)
    {
        if ((d>0) && (d<100))
        {
            downsampling=d;
            return true;
        }
        else
            return false;
    }

    /************************************************************************/
    vector<int> get_color()
    {
        vector<int> rgb;
        rgb.clear();
        rgb.push_back(r);
        rgb.push_back(g);
        rgb.push_back(b);
        return rgb;
    }

    /**********************************************************************/
    bool set_color(const int red, const int green, const int blue)
    {
        if ((red<=255) && (green<=255) && (blue<=255) && (red>=0) && (green>=0) && (blue>=0))
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
    int get_optimizer_points()
    {
        return optimizer_points;
    }

    /**********************************************************************/
    bool set_optimizer_points(const int max)
    {
        if ((max>0) && (max<500))
        {
            optimizer_points=max;
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
    bool set_visualized_points(const int v)
    {
        if ((v>=10) && (v<=1000))
        {
            vis_points=v;
            return true;
        }
        else
            return false;
    }

    /**********************************************************************/
    vector<double> get_superq(const string &name)
    {
        vector<double> parameters;
        parameters.clear();

        if (name==objname)
        {
            for (size_t i=0; i<x.size(); i++)
                parameters.push_back(x[i]);
        }

        return parameters;
    }

public:
    /***********************************************************************/
    double getPeriod()
    {
        return 0.0;
    }

    /***********************************************************************/
    bool updateModule()
    {        
        t0=Time::now();

        if (mode_online)
        {
            go_on=acquirePointsFromBlob();

            if ((go_on==false) && (!isStopping()))
            {
                yError("No image available! ");
                return false;
            }

            if ((filter_on==true) && (points.size()>0) && (go_on==true))
            {
                go_on=filter();
            }

            if (points.size()>0)
            {
                yInfo()<<"number of points acquired:"<< points.size();
                go_on=computeSuperq();
            }

            if ((go_on==false) && (!isStopping()))
            {
                yError("Not found a suitable superquadric! ");
            }
            else if (go_on==true)
            {
                go_on=showSuperq();

                if ((go_on==false) && (!isStopping()))
                {
                    yError("No image available! ");
                    return false;
                }
            }
        }
        else
        {
            if (!file_on)
            {                
                go_on=acquirePointsFromBlob();

                if ((go_on==false) && (!isStopping()))
                {
                    yError("No image available! ");
                    return false;
                }
            }
            else
                go_on=readPointCloud();

            if ((filter_on==true) && (points.size()>0))
                go_on=filter();

            if ((go_on==false) && (!isStopping()))
            {
                yError("Something wrong in point filtering! ");
                return false;
            }

            if (points.size()>0)
            {
                yInfo()<<"number of points received for superquadric computation:"<< points.size();
                go_on=computeSuperq();
            }

            if ((go_on==false) && (!isStopping()))
            {
                yError("Not found a suitable superquadric! ");
            }
            else if (go_on==true)
            {
                saveSuperq();
                go_on=showSuperq();
            }

            t=Time::now()-t0;

            if( norm(x)!=0.0)
                return false;
            else
                return true;
        }

        count++;

        t=Time::now()-t0;

        count++;
        return true;
    }

    /***********************************************************************/
    bool configure(ResourceFinder &rf)
    {
        bool config_ok;

        config_ok=configOnOff(rf);

        if (filter_on==true)
            config_ok=configFilter(rf);

        if (config_ok)
            config_ok=config3Dpoints(rf);

        if (config_ok)
            config_ok=configSuperq(rf);

        if ((config_ok==true) && (file_on==false))
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
        portOPCrpc.interrupt();
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

        if (portOPCrpc.asPort().isOpen())
            portOPCrpc.close();

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
    bool configOnOff(ResourceFinder &rf)
    {
        count=0;

        homeContextPath=rf.getHomeContextPath().c_str();
        pointCloudFileName=rf.findFile("pointCloudFile");
        mode_online=(rf.check("online", Value("yes"))=="yes");

        yDebug()<<"file points "<<pointCloudFileName;

        if (rf.find("pointCloudFile").isNull())
        {
            file_on=false;
            if (rf.find("outputFile").isNull())
                outputFileName=homeContextPath+"/output.off";

            yDebug()<<"file output "<<outputFileName;
        }
        else
        {
            mode_online=false;
            file_on=true;
            outputFileName=rf.findFile("outputFile");

            if (rf.find("outputFile").isNull())
                outputFileName=homeContextPath+"/output.off";

            yDebug()<<"file output "<<outputFileName;
        }

        filter_on=(rf.check("filter_on", Value("no"))=="yes");

        return true;
    }

    /***********************************************************************/
    bool configFilter(ResourceFinder &rf)
    {
        radius=rf.check("radius", Value(0.0002)).asDouble();
        nnThreshold=rf.check("nn-threshold", Value(60)).asInt();
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

        return true;
    }

    /***********************************************************************/
    bool configSuperq(ResourceFinder &rf)
    {
        this->rf=&rf;
        x.resize(11,0.0);

        if (mode_online)
        {
            optimizer_points=rf.check("optimizer_points", Value(80)).asInt();
            max_cpu_time=rf.check("max_cpu_time", Value(0.3)).asDouble();
        }
        else
        {            
            optimizer_points=rf.check("optimizer_points", Value(100)).asInt();
            max_cpu_time=rf.check("max_cpu_time", Value(5.0)).asDouble();
        }

        tol=rf.check("tol",Value(1e-5)).asDouble();
        acceptable_iter=rf.check("acceptable_iter",Value(0)).asInt();
        max_iter=rf.check("max_iter",Value(numeric_limits<int>::max())).asInt();

        mu_strategy=rf.find("mu_strategy").asString().c_str();
        if (rf.find("mu_strategy").isNull())
            mu_strategy="monotone";
        nlp_scaling_method=rf.find("nlp_scaling_method").asString().c_str();
        if (rf.find("nlp_scaling_method").isNull())
            nlp_scaling_method="gradient-based";

        return true;
    }

    /***********************************************************************/
    bool configViewer(ResourceFinder &rf)
    {        
        portImgIn.open("/superquadric-detection/img:i");
        portImgOut.open("/superquadric-detection/img:o");

        eye=rf.check("eye", Value("left")).asString();

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

        vis_points=16;

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

        if (method=="point")
        {
            if (contour.size()>0)
            {
                getBlob(imgDispOut,color);

                if (blob_points.size()>0)
                {
                    get3Dpoints(imgDispOut, color);
                }
            }
        }
        else if (method=="name")
        {
            pointFromName();

            if (contour.size()>0)
            {
                getBlob(imgDispOut,color);

                if (blob_points.size()>0)
                {
                    get3Dpoints(imgDispOut, color);
                }
            }
        }

        portDispOut.write();

        return true;
    }

    /***********************************************************************/
    void getBlob(ImageOf<PixelRgb> &imgDispOut,const PixelRgb &color)
    {
        Bottle cmd,reply;
        blob_points.clear();
        points.clear();
        cmd.addString("get_component_around");
        cmd.addInt(contour[0].x); cmd.addInt(contour[0].y);


        if (portBlobRpc.write(cmd,reply))
        {
            if (Bottle *blob_list=reply.get(0).asList())
            {
                for (int i=0; i<blob_list->size();i++)
                {
                    if (Bottle *blob_pair=blob_list->get(i).asList())
                    {
                        cv:: Point pix=cv::Point(blob_pair->get(0).asInt(),blob_pair->get(1).asInt());
                        blob_points.push_back(cv::Point(blob_pair->get(0).asInt(),blob_pair->get(1).asInt()));
                        imgDispOut.pixel(pix.x, pix.y)=color;
                    }
                    else
                    {
                        yError()<<"Some problems in blob pixels!";
                    }
                }

                //if (blob_points.size()>0)
                //{
                   // get3Dpoints(imgDispOut,color);
                //}
                //else
                //{
                    //yError()<<"no blob received!";
                //}
            }
            else
            {
                yError()<<"Some problem  in object blob!";
            }
        }
        else
        {
            //contour.clear();
            points.clear();
            yError("lbpExtract reply is fail!");
        }
    }

    /***********************************************************************/
    void get3Dpoints(ImageOf<PixelRgb> &imgDispOut, const PixelRgb &color)
    {
        Bottle cmd,reply;
        cmd.addString("Points");
        count=0;

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
                count++;

                if ((norm(point)>0) && (count==downsampling-1))
                {
                    points.push_back(point);
                    count=0;
                }
            }

            if (points.size()<=0)
            {
                yError("Some problems in point acquisition!");
            }
            else
            {
                Vector colors(3,0.0);
                colors[0]=255;
                savePoints("/SFM-"+objname, colors);
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

        portOPCrpc.write(cmd,reply);
        if(reply.size()>1)
        {
            if(reply.get(0).asVocab()==Vocab::encode("ack"))
            {
                if (Bottle *b=reply.get(1).asList())
                {
                    if (Bottle *b1=b->get(1).asList())
                    {
                        cmd.clear();
                        int id=b1->get(0).asInt();
                        cmd.addVocab(Vocab::encode("get"));
                        Bottle &info=cmd.addList();
                        Bottle &info2=info.addList();
                        info2.addString("id");
                        info2.addInt(id);
                        Bottle &info3=info.addList();
                        info3.addString("propSet");
                        Bottle &info4=info3.addList();
                        info4.addString("position_2d_left");
                    }
                    else
                    {
                        yError("no object id provided by OPC!");
                        contour.clear();
                    }
                }
                else
                {
                    yError("uncorrect reply from OPC!");
                    contour.clear();
                }

                Bottle reply;
                portOPCrpc.write(cmd,reply);
                if (reply.size()>1)
                {
                    if (reply.get(0).asVocab()==Vocab::encode("ack"))
                    {
                        if (Bottle *b=reply.get(1).asList())
                        {                           
                            if (Bottle *b1=b->find("position_2d_left").asList())
                            {
                                cv::Point p1,p2,p;
                                p1.x=b1->get(0).asInt();
                                p1.y=b1->get(1).asInt();
                                p2.x=b1->get(2).asInt();
                                p2.y=b1->get(3).asInt();
                                p.x=p1.x+(p2.x-p1.x)/2;
                                p.y=p1.y+(p2.y-p1.y)/2;
                                //contour.clear();
                                contour.push_back(p);
                            }
                            else
                            {
                                yError("position_2d_left field not found in the OPC reply!");
                                contour.clear();
                            }
                        }
                        else
                        {
                            yError("uncorrect reply structure received!");
                            contour.clear();
                        }
                    }
                    else
                    {
                        yError("Failure in reply for object 2D point!");
                        contour.clear();
                    }
                }
                else
                {
                    yError("reply size for 2D point less than 1!");
                    contour.clear();
                }
            }
            else
            {
                yError("Failure in reply for object id!");
                contour.clear();
            }
        }
        else
        {
            yError("reply size for object id less than 1!");
            contour.clear();
        }
    }


    /***********************************************************************/
    void savePoints(const string &namefile, const Vector &colors)
    {
        ofstream fout;
        stringstream ss2;
        ss2 << count;
        string str_i = ss2.str();        
        fout.open((homeContextPath+namefile+str_i+".off").c_str());

        if (fout.is_open())
        {
            fout<<"COFF"<<endl;
            fout<<points.size()<<" 0 0"<<endl;
            fout<<endl;
            for (size_t i=0; i<points.size(); i++)
            {
                fout<<points[i].subVector(0,2).toString(3,4).c_str()<<
                      " "<<colors[0]<<" "<<colors[1]<<" "<<colors[2]<<endl;
            }

            fout<<endl;
        }
        else
            yError()<<"Some problems in opening output file!";

        fout.close();
    }

    /***********************************************************************/
    void saveSuperq()
    {
        ofstream fout;        
        fout.open(outputFileName.c_str());
        if (fout.is_open())
        {
            fout<<"*****Result*****"<<endl;
            fout<<"Computed superquadric: "<<endl;
            fout<<" "<<x.toString(3,3);
            fout<<endl;
            fout<<"Execution time: "<<endl;
            fout<<" "<<t_superq <<endl;
            fout<<"Update module time"<<endl;
            fout<<" "<<t_superq+Time::now()-t0<<endl<<endl;
            fout<<"*****Optimizer parameters*****"<<endl;
            fout<<"Optimizer points: "<<optimizer_points<<endl;
            fout<<"Tolerance :"<<tol<<endl;
            fout<<"Nlp scaling method: "<<nlp_scaling_method<<endl;
            fout<<"Mu strategy: "<<mu_strategy<<endl;
        }
        fout.close();
    }

    /***********************************************************************/
    bool readPointCloud()
    {
        ifstream pointsFile(pointCloudFileName.c_str());
        points.clear();
        int nPoints;
        int state=0;
        char line[255];

        if (!pointsFile.is_open())
        {
            yError()<<"problem opening point cloud file!";
            return false;
        }

        while (!pointsFile.eof())
        {
            pointsFile.getline(line,sizeof(line),'\n');
            Bottle b(line);
            Value firstItem=b.get(0);
            bool isNumber=firstItem.isInt() || firstItem.isDouble();

            if (state==0)
            {
                string tmp=firstItem.asString().c_str();
                std::transform(tmp.begin(),tmp.end(),tmp.begin(),::toupper);
            if (tmp=="OFF")
                state++;
            }
            else if (state==1)
            {
                if (isNumber)
                {
                    nPoints=firstItem.asInt();
                    state++;
                }
            }
            else if (state==2)
            {
                if (isNumber && (b.size()>=3))
                {
                    Vector point(3,0.0);
                    point[0]=b.get(0).asDouble();
                    point[1]=b.get(1).asDouble();
                    point[2]=b.get(2).asDouble();
                    points.push_back(point);

                    if (--nPoints<=0)
                        return true;
                }
            }
        }

        return false;
    }

    /***********************************************************************/
    bool filter()
    {
        numVertices=points.size();

        cv:: Mat data(numVertices,3,CV_32F);
        for (int i=0; i<numVertices; i++)
        {
            Vector point=points[i];
            data.at<float>(i,0)=point[0];
            data.at<float>(i,1)=point[1];
            data.at<float>(i,2)=point[2];
        }

        points.clear();

        yInfo()<<"Processing points...";
        double t0=yarp::os::Time::now();
        SpatialDensityFilter::filter(data,radius,nnThreshold+1, points);
        double t1=yarp::os::Time::now();
        yInfo()<<"Processed in "<<1e3*(t1-t0)<<" [ms]";

        Vector colors(3,0.0);
        colors[1]=255;
        savePoints("/filtered-"+objname, colors);

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
        app->Options()->SetNumericValue("max_cpu_time",max_cpu_time);
        app->Options()->SetStringValue("nlp_scaling_method",nlp_scaling_method);
        app->Options()->SetStringValue("hessian_approximation","limited-memory");
        app->Options()->SetIntegerValue("print_level",0);
        app->Initialize();

        Ipopt::SmartPtr<SuperQuadric_NLP> superQ_nlp= new SuperQuadric_NLP;

        superQ_nlp->init();
        superQ_nlp->configure(this->rf);
        superQ_nlp->setPoints(points, mode_online,optimizer_points);

        double t0_superq=Time::now();

        yDebug()<<"start IPOPT ";

        Ipopt::ApplicationReturnStatus status=app->OptimizeTNLP(GetRawPtr(superQ_nlp));

        yDebug()<<"finish IPOPT ";

        t_superq=Time::now()-t0_superq;

        points.clear();

        if (status==Ipopt::Solve_Succeeded)
        {
            x=superQ_nlp->get_result();
            yInfo("Solution of the optimization problem: %s", x.toString(3,3).c_str());
            yInfo("Execution time : %f", t_superq);
            return true;
        }
        else if(status==Ipopt::Maximum_CpuTime_Exceeded)
        {
            x=superQ_nlp->get_result();
            yWarning("Solution after maximum time exceeded: %s", x.toString(3,3).c_str());
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
        Stamp *stamp=NULL;

        ImageOf<PixelRgb> *imgIn=portImgIn.read();
        if (imgIn==NULL)
            return false;

        ImageOf<PixelRgb> &imgOut=portImgOut.prepare();
        imgOut=*imgIn;

        R=euler2dcm(x.subVector(8,10));

        if ((norm(x)>0.0))
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

                     if ((target_point.x<0) || (target_point.y<0) || (target_point.x>=320) || (target_point.y>=240))
                     {
                         yError("Not acceptable pixels!");
                     }
                     else
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
    rf.setDefaultContext("superquadric-detection");
    rf.configure(argc,argv);
    return mod.runModule(rf);
}


