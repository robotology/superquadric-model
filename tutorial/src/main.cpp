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
#include <string>
#include <sstream>
#include <deque>
#include <map>
#include <set>
#include <fstream>
#include <iomanip>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <opencv2/opencv.hpp>


#include "src/testingModule_IDL.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

class AcquireBlob : public RFModule,
                    testingModule_IDL
{
    string method;
    string objname;
    bool streaming;
    bool color;

    vector<cv::Point> object_center;
    deque<Vector> blob_points;
    deque<Vector> points;
     deque<Vector> points_rotated;

    RpcClient portBlobRpc;
    RpcClient portOPCrpc;
    RpcClient portRGBDRpc;
    RpcClient superqRpc;
    RpcServer portRpc;

    BufferedPort<Bottle > pointPort;
    BufferedPort<Property > portFrame;

    Mutex mutex;

public:

    /************************************************************************/
    bool attach(RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

//    /************************************************************************/
//    Bottle  get_Blob()
//    {
//        Bottle blob;
//        for (size_t i=0; i<blob_points.size(); i++)
//        {
//            Bottle &b=blob.addList();
//            b.addDouble(blob_points[i].x); b.addDouble(blob_points[i].y);
//        }

//        return blob;
//    }

    /************************************************************************/
    bool  set_streaming_mode(const string &entry)
    {
        if (entry=="on" || entry=="off")
        {
            streaming=(entry=="on");
            return true;
        }
        else
            return false;
    }

    /************************************************************************/
    void  sendPoints()
    {
        Bottle &point=pointPort.prepare();
        point.clear();
        Bottle &b1=point.addList();
        
        for (size_t i=0; i<points_rotated.size(); i++)
        {
            Bottle &b=b1.addList();
            b.addDouble(points_rotated[i][0]); b.addDouble(points_rotated[i][1]); b.addDouble(points_rotated[i][2]);
            b.addInt(points_rotated[i][3]); b.addInt(points_rotated[i][4]); b.addInt(points_rotated[i][5]);
        }

        pointPort.write();
        
    }

    /************************************************************************/
    bool getperiod()
    {
        return 0.0;
    }

    /************************************************************************/
    bool set_object_name(const string &object_name)
    {
        LockGuard lg(mutex);
        objname=object_name;
        method="name";

        return true;
    }

    /**********************************************************************/
    bool configure(ResourceFinder &rf)
    {
        method=rf.find("method").asString().c_str();
        if (rf.find("method").isNull())
            method="name";

        objname=rf.find("object_name").asString().c_str();
        if (rf.find("object_name").isNull())
            objname="object";

        streaming=(rf.check("streaming", Value("off")).asString()=="on");
        color=(rf.check("color", Value("off")).asString()=="on");

        portBlobRpc.open("/testing-module/blob:rpc");
        portOPCrpc.open("/testing-module/OPC:rpc");
        portRGBDRpc.open("/testing-module/RGBD:rpc");
        superqRpc.open("/testing-module/superq:rpc");
        portRpc.open("/testing-module/rpc");

        pointPort.open("/testing-module/point:o");

        portFrame.open("/testing-module/frame:i");

        attach(portRpc);
        return true;
    }

    /**********************************************************************/
    bool close()
    {

        if (portBlobRpc.asPort().isOpen())
            portBlobRpc.close();
        if (portOPCrpc.asPort().isOpen())
            portOPCrpc.close();
        if (portRGBDRpc.asPort().isOpen())
            portRGBDRpc.close();
        if (superqRpc.asPort().isOpen())
            superqRpc.close();
        if (!portFrame.isClosed())
            portFrame.close();

        return true;
    }

    /**********************************************************************/
    bool updateModule()
    {        
        if (method=="point")
        {
            blob_points.clear();

            if (object_center.size()>0)
            {
                getBlob();
            }
            else
            {
                Vector aux(2);
                aux.resize(2,0.0);
                blob_points.push_back(aux);
            }
        }
        else if (method=="name")
        {
            pointFromName();

            if ((object_center.size()>0) )
            {
                getBlob();
            }
            else
            {
                Vector aux(2);
                aux.resize(2,0.0);
                blob_points.push_back(aux);
            }
        }

        yDebug()<<"blob_points size "<<blob_points.size();

        if (blob_points.size()>1)
        {
            get3Dpoints();
        }

        if (points.size()>0)
            fromCameraToRoot();

        if (points_rotated.size()>0 && streaming==true)
            sendPoints();
        else if (points_rotated.size()>0 && (streaming==false))
        {
            Bottle cmd, reply;
            cmd.addString("get_superq");

            Bottle &in1=cmd.addList();
        
            for (size_t i=0; i<points_rotated.size(); i++)
            {
                Bottle &in=in1.addList();
                in.addDouble(points_rotated[i][0]);
                in.addDouble(points_rotated[i][1]);
                in.addDouble(points_rotated[i][2]);
            }
            
            // Add 1 instead of 0 if you want the filtered superquadric
            cmd.addInt(0);

            superqRpc.write(cmd, reply);

            cout<<"Received superquadric: "<<reply.toString()<<endl;
        }

        return true;
    }

    /***********************************************************************/
    void getBlob()
    {
        Bottle cmd,reply;
        blob_points.clear();

        Vector aux(2);
        
        cmd.addString("get_component_around");
        cmd.addInt(object_center[0].x); cmd.addInt(object_center[0].y);

        if (portBlobRpc.write(cmd,reply))
        {         
            if (Bottle *blob_list=reply.get(0).asList())
            {
                for (int i=0; i<blob_list->size();i++)
                {
                    if (Bottle *blob_pair=blob_list->get(i).asList())
                    {
                        aux[0]=blob_pair->get(0).asDouble();
                        aux[1]=blob_pair->get(1).asDouble();

                        blob_points.push_back(aux);
                    }
                    else
                    {
                        yError()<<"Some problems in blob pixels!";
                    }
                }
            }
            else
            {
                yError()<<"Some problem  in object blob!";
            }
        }
        else
        {
            yError("lbpExtract query is fail!");
        }
    }

    /***********************************************************************/
    void pointFromName()
    {
        Bottle cmd,reply;
        blob_points.clear();
        object_center.clear();
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
                        info4.addString("position_2d");
                    }
                    else
                    {
                        yError("no object id provided by OPC!");
                        object_center.clear();
                    }
                }
                else
                {
                    yError("uncorrect reply from OPC!");
                    object_center.clear();
                }

                Bottle reply;
                if (portOPCrpc.write(cmd,reply))
                {

                    if (reply.size()>1)
                    {
                        if (reply.get(0).asVocab()==Vocab::encode("ack"))
                        {
                            if (Bottle *b=reply.get(1).asList())
                            {
                                if (Bottle *b1=b->find("position_2d").asList())
                                {
                                    cv::Point p1,p2,p;
                                    p1.x=b1->get(0).asInt();
                                    p1.y=b1->get(1).asInt();
                                    p2.x=b1->get(2).asInt();
                                    p2.y=b1->get(3).asInt();
                                    p.x=p1.x+(p2.x-p1.x)/2;
                                    p.y=p1.y+(p2.y-p1.y)/2;
                                    object_center.push_back(p);
                                }
                                else
                                {
                                    yError("position_2d field not found in the OPC reply!");
                                    object_center.clear();
                                }
                            }
                            else
                            {
                                yError("uncorrect reply structure received!");
                                object_center.clear();
                            }
                        }
                        else
                        {
                            yError("Failure in reply for object 2D point!");
                            object_center.clear();
                        }
                    }
                    else
                    {
                        yError("reply size for 2D point less than 1!");
                        object_center.clear();
                    }
                }
                else
                    yError("no reply from second OPC query!");
            }
            else
            {
                yError("Failure in reply for object id!");
                object_center.clear();
            }
        }
        else
        {
            yError("reply size for object id less than 1!");
            object_center.clear();
        }
    }

    /***********************************************************************/
    void get3Dpoints()
    {
        Bottle cmd,reply;
        cmd.addString("get_3D_points");
        int count_blob=0;

        points.clear();

        Bottle &content=cmd.addList();

        for (size_t i=0; i<blob_points.size(); i++)
        {
            Bottle &vector=content.addList();
            vector.addDouble(blob_points[i][0]);
            vector.addDouble(blob_points[i][1]);
        }

        cmd.addInt(color);

        yDebug()<<"cmd "<<cmd.toString();

        if (portRGBDRpc.write(cmd,reply))
        {
            yDebug()<<"reply "<<reply.toString();
            count_blob=0;

            Bottle *content=reply.get(0).asList();

            Vector aux;

            Bottle *in1=content->get(0).asList();

            for (size_t i=0; i<in1->size(); i++)
            {
                Bottle *in=in1->get(i).asList();
                if (in->size()==3)
                {
                    aux.resize(6,0.0);
                    aux[0]=in->get(0).asDouble();
                    aux[1]=in->get(1).asDouble();
                    aux[2]=in->get(2).asDouble();
                    aux[3]=255.0;
                    aux[4]=0.0;
                    aux[5]=0.0;
                }
                else if (in->size()==6)
                {
                    aux.resize(6,0.0);
                    aux[0]=in->get(0).asDouble();
                    aux[1]=in->get(1).asDouble();
                    aux[2]=in->get(2).asDouble();
                    aux[3]=in->get(3).asDouble();
                    aux[4]=in->get(4).asDouble();
                    aux[5]=in->get(5).asDouble();
                }

                count_blob+=2;

                if ((norm(aux)>0))
                    points.push_back(aux);;
            }

            if (points.size()<=0)
            {
                yError(" Some problems in point acquisition!");
            }
            else
            {
                Vector colors(3,0.0);
                colors[0]=255;
            }
        }
        else
        {
            yError(" RGBD reply is fail!");
            points.clear();
        }
    }

    /***********************************************************************/
    void fromCameraToRoot()
    {
        Property *frame_info=portFrame.read(false);
        Vector x(3);
        Vector o(4);

        if (frame_info!=NULL)
        {
            Bottle &pose_b=frame_info->findGroup("depth");
            cout<<" Bottle pose "<<pose_b.toString();
            Bottle *pose=pose_b.get(1).asList();
            x[0]=pose->get(0).asDouble();
            x[1]=pose->get(1).asDouble();
            x[2]=pose->get(2).asDouble();

            cout<<"pose 0 "<<pose->get(0).asDouble()<<endl;

            o[0]=pose->get(3).asDouble();
            o[1]=pose->get(4).asDouble();
            o[2]=pose->get(5).asDouble();
            o[3]=pose->get(6).asDouble();

            Matrix H;
            H.resize(4,4);
            H=axis2dcm(o);
            H.setSubcol(x,0,3);
            H(3,3)=1;

            cout<<"H "<<H.toString()<<endl;
            //H=SE3inv(H);

            cout<<"Out from depth "<<frame_info->toString()<<endl;
            cout<<"x "<<x.toString()<<endl;
            cout<<"o "<<o.toString()<<endl;

            if (norm(x)!=0.0 && norm(o)!=0.0)
            {
                points_rotated.clear();
                for (size_t i=0; i<points.size(); i++)
                {
                    if (norm(points[i].subVector(0,2))>0.0)
                    {
                        Vector aux;
                        aux.resize(4,1.0);
                        aux.setSubvector(0,points[i].subVector(0,2));

                        Vector aux2(6);
                        aux2.setSubvector(0,(H*aux).subVector(0,2));
                        aux2[3]=points[i][3]; aux2[4]=points[i][4]; aux2[5]=points[i][5];
                        points_rotated.push_back(aux2);
                    }
                }
            }
        }
        else
          yError()<<"Frame info null";
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

    AcquireBlob mod;
    ResourceFinder rf;
    rf.setDefaultContext("testing-module");
    rf.configure(argc,argv);
    return mod.runModule(rf);
}
