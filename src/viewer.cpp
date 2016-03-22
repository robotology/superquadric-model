#include <vector>
#include <algorithm>
#include <string>
#include <fstream>
#include <iomanip>

#include <opencv2/opencv.hpp>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/PolyDriver.h>
# include <yarp/dev/CartesianControl.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

/*******************************************************************************/
class ViewerModule
{
protected:

    BufferedPort<ImageOf<PixelRgb> > portImgIn;
    BufferedPort<ImageOf<PixelRgb> > portImgOut;
    Vector x;

    PolyDriver clientGazeCtrl;
    PolyDriver client;
    IGazeControl *igaze;
    ICartesianControl *icart;

    Matrix R;
    Vector euler;
    Vector point,point1;
    Vector point2D;

public:

/*******************************************************************************/
    bool configure(ResourceFinder &rf)
    {
       portImgIn.open("/viewer/img:i");
       portImgOut.open("/viewer/img:o");

       x.resize(11,0.0);

       Property optionG;
       optionG.put("device","gazecontrollerclient");
       optionG.put("remote","/iKinGazeCtrl");
       optionG.put("local","/client/gaze");

       clientGazeCtrl.open(optionG);
       igaze=NULL;

       if (!clientGazeCtrl.open(optionG))
       {
           clientGazeCtrl.view(igaze);
       }

        R.resize(4,4);
        point2D.resize(2,0.0);
        euler.resize(3,0.0);
        point.resize(3,0.0);
        point1.resize(3,0.0);

       /**Property option("(device cartesiancontrollerclient)");
       option.put("remote","/icubSim/cartesianController/left_arm");
       option.put("local","/cartesian_client/left_arm");
       if (!client.open(option))
            return false;

        client.view(icart);

        Vector newDof, curDof;
        icart->getDOF(curDof);
        newDof=curDof;

        newDof[0]=1;
        newDof[1]=0;
        newDof[2]=1;*/

       return true;

    }

/*******************************************************************************/

    bool interruptModule()
    {
        portImgIn.interrupt();
        portImgOut.interrupt();
        portSuperqIn.interrupt();
        return true;
    }

/*******************************************************************************/

    bool close()
    {
        portImgIn.close();
        portImgOut.close();
        portSuperqIn.close();

        clientGazeCtrl.close();
        return true;
    }

/*******************************************************************************/
    bool updateModule()
    {
        receiveSuperq();

        PixelRgb color(255,255,0);


        ImageOf<PixelRgb> *imgIn=portImgIn.read();
        if (imgIn==NULL)
            return false;

        ImageOf<PixelRgb> &imgOut=portImgOut.prepare();
        imgOut.resize(imgIn->width(),imgIn->height());

        cv::Mat imgInMat=cv::Mat((IplImage*)imgIn->getIplImage());
        cv::Mat imgOutMat=cv::Mat((IplImage*)imgOut.getIplImage());
        imgInMat.copyTo(imgOutMat);


        euler[0]=x[8];
        euler[1]=x[9];
        euler[2]=x[10];
        R=euler2dcm(euler);


        for(double eta=-3.14; eta<3.14; eta=eta+0.4)
        {
             for(double omega=-3.14; omega<3.14;omega=omega+0.4)
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


        portImgOut.write();

        return true;
    }

/*******************************************************************************/
    void receiveSuperq()
    {

        Bottle *xIn=portSuperqIn.read(true);
        if (xIn!=NULL)
        {
            Bottle *pointList=xIn->get(0).asList();

                x[0]=pointList->get(0).asDouble();
                x[1]=pointList->get(1).asDouble();
                x[2]=pointList->get(2).asDouble();
                x[3]=pointList->get(3).asDouble();
                x[4]=pointList->get(4).asDouble();
                x[5]=pointList->get(5).asDouble();
                x[6]=pointList->get(6).asDouble();
                x[7]=pointList->get(7).asDouble();
                x[8]=pointList->get(8).asDouble();
                x[9]=pointList->get(9).asDouble();
                x[10]=pointList->get(10).asDouble();

        }


     }


};

/*******************************************************************************/
int main(int argc,char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("unable to find YARP server!");
        return 1;
    }

    ViewerModule mod;
    ResourceFinder rf;
    rf.setDefaultContext("viewer");
    rf.configure(argc,argv);
    return mod.runModule(rf);
