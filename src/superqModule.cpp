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
#include <sstream>
#include <set>
#include <fstream>

#include <yarp/math/Math.h>

#include "superqModule.h"

#include "src/superquadricModel_IDL.h"

using namespace yarp::math;


/************************************************************************/
bool SuperqModule::attach(RpcServer &source)
{
    return this->yarp().attachAsServer(source);
}

/************************************************************************/
bool SuperqModule::set_object_name(const string &object_name)
{
    LockGuard lg(mutex);
    objname=object_name;
    method="name";
    outputFileName=homeContextPath+"/"+objname+".txt";
    yDebug()<<"file output "<<outputFileName;
    x.resize(11,0.0);
    x_filtered.resize(11,0.0);

    superqCom->setPar("object_name", objname);
    superqCom->setPar("method", method);

    return true;
}

/************************************************************************/
bool SuperqModule::set_seed_point(const int x, const int y)
{
    if ((x>0) && (y>0))
    {
        LockGuard lg(mutex);
        cv::Point p;
        p.x=x;
        p.y=y;
        superqCom->setContour(p);
        method="point";
    }

    return true;
}

/************************************************************************/
string SuperqModule::get_object_name()
{
    return objname;
}

/************************************************************************/
string SuperqModule::get_method()
{
    return method;
}

/************************************************************************/
vector<int> SuperqModule::get_color()
{
    vector<int> rgb;
    rgb.clear();
    rgb.push_back(r);
    rgb.push_back(g);
    rgb.push_back(b);
    return rgb;
}

/**********************************************************************/
bool SuperqModule::set_color(const int red, const int green, const int blue)
{
    if ((red<=255) && (green<=255) && (blue<=255) && (red>=0) && (green>=0) && (blue>=0))
    {
        LockGuard lg(mutex);
        r=red;
        g=green;
        b=blue;
        superqVis->setColor(r,g,b);
        return true;
    }
    else
    {
        return false;
    }
}

/**********************************************************************/
string SuperqModule::get_eye()
{
    return eye;
}

/**********************************************************************/
bool SuperqModule::set_eye(const string &e)
{
    if ((e=="left") || (e=="right"))
    {
        LockGuard lg(mutex);
        eye=e;
        superqVis->setPar("eye", eye);
        return true;
    }
    else
    {
        return false;
    }
}

/**********************************************************************/
string SuperqModule::get_visualization_on()
{
    if (visualization_on)
        return "on";
    else
        return "off";
}

/**********************************************************************/
bool SuperqModule::set_visualization_on(const string &e)
{
    if ((e=="on") || (e=="off"))
    {
        LockGuard lg(mutex);
        if (visualization_on==false && e=="on")
        {
            superqVis= new SuperqVisualization(rate_vis,eye, what_to_plot, Color, igaze, K,vis_points, vis_step);

            bool thread_started=superqVis->start();

            cout<<endl;
            if (thread_started)
                cout<<"[SuperqVisualization] thread started!"<<endl;
            else
                yError()<<"[SuperqVisualization] problems in starting the thread!";
            cout<<endl;

            visualization_on=true;

        }
        else if (visualization_on==true && e=="off")
        {
            superqVis->stop();
            delete superqVis;
            visualization_on=false;
        }
        return true;
    }
    else
    {
        return false;
    }
}

/**********************************************************************/
int SuperqModule::get_visualized_points()
{
    return vis_points;
}

/**********************************************************************/
bool SuperqModule::set_visualized_points(const int v)
{
    if ((v>=10) && (v<=1000))
    {
        LockGuard lg(mutex);
        vis_points=v;
        superqVis->setPar("vis_points", vis_points);
        return true;
    }
    else
    {
        return false;
    }
}

/**********************************************************************/
vector<double> SuperqModule::get_superq(const string &name, bool filtered_or_not)
{
    vector<double> parameters;
    parameters.clear();

    if (!one_shot_mode)
    {
        Vector sol=superqCom->getSolution(name, filtered_or_not);

        for (size_t i=0; i<sol.size(); i++)
        {
                parameters.push_back(sol[i]);
        }
    }
    else
    {
        superqCom->threadInit();
        superqCom->step();
        Vector sol=superqCom->getSolution(name, filtered_or_not);

        for (size_t i=0; i<sol.size(); i++)
        {
                parameters.push_back(sol[i]);
        }
    }

    return parameters;
}

/**********************************************************************/
bool SuperqModule::set_filtering(const string &entry)
{
    if ((entry=="on") || (entry=="off"))
    {
        LockGuard lg(mutex);
        filter_points= (entry=="on");
        if (filter_points==true)
        {
            radius=0.005;
            nnThreshold=100;
            Property options;
            options.put("filter_radius_advanced", radius);
            options.put("filter_nnThreshold_advanced", nnThreshold);
            superqCom->setPointsFilterPar(options);
            superqCom->setPar("filter_points", "on");
        }
        else
            superqCom->setPar("filter_points", "off");
        
        cout<<endl;
        cout<<"[SuperqModule]: filter_points "<<filter_points<<endl;
        cout<<"[SuperqModule]: radius        "<<radius<<endl;
        cout<<"[SuperqModule]: nn-thrshold   "<<nnThreshold<<endl;
        cout<<endl;

        return true;
    
    }
    else
    {
        
        return false;
    }
}

/**********************************************************************/
string SuperqModule::get_filtering()
{
    if (filter_points==1)
    {
        return "on";
    }
    else
    {
        return "off";
    }
}

/**********************************************************************/
bool SuperqModule::set_filtering_superq(const string &entry)
{
    if ((entry=="on") || (entry=="off"))
    {
        LockGuard lg(mutex);
        filter_superq= (entry=="on");
        if (filter_superq==true)
        {
            median_order=5; 
            fixed_window=false;
            Property options;
            options.put("median_order_advanced", median_order);
            superqCom->setSuperqFilterPar(options);
            superqCom->setPar("filter_superq", "on");
        }
        else
            superqCom->setPar("filter_superq", "off");
        
        cout<<endl;
        cout<<"[SuperqModule]: filter_superq         "<<filter_superq<<endl;
        cout<<"[SuperqModule]: fixed_window          "<<fixed_window<<endl;
        cout<<"[SuperqModule]: median_order          "<<median_order<<endl;        
        cout<<endl;

        return true;
        
    }
    else
    {
        return false;
    }
}

/**********************************************************************/
string SuperqModule::get_filtering_superq()
{
    if (filter_superq==true)
    {
        return "on";
    }
    else
    {
        return "off";
    }
}

/**********************************************************************/
string SuperqModule::get_save_points()
{
    if (save_points==true)
    {
        return "on";
    }
    else
    {
        return "off";
    }
}

/**********************************************************************/
bool SuperqModule::set_save_points(const string &entry)
{
    if ((entry=="on") || (entry=="off"))
    {
        save_points=(entry=="on");
        superqCom->setPar("save_points", "on");
    } 
    
    
}

/**********************************************************************/
Property SuperqModule::get_advanced_options(const string &field)
{
    Property advOptions;
    if (field=="points_filter")
        advOptions=superqCom->getPointsFilterPar();
    else if (field=="superq_filter")
        advOptions=superqCom->getSuperqFilterPar();
    else if (field=="optimization")
        advOptions=superqCom->getIpoptPar();


    return advOptions;
}

/**********************************************************************/
bool SuperqModule::set_advanced_options(const Property &newOptions, const string &field)
{
    if (field=="points_filter")
        superqCom->setPointsFilterPar(newOptions);
    else if (field=="superq_filter")
        superqCom->setSuperqFilterPar(newOptions);
    else if (field=="optimization")
        superqCom->setIpoptPar(newOptions);
    else
        return false;

    return true;
}

/**********************************************************************/
bool SuperqModule::set_plot(const string &plot)
{
    yDebug() <<"PLOT "<<plot;
    if ((plot=="superq") || (plot=="points") || (plot=="filtered-points"))
    {
        LockGuard lg(mutex);
        what_to_plot=plot;
        return true;
    }
    else
        return false;
}

/**********************************************************************/
string SuperqModule::get_plot()
{
    return what_to_plot;
}

/**********************************************************************/
bool SuperqModule::set_visualized_points_step(const int step)
{
    if (step>0)
    {
        vis_step=step;
        return true;
    }
    else
    {
        yError()<<"Negativa step for visualized point downsampling!";
        return false;
    }
}

/**********************************************************************/
int SuperqModule::get_visualized_points_step()
{
    return vis_step;
}

/**********************************************************************/
bool SuperqModule::set_fixed_window(const string &entry)
{
    fixed_window=(entry=="yes");

    if (!fixed_window)
    {
        median_order=5;
        min_median_order=1;
        max_median_order=30;
        threshold_median=0.1;
        min_norm_vel=0.01;
        Property options;
        options.put("median_order_advanced", median_order);
        options.put("min_median_order_advanced", min_median_order);
        options.put("max_median_order_advanced", max_median_order);
        options.put("threshold_median_advanced", threshold_median);
        options.put("min_norm_vel_advanced", min_norm_vel);
        superqCom->setSuperqFilterPar(options);
    }

    cout<<endl;
    cout<<"[SuperqModule]: min_median_order      "<<min_median_order<<endl;
    cout<<"[SuperqModule]: max_median_order      "<<max_median_order<<endl;
    cout<<"[SuperqModule]: threshold_median      "<<threshold_median<<endl;
    cout<<"[SuperqModule]: min_norm_vel          "<<min_norm_vel<<endl;
    cout<<endl;

    return true;
}

/**********************************************************************/
bool SuperqModule::get_fixed_window()
{
    return fixed_window;
}

/**********************************************************************/
bool SuperqModule::set_one_shot_mode(const string &entry)
{
    if (entry=="on" || entry=="off")
    {
        one_shot_mode=(entry=="on");
        return true;
    }
    else
        return false;
}

/**********************************************************************/
bool SuperqModule::get_one_shot_mode()
{
    return one_shot_mode;
}

/***********************************************************************/
double SuperqModule::getPeriod()
{
    return 0.0;
}

/***********************************************************************/
bool SuperqModule::updateModule()
{
    t0=Time::now();
    LockGuard lg(mutex);

    if (!one_shot_mode)
    {

        ImageOf<PixelRgb> *imgIn=portImgIn.read();

        superqCom->sendImg(imgIn);

        x=superqCom->getSolution(objname,false);

        x_filtered=superqCom->getSolution(objname,true);

        if (visualization_on)
        {
            superqVis->sendImg(imgIn);
            if (what_to_plot=="superq")
            {
                if (filter_superq)
                    superqVis->sendSuperq(x_filtered);
                else
                    superqVis->sendSuperq(x);
            }
            else if (what_to_plot=="points")
            {
                superqCom->getPoints(points);
                superqVis->sendPoints(points);
            }
        }
    }

    t=Time::now()-t0;
    return true;
}

/***********************************************************************/
bool SuperqModule::configure(ResourceFinder &rf)
{
    bool config_ok;

    cout<<endl<<"[SuperqModule] configuring ... "<<endl<<endl;

    config_ok=configOnOff(rf);

    if (filter_points==true)
        config_ok=configFilter(rf);
    if (filter_superq==true)
        config_ok=configFilterSuperq(rf);

    if (config_ok)
        config_ok=configServices(rf);

    if (config_ok)
        config_ok=configSuperq(rf);

    if ((config_ok==true))
        config_ok=configViewer(rf);

    superqCom= new SuperqComputation(rate, filter_points, filter_superq,fixed_window, objname,
                                     method,filter_points_par, filter_superq_par, ipopt_par, homeContextPath, save_points);

    if (!one_shot_mode)
    {
        bool thread_started=superqCom->start();

        cout<<endl;
        if (thread_started)
            cout<<"[SuperqComputation] thread started!"<<endl;
        else
            yError()<<"[SuperqComputation] problems in starting the thread!";
        cout<<endl;
    }

    if (visualization_on)
    {
        superqVis= new SuperqVisualization(rate_vis,eye, what_to_plot, Color, igaze, K,vis_points, vis_step);

        bool thread_started=superqVis->start();

        cout<<endl;
        if (thread_started)
            cout<<"[SuperqVisualization] thread started!"<<endl;
        else
            yError()<<"[SuperqVisualization] problems in starting the thread!";
        cout<<endl;
    }

    return config_ok;
}

/***********************************************************************/
bool SuperqModule::interruptModule()
{
    cout<<endl<<"[SuperqModule] interruping ... "<<endl<<endl;
    portImgIn.interrupt();
    return true;
}

/***********************************************************************/
bool SuperqModule::close()
{
    cout<<endl<<"[SuperqModule] closing ... "<<endl<<endl;
    saveSuperq();

    if (!one_shot_mode)
        superqCom->stop();

    delete superqCom;


    if (visualization_on)
    {
        superqVis->stop();
        delete superqVis;
    }

    if (portBlobRpc.asPort().isOpen())
        portBlobRpc.close();

    if (portSFMrpc.asPort().isOpen())
        portSFMrpc.close();

    if (portOPCrpc.asPort().isOpen())
        portOPCrpc.close();

    if (portRpc.asPort().isOpen())
        portRpc.close();

    if (!portImgIn.isClosed())
        portImgIn.close();

    GazeCtrl.close();  

    return true;
}

/***********************************************************************/
bool SuperqModule::configOnOff(ResourceFinder &rf)
{
    homeContextPath=rf.getHomeContextPath().c_str();
    pointCloudFileName=rf.findFile("pointCloudFile");
    visualization_on=(rf.check("visualization_on", Value("yes")).asString()=="yes");
    one_shot_mode=(rf.check("one_shot_mode", Value("no")).asString()=="yes");
    save_points=(rf.check("save_points", Value("no")).asString()=="yes");

    rate=rf.check("rate", Value(100)).asInt();
    rate_vis=rf.check("rate_vis", Value(100)).asInt();

    if (rf.find("pointCloudFile").isNull())
    {
        //mode_online=true;
    }
    else
    {
        //mode_online=false;
        outputFileName=rf.findFile("outputFile");

        if (rf.find("outputFile").isNull())
            outputFileName=homeContextPath+"/output.txt";

        yDebug()<<"file output "<<outputFileName;
    }

    filter_points=(rf.check("filter_points", Value("off")).asString()=="on");
    filter_superq=(rf.check("filter_superq", Value("off")).asString()=="on");

    cout<<endl;
    cout<<"[SuperqModule]: rate          "<<rate<<endl;
    cout<<"[SuperqModule]: filter_points "<<filter_points<<endl;
    cout<<"[SuperqModule]: filter_superq "<<filter_superq<<endl;
    cout<<endl;

    return true;
}

/***********************************************************************/
bool SuperqModule::configFilter(ResourceFinder &rf)
{
    radius=rf.check("radius", Value(0.005)).asDouble();
    nnThreshold=rf.check("nn-threshold", Value(100)).asInt();

    filter_points_par.put("filter_radius_advanced",radius);
    filter_points_par.put("filter_nnThreshold_advanced",nnThreshold);

    cout<<endl;
    cout<<"[SuperqModule]: radius         "<<radius<<endl;
    cout<<"[SuperqModule]: nn-threshold   "<<nnThreshold<<endl;
    cout<<endl;

    return true;
}

/***********************************************************************/
bool SuperqModule::configFilterSuperq(ResourceFinder &rf)
{
    fixed_window=(rf.check("fixed_window", Value("no")).asString()=="yes");
    median_order=rf.check("median_order", Value(1)).asInt();
    min_median_order=rf.check("min_median_order", Value(1)).asInt();
    max_median_order=rf.check("max_median_order", Value(30)).asInt();
    threshold_median=rf.check("threshold_median", Value(0.1)).asDouble();
    min_norm_vel=rf.check("min_norm_vel", Value(0.01)).asDouble();
    x.resize(11,0.0);

    filter_superq_par.put("median_order_advanced",median_order);
    filter_superq_par.put("min_median_order_advanced",min_median_order);
    filter_superq_par.put("max_median_order_advanced",max_median_order);
    filter_superq_par.put("threshold_median_advanced",threshold_median);
    filter_superq_par.put("min_norm_vel_advanced",min_norm_vel);

    cout<<endl;
    cout<<"[SuperqModule]: fixed_window          "<<fixed_window<<endl;
    cout<<"[SuperqModule]: median_order          "<<median_order<<endl;
    cout<<"[SuperqModule]: min_median_order      "<<min_median_order<<endl;
    cout<<"[SuperqModule]: max_median_order      "<<max_median_order<<endl;
    cout<<"[SuperqModule]: threshold_median      "<<threshold_median<<endl;
    cout<<"[SuperqModule]: min_norm_vel          "<<min_norm_vel<<endl;
    cout<<endl;

    return true;
}

/***********************************************************************/
bool SuperqModule::configServices(ResourceFinder &rf)
{
    portRpc.open("/superquadric-model/rpc");

    attach(portRpc);

    return true;
}

/***********************************************************************/
bool SuperqModule::configSuperq(ResourceFinder &rf)
{
    this->rf=&rf;
    x_filtered.resize(11,0.0);

    optimizer_points=rf.check("optimizer_points", Value(300)).asInt();
    max_cpu_time=rf.check("max_cpu_time", Value(5.0)).asDouble();

    tol=rf.check("tol",Value(1e-5)).asDouble();
    acceptable_iter=rf.check("acceptable_iter",Value(0)).asInt();
    max_iter=rf.check("max_iter",Value(numeric_limits<int>::max())).asInt();

    mu_strategy=rf.find("mu_strategy").asString().c_str();
    if (rf.find("mu_strategy").isNull())
        mu_strategy="adaptive";

    nlp_scaling_method=rf.find("nlp_scaling_method").asString().c_str();
    if (rf.find("nlp_scaling_method").isNull())
        nlp_scaling_method="none";

    ipopt_par.put("optimizer_points_advanced",optimizer_points);
    ipopt_par.put("max_cpu_time_advanced", max_cpu_time);
    ipopt_par.put("tol_advanced",tol);
    ipopt_par.put("acceptable_iter_advanced",acceptable_iter);
    ipopt_par.put("max_iter_advanced",max_iter);
    ipopt_par.put("mu_strategy_advanced",mu_strategy);
    ipopt_par.put("nlp_scaling_method_advanced",nlp_scaling_method);

    cout<<endl;
    cout<<"[Superqcomputation]: optimizer_points      "<<optimizer_points<<endl;
    cout<<"[Superqcomputation]: max_cpu_time          "<<max_cpu_time<<endl;
    cout<<"[Superqcomputation]: tol                   "<<tol<<endl;
    cout<<"[Superqcomputation]: acceptable_iter       "<<acceptable_iter<<endl;
    cout<<"[Superqcomputation]: max_iter              "<<max_iter<<endl;
    cout<<"[Superqcomputation]: mu_strategy           "<<mu_strategy<<endl;
    cout<<"[Superqcomputation]: nlp_scaling_method    "<<nlp_scaling_method<<endl;
    cout<<endl;

    return true;
}

/***********************************************************************/
bool SuperqModule::configViewer(ResourceFinder &rf)
{
    portImgIn.open("/superquadric-model/img:i");
    portImgOut.open("/superquadric-model/img:o");

    eye=rf.check("eye", Value("left")).asString();
    what_to_plot=rf.find("plot").asString().c_str();
    if (rf.find("plot").isNull())
        what_to_plot="superq";

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
    optionG.put("local","/superquadric-model/gaze");

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

    vis_points=50;
    vis_step=10;

    return true;
}

/***********************************************************************/
void SuperqModule::saveSuperq()
{
    ofstream fout;
    fout.open(outputFileName.c_str());
    if (fout.is_open())
    {
        fout<<"*****Result*****"<<endl;
        fout<<"Computed superquadric: "<<endl;
        fout<<" "<<x.toString(3,3);
        fout<<endl;
        fout<<"Filtered superquadric: "<<endl;
        fout<<" "<<x_filtered.toString(3,3);
        fout<<endl;
        fout<<"Execution time: "<<endl;
        fout<<" "<<t_superq <<endl;
        fout<<"Visualization time: "<<endl;
        fout<<" "<<t_shows2 <<endl;
        fout<<"Update module time"<<endl;
        fout<<" "<<t<<endl<<endl;
        fout<<"*****Optimizer parameters*****"<<endl;
        fout<<"Optimizer points: "<<optimizer_points<<endl;
        fout<<"Tolerance :"<<tol<<endl;
        fout<<"Nlp scaling method: "<<nlp_scaling_method<<endl;
        fout<<"Mu strategy: "<<mu_strategy<<endl;
    }
    fout.close();
}

///***********************************************************************/
//bool SuperqModule::readPointCloud()
//{
//    ifstream pointsFile(pointCloudFileName.c_str());
//    points.clear();
//    int nPoints;
//    int state=0;
//    char line[255];

//    if (!pointsFile.is_open())
//    {
//        yError()<<"problem opening point cloud file!";
//        return false;
//    }

//    while (!pointsFile.eof())
//    {
//        pointsFile.getline(line,sizeof(line),'\n');
//        Bottle b(line);
//        Value firstItem=b.get(0);
//        bool isNumber=firstItem.isInt() || firstItem.isDouble();

//        if (state==0)
//        {
//            string tmp=firstItem.asString().c_str();
//            std::transform(tmp.begin(),tmp.end(),tmp.begin(),::toupper);
//            if (tmp=="OFF" || tmp=="COFF")
//                state++;
//        }
//        else if (state==1)
//        {
//            if (isNumber)
//            {
//                nPoints=firstItem.asInt();
//                state++;
//            }
//        }
//        else if (state==2)
//        {
//            if (isNumber && (b.size()>=3))
//            {
//                Vector point(3,0.0);
//                point[0]=b.get(0).asDouble();
//                point[1]=b.get(1).asDouble();
//                point[2]=b.get(2).asDouble();
//                points.push_back(point);

//                if (--nPoints<=0)
//                    return true;
//            }
//        }
//    }

//    return false;
//}










