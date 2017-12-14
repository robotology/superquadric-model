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

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

/************************************************************************/
bool SuperqModule::attach(RpcServer &source)
{
    return this->yarp().attachAsServer(source);
}

/************************************************************************/
bool SuperqModule::set_tag_file(const string &tag)
{
    LockGuard lg(mutex);

    tag_file=tag;
    outputFileName=homeContextPath+"/"+tag_file+".txt";
    yDebug()<<" [SuperqModule]: File output "<<outputFileName;

    superqCom->setPar("tag_file", tag_file);

    return true;
}

/************************************************************************/
string SuperqModule::get_tag_file()
{
    return tag_file;
}

/**********************************************************************/
string SuperqModule::get_visualization()
{
    if (visualization_on)
        return "on";
    else
        return "off";
}

/**********************************************************************/
bool SuperqModule::set_visualization(const string &e)
{
    if ((e=="on") || (e=="off"))
    {
        LockGuard lg(mutex);

        if ((visualization_on==false) && (e=="on"))
        {
            superqVis->resume();

            visualization_on=true;

        }
        else if ((visualization_on==true) && (e=="off"))
        {
            superqVis->suspend();
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
Property SuperqModule::get_superq()
{
    Property superq;

    Vector sol(11,0.0);
    if (single_superq)
    {
        sol=superqCom->getSolution(0);
        superq=fillProperty(sol);
    }
    else
    {
        yDebug()<<"[SuperqComputation]: Superq has been computed "<<superqCom->superq_computed;
        while (!superqCom->superq_computed)
        {
            Time::delay(0.1);
        }

        superq=fillMultipleSolutions(superq_tree->root);
    }

    deque<Vector> p_aux;
    p_aux.clear();
    superqCom->sendPoints(p_aux);

    return superq;
}

/**********************************************************************/
bool SuperqModule::send_point_clouds(const vector<Vector> &p)
{
    double t, t_fin;

    t=Time::now();

    superqCom->setPar("object_class", object_class);

    yDebug()<<"Time operations  after set par 1"<<Time::now() - t;

    superqCom->setPar("one_shot", "on");

    yDebug()<<"Time operations after set par 2"<<Time::now() - t;

    superqCom->superq_computed=false;

    deque<Vector> p_aux;

    for (size_t i=0; i<p.size(); i++)
        p_aux.push_back(p[i]);

    superqCom->sendPoints(p_aux);
    yDebug()<<"Time operations send points "<<Time::now() - t;

    t_fin= Time::now() - t;

    yDebug()<<"[SuperqModule]: Time operations  final"<<t_fin;

    return true;
}

/**********************************************************************/
bool SuperqModule::reset_filter()
{
    superqCom->resetMedianFilter();

    return true;
}

/**********************************************************************/
Property SuperqModule::get_superq_filtered()
{
    Property superq;
    Vector sol(11,0.0);
    sol=superqCom->getSolution(1);

    superqCom->setPar("one_shot", "off");
    deque<Vector> p_aux;
    p_aux.clear();
    superqCom->sendPoints(p_aux);

    superq=fillProperty(sol);

    return superq;
}

/**********************************************************************/
Property SuperqModule::fillProperty(const Vector &sol)
{
    Property superq;

    Bottle bottle;
    Bottle &b1=bottle.addList();
    b1.addDouble(sol[0]); b1.addDouble(sol[1]); b1.addDouble(sol[2]);
    superq.put("dimensions", bottle.get(0));

    Bottle &b2=bottle.addList();
    b2.addDouble(sol[3]); b2.addDouble(sol[4]);
    superq.put("exponents", bottle.get(1));

    Bottle &b3=bottle.addList();
    b3.addDouble(sol[5]); b3.addDouble(sol[6]); b3.addDouble(sol[7]);
    superq.put("center", bottle.get(2));

    Bottle &b4=bottle.addList();
    Vector orient=dcm2axis(euler2dcm(sol.subVector(8,10)));
    b4.addDouble(orient[0]); b4.addDouble(orient[1]); b4.addDouble(orient[2]); b4.addDouble(orient[3]);
    superq.put("orientation", bottle.get(3));
    return superq;
}


/**********************************************************************/
Property SuperqModule::fillMultipleSolutions(node *leaf)
{
    int count=0;
    Property sup;

    addSuperqInProp(leaf, count, sup);

    return sup;
}

/**********************************************************************/
void SuperqModule::addSuperqInProp(node *leaf, int &count, Property &superq_pr)
{
    Vector sup(11,0.0);

    if (leaf!=NULL)
    {
        if (norm(leaf->superq.subVector(0,2))>0.0)
        {            
            stringstream ss;
 
            sup=leaf->superq;

            if (leaf->right!=NULL)
            {     
                addSuperqInProp(leaf->right, count, superq_pr);

                count++;
            }

            if (leaf->left!=NULL)
            {
                addSuperqInProp(leaf->left, count, superq_pr);

                count++;
            }

            if (leaf->right==NULL && leaf->left==NULL)
            {
                ss<<count;
                string count_str=ss.str();
                Bottle bottle;
                Bottle &b1=bottle.addList();
                b1.addDouble(sup[0]); b1.addDouble(sup[1]); b1.addDouble(sup[2]);
                superq_pr.put("dimensions_"+count_str, bottle.get(0));

                Bottle &b2=bottle.addList();
                b2.addDouble(sup[3]); b2.addDouble(sup[4]);
                superq_pr.put("exponent_"+count_str, bottle.get(1));

                Bottle &b3=bottle.addList();
                b3.addDouble(sup[5]); b3.addDouble(sup[6]); b3.addDouble(sup[7]);
                superq_pr.put("center_"+count_str, bottle.get(2));

                Bottle &b4=bottle.addList();
                Vector orient=dcm2axis(euler2dcm(sup.subVector(8,10)));
                b4.addDouble(orient[0]); b4.addDouble(orient[1]); b4.addDouble(orient[2]); b4.addDouble(orient[3]);
                superq_pr.put("orientation_"+count_str, bottle.get(3));

                yDebug()<<"Property "<<count<<superq_pr.toString();
                //count++;

            }   
            //count++;         
            
        }
        else
        {
            count++;
            if (leaf->right!=NULL)
            {
                addSuperqInProp(leaf->right, count, superq_pr);
                count++;
            }
            if (leaf->left!=NULL)
            {
                //count++;
                addSuperqInProp(leaf->left, count, superq_pr);               
            }
        }

    }
    else
        yDebug()<<"[SuperqModule]:Finished property fill!";
}

/**********************************************************************/
bool SuperqModule::set_points_filtering(const string &entry)
{
    if ((entry=="on") || (entry=="off"))
    {
        LockGuard lg(mutex);
        filter_points=(entry=="on");
        if (filter_points)
        {
            Property options;
            options.put("filter_radius", radius);
            options.put("filter_nnThreshold", nnThreshold);
            superqCom->setPointsFilterPar(options, false);
            superqCom->setPar("filter_points", "on");
        }
        else
            superqCom->setPar("filter_points", "off");

        yInfo()<<"[SuperqModule]: filter_points "<<filter_points;
        yInfo()<<"[SuperqModule]: radius        "<<radius;
        yInfo()<<"[SuperqModule]: nn-thrshold   "<<nnThreshold;

        return true;
    }
    else
    {
        return false;
    }
}

/**********************************************************************/
string SuperqModule::get_points_filtering()
{
    if (filter_points)
    {
        return "on";
    }
    else
    {
        return "off";
    }
}

/**********************************************************************/
bool SuperqModule::set_superq_filtering(const string &entry)
{
    if ((entry=="on") || (entry=="off"))
    {
        LockGuard lg(mutex);
        filter_superq= (entry=="on");
        if (filter_superq)
        {
            Property options;
            options.put("median_order", median_order);
            if (fixed_window)
                options.put("fixed_window", "on");
            else
                options.put("fixed_window", "off");

            superqCom->setSuperqFilterPar(options, false);
            superqCom->setPar("filter_superq", "on");
        }
        else
            superqCom->setPar("filter_superq", "off");

        yInfo()<<"[SuperqModule]: filter_superq         "<<filter_superq;
        yInfo()<<"[SuperqModule]: fixed_window          "<<fixed_window;
        yInfo()<<"[SuperqModule]: median_order          "<<median_order;
        yInfo()<<"[SuperqModule]: min_median_order      "<<min_median_order;
        yInfo()<<"[SuperqModule]: max_median_order      "<<max_median_order;

        return true;
    }
    else
    {
        return false;
    }
}

/**********************************************************************/
string SuperqModule::get_superq_filtering()
{
    if (filter_superq)
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
    if (save_points)
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
        if (save_points)
            superqCom->setPar("save_points", "on");
        else
            superqCom->setPar("save_points", "off");
    }
}

/**********************************************************************/
Property SuperqModule::get_options(const string &field)
{
    Property advOptions;
    if (field=="points_filter")
        advOptions=superqCom->getPointsFilterPar();
    else if (field=="superq_filter")
        advOptions=superqCom->getSuperqFilterPar();
    else if (field=="optimization")
        advOptions=superqCom->getIpoptPar();
    else if (field=="visualization")
        advOptions=superqVis->getPar();
    else if (field=="statistics")
    {
        advOptions.put("average_computation_time", t_superq);
        advOptions.put("average_visualization_time", t_vis);
    }

    return advOptions;
}

/**********************************************************************/
bool SuperqModule::set_options(const Property &newOptions, const string &field)
{
    if (field=="points_filter")
        superqCom->setPointsFilterPar(newOptions, false);
    else if (field=="superq_filter")
        superqCom->setSuperqFilterPar(newOptions, false);
    else if (field=="optimization")
        superqCom->setIpoptPar(newOptions, false);
    else if (field=="visualization")
        superqVis->setPar(newOptions, false);
    else
        return false;

    return true;
}

/**********************************************************************/
bool SuperqModule::set_object_class(const string &objclass)
{
    object_class=objclass;
    if (object_class=="box" || object_class=="sphere" || object_class=="cylinder")
        single_superq=true;
    else
        single_superq=false;

    superqCom->single_superq=single_superq;

    return true;
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


    if (mode_online)
    {
        superqCom->setPar("object_class", object_class);

        Property &x_to_send=portSuperq.prepare();

        imgIn=portImgIn.read();

        if (times_superq.size()<10)
            times_superq.push_back(superqCom->getTime());
        else if (times_superq.size()==10)
        {
            for (size_t i=0; i<times_superq.size(); i++)
            {
                t_superq+=times_superq[i];
            }
            t_superq=t_superq/times_superq.size();
            times_superq.push_back(0.0);
        }
        else
            times_superq.clear();


        if (!filter_superq)
            x_to_send=fillProperty(x);
        else
            x_to_send=fillProperty(x_filtered);

        portSuperq.write();

        if (visualization_on)
        {
            if (times_vis.size()<10)
            times_vis.push_back(superqVis->getTime());
            else if (times_vis.size()==10)
            {
                for (size_t i=0; i<times_vis.size(); i++)
                {
                    t_vis+=times_vis[i];
                }
            t_vis=t_vis/times_vis.size();
            times_vis.push_back(0.0);
        }
        else
            times_vis.clear();
        }
    }
    else
    {
        readPointCloud();
        superqCom->threadInit();
        superqCom->sendPoints(points_aux);

        superqCom->setPar("object_class", object_class);

        if ((filter_points==true) && (points.size()>0))
        {
            superqCom->filter();
        }

        if (points.size()>0)
        {
            if (single_superq)
            {
                yInfo()<<"[SuperqModule]: number of points acquired:"<< points.size();
                go_on=superqCom->computeSuperq();
            }
            else
            {
                t_mult=Time::now();
                superqCom->iterativeModeling();
                superqCom->mergeModeling(superq_tree->root, true);
                t_mult=Time::now()-t_mult;
                superq_tree->printTree(superq_tree->root);
                yInfo()<<"             Execution Time           :"<<t_mult;
            }
        }

        if ((go_on==false) && (points.size()>0))
        {
            yError("[SuperqModule]: Not found a suitable superquadric! ");
        }
        else if ((go_on==true) && (norm(x)>0.0))
        {
            if (filter_superq)
                superqCom->filterSuperq();
        }

        x=superqCom->getSolution(false);
        x_filtered=superqCom->getSolution(true);

        return false;
    }


    t=Time::now()-t0;
    return true;
}

/***********************************************************************/
bool SuperqModule::configure(ResourceFinder &rf)
{
    bool config_ok;

    yInfo()<<"[SuperqModule]: Configuring ... ";

    configOnOff(rf);

    if (filter_points==true)
        configFilter(rf);
    if (filter_superq==true)
        configFilterSuperq(rf);

    configServices(rf);
    configSuperq(rf);

    config_ok=configViewer(rf);
    if (config_ok==false)
        return false;


    imgIn=portImgIn.read();
    superq_tree= new superqTree();

    superqCom= new SuperqComputation(mutex_shared,rate, filter_points, filter_superq, single_superq, fixed_window, points, imgIn, tag_file,
                                     threshold_median,filter_points_par, x, x_filtered, filter_superq_par, ipopt_par, homeContextPath, save_points, this->rf, superq_tree);

    if (mode_online)
    {
        bool thread_started=superqCom->start();

        if (thread_started)
            yInfo()<<"[SuperqModule]: Computation hread started!";
        else
            yError()<<"[SuperqModule]: Problems in starting the computation thread!";

        superqVis= new SuperqVisualization(rate_vis,eye, what_to_plot,x, x_filtered, Color, igaze, K, points, vis_points, vis_step, imgIn, superq_tree, single_superq);

    if (visualization_on)
    {
        bool thread_started=superqVis->start();

            if (thread_started)
                yInfo()<<"[SuperqModule]: Visualization thread started!";
            else
                yError()<<"[SuperqModule]: Problems in starting the visualization thread!";
        }
        else
        {
            superqVis->threadInit();
        }
    }

    return true;
}

/***********************************************************************/
bool SuperqModule::interruptModule()
{
    yInfo()<<"[SuperqModule]: Interruping ... ";

    portImgIn.interrupt();
    portSuperq.interrupt();

    return true;
}

/***********************************************************************/
bool SuperqModule::close()
{
    yInfo()<<"[SuperqModule]: Closing ... ";
    saveSuperq();

    superqCom->stop();
    delete superqCom;

    if (mode_online)
    {
        superqVis->stop();
        delete superqVis;
    }

    if (portRpc.asPort().isOpen())
        portRpc.close();

    if (!portImgIn.isClosed())
        portImgIn.close();

     if (!portSuperq.isClosed())
        portSuperq.close();

    if (mode_online)
        GazeCtrl.close();

    return true;
}

/***********************************************************************/
bool SuperqModule::configOnOff(ResourceFinder &rf)
{
    homeContextPath=rf.getHomeContextPath().c_str();
    pointCloudFileName=rf.findFile("pointCloudFile");
    save_points=(rf.check("save_points", Value("no")).asString()=="yes");

    visualization_on=(rf.check("visualization_on", Value("yes")).asString()=="yes");

    rate=rf.check("rate", Value(100)).asInt();
    rate_vis=rf.check("rate_vis", Value(100)).asInt();

    threshold_median=rf.check("threshold_median", Value(0.1)).asDouble();

    object_class=rf.check("object_class", Value("default")).asString();

    if (rf.find("pointCloudFile").isNull())
    {
        mode_online=true;
    }
    else
    {
        mode_online=false;
        outputFileName=rf.findFile("outputFile");

        if (rf.find("outputFile").isNull())
            outputFileName=homeContextPath+"/output.txt";

        yDebug()<<"file output "<<outputFileName;
    }

    filter_points=(rf.check("filter_points", Value("off")).asString()=="on");
    filter_superq=(rf.check("filter_superq", Value("off")).asString()=="on");
    single_superq=(rf.check("single_superq", Value("on")).asString()=="on");

    yInfo()<<"[SuperqModule]: rate          "<<rate;
    yInfo()<<"[SuperqModule]: filter_points "<<filter_points;
    yInfo()<<"[SuperqModule]: filter_superq "<<filter_superq;

    return true;
}

/***********************************************************************/
bool SuperqModule::configFilter(ResourceFinder &rf)
{
    radius=rf.check("radius", Value(0.005)).asDouble();
    nnThreshold=rf.check("nn-threshold", Value(100)).asInt();

    filter_points_par.put("filter_radius",radius);
    filter_points_par.put("filter_nnThreshold",nnThreshold);

    yInfo()<<"[SuperqModule]: radius         "<<radius;
    yInfo()<<"[SuperqModule]: nn-threshold   "<<nnThreshold;

    return true;
}

/***********************************************************************/
bool SuperqModule::configFilterSuperq(ResourceFinder &rf)
{
    fixed_window=(rf.check("fixed_window", Value("off")).asString()=="on");
    reset=(rf.check("reset", Value("off")).asString()=="on");
    median_order=rf.check("median_order", Value(1)).asInt();
    min_median_order=rf.check("min_median_order", Value(1)).asInt();
    max_median_order=rf.check("max_median_order", Value(30)).asInt();
    min_norm_vel=rf.check("min_norm_vel", Value(0.01)).asDouble();
    x.resize(11,0.0);

    filter_superq_par.put("fixed_window",fixed_window);
    filter_superq_par.put("median_order",median_order);
    filter_superq_par.put("min_median_order",min_median_order);
    filter_superq_par.put("max_median_order",max_median_order);
    filter_superq_par.put("threshold_median",threshold_median);
    filter_superq_par.put("min_norm_vel",min_norm_vel);

    yInfo()<<"[SuperqModule]: fixed_window          "<<fixed_window;
    yInfo()<<"[SuperqModule]: median_order          "<<median_order;
    yInfo()<<"[SuperqModule]: min_median_order      "<<min_median_order;
    yInfo()<<"[SuperqModule]: max_median_order      "<<max_median_order;
    yInfo()<<"[SuperqModule]: threshold_median      "<<threshold_median;
    yInfo()<<"[SuperqModule]: min_norm_vel          "<<min_norm_vel;

    return true;
}

/***********************************************************************/
bool SuperqModule::configServices(ResourceFinder &rf)
{
    portRpc.open("/superquadric-model/rpc");
    portSuperq.open("/superquadric-model/superq:o");

    attach(portRpc);

    return true;
}

/***********************************************************************/
bool SuperqModule::configSuperq(ResourceFinder &rf)
{
    this->rf=&rf;

    optimizer_points=rf.check("optimizer_points", Value(300)).asInt();
    max_cpu_time=rf.check("max_cpu_time", Value(5.0)).asDouble();

    tree_splitting=rf.check("tree_splitting", Value(2)).asInt();
    f_thres=rf.check("f_thres", Value(1.0)).asInt();

    tol=rf.check("tol",Value(1e-5)).asDouble();
    acceptable_iter=rf.check("acceptable_iter",Value(0)).asInt();
    max_iter=rf.check("max_iter",Value(numeric_limits<int>::max())).asInt();

    mu_strategy=rf.find("mu_strategy").asString().c_str();
    if (rf.find("mu_strategy").isNull())
        mu_strategy="adaptive";

    nlp_scaling_method=rf.find("nlp_scaling_method").asString().c_str();
    if (rf.find("nlp_scaling_method").isNull())
        nlp_scaling_method="none";

    ipopt_par.put("optimizer_points",optimizer_points);
    ipopt_par.put("max_cpu_time", max_cpu_time);
    ipopt_par.put("tol",tol);
    ipopt_par.put("acceptable_iter",acceptable_iter);
    ipopt_par.put("max_iter",max_iter);
    ipopt_par.put("mu_strategy",mu_strategy);
    ipopt_par.put("nlp_scaling_method",nlp_scaling_method);
    ipopt_par.put("tree_splitting",tree_splitting);

    yInfo()<<"[SuperqModule]: optimizer_points      "<<optimizer_points;
    yInfo()<<"[SuperqModule]: max_cpu_time          "<<max_cpu_time;
    yInfo()<<"[SuperqModule]: tol                   "<<tol;
    yInfo()<<"[SuperqModule]: acceptable_iter       "<<acceptable_iter;
    yInfo()<<"[SuperqModule]: max_iter              "<<max_iter;
    yInfo()<<"[SuperqModule]: mu_strategy           "<<mu_strategy;
    yInfo()<<"[SuperqModule]: nlp_scaling_method    "<<nlp_scaling_method;
    yInfo()<<"[SuperqModule]: tree_splitting        "<<tree_splitting;
    yInfo()<<"[SuperqModule]: f_thres               "<<f_thres;

    return true;
}

/***********************************************************************/
bool SuperqModule::configViewer(ResourceFinder &rf)
{
    portImgIn.open("/superquadric-model/img:i");

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

    if (mode_online)
    {
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
    }

    return true;
}

/***********************************************************************/
void SuperqModule::saveSuperq()
{
    ofstream fout;
    fout.open(outputFileName.c_str());
    if (fout.is_open() && single_superq==true)
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
        fout<<"Update module time"<<endl;
        fout<<" "<<t<<endl<<endl;
        fout<<"*****Optimizer parameters*****"<<endl;
        fout<<"Optimizer points: "<<optimizer_points<<endl;
        fout<<"Tolerance :"<<tol<<endl;
        fout<<"Nlp scaling method: "<<nlp_scaling_method<<endl;
        fout<<"Mu strategy: "<<mu_strategy<<endl;
    }
    else if (fout.is_open() && single_superq==false)
    {
        fout<<"*****Result*****"<<endl;
        fout<<"Computed superquadric: "<<endl;
        superq_tree->saveTree(fout, superq_tree->root);
        fout<<endl;

        fout<<"Execution time: "<<endl;
        fout<<" "<<t_mult<<endl;

    }
    fout.close();
}

/***********************************************************************/
bool SuperqModule::readPointCloud()
{
    ifstream pointsFile(pointCloudFileName.c_str());
    points_aux.clear();
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
            if (tmp=="OFF" || tmp=="COFF")
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
                points_aux.push_back(point);

                if (--nPoints<=0)
                    return true;
            }
        }
    }

    return false;
}
