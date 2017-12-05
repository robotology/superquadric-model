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
#include <yarp/math/SVD.h>

#include "superqComputation.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;

/*******************************************************************************/
vector<int>  SpatialDensityFilter::filter(const cv::Mat &data,const double radius, const int maxResults, deque<Vector> &points)
{
    deque<Vector> ind;
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

/***********************************************************************/
SuperqComputation::SuperqComputation(Mutex &_mutex_shared, int _rate, bool _filter_points, bool _filter_superq, bool _single_superq, bool _fixed_window,deque<yarp::sig::Vector> &_points, ImageOf<PixelRgb> *_imgIn, string _tag_file, double _threshold_median,
                                const Property &_filter_points_par, Vector &_x, Vector &_x_filtered, const Property &_filter_superq_par, const Property &_ipopt_par, const string &_homeContextPath, bool _save_points, ResourceFinder *_rf, superqTree *_superq_tree):
                                mutex_shared(_mutex_shared),filter_points(_filter_points), filter_superq(_filter_superq), single_superq(_single_superq),fixed_window( _fixed_window),tag_file(_tag_file),  threshold_median(_threshold_median), save_points(_save_points), imgIn(_imgIn),
                                filter_points_par(_filter_points_par),filter_superq_par(_filter_superq_par),ipopt_par(_ipopt_par), Thread(), homeContextPath(_homeContextPath), x(_x), x_filtered(_x_filtered), points(_points), rf(_rf), superq_tree(_superq_tree)
{
}

/***********************************************************************/
void SuperqComputation::setPointsFilterPar(const Property &newOptions, bool first_time)
{
    LockGuard lg(mutex);

    double radiusValue=newOptions.find("filter_radius").asDouble();
    if (newOptions.find("filter_radius").isNull() && (first_time==true))
    {
        radius=0.01;
    }
    else if (!newOptions.find("filter_radius").isNull())
    {
        if ((radiusValue>0.0000001) && (radiusValue<0.01))
        {
            radius=radiusValue;
        }
        else if ((radiusValue>=0.01))
        {
            radius=0.01;
        }
        else if ((radiusValue<=0.0000001))
        {
            radius=0.0000001;
        }
    }

   int nnThreValue=newOptions.find("filter_nnThreshold").asInt();
    if (newOptions.find("filter_nnThreshold").isNull() && (first_time==true))
    {
        nnThreshold=100;
    }
    else if (!newOptions.find("filter_nnThreshold").isNull())
    {
        if ((nnThreValue>0) && (nnThreValue<100))
        {
                nnThreshold=nnThreValue;
        }
        else
        {
            nnThreshold=100;
        }
    }
}

/***********************************************************************/
Property SuperqComputation::getPointsFilterPar()
{
    LockGuard lg(mutex);

    Property advOptions;
    advOptions.put("filter_radius",radius);
    advOptions.put("filter_nnThreshold",nnThreshold);
    return advOptions;
}

/***********************************************************************/
void SuperqComputation::setSuperqFilterPar(const Property &newOptions, bool first_time)
{
    LockGuard lg(mutex);

    int mOrderValue=newOptions.find("median_order").asInt();
    if (newOptions.find("median_order").isNull() && (first_time==true))
    {
        std_median_order=3;
    }
    else if (!newOptions.find("median_order").isNull())
    {
        if((mOrderValue>=1) && (mOrderValue<=50))
        {
            std_median_order=mOrderValue;
        }
        else if (mOrderValue<1)
        {
            std_median_order=1;
        }
        else if (mOrderValue>50)
        {
            std_median_order=50;
        }
    }

    mOrderValue=newOptions.find("min_median_order").asInt();
    if (newOptions.find("min_median_order").isNull() && (first_time==true))
    {
        min_median_order=1;
    }
    else if (!newOptions.find("min_median_order").isNull())
    {
        if ((mOrderValue>=1) && (mOrderValue<=50))
        {
            min_median_order=mOrderValue;
        }
        else if (mOrderValue<1)
        {
            min_median_order=1;
        }
        else if (mOrderValue>50)
        {
            min_median_order=50;
        }
    }

    mOrderValue=newOptions.find("max_median_order").asInt();
    if (newOptions.find("max_median_order").isNull() && (first_time==true))
    {
        max_median_order=30;
    }
    else if (!newOptions.find("max_median_order").isNull())
    {
        if ((mOrderValue>=1) && (mOrderValue<=50) && (mOrderValue>=min_median_order))
        {
            max_median_order=mOrderValue;
        }
        else if ((mOrderValue<1) || (mOrderValue<min_median_order))
        {
            max_median_order=min_median_order;
        }
        else if (mOrderValue>50)
        {
            max_median_order=50;
        }
    }

    double threValue=newOptions.find("threshold_median").asDouble();
    if (newOptions.find("threhsold_median").isNull() && (first_time==true))
    {
        threshold_median=0.1;
    }
    else if (!newOptions.find("threhsold_median").isNull())
    {
        if ((threValue>0.005) && (threValue<=2.0))
        {
            threshold_median=threValue;
        }
        else if (threValue<0.005)
        {
            threshold_median=0.005;
        }
        else if (threValue>2.0)
        {
            threshold_median=2.0;
        }
    }

    double minNormVel=newOptions.find("min_norm_vel").asDouble();
    if (newOptions.find("min_norm_vel").isNull() && (first_time==true))
    {
        min_norm_vel=0.01;
    }
    else if (!newOptions.find("min_norm_vel").isNull())
    {
        if ((minNormVel>0.005) && (minNormVel<=0.1))
        {
            min_norm_vel=minNormVel;
        }
        else if (minNormVel<0.005)
        {
            min_norm_vel=0.005;
        }
        else if (minNormVel>0.1)
        {
            min_norm_vel=0.1;
        }
    }

    string par=newOptions.find("fixed_window").asString();
    if (newOptions.find("fixed_window").isNull() && (first_time==true))
    {
        fixed_window=false;
    }
    else if (!newOptions.find("fixed_window").isNull())
    {
        if ((par=="on") || (par=="off"))
        {
            fixed_window=(par=="on");
        }
        else
        {
            fixed_window=false;
        }
    }
}

/***********************************************************************/
Property SuperqComputation::getSuperqFilterPar()
{
    LockGuard lg(mutex);

    Property advOptions;
    if (fixed_window)
        advOptions.put("fixed_window","on");
    else
        advOptions.put("fixed_window","off");
    advOptions.put("median_order",std_median_order);
    advOptions.put("min_median_order",min_median_order);
    advOptions.put("max_median_order",max_median_order);
    advOptions.put("threshold_median",threshold_median);
    advOptions.put("min_norm_vel",min_norm_vel);
    return advOptions;
}

/***********************************************************************/
void SuperqComputation::setIpoptPar(const Property &newOptions, bool first_time)
{
    LockGuard lg(mutex);

    int points=newOptions.find("optimizer_points").asInt();


    if (newOptions.find("optimizer_points").isNull() && (first_time==true))
    {
        optimizer_points=50;
    }
    else if (!newOptions.find("optimizer_points").isNull())
    {
        if ((points>=10) && (points<=300))
        {
            optimizer_points=points;
        }
        else if (points<10)
        {
            optimizer_points=10;
        }
        else if (points>300)
        {
            optimizer_points=300;
        }
    }

    int ts=newOptions.find("tree_splitting").asInt();
    if (newOptions.find("tree_splitting").isNull() && (first_time==true))
    {
        tree_splitting=2;
    }
    else if (!newOptions.find("tree_splitting").isNull())
    {
        if ((ts>=1))
        {
            tree_splitting=ts;
        }
        else
        {
            tree_splitting=2;
        }
    }

    double maxCpuTime=newOptions.find("max_cpu_time").asDouble();
    if (newOptions.find("max_cpu_time").isNull() && (first_time==true))
    {
        max_cpu_time=5.0;
    }
    else if (!newOptions.find("max_cpu_time").isNull())
    {
        if ((maxCpuTime>=0.01) && (maxCpuTime<=10.0))
        {
            max_cpu_time=maxCpuTime;
        }
        else if (maxCpuTime<0.01)
        {
            max_cpu_time=0.01;
        }
        else if (maxCpuTime>10.0)
        {
            max_cpu_time=10.0;
        }
    }

    double tolValue=newOptions.find("tol").asDouble();
    if (newOptions.find("tol").isNull() && (first_time==true))
    {
        tol=1e-5;
    }
    else if (!newOptions.find("tol").isNull())
    {
        if ((tolValue>1e-8) && (tolValue<=0.01))
        {
            tol=tolValue;
        }
        else if (tolValue<1e-8)
        {
            tol=1e-8;
        }
        else if (tolValue>0.01)
        {
            tol=0.01;
        }
    }

    int accIter=newOptions.find("acceptable_iter").asInt();
    if (newOptions.find("acceptable_iter").isNull() && (first_time==true))
    {
        acceptable_iter=0;
    }
    else if (!newOptions.find("acceptable_iter").isNull())
    {
        if ((accIter>=0 )&& (accIter<=10))
        {
             acceptable_iter=accIter;
        }
        else if (accIter<0)
        {
            acceptable_iter=0;
        }
        else if (accIter>10)
        {
            acceptable_iter=10;
        }
    }

    int maxIter=newOptions.find("max_iter").asInt();
    if (newOptions.find("max_iter").isNull() && (first_time==true))
    {
        max_iter=100;
    }
    else if (!newOptions.find("max_iter").isNull())
    {
        if ((maxIter>1))
        {
            max_iter=maxIter;
        }
        else
        {
            max_iter=100;
        }
    }

    string mu_str=newOptions.find("mu_strategy").asString();
    if (newOptions.find("mu_strategy").isNull() && (first_time==true))
    {
        mu_strategy="monotone";
    }
    else if (!newOptions.find("mu_strategy").isNull())
    {
        if ((mu_str=="adaptive") || (mu_str=="monotone"))
        {
            mu_strategy=mu_str;
        }
        else
        {
            mu_strategy="monotone";
        }
    }

    string nlp=newOptions.find("nlp_scaling_method").asString();
    if (newOptions.find("nlp_scaling_method").isNull() && (first_time==true))
    {
        nlp_scaling_method="gradient-based";
    }
    else if (!newOptions.find("nlp_scaling_method").isNull())
    {
        if ((nlp=="none") || (nlp=="gradient-based"))
        {
            nlp_scaling_method=nlp;
        }
        else
        {
            nlp_scaling_method="gradient-based";
        }
    }
}

/***********************************************************************/
Property SuperqComputation::getIpoptPar()
{
    LockGuard lg(mutex);

    Property advOptions;
    advOptions.put("optimizer_points",optimizer_points);
    advOptions.put("max_cpu_time",max_cpu_time);
    advOptions.put("tol",tol);
    advOptions.put("max_iter",max_iter);
    advOptions.put("acceptable_iter",acceptable_iter);
    advOptions.put("IPOPT_mu_strategy",mu_strategy);
    advOptions.put("IPOPT_nlp_scaling_method",nlp_scaling_method);
    return advOptions;
}

/***********************************************************************/
void SuperqComputation::setPar(const string &par_name, const string &value)
{
    //LockGuard lg(mutex);
    if (par_name=="tag_file")
        tag_file=value;
    else if (par_name=="filter_points")
        filter_points=(value=="on");
    else if (par_name=="filter_superq")
        filter_superq=(value=="on");
    else if (par_name=="save_points")
        save_points=(value=="on");
    else if (par_name=="one_shot")
        one_shot=(value=="on");
    else if (par_name=="object_class")
        ob_class=value;
    else if (par_name=="single_superq")
        single_superq=(value=="on");
}

/***********************************************************************/
double SuperqComputation::getTime()
{
    LockGuard lg(mutex);
    return t_superq;
}

/***********************************************************************/
bool SuperqComputation::threadInit()
{
    yInfo()<<"[SuperqComputation]: Thread initing ... ";

    if (filter_points==true)
        setPointsFilterPar(filter_points_par, true);

    if (filter_superq==true)
        setSuperqFilterPar(filter_superq_par, true);

    setIpoptPar(ipopt_par, true);

    configFilterSuperq();
    config3Dpoints();

    bounds_automatic=true;
    one_shot=false;

    superq_computed=false;

    yDebug()<<"[SuperqComputation]: Resize of x";
    x.resize(11,0.00);
    x_filtered.resize(11,0.0);
    yDebug()<<"[SuperqComputation]: After resize of x";

    count_file=0;

    return true;
}

/***********************************************************************/
void SuperqComputation::run()
{
    yDebug()<<"is stopping "<<isStopping();
    while (!isStopping())
    {
        t0=Time::now();

        //yDebug()<<"points size "<<points.size();

        if (points.size()>0)
        {
            if (one_shot==false)
                getPoints3D();

            if (points.size()>0)
            {
                if (single_superq)
                {
                    if ((filter_points==true))
                    {
                        filter();
                    }

                    if (points.size()>0)
                    {
                        yInfo()<<"[SuperqComputation]: number of points acquired:"<< points.size();
                        go_on=computeSuperq();
                    }
                }
                else
                {
                    iterativeModeling();
                    //if(merge)
                    mergeModeling(superq_tree->root, true);
                    Time::delay(1.5);
                    superq_computed=true;
                }
            }

            yDebug()<<__LINE__;

            if ((go_on==false) && (points.size()>0))
            {
                yError("[SuperqComputation]: Not found a suitable superquadric! ");
            }
            else if (go_on==true && norm(x)>0.0 && (points.size()>0))
            {
                if (filter_superq)
                    filterSuperq();
                else
                    x_filtered=x;
            }
            else
            {
                x_filtered.resize(11,0.0);
            }

            mutex.unlock();
        }
        else
        {
            x_filtered.resize(11,0.0);

            Time::delay(0.15);
        }

        t_superq=Time::now() - t0;
    }
}

/***********************************************************************/
void SuperqComputation::threadRelease()
{
    yInfo()<<"[SuperComputation]: Thread releasing ... ";

    if (!pointPort.isClosed())
        pointPort.close();

    if (mFilter!=NULL)
        delete mFilter;

    if (PolyEst!=NULL)
        delete PolyEst;

    if (superq_tree!=NULL)
        delete superq_tree;
}

/***********************************************************************/
bool SuperqComputation::configFilterSuperq()
{
    x.resize(11,0.0);
    new_median_order=1;

    std_median_order=5;
    max_median_order=30;

    mFilter = new MedianFilter(median_order, x);
    PolyEst =new AWLinEstimator(max_median_order, threshold_median);

    return true;
}

/***********************************************************************/
bool SuperqComputation::config3Dpoints()
{
    pointPort.open("/superquadric-model/point:i");

    return true;
}

/***********************************************************************/
void SuperqComputation::getPoints3D()
{
    Bottle *reply;

    reply=pointPort.read(false);

    if (reply!=NULL)
    {
        points.clear();

        if (Bottle *list=reply->get(0).asList())
        {
            for (int i=0; i<list->size();i++)
            {
                if (Bottle *pp=list->get(i).asList())
                {
                    Vector tmp(3,0.0);
                    tmp[0]=pp->get(0).asDouble();
                    tmp[1]=pp->get(1).asDouble();
                    tmp[2]=pp->get(2).asDouble();


                    points.push_back(tmp);
                }
                else
                {
                    yError()<<"[SuperqComputation]: Some problems in blob pixels!";
                }
            }
        }
        else
        {
            yError()<<"[SuperqComputation]: Some problem  in object blob!";
        }
    }
    else
    {
        yWarning("[SuperqComputation]: 3D points not received!");
    }
}

/***********************************************************************/
void SuperqComputation::savePoints(const string &namefile, const Vector &colors)
{
    ofstream fout;
    stringstream ss;
    ss << count_file;
    string count_file_str=ss.str();
    fout.open((homeContextPath+namefile+count_file_str+".off").c_str());

    if (fout.is_open())
    {
        fout<<"COFF"<<endl;
        fout<<points.size()<<" 0 0"<<endl;
        fout<<endl;
        for (size_t i=0; i<points.size(); i++)
        {
            int r=points[i][3];
            int g=points[i][4];
            int b=points[i][5];
            fout<<points[i].subVector(0,2).toString(3,3)<<" "<<r<<" "<<g<<" "<<b<<endl;
        }

        fout<<endl;
    }
    else
        yError()<<"[SuperqComputation]: Some problems in opening output file!";

    fout.close();

    count_file++;
}

/***********************************************************************/
bool SuperqComputation::readPointCloud()
{
    ifstream pointsFile(pointCloudFileName.c_str());
    points.clear();
    int nPoints;
    int state=0;
    string line;

    if (!pointsFile.is_open())
    {
        yError()<<"[SuperqComputation]: problem opening point cloud file!";
        return false;
    }

    while (!pointsFile.eof())
    {
        getline(pointsFile,line);
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
                points.push_back(point);

                if (--nPoints<=0)
                    return true;
            }
        }
    }

    return false;
}

/***********************************************************************/
void SuperqComputation::filter()
{
    numVertices=points.size();

    cv:: Mat data(numVertices,3,CV_32F);

    for (int i=0; i<numVertices; i++)
    {
        Vector point=points[i];
        data.at<float>(i,0)=(float)point[0];
        data.at<float>(i,1)=(float)point[1];
        data.at<float>(i,2)=(float)point[2];
    }

    points.clear();

    yInfo()<<"[SuperqComputation]: Processing points...";
    double t0=yarp::os::Time::now();
    SpatialDensityFilter::filter(data,radius,nnThreshold+1, points);
    double t1=yarp::os::Time::now();
    yInfo()<<"[SuperqComputation]: Processed in "<<1e3*(t1-t0)<<" [ms]";

    Vector colors(3,0.0);
    colors[1]=255;

    savePoints("/filtered-"+tag_file, colors);
}

/***********************************************************************/
bool SuperqComputation::computeSuperq()
{
    Vector colors(3,0.0);
    colors[1]=255;

    savePoints("/3Dpoints-"+tag_file, colors);

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
    superQ_nlp->configure(this->rf,bounds_automatic, ob_class);

    superQ_nlp->setPoints(points, optimizer_points);

    double t0_superq=Time::now();

    yDebug()<<"[SuperqComputation]: Start IPOPT ";

    Ipopt::ApplicationReturnStatus status=app->OptimizeTNLP(GetRawPtr(superQ_nlp));

    yDebug()<<"[SuperqComputation]: Finish IPOPT ";

    double t_s=Time::now()-t0_superq;

    if (status==Ipopt::Solve_Succeeded)
    {
        x=superQ_nlp->get_result();
        yInfo("[SuperqComputation]: Solution of the optimization problem: %s", x.toString(3,3).c_str());
        yInfo("[SuperqComputation]: Execution time : %f", t_s);
        return true;
    }
    else if(status==Ipopt::Maximum_CpuTime_Exceeded)
    {
        x=superQ_nlp->get_result();
        yWarning("[SuperqComputation]: Solution after maximum time exceeded: %s", x.toString(3,3).c_str());
        return true;
    }
    else
    {
        x.resize(11,0.0);
        return false;
    }
}

/***********************************************************************/
Vector SuperqComputation::computeMultipleSuperq(const deque<Vector> &points_splitted)
{
    Vector colors(3,0.0);
    colors[1]=255;

    //savePoints("/3Dpoints-"+tag_file, colors);

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
    superQ_nlp->configure(this->rf,bounds_automatic, ob_class);

    superQ_nlp->setPoints(points_splitted, optimizer_points);

    double t0_superq=Time::now();

    yDebug()<<"[SuperqComputation]: Start IPOPT ";

    Ipopt::ApplicationReturnStatus status=app->OptimizeTNLP(GetRawPtr(superQ_nlp));

    yDebug()<<"[SuperqComputation]: Finish IPOPT ";

    double t_s=Time::now()-t0_superq;

    if (status==Ipopt::Solve_Succeeded)
    {
        x=superQ_nlp->get_result();
        yInfo("[SuperqComputation]: Solution of the optimization problem: %s", x.toString(3,3).c_str());
        yInfo("[SuperqComputation]: Execution time : %f", t_s);
        return x;
    }
    else if(status==Ipopt::Maximum_CpuTime_Exceeded)
    {
        x=superQ_nlp->get_result();
        yWarning("[SuperqComputation]: Solution after maximum time exceeded: %s", x.toString(3,3).c_str());
        return x;
    }
    else
    {
        x.resize(11,0.0);
        return x;
    }
}

/***********************************************************************/
void SuperqComputation::filterSuperq()
{
    yInfo()<< "[SuperqComputation]: Filtering the last "<< median_order << " superquadrics...";

    yInfo()<<"[SuperqComputation]: x "<<x.toString();

    if (fixed_window)
    {
        if (median_order != std_median_order)
        {
            median_order=std_median_order;
            mFilter->setOrder(median_order);
        }
        x_filtered=mFilter->filt(x);
    }
    else
    {
        int new_median_order=adaptWindComputation();

        if (median_order != new_median_order)
        {
            median_order=new_median_order;
            mFilter->setOrder(median_order);
            x_filtered=mFilter->filt(x);
        }
        else
            x_filtered=mFilter->filt(x);
    }

    if (norm(x_filtered)==0.0)
        x_filtered=x;

    yInfo()<< "[SuperqComputation]: Filtered superq "<< x_filtered.toString(3,3);
}

/***********************************************************************/
void SuperqComputation::resetMedianFilter()
{
    x.resize(11,0.0);
    x_filtered.resize(11,0.0);

    mFilter->init(x);
}

/***********************************************************************/
int SuperqComputation::adaptWindComputation()
{
    elem_x.resize(3,0.0);
    elem_x=x.subVector(5,7);
    yInfo()<<"[SuperqComputation]: Old median order "<<median_order;

    AWPolyElement el(elem_x,Time::now());
    Vector vel=PolyEst->estimate(el);
    yInfo()<<"[SuperqComputation]: Velocity estimate "<<PolyEst->estimate(el).toString();


    if (norm(vel)>=min_norm_vel)
        new_median_order=min_median_order;
    else
    {
        if (new_median_order<max_median_order)
            new_median_order++;
    }

    yInfo()<<"[SuperqComputation]: New median order "<<new_median_order;
    return new_median_order;
}

/***********************************************************************/
Vector SuperqComputation::getSolution(bool filtered_superq)
{
    LockGuard lg(mutex);

    if (filtered_superq==false)
        return x;
    else
        return x_filtered;
}

/***********************************************************************/
void SuperqComputation::sendPoints(const deque<Vector> &p)
{
    //LockGuard lg_shared(mutex_shared);
    LockGuard lg(mutex);

    yDebug()<<"points size p"<<p.size();

    points.clear();

    for (int i=0; i<p.size(); i++)
    {
        points.push_back(p[i]);
    }
}

/***********************************************************************/
void SuperqComputation::iterativeModeling()
{
    Vector f(2,0.0);
    bool first_time=true;

    superq_tree->setPoints(&points);

    node *newnode=superq_tree->root;

    int i=0;
    int j=tree_splitting;
    int count=0;
    int count2=0;
    bool final=false;

    computeNestedSuperq(newnode, i,j, first_time, count, final, count2);

    //superq_tree->printTree(superq_tree->root);
}

/***********************************************************************/

void SuperqComputation::computeNestedSuperq(node *newnode, int &i,int &j, bool first_time,int &count, bool final, int &count2)
{
    if ((newnode!=NULL) && (i<tree_splitting))
    {
        cout<<endl;
        Vector superq1(11,0.0);
        Vector superq2(11,0.0);

        nodeContent node_c1;
        nodeContent node_c2;

        splitPoints(false, newnode);

        superq1=computeMultipleSuperq(points_splitted1);
        superq2=computeMultipleSuperq(points_splitted2);

        node_c1.f_value=evaluateLoss(superq1, points_splitted1);
        node_c2.f_value=evaluateLoss(superq2, points_splitted2);

        node_c1.superq=superq1;
        node_c2.superq=superq2;
        node_c1.point_cloud= new deque<Vector>;
        node_c2.point_cloud= new deque<Vector>;

        *node_c1.point_cloud=points_splitted1;
        *node_c2.point_cloud=points_splitted2;

        superq_tree->insert(node_c1, node_c2, newnode);

        i++;
        first_time=true;

        computeNestedSuperq(newnode->left,i,j, first_time, count, final, count2);

        first_time=true;
    }
    else if ((i==tree_splitting))
    {
        yDebug()<<"Completing Tree..";

        if (first_time==true)
        {
            cout<<endl;

            newnode=superq_tree->root;
            yDebug()<<newnode->right->point_cloud->size();

            first_time=false;
            j=0;

            computeNestedSuperq(newnode->right,i,j, first_time, count, final, count2);


            first_time=false;
            cout<<endl;

        }
        else if ((newnode!=NULL) && (j<tree_splitting-1))
        {

            j++;
            Vector superq1(11,0.0);
            Vector superq2(11,0.0);

            nodeContent node_c1;
            nodeContent node_c2;

            if ((!newnode->right) && (!newnode->left))
            {
                //if ((norm(newnode->right->superq)==0) && (norm(newnode->left->superq)==0))
                //{
                yDebug()<<"Computing right and left";
                splitPoints(false, newnode);
                superq1=computeMultipleSuperq(points_splitted1);
                superq2=computeMultipleSuperq(points_splitted2);

                node_c1.f_value=evaluateLoss(superq1, points_splitted1);
                node_c2.f_value=evaluateLoss(superq2, points_splitted2);

                node_c1.superq=superq1;
                node_c2.superq=superq2;
                node_c1.point_cloud= new deque<Vector>;
                node_c2.point_cloud= new deque<Vector>;

                *node_c1.point_cloud=points_splitted1;
                *node_c2.point_cloud=points_splitted2;


                superq_tree->insert(node_c1, node_c2, newnode);
               // }
            }
            else
                yDebug()<<"Skipping computation because already computed";

            computeNestedSuperq(newnode->right,i,j, first_time, count, final, count2);

        }

        if ((j==tree_splitting-1) && (count<tree_splitting))
        {
            if (count==0)
            {
                count++;
                newnode=superq_tree->root;
                yDebug()<<newnode->right->point_cloud->size();

                first_time=false;

                computeNestedSuperq(newnode->right,i,j, first_time, count, final, count2);
            }
            else if ((newnode!=NULL) && (count<tree_splitting))
            {
                Vector superq1(11,0.0);
                Vector superq2(11,0.0);

                nodeContent node_c1;
                nodeContent node_c2;

                if ((!newnode->right) && (!newnode->left))
                {
                    yDebug()<<"Computing right and left";
                    //if ((norm(newnode->right->superq)==0) && (norm(newnode->left->superq)==0))
                    //{
                        splitPoints(false, newnode);

                        superq1=computeMultipleSuperq(points_splitted1);
                        superq2=computeMultipleSuperq(points_splitted2);

                        node_c1.f_value=evaluateLoss(superq1, points_splitted1);
                        node_c2.f_value=evaluateLoss(superq2, points_splitted2);

                        node_c1.superq=superq1;
                        node_c2.superq=superq2;
                        node_c1.point_cloud= new deque<Vector>;
                        node_c2.point_cloud= new deque<Vector>;

                        *node_c1.point_cloud=points_splitted1;
                        *node_c2.point_cloud=points_splitted2;


                        superq_tree->insert(node_c1, node_c2, newnode);
                    //}
                }
                else
                    yDebug()<<"Skipping computation because already computed";

                if (count %2 !=0)
                {
                    count++;
                    computeNestedSuperq(newnode->left,i,j, first_time, count, final, count2);
                }
                else
                {
                    count++;
                    computeNestedSuperq(newnode->right,i,j, first_time, count, final, count2);
                }
            }
        }
        else if ((count==tree_splitting) && (count2<tree_splitting))
        {
            if (count2==0)
            {
                count2++;
                newnode=superq_tree->root;
                yDebug()<<newnode->right->point_cloud->size();

                first_time=false;

                computeNestedSuperq(newnode->left,i,j, first_time, count, final, count2);
            }
            else if ((newnode!=NULL) && (count2<tree_splitting))
            {
                Vector superq1(11,0.0);
                Vector superq2(11,0.0);

                nodeContent node_c1;
                nodeContent node_c2;

                if ((!newnode->right) && (!newnode->left))
                {
                    yDebug()<<"Computing right and left";
                    //if ((norm(newnode->right->superq)==0) && (norm(newnode->left->superq)==0))
                    //{
                        splitPoints(false, newnode);

                        superq1=computeMultipleSuperq(points_splitted1);
                        superq2=computeMultipleSuperq(points_splitted2);

                        node_c1.f_value=evaluateLoss(superq1, points_splitted1);
                        node_c2.f_value=evaluateLoss(superq2, points_splitted2);

                        node_c1.superq=superq1;
                        node_c2.superq=superq2;
                        node_c1.point_cloud= new deque<Vector>;
                        node_c2.point_cloud= new deque<Vector>;

                        *node_c1.point_cloud=points_splitted1;
                        *node_c2.point_cloud=points_splitted2;


                        superq_tree->insert(node_c1, node_c2, newnode);
                    //}
                }
                else
                    yDebug()<<"Skipping computation because already computed";

                if (count2 %2 !=0)
                {
                    count2++;
                    computeNestedSuperq(newnode->right,i,j, first_time, count, final, count2);
                }
                else
                {
                    count2++;
                    computeNestedSuperq(newnode->left,i,j, first_time, count, final, count2);
                }
            }
        }
        else
            yDebug()<<"stop";
    }

}

/***********************************************************************/
void SuperqComputation::splitPoints(bool merging, node *leaf)
{
    deque<Vector> points_splitted;

    points_splitted=*leaf->point_cloud;

    points_splitted1.clear();
    points_splitted2.clear();

    if (!merging)
    {
        Vector center(3,0.0);

        for (size_t k=0; k<points_splitted.size();k++)
        {
            Vector &point=points_splitted[k];
            center[0]+=point[0];
            center[1]+=point[1];
            center[2]+=point[2];
        }

        center[0]/=points_splitted.size();
        center[1]/=points_splitted.size();
        center[2]/=points_splitted.size();

        Matrix M=zeros(3,3);
        Matrix u(3,3);
        Matrix v(3,3);

        Vector s(3,0.0);
        Vector n(3,0.0);
        Vector o(3,0.0);
        Vector a(3,0.0);

        for (size_t i=0;i<points_splitted.size(); i++)
        {
            Vector &point=points_splitted[i];
            M(0,0)= M(0,0) + (point[1]-center[1])*(point[1]-center[1]) + (point[2]-center[2])*(point[2]-center[2]);
            M(0,1)= M(0,1) - (point[1]-center[1])*(point[0]-center[0]);
            M(0,2)= M(0,2) - (point[2]-center[2])*(point[0]-center[0]);
            M(1,1)= M(1,1) + (point[0]-center[0])*(point[0]-center[0]) + (point[2]-center[2])*(point[2]-center[2]);
            M(2,2)= M(2,2) + (point[1]-center[1])*(point[1]-center[1]) + (point[0]-center[0])*(point[0]-center[0]);
            M(1,2)= M(1,2) - (point[2]-center[2])*(point[1]-center[1]);
        }

        M(0,0)= M(0,0)/points_splitted.size();
        M(0,1)= M(0,1)/points_splitted.size();
        M(0,2)= M(0,2)/points_splitted.size();
        M(1,1)= M(1,1)/points_splitted.size();
        M(2,2)= M(2,2)/points_splitted.size();
        M(1,2)= M(1,2)/points_splitted.size();

        M(1,0)= M(0,1);
        M(2,0)= M(0,2);
        M(2,1)= M(1,2);

        SVDJacobi(M,u,s,v);
        n=u.getCol(2);

        Vector plane(4,0.0);

        plane[0]=n[0];
        plane[1]=n[1];
        plane[2]=n[2];
        plane[3]=(plane[0]*center[0]+plane[1]*center[1]+plane[2]*center[2]);


        leaf->plane=plane;


        for (size_t j=0; j<points_splitted.size(); j++)
        {
            Vector point=points_splitted[j];
            if (plane[0]*point[0]+plane[1]*point[1]+plane[2]*point[2]- plane[3] > 0)
                points_splitted1.push_back(point);
            else
                points_splitted2.push_back(point);
        }

        yDebug()<<"points_splitted 1 "<<points_splitted1.size();
        yDebug()<<"points_splitted 2 "<<points_splitted2.size();
    }
}

/****************************************************************/
double SuperqComputation::evaluateLoss(Vector &superq, deque<Vector> &points_splitted)
{
    double value=0.0;
    int count_subsample=0.0;
    int c;

    c=points_splitted.size()/optimizer_points;

    if (c==0)
        c=1;

    for(size_t i=0;i<points_splitted.size();i+=c)
    {
        double tmp=pow(f(superq,points_splitted[i]),superq[3])-1;
        value+=tmp*tmp;
        count_subsample++;
    }

    value/=count_subsample;
    return value;
}

/****************************************************************/
double SuperqComputation::f(Vector &x, Vector &point_cloud)
{
    Vector euler(3,0.0);
    euler[0]=x[8];
    euler[1]=x[9];
    euler[2]=x[10];
    Matrix R=euler2dcm(euler);

    double num1=R(0,0)*point_cloud[0]+R(0,1)*point_cloud[1]+R(0,2)*point_cloud[2]-x[5]*R(0,0)-x[6]*R(0,1)-x[7]*R(0,2);
    double num2=R(1,0)*point_cloud[0]+R(1,1)*point_cloud[1]+R(1,2)*point_cloud[2]-x[5]*R(1,0)-x[6]*R(1,1)-x[7]*R(1,2);
    double num3=R(2,0)*point_cloud[0]+R(2,1)*point_cloud[1]+R(2,2)*point_cloud[2]-x[5]*R(2,0)-x[6]*R(2,1)-x[7]*R(2,2);
    double tmp=pow(abs(num1/x[0]),2.0/x[4]) + pow(abs(num2/x[1]),2.0/x[4]);

    return pow( abs(tmp),x[4]/x[3]) + pow( abs(num3/x[2]),(2.0/x[3]));
}

/****************************************************************/
void SuperqComputation::mergeModeling(node *node, bool go_on)
{
    if (node!=NULL)
    {
        if (node->f_value==0)
        {
            yDebug()<<"That's root!";
            double cost_right_2=(node->right->right->f_value +
                                 node->right->left->f_value)/2.0;

            yDebug()<<"cost right "<<cost_right_2;
            yDebug()<<"node right f value"<<node->right->f_value;

            if (cost_right_2 < node->right->f_value)
            {
                mergeModeling(node->right, go_on);
            }
            else
            {
                node->right->right=NULL;
                node->right->left=NULL;
            }

            double cost_left_2=(node->left->right->f_value +
                                 node->left->left->f_value)/2.0;

            yDebug()<<"cost left "<<cost_left_2;
            yDebug()<<"node left f value"<<node->left->f_value;

            if (cost_left_2 < node->left->f_value)
                mergeModeling(node->left, go_on);
            else
            {
                node->left->right=NULL;
                node->left->left=NULL;
            }
        }
        else if (node->right!=NULL && node->left!=NULL)
        {
            cout<<endl;

            double cost_2=(node->right->f_value +
                                 node->left->f_value)/2.0;
            yDebug()<<"cost "<<cost_2;
            yDebug()<<"f_value "<<node->f_value;

            if (((node->right->right!=NULL && node->right->left!=NULL)||
                 (node->left->right!=NULL && node->left->left!=NULL)) && go_on==true)
            {
                mergeModeling(node->right, go_on);
                mergeModeling(node->left, go_on);

            }
            else if ((cost_2<node->f_value))
            {
                /***************************************/

                yDebug()<<"point cloud father size"<<node->father->point_cloud->size();
                points_splitted1.clear();
                points_splitted2.clear();

                for (size_t j=0; j<node->father->point_cloud->size(); j++)
                {
                    deque<Vector> p_tmp=*node->father->point_cloud;
                    Vector point=p_tmp[j];
                    if (node->plane[0]*point[0]+node->plane[1]*point[1]+node->plane[2]*point[2]- node->plane[3] > 0)
                        points_splitted1.push_back(point);
                    else
                        points_splitted2.push_back(point);
                }

                yDebug()<<"points_splitted 1 "<<points_splitted1.size();
                yDebug()<<"points_splitted 2 "<<points_splitted2.size();
                /***************************************/

                Vector superq1(11,0.0);
                Vector superq2(11,0.0);

                nodeContent node_c1;
                nodeContent node_c2;


                superq1=computeMultipleSuperq(points_splitted1);
                superq2=computeMultipleSuperq(points_splitted2);

                node_c1.f_value=evaluateLoss(superq1, points_splitted1);
                node_c2.f_value=evaluateLoss(superq2, points_splitted2);

                double cost_merged=(node_c1.f_value + node_c2.f_value)/2.0;
                double cost_old;
                if (node->father->right !=node)
                    cost_old = (cost_2*2 +node->father->right->f_value)/3.0;
                else
                    cost_old = (cost_2*2 +node->father->left->f_value)/3.0;

                yDebug()<<"cost merged "<<cost_merged;
                yDebug()<<"cost old "<<cost_old;

                if(cost_merged + 0.01 < cost_old)
                {
                    node_c1.superq=superq1;
                    node_c2.superq=superq2;
                    node_c1.point_cloud= new deque<Vector>;
                    node_c2.point_cloud= new deque<Vector>;

                    *node_c1.point_cloud=points_splitted1;
                    *node_c2.point_cloud=points_splitted2;

                    node->father->right=NULL;
                    node->father->left=NULL;
                    superq_tree->insert(node_c1, node_c2, node->father);

                }

                if (node->father !=superq_tree->root)
                    mergeModeling(node->father, false);
                else
                    yDebug()<<"stop 1";
            }
            else
            {
                node->right=NULL;
                node->left=NULL;
                if (node->father !=superq_tree->root)
                    mergeModeling(node->father, false);
                else
                    yDebug()<<"stop 2";
            }
            yDebug()<<"stop fuori";
        }
    }

    yDebug()<<__LINE__;
}
