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

#include <csignal>
#include <cmath>
#include <limits>
#include <algorithm>
#include <string>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <deque>

#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>
#include <IpReturnCodes.hpp>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

class  SuperQuadric_NLP : public Ipopt::TNLP
{
    bool bounds_automatic;
    Vector x_v;
    Vector x0;
    Matrix bounds;
    double aux_objvalue;
    Vector aux_gradf;

public:
    Vector solution;
    deque<Vector> points_downsampled;

    /****************************************************************/
    void init()
    {
        points_downsampled.clear();
        aux_objvalue=0.0;
    }

    /****************************************************************/
    void setPoints(const deque<Vector> &point_cloud, bool &mode_on, const unsigned int &optimizer_points)
    {
        if (point_cloud.size()<optimizer_points)
        {
            for (size_t i=0;i<point_cloud.size();i++)
            {
                points_downsampled.push_back(point_cloud[i]);
            }
        }
        else
        {
            int count=point_cloud.size()/optimizer_points;

            for (size_t i=0; i<point_cloud.size(); i+=count)
            {
                points_downsampled.push_back(point_cloud[i]);
            }
        }

        yInfo("points actually used for modeling: %lu ",points_downsampled.size());

        x0.resize(11,0.0);
        computeX0(x0, points_downsampled);
    }

    /****************************************************************/
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m,Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag, Ipopt::TNLP::IndexStyleEnum &index_style)
    {
        n=11;
        m=nnz_jac_g=nnz_h_lag=0;
        index_style=TNLP::C_STYLE;
        x_v.resize(n,0.0);
        aux_gradf.resize(n,0.0);

        return true;
    }

    /****************************************************************/
    void computeBounds()
    {
        if (bounds_automatic==true)
        {
            bounds(0,1)=x0[0]*2.0;
            bounds(1,1)=x0[1]*2.0;
            bounds(2,1)=x0[2]*2.0;
            bounds(0,0)=0.02;
            bounds(1,0)=0.02;
            bounds(2,0)=0.02;
            bounds(3,0)=0.1;
            bounds(4,0)=0.1;
            bounds(3,1)=1.9;
            bounds(4,1)=1.9;

            bounds(5,0)=x0[5]-bounds(0,1);
            bounds(6,0)=x0[6]-bounds(1,1);
            bounds(7,0)=x0[7]-bounds(2,1);
            bounds(5,1)=x0[5]+bounds(0,1);
            bounds(6,1)=x0[6]+bounds(1,1);
            bounds(7,1)=x0[7]+bounds(2,1);
        }

        bounds(8,0)=0;
        bounds(9,0)=0;
        bounds(10,0)=0;
        bounds(8,1)=2*M_PI;
        bounds(9,1)=M_PI;
        bounds(10,1)=2*M_PI;
    }

    /****************************************************************/
    bool get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                         Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u)
    {
        computeBounds();
        for (Ipopt::Index i=0; i<n; i++)
        {
           x_l[i]=bounds(i,0);
           x_u[i]=bounds(i,1);
        }

        return true;
    }

    /****************************************************************/
     bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number *x,
                                bool init_z, Ipopt::Number *z_L, Ipopt::Number *z_U,
                                Ipopt::Index m, bool init_lambda, Ipopt::Number *lambda)
     {
         for (Ipopt::Index i=0;i<n;i++)
         {
             x[i]=x0[i];
         }
         return true;
     }

     /****************************************************************/
     bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                    Ipopt::Number &obj_value)
     {
         F(x,points_downsampled, new_x);
         obj_value=aux_objvalue;

         return true;
     }

     /****************************************************************/
     void F(const Ipopt::Number *x, deque<Vector> &points, bool &new_x)
     {
         if (new_x)
         {
             double value=0.0;

             for(size_t i=0;i<points.size();i++)
                value+=(pow(f(x,points[i]),x[3])-1)*(pow(f(x,points[i]),x[3])-1);
             value*=x[0]*x[1]*x[2]/points.size();
             aux_objvalue=value;
         }
     }

      /****************************************************************/
     double f(const Ipopt::Number *x, Vector &point_cloud)
     {
         Vector euler(3,0.0);
         Matrix R(4,4);

         euler[0]=x[8];
         euler[1]=x[9];
         euler[2]=x[10];
         R=euler2dcm(euler);

         double num1, num2, num3;
         num1=R(0,0)*point_cloud[0]+R(0,1)*point_cloud[1]+R(0,2)*point_cloud[2]-x[5]*R(0,0)-x[6]*R(0,1)-x[7]*R(0,2);
         num2=R(1,0)*point_cloud[0]+R(1,1)*point_cloud[1]+R(1,2)*point_cloud[2]-x[5]*R(1,0)-x[6]*R(1,1)-x[7]*R(1,2);
         num3=R(2,0)*point_cloud[0]+R(2,1)*point_cloud[1]+R(2,2)*point_cloud[2]-x[5]*R(2,0)-x[6]*R(2,1)-x[7]*R(2,2);

         double tmp=pow(abs(num1/x[0]),2.0/x[4]) + pow(abs(num2/x[1]),2.0/x[4]);

         return pow( abs(tmp),x[4]/x[3]) + pow( abs(num3/x[2]),(2.0/x[3]));
     }

     /****************************************************************/
     double F_v(const Vector &x, const deque<Vector> &points)
     {
         double value=0.0;

         for (size_t i=0;i<points.size();i++)
            value+=(pow(f_v(x,points.at(i)),x[3])-1)*(pow(f_v(x,points.at(i)),x[3])-1) ;

         value*=x[0]*x[1]*x[2]/points.size();
         return value;
     }

      /****************************************************************/
     double f_v(const Vector &x, const Vector &point_cloud)
     {
         Vector euler(3,0.0);
         Matrix R(4,4);

         euler[0]=x[8];
         euler[1]=x[9];
         euler[2]=x[10];
         R=euler2dcm(euler);

         double num1, num2, num3;
         num1=R(0,0)*point_cloud[0]+R(0,1)*point_cloud[1]+R(0,2)*point_cloud[2]-x[5]*R(0,0)-x[6]*R(0,1)-x[7]*R(0,2);
         num2=R(1,0)*point_cloud[0]+R(1,1)*point_cloud[1]+R(1,2)*point_cloud[2]-x[5]*R(1,0)-x[6]*R(1,1)-x[7]*R(1,2);
         num3=R(2,0)*point_cloud[0]+R(2,1)*point_cloud[1]+R(2,2)*point_cloud[2]-x[5]*R(2,0)-x[6]*R(2,1)-x[7]*R(2,2);

         double tmp;
         tmp= pow(abs(num1/x[0]),2.0/x[4]) + pow(abs(num2/x[1]),2.0/x[4]);

         return pow( abs(tmp),x[4]/x[3]) + pow( abs(num3/x[2]),(2.0/x[3]) );
     }

     /****************************************************************/
     bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                          Ipopt::Number *grad_f)
     {
         Vector x_tmp(n,0.0);
         double grad_p, grad_n;
         double eps=1e-8;

         for (Ipopt::Index j=0;j<n;j++)
             x_tmp[j]=x[j];

         for (Ipopt::Index j=0;j<n;j++)
         {
             x_tmp[j]=x_tmp[j]+eps;

             grad_p=F_v(x_tmp,points_downsampled);

             x_tmp[j]=x_tmp[j]-eps;

             grad_n=F_v(x_tmp,points_downsampled);

             aux_gradf[j]=(grad_p-grad_n)/eps;
          }

         for (Ipopt::Index j=0;j<n;j++)
            grad_f[j]=aux_gradf[j];

         return true;
     }

     /****************************************************************/
     bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                 Ipopt::Index m, Ipopt::Number *g)
     {
         return false;
     }

     /****************************************************************/
     bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                     Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
                     Ipopt::Index *jCol, Ipopt::Number *values)
     {
         return false;
     }

    /****************************************************************/
    void configure(ResourceFinder *rf)
    {
        bounds.resize(11,2);

        bounds_automatic=(rf->check("bounds_automatic",Value("yes")).asString()=="yes");

        if (bounds_automatic==false)
            readMatrix("bounds",bounds, 11, rf);
    }

    /****************************************************************/
    void computeX0(Vector &x0, deque<Vector> &point_cloud)
    {
        x0[3]=1.0;
        x0[4]=1.0;
        x0[5]=0.0;
        x0[6]=0.0;
        x0[7]=0.0;

        for (size_t i=0; i<point_cloud.size();i++)
        {
            Vector &point=point_cloud[i];
            x0[5]+=point[0];
            x0[6]+=point[1];
            x0[7]+=point[2];
        }

        x0[5]/=point_cloud.size();
        x0[6]/=point_cloud.size();
        x0[7]/=point_cloud.size();

        computeInitialOrientation(x0,point_cloud);

        Matrix bounding_box(3,2);
        bounding_box=computeBoundingBox(point_cloud,x0);

        x0[0]=(-bounding_box(0,0)+bounding_box(0,1))/2;
        x0[1]=(-bounding_box(1,0)+bounding_box(1,1))/2;
        x0[2]=(-bounding_box(2,0)+bounding_box(2,1))/2;
    }

    /****************************************************************/
    void computeInitialOrientation(Vector &x0,deque<Vector> &point_cloud)
    {
        Matrix M=zeros(3,3);
        Matrix R(3,3);
        Matrix u(3,3);
        Matrix v(3,3);

        Vector s(3,0.0);
        Vector n(3,0.0);
        Vector o(3,0.0);
        Vector a(3,0.0);

        for (size_t i=0;i<point_cloud.size(); i++)
        {
            Vector &point=point_cloud[i];
            M(0,0)= M(0,0) + (point[1]-x0[6])*(point[1]-x0[6]) + (point[2]-x0[7])*(point[2]-x0[7]);
            M(0,1)= M(0,1) - (point[1]-x0[6])*(point[0]-x0[5]);
            M(0,2)= M(0,2) - (point[2]-x0[7])*(point[0]-x0[5]);
            M(1,1)= M(1,1) + (point[0]-x0[5])*(point[0]-x0[5]) + (point[2]-x0[7])*(point[2]-x0[7]);
            M(2,2)= M(2,2) + (point[1]-x0[6])*(point[1]-x0[6]) + (point[0]-x0[5])*(point[0]-x0[5]);
            M(1,2)= M(1,2) - (point[2]-x0[7])*(point[1]-x0[6]);
        }

        M(0,0)= M(0,0)/point_cloud.size();
        M(0,1)= M(0,1)/point_cloud.size();
        M(0,2)= M(0,2)/point_cloud.size();
        M(1,1)= M(1,1)/point_cloud.size();
        M(2,2)= M(2,2)/point_cloud.size();
        M(1,2)= M(1,2)/point_cloud.size();

        M(1,0)= M(0,1);
        M(2,0)= M(0,2);
        M(2,1)= M(1,2);

        SVDJacobi(M,u,s,v);
        n=u.getCol(0);
        o=u.getCol(1);
        a=u.getCol(2);

        R.setCol(0,n);
        R.setCol(1,o);
        R.setCol(2,a);

        x0.setSubvector(8,dcm2euler(R));
    }

    /****************************************************************/
    Matrix computeBoundingBox(deque<Vector> &points, const Vector &x0)
    {
        Matrix BB(3,2);
        Matrix R3(3,3);

        R3=euler2dcm(x0.subVector(8,10)).submatrix(0,2,0,2);

        Vector point(3,0.0);
        point=R3.transposed()*points[0];

        BB(0,0)=point[0];
        BB(1,0)=point[1];
        BB(2,0)=point[2];
        BB(0,1)=point[0];
        BB(1,1)=point[1];
        BB(2,1)=point[2];

        for (size_t i=0; i<points.size();i++)
        {
            Vector &pnt=points[i];
            point=R3.transposed()*pnt;
            if(BB(0,0)>point[0])
               BB(0,0)=point[0];

            if(BB(0,1)<point[0])
                BB(0,1)=point[0];

            if(BB(1,0)>point[1])
                BB(1,0)=point[1];

            if(BB(1,1)<point[1])
                BB(1,1)=point[1];

            if(BB(2,0)>point[2])
                BB(2,0)=point[2];

            if(BB(2,1)<point[2])
                BB(2,1)=point[2];
        }

        return BB;
    }

    /****************************************************************/
   bool readMatrix(const string &tag, Matrix &matrix, const int &dimension, ResourceFinder *rf)
   {
       string tag_x=tag+"_x";
       string tag_y=tag+"_y";
       bool check_x;

       if (tag=="x0")
       {
           if (Bottle *b=rf->find(tag.c_str()).asList())
           {
               Vector col;
               if (b->size()>=dimension)
               {
                   for(int i=0; i<b->size();i++)
                       col.push_back(b->get(i).asDouble());

                   matrix.setCol(0, col);
               }
               return true;
           }
       }
       else
       {
           if (tag=="bounds")
           {
               tag_x=tag+"_l";
               tag_y=tag+"_u";
           }

           if (Bottle *b=rf->find(tag_x.c_str()).asList())
           {
               Vector col;
               if (b->size()>=dimension)
               {
                   for(int i=0; i<b->size();i++)
                       col.push_back(b->get(i).asDouble());

                   matrix.setCol(0, col);
               }
               check_x=true;

           }
           if (Bottle *b=rf->find(tag_y.c_str()).asList())
           {
               Vector col;
               if (b->size()>=dimension)
               {
                   for (int i=0; i<b->size();i++)
                       col.push_back(b->get(i).asDouble());
                   matrix.setCol(1, col);
               }

               if (check_x==true)
                   return true;
           }
       }
       return false;
   }

   /****************************************************************/
   void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n,
                          const Ipopt::Number *x, const Ipopt::Number *z_L,
                          const Ipopt::Number *z_U, Ipopt::Index m,
                          const Ipopt::Number *g, const Ipopt::Number *lambda,
                          Ipopt::Number obj_value, const Ipopt::IpoptData *ip_data,
                          Ipopt::IpoptCalculatedQuantities *ip_cq)
   {
       solution.resize(n);
       for (Ipopt::Index i=0; i<n; i++)
           solution[i]=x[i];
   }

   /****************************************************************/
   Vector get_result() const
   {
       return solution;
   }
};

