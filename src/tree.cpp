#include <cmath>
#include <algorithm>
#include <sstream>
#include <set>
#include <fstream>

#include <yarp/math/Math.h>

#include "tree.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

/***********************************************************************/
superqTree::superqTree()
{
    root= new node;
    root->superq.resize(11,0.0);
    root->plane.resize(4,0.0);
    root->f_penalized=0;
    root->left=NULL;
    root->right=NULL;
    root->height=1;
    root->axis_x.resize(3,0.0);
    root->axis_y.resize(3,0.0);
    root->axis_z.resize(3,0.0);
    root->R;
    root->plane_important=false;
}

/***********************************************************************/
superqTree::~superqTree()
{
    destroy_tree();
}

/***********************************************************************/
void superqTree::setPoints(deque<Vector> *point_cloud)
{
    root->point_cloud=point_cloud;
}

/***********************************************************************/
void superqTree::destroy_tree()
{
    destroy_tree(root);
}

/***********************************************************************/
void superqTree::destroy_tree(node *leaf)
{
    if (leaf!=NULL)
    {
        destroy_tree(leaf->left);
        destroy_tree(leaf->right);
        delete leaf;
    }
}

/***********************************************************************/
void superqTree::insert(nodeContent &node_content1, nodeContent &node_content2,  node *leaf)
{
    double f_v1=node_content1.f_value;
    double f_v2=node_content2.f_value;

    if (f_v1 < f_v2)
    {
        if (leaf->right==NULL)
            leaf->right=new node;

        leaf->right->f_value=f_v1;
        leaf->right->superq=node_content1.superq;
        leaf->right->plane=node_content1.plane;
        leaf->right->point_cloud=node_content1.point_cloud;
        leaf->right->left=NULL;
        leaf->right->right=NULL;
        leaf->right->father=leaf;
        leaf->right->height=node_content1.height;
        leaf->right->plane_important=false;

        if (leaf->left==NULL)
            leaf->left=new node;


        leaf->left->f_value=f_v2;
        leaf->left->superq=node_content2.superq;
        leaf->left->plane=node_content2.plane;
        leaf->left->point_cloud=node_content2.point_cloud;
        leaf->left->left=NULL;
        leaf->left->right=NULL;
        leaf->left->father=leaf;
        leaf->left->height=node_content1.height;
        leaf->left->plane_important=false;


        yDebug()<<"First case";
    }
    else
    {
        if (leaf->left==NULL)
            leaf->left=new node;

        leaf->left->f_value=f_v1;
        leaf->left->superq=node_content1.superq;
        leaf->left->plane=node_content1.plane;
        leaf->left->point_cloud=node_content1.point_cloud;
        leaf->left->left=NULL;
        leaf->left->right=NULL;
        leaf->left->father=leaf;
        leaf->left->height=node_content1.height;
        leaf->left->plane_important=false;


        if (leaf->right==NULL)
            leaf->right=new node;

        leaf->right->f_value=f_v2;
        leaf->right->superq=node_content2.superq;
        leaf->right->plane=node_content2.plane;
        leaf->right->point_cloud=node_content2.point_cloud;
        leaf->right->left=NULL;
        leaf->right->right=NULL;
        leaf->right->father=leaf;
        leaf->right->height=node_content1.height;
        leaf->right->plane_important=false;
    }
}

/***********************************************************************/
node *superqTree::search(double f_value, node *leaf)
{
    if(leaf!=NULL)
    {
        if(f_value==leaf->f_value)
          return leaf;
        if(f_value<leaf->f_value)
          return search(f_value, leaf->left);
        else
          return search(f_value, leaf->right);
    }
    else
        return NULL;
}

/***********************************************************************/
node *superqTree::search(double f_value)
{
    return search(f_value, root);
}

/***********************************************************************/
void superqTree::printNode(node *leaf)
{
    if(leaf!=NULL)
    {
        //yInfo()<<"Node content:";
        //yInfo()<<"point_cloud size:"<<leaf->point_cloud->size();
        //yInfo()<<"superquadric    :"<<leaf->superq.toString();
        //yInfo()<<"plane           :"<<leaf->plane.toString();
        //yInfo()<<"f value         :"<<leaf->f_value;
        //yDebug()<<"print right";
        if (leaf->right!=NULL)
            printNode(leaf->right);

        //Debug()<<"print left";
        if (leaf->left!=NULL)
            printNode(leaf->left);
    }
    else
        yDebug()<<"Finished";
}

/***********************************************************************/
void superqTree::printTree(node *leaf)
{
    //yDebug()<<"Print tree...";

    printNode(leaf);

}


/***********************************************************************/
void superqTree::saveNode(ofstream &fout, node *leaf)
{
    if(leaf!=NULL)
    {
        fout<<"Superquadric "<<endl;
        fout<<" "<<leaf->superq.subVector(0,7).toString(3,3);
        fout<<dcm2axis(euler2dcm(leaf->superq.subVector(8,10))).toString()<<endl;
        fout<<"Point cloud size "<<endl;
        fout<<" "<<leaf->point_cloud->size()<<endl;
        fout<<"Plane"<<endl;
        fout<<" "<<leaf->plane.toString(3,3)<<endl;
        fout<<"Finale value"<<endl;
        fout<<" "<<leaf->f_value<<endl;
        fout<<endl;

        if (leaf->right!=NULL)
            saveNode(fout, leaf->right);

        if (leaf->left!=NULL)
            saveNode(fout, leaf->left);
    }
    else
        yDebug()<<"Finished";
}

/***********************************************************************/
void superqTree::saveTree(ofstream &fout, node *leaf)
{
    yDebug()<<"Save tree...";

    saveNode(fout, leaf);

}

