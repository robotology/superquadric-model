#include <cmath>
#include <algorithm>
#include <sstream>
#include <set>
#include <fstream>

#include "tree.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

/***********************************************************************/
superqTree::superqTree(deque<Vector> *point_cloud)
{
    root= new node;
    root->point_cloud=point_cloud;
    root->superq.resize(11,0.0);
    root->plane.resize(4,0.0);
    root->f_value=0;
    root->left=NULL;
    root->right=NULL;
    count=0;
}

/***********************************************************************/
superqTree::~superqTree()
{
    destroy_tree();
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

        if (leaf->left==NULL)
            leaf->left=new node;

        leaf->left->f_value=f_v2;
        leaf->left->superq=node_content2.superq;
        leaf->left->plane=node_content2.plane;
        leaf->left->point_cloud=node_content2.point_cloud;
        leaf->left->left=NULL;
        leaf->left->right=NULL;
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


        if (leaf->right==NULL)
            leaf->right=new node;

        leaf->right->f_value=f_v2;
        leaf->right->superq=node_content2.superq;
        leaf->right->plane=node_content2.plane;
        leaf->right->point_cloud=node_content2.point_cloud;
        leaf->right->left=NULL;
        leaf->right->right=NULL;
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
        yDebug()<<"count"<<count;
        count++;
        yInfo()<<"Node content:";
        yInfo()<<"point_cloud size:"<<leaf->point_cloud->size();
        yInfo()<<"superquadric    :"<<leaf->superq.toString();
        yInfo()<<"plane           :"<<leaf->plane.toString();
        yInfo()<<"f value         :"<<leaf->f_value;
        yDebug()<<"print right";
        printNode(leaf->right);
        yDebug()<<"print left";
        printNode(leaf->left);       
    }
    else
        yDebug()<<"Finished";
}

/***********************************************************************/
void superqTree::printTree(node *leaf)
{
    yDebug()<<"Print tree...";

    printNode(leaf);

}

