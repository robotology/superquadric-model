
#ifndef __TREE_H__
#define __TREE_H__

#include <string>
#include <deque>
#include <map>
#include <fstream>

#include <yarp/dev/all.h>

struct node
{
    std::deque<yarp::sig::Vector> *point_cloud;
    yarp::sig::Vector superq;
    yarp::sig::Vector plane;   
    node *left;
    node *right;
    node *father;
    double f_penalized;
    double f_value;
    int height;
    yarp::sig::Vector axis_x;
    yarp::sig::Vector axis_y;
    yarp::sig::Vector axis_z;
    yarp::sig::Matrix R;
    bool plane_important;

};

struct nodeContent
{
    std::deque<yarp::sig::Vector> *point_cloud;
    yarp::sig::Vector superq;
    yarp::sig::Vector plane;
    double f_value;
    int height;
    bool plane_important;
};


class superqTree
{
    /***********************************************************************/
    void destroy_tree(node *leaf);

    /***********************************************************************/
    node *search(double f_value, node *leaf);

public:

    int count;

    node *root;

    /***********************************************************************/
    superqTree();

    /***********************************************************************/
    void setPoints(std::deque<yarp::sig::Vector> *point_cloud);

    /***********************************************************************/
    ~superqTree();

    /***********************************************************************/
    node *search(double f_value);

    /***********************************************************************/
    void destroy_tree();

    /***********************************************************************/
    void printNode(node *leaf);

    /***********************************************************************/
    void printTree(node *leaf);

    /***********************************************************************/
    void saveNode(std::ofstream &fout,node *leaf);

    /***********************************************************************/
    void saveTree(std::ofstream &fout, node *leaf);

    /***********************************************************************/
    void insert(nodeContent &node_content1, nodeContent &node_content2, node *leaf);
};

#endif
