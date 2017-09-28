
#ifndef __TREE_H__
#define __TREE_H__

#include <string>
#include <deque>
#include <map>

#include <yarp/dev/all.h>

struct node
{
    std::deque<yarp::sig::Vector> *point_cloud;
    yarp::sig::Vector superq;
    yarp::sig::Vector plane;
    double f_value;
    node *left;
    node *right;
};

struct nodeContent
{
    std::deque<yarp::sig::Vector> *point_cloud;
    yarp::sig::Vector superq;
    yarp::sig::Vector plane;
    double f_value;
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
    superqTree(std::deque<yarp::sig::Vector> *point_cloud);

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
    void insert(nodeContent &node_content1, nodeContent &node_content2, node *leaf);
};

#endif
