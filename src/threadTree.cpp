
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

#include "threadTree.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;


/***********************************************************************/
ThreadTree::ThreadTree()
{
}

bool ThreadTree::threadInit(superqTree *_tree, int _tree_splitting)
{
    tree=_tree;
    tree_splitting=_tree_splitting;
    i=0;

    return true;
}

/***********************************************************************/
bool ThreadTree::threadRelease()
{
    return true;
}

/***********************************************************************/
bool ThreadTree::run(node *newnode)
{
    if ((newnode!=NULL) && (i<tree_splitting))
    {
        Vector superq1(11,0.0);
        Vector superq2(11,0.0);
        deque<Vector> points_splitted1, points_splitted2;

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

        run(newnode->left);
        run(newnode->right);

        return true;
    }
    else
        return false;
}


