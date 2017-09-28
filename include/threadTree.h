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

#ifndef __TREETHREAD_H__
#define __TREETHREAD_H__

#include <yarp/os/all.h>
#include "tree.h"


/*******************************************************************************/
class ThreadTree
{
public:

    int i;
    superqTree *tree;
    int tree_splitting;

    /*******************************************************************************/
    bool threadInit(superqTree *_tree, int _tree_splitting);

    /*******************************************************************************/
    bool threadRelease();

    /*******************************************************************************/
    bool run(node *newnode);

};



#endif
