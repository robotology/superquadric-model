# superquadric-detection
Framework for modeling, detecting and visualizing objects through superquadric functions. 

## Theoretical background
The **superquadric-detection** framework is based on the idea that _low-dimensional_, _compact_, _mathematical_ representation of objects
can provide computational and theoretical advantages in hard problems trackled in robotics, such as trajectory planning for exploration, grasping and approaching towards objects.
It takes inspiration from theories conceived during the 90's and 2000's (Jaklic, A., Leonardis, A., Solina, F., **Segmentation and Recovery of Superquadrics**, Ch. no.
2, Springer, 2000 (1)) since it uses **superquadric functions** as a _mathematical and low dimensional model_ for representing objects. The novelty of the superquadric-detection module consists in the implementing of the standard optimization problem provided by (1) by exploiting the **Ipopt** software pacakge (for software documentation: [Ipopt](https://projects.coin-or.org/Ipopt), for the theroetical background: A. Watcher, L.T.Biegler, **On the Implementation of an Interior-Point Filter Line-Search
Algorithm for Large-Scale Nonlinear Programming**, 2004 (2)). Furthermore, the reconstructed superquadric is overlapped in _real-time_ on the real scene seen by the robot's cameras.

<p> <img src=https://github.com/giuliavezzani/superquadric-detection/blob/master/img/superq-eq.jpg"/> (zoom:50%)</p>

## Dependencies
- [YARP](https://github.com/robotology/yarp)
- [iCub](https://github.com/robotology/icub-main)
- [icub-contrib-common](https://github.com/robotology/icub-contrib-common)
- [IPOPT](https://projects.coin-or.org/Ipopt)
- [OpenCV](http://opencv.org/) 

## Documentation
Online documentation is available here:  http://giuliavezzani.github.com/superquadric-detection [work in progress]

## Code snippets

## License
Material included here is Copyright of _iCub Facility - Istituto Italiano di Tecnologia_
and is released under the terms of the GPL v2.0 or later. See the file LICENSE for details.
