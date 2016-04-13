# superquadric-detection
Framework for modeling, detecting and visualizing objects through superquadric functions. 

## Theoretical background
The **superquadric-detection** framework is based on the idea that _low-dimensional_, _compact_, _mathematical_ representation of objects
can provide computational and theoretical advantages in hard problems trackled in robotics, such as trajectory planning for exploration, grasping and approaching towards objects.
It takes inspiration from theories conceived during the 90's and 2000's (Jaklic, A., Leonardis, A., Solina, F., **Segmentation and Recovery of Superquadrics**, Ch. no.
2, Springer, 2000 (1)) since it uses **superquadric functions** as a _mathematical and low dimensional model_ for representing objects. The novelty of the superquadric-detection module consists in the implementing of the standard optimization problem provided by (1) by exploiting the **Ipopt** software pacakge (for software documentation: [Ipopt](https://projects.coin-or.org/Ipopt), for the theroetical background: A. Watcher, L.T.Biegler, **On the Implementation of an Interior-Point Filter Line-Search
Algorithm for Large-Scale Nonlinear Programming**, 2004 (2)). Furthermore, the reconstructed superquadric is overlapped in _real-time_ on the real scene seen by the robot's cameras.

The so called _inside-outside_ superquadric function is given by:

<img src="https://github.com/giuliavezzani/superquadric-detection/blob/master/img/superq-eq.jpg" width=486 height=97> 

and it provides a simple test whether a given point lies inside or outside the superquadric. If _F < 1_, the given point _( x, y, z)_ is inside the superquadric, if _F = 1_ the corresponding point lies on the surface of the superquadric, and if _F > 1_
the point lies outside the superquadric.

The inside-outside function can be used for superquadric recovery. Suppose we have a set of _3D_ surface points, for example coming from a stereo video system, _( xi, yi, zi ), i = 1, .., n_. The superquadric in general position is defined by the following equation:

We want to find such values for the 11 parameters _&Lambda;_ ( _&lambda;j_, _j = 1, .. 11 )_ that most of the _n_ 3D points will lay on, or close to the superquadric surface.
The problem can be solved by minimizing the following quantity:

<img src="https://github.com/giuliavezzani/superquadric-detection/blob/master/img/min-eq.jpg" width=646 height=91> 

The _F_ function is raised to the power of _&epsilon;1_ in order to make the error metric independent from the shape of the superquadric and provide faster convergence. This change causes a bias towards larger superquadrics. This effect is 
compensated by the multiplication with the term _&lambda;1, &lambda;2, &lambda;3_ which is proportional to the volume of the superquadric.



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
