# superquadric-detection
Framework for modeling, detecting and visualizing objects through superquadric functions.

## Theoretical background
The **superquadric-detection** framework is based on the idea that _low-dimensional_, _compact_, _mathematical_ representation of objects
can provide computational and theoretical advantages in hard problems trackled in robotics, such as trajectory planning for exploration, grasping and approaching towards objects.
It takes inspiration from theories conceived during the 90's and 2000's (Jaklic, A., Leonardis, A., Solina, F., **Segmentation and Recovery of Superquadrics**, Ch. no.
2, Springer, 2000 (1)) since it uses **superquadric functions** as a _mathematical and low dimensional model_ for representing objects. The novelty of the superquadric-detection module consists in the implemention of the standard optimization problem provided by (1) by exploiting the **Ipopt** software pacakge (for software documentation: [Ipopt](https://projects.coin-or.org/Ipopt), for theroetical background: A. Watcher, L.T.Biegler, **On the Implementation of an Interior-Point Filter Line-Search
Algorithm for Large-Scale Nonlinear Programming**, 2004 (2)). Furthermore, the reconstructed superquadric is overlapped in _real-time_ on the real scene seen by the robot's cameras.

The so called _inside-outside_ superquadric function is given by:

<img src="https://github.com/giuliavezzani/superquadric-detection/blob/master/misc/superq-eq.jpg" width=486 height=97>

and it provides a simple test whether a given point lies inside or outside the superquadric. If _F < 1_, the given point _( x, y, z)_ is inside the superquadric, if _F = 1_ the corresponding point lies on the surface of the superquadric, and if _F > 1_
the point lies outside the superquadric.

The inside-outside function can be used for superquadric recovery. Suppose we have a set of _3D_ surface points, for example coming from a stereo video system, _( xi, yi, zi ), i = 1, .., n_. The superquadric in general position is defined by the following equation:

<img src="https://github.com/giuliavezzani/superquadric-detection/blob/master/misc/fgen-eq.jpg" width=469 height=75>

We want to find such values for the 11 parameters _&Lambda;_ ( _&lambda;j_, _j = 1, .. 11 )_ that most of the _n_ 3D points will lay on, or close to the superquadric surface. The superquadric parameters have the following meanings:
- _&lambda;1_, _&lambda;2_, and _&lambda;3_ are the semi-axes lengths;
- _&lambda;4_ and _&lambda;5_ are the exponents, responsible for the superquadric shape;
- _&lambda;6_, _&lambda;7_, and _&lambda;8_ are the coordinates of the superquadric center;
- _&lambda;9_, _&lambda;10_, and _&lambda;11_ are the Euler angles, representing the superquadric pose.

The problem can be solved by minimizing the following quantity:

<img src="https://github.com/giuliavezzani/superquadric-detection/blob/master/misc/min-eq.jpg" width=646 height=91>

The _F_ function is raised to the power of _&epsilon;1_ in order to make the error metric independent from the shape of the superquadric and provide faster convergence. This change causes a bias towards larger superquadrics. This effect is
compensated by the multiplication with the term _&lambda;1, &lambda;2, &lambda;3_ which is proportional to the volume of the superquadric.

## Module description

You can find an overview on superquadric-detection module in the following pdf: [superquadric-detection.pdf](https://github.com/giuliavezzani/superquadric-detection/blob/master/misc/superquadric-detection.pdf). 
If you want to _browse_ the prezi version of the presentation, you can have a look at the link: [superquadric-detection-prezi](https://prezi.com/zlx2l4ekonuc/superquadric-detection/).

## Dependencies
- [YARP](https://github.com/robotology/yarp)
- [iCub](https://github.com/robotology/icub-main)
- [icub-contrib-common](https://github.com/robotology/icub-contrib-common)
- [IPOPT](https://projects.coin-or.org/Ipopt)
- [OpenCV](http://opencv.org/)
- [IOL](https://github.com/robotology/iol)
  

## Module pipeline
There are <b>two different modes</b> for running the module:
 1.  <b>online</b> mode, if you want the object to be detected and modeled in _real-time_;
 2.  <b>one-shot</b> mode, if you want to model the object described by a point cloud in a _.off file_.


In <b>online</b> mode you can <b>select the object</b> to be detected and modeled by sending through the rpc service:
- the  <b>object seed-point</b>, i.e. a 2D point ( _(x, y)_ pixel) representing the center of the object blob.
- or the <b>object name</b> that is associated to the object and stored in the robot memory.

Both in <b>online</b> and <b>one-shot</b> mode, the 3D points can be <b>filtered</b> in order to remove possible outliers. The filter can be enabled by the users with a specific parameter.

The module pipeline in <b>online</b> mode is the following:


<img src="https://github.com/giuliavezzani/superquadric-detection/blob/master/misc/pipeline2.png" width=1011 height=297>



The seed-point/object name is used to get the 2D object blob. Then, the 2D pixels are converted in 3D points. The 3D points can be filtered (if the filter_on option is set on). Then, the superquadric is computed and shown in the image coming from the camera, overlapped on the object.


## Documentation
Online documentation is available here:  [http://giuliavezzani.github.com/superquadric-detection](http://giuliavezzani.github.com/superquadric-detection).

## License
Material included here is Copyright of _iCub Facility - Istituto Italiano di Tecnologia_
and is released under the terms of the GPL v2.0 or later. See the file LICENSE for details.
