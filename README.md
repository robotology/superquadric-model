# superquadric-model
Framework for modeling, detecting and visualizing objects through superquadric functions.

#### Overview:
- [Theoretical background](#theoretical-background)
- [Dendenciens & how to compile](#dependencies)
- [Module structure](#module-structure)
- [Use case & results](#use-case)
- [Documentaion & more](#more-information)


## Theoretical background
The **superquadric-model** framework is based on the idea that _low-dimensional_, _compact_, _mathematical_ representation of objects
can provide computational and theoretical advantages in hard problems trackled in robotics, such as trajectory planning for exploration, grasping and approaching towards objects.
It takes inspiration from theories conceived during the 90's and 2000's (Jaklic, A., Leonardis, A., Solina, F., **Segmentation and Recovery of Superquadrics**, Ch. no.
2, Springer, 2000 (1)) since it uses **superquadric functions** as a _mathematical and low dimensional model_ for representing objects. The novelty of the superquadric-model module consists in the implemention of the standard optimization problem provided by (1) by exploiting the **Ipopt** software pacakge (for software documentation: [Ipopt](https://projects.coin-or.org/Ipopt), for theroetical background: A. Watcher, L.T.Biegler, **On the Implementation of an Interior-Point Filter Line-Search
Algorithm for Large-Scale Nonlinear Programming**, 2004 (2)). Furthermore, the reconstructed superquadric is overlapped in _real-time_ on the real scene seen by the robot's cameras.

The so called _inside-outside_ superquadric function is given by:

<img src="https://github.com/robotology/superquadric-model/blob/master/misc/superq-eq.jpg" width=486 height=97>

and it provides a simple test whether a given point lies inside or outside the superquadric. If _F < 1_, the given point _( x, y, z)_ is inside the superquadric, if _F = 1_ the corresponding point lies on the surface of the superquadric, and if _F > 1_
the point lies outside the superquadric.

The inside-outside function can be used for superquadric recovery. Suppose we have a set of _3D_ surface points, for example coming from a stereo video system, _( xi, yi, zi ), i = 1, .., n_. The superquadric in general position is defined by the following equation:

<img src="https://github.com/robotology/superquadric-model/blob/master/misc/fgen-eq.jpg" width=469 height=75>

We want to find such values for the 11 parameters _&Lambda;_ ( _&lambda;j_, _j = 1, .. 11 )_ that most of the _n_ 3D points will lay on, or close to the superquadric surface. The superquadric parameters have the following meanings:
- _&lambda;1_, _&lambda;2_, and _&lambda;3_ are the semi-axes lengths;
- _&lambda;4_ and _&lambda;5_ are the exponents, responsible for the superquadric shape;
- _&lambda;6_, _&lambda;7_, and _&lambda;8_ are the coordinates of the superquadric center;
- _&lambda;9_, _&lambda;10_, and _&lambda;11_ are the Euler angles, representing the superquadric pose.

The problem can be solved by minimizing the following quantity:

<img src="https://github.com/robotology/superquadric-model/blob/master/misc/min-eq.jpg" width=646 height=91>

The _F_ function is raised to the power of _&epsilon;1_ in order to make the error metric independent from the shape of the superquadric and provide faster convergence. This change causes a bias towards larger superquadrics. This effect is
compensated by the multiplication with the term _&lambda;1, &lambda;2, &lambda;3_ which is proportional to the volume of the superquadric.

[`Go to the top`](#superquadric-model)

## Dependencies
- [YARP](https://github.com/robotology/yarp)
- [iCub](https://github.com/robotology/icub-main)
- [icub-contrib-common](https://github.com/robotology/icub-contrib-common)
- [IPOPT](https://projects.coin-or.org/Ipopt)
- [OpenCV](http://opencv.org/)

## How to compile

In `Linux systems` code can be compiled as follows:
```
git clone https://github.com/robotology/superquadric-model.git
cd superquadric-model
mkdir build; cd build
ccmake ..
make install
```
[`Go to the top`](#superquadric-model)

## Module structure
The module structure is outlined in the following picture:

<p align="center" > <img src="https://raw.githubusercontent.com/robotology/superquadric-model/master/misc/superquadric-model-3d.png" width=600 > </p>

The superquadric-model module launches two separate threads:
- `SuperqComputation`, computing the superquadric given the 3D point cloud of the object;
- `SuperqVisualization`, showing the estimated superquadric or the points used for the computation (optional).

The superquadric-model also provides some `thrift services` through a `rpc port`. The user can communicate with the module through these services in order to ask the state of the two threads or to modify some parameters on the fly.
The module also receive the camera image and, if the `SuperqVisualization` is enabled, the output is shown on a yarpview.
The `SuperqComputation` provides two buffered ports, respectively for receiving continuosly the 3D point cloud of the object and for sending the estimated superquadric. The computation and visualization threads also share some variables, such as the computed superquadric.

In order to improve the superquadric modeling, the `SuperqComputation` thread provides **two filtering process**:

- on the 3D point cloud;
- on the estimated superquadric.

The former is a **density filter**, discarding noisy queues of the point cloud, that can arise from the noise of the disparity map.
The latter is a **median filter with an adaptive moving window**. Each filtered superquadric is obtained by applying a median filter on the last _m_ estimated superquadrics. The value of _m_ depends on the estimated velocity of the object. If the object is moving the window width is low in order to correctly track the object (e.g.` _m_=1`), otherwise _m_ is incrementally increased up to a maximum value.

[`Go to the top`](#superquadric-model)

## Use case
The superquadric-model module requires the 3D point cloud of the object we want to model with a superquadric function.
An example code for retriving this information, together with a tutorial, is provided in the folder [`tutorial`](https://github.com/robotology/superquadric-model/tree/master/tutorial) in this repository.



### Some results
Here is an example of a reconstructed superquadric:

<img src="https://github.com/robotology/superquadric-model/blob/master/misc/superq-ex.png" width=900>

The execution times are respectively:
- for **superquadric computation** nearly 0.1 s (including the median filter on the estimated superquadrics)
- for **point cloud filtereing** nearly 0.1 s (optional)
- for **visualization** nearly 0.01 s (optional)

This [video](https://www.youtube.com/watch?v=MViX4Ppo4WQ&feature=youtu.be) shows the superquadric modeling of several objects.

[`Go to the top`](#superquadric-model)

### More information
You can find an overview of the entire pipeline `superquadric-model module + example code` in the following pdf: [superquadric-model.pdf](https://github.com/robotology/superquadric-model/blob/master/misc/superquadric-model.pdf).
If you want to _browse_ the prezi version of the presentation, you can have a look at the link: [superquadric-model-prezi](https://prezi.com/zlx2l4ekonuc/superquadric-model/).

## Documentation
Online documentation is available here:  [http://robotology.github.com/superquadric-model](http://robotology.github.com/superquadric-model).

[`Go to the top`](#superquadric-model)
## License
Material included here is Copyright of _iCub Facility - Istituto Italiano di Tecnologia_
and is released under the terms of the GPL v2.0 or later. See the file LICENSE for details.

[![DOI](https://zenodo.org/badge/54477564.svg)](https://zenodo.org/badge/latestdoi/54477564)
