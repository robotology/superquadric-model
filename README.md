# Superquadric-detection
Framework for modeling, detecting and visualizing objects through superquadrics 

## Theoretical background
The **Superquadric-detection** framework is based on the idea that _low-dimesional_, _compact_, _mathematical_ representation of objects
can provide computation and theoretical advantages in very hard problems in robotics, such as trajectory planning for exploration, grasping and approaching towards objects.
It takes inspiration from theories conceived during the 90's and 2000's (Jaklic, A., Leonardis, A., Solina, F., **Segmentation and Recovery of Superquadrics**, Ch. no.
2, Springer, 2000 (1)) since it uses **superquadric functions** as a _mathematical and low dimensional model_ for representing objects. The novelty of the superquadric-detection module consists in the implementing of the standard optimization problem provided by (1) by exploiting the **Ipopt** software pacakge (for software documentation:[Ipopt](https://projects.coin-or.org/Ipopt), for the theroetical background: A. Watcher, L.T.Biegler, **On the Implementation of an Interior-Point Filter Line-Search
Algorithm for Large-Scale Nonlinear Programming**, 2004 (2)). Furthermore, the reconstructed superquadric is overlapped in _real-time_ on the real scene seen by the robot's cameras.

## Dependencies

## Documentation
Online documentation is available here:  http://giuliavezzani.github.com/superquadric-detection [work in progress]

## Code snippets

## License
