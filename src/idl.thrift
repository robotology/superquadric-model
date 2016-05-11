# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Giulia Vezzani
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# idl.thrift
/**
* Property
*
* IDL structure to set/show advanced parameters.
*/
struct Property
{
} (
   yarp.name = "yarp::os::Property"
   yarp.includefile="yarp/os/Property.h"
  )

/**
* superquadricDetection_IDL
*
* IDL Interface to \ref superquadric-detection services.
*/

service superquadricDetection_IDL
{
    /**
    * Set the name of the object
    * to be detected and modeled.
    * @param entry is the name of the object.
    * @return true/false is the object is known/unknown.
    */
    bool set_object_name(1:string entry);

    /**
    * Get the seed point of the object
    * without using the name object.
    * @param x_pixel is the x pixel value of the object seed point.
    * @param y_pixel is the x pixel value of the object seed point.
    * return true/false on success/failure.
    */
    bool set_seed_point(1:i32 x_pixel, 2:i32 y_pixel);

    /**
    * Return the name of the object that is
    * being detected and modeled.
    * @return the name of the object.
    */
    string get_object_name();

    /**
    * Say which method is set for getting
    * blob.
    * @return "name" if you have to type the name
    * "point" if you have to click on the camera.
    */
    string get_method();

    /**
    * Get RGB values
    * @return a list with the RGB values of the
    * superquadric and blob visualization.
    */
    list<i32> get_color();

    /**
    * Set the RGB values of the superquadric
    * and blob visualization.
    * @param r is red value in [0,255]. 
    * @param g is green value in [0,255].
    * @param b is blue value in [0,255].  
    * @return true/false on success/failure.
    */
    bool set_color(1: i32 r, 2: i32 g, 3: i32 b);

    /**
    * Get the eye used for projection of the
    * 3D points on the superquadric surface
    * to the 2D pixels.
    * @return the name of the eye used.
    */
    string get_eye();

    /**
    * Set the eye used for projection of the
    * 3D points on the superquadric surface
    * to the 2D pixels.
    * @param eye is a string "left" or "right" for selecting the left or right eye.
    * @return true/false on success/failure.
    */
    bool set_eye(1:string eye);

    /**
    * Get the maximum number of points used
    * for the superquadric reconstruction.
    * @return the maximum number of the points.
    */
    i32 get_optimizer_points();

    /**
    * Set the maximum number of points used
    * for the superquadric reconstruction.
    * @param max is the number of points for superquadric reconstruction
    * @return true/false on success/failur.
    */
    bool set_optimizer_points(1:i32 max);

    /**
    * Get max number of visualized points
    * on superquadric surface.
    * @return the number of the visualized points.
    */
    i32 get_visualized_points();

    /**
    * Set max number of visualized points
    * on superquadric surface.
    * @param vis is the number of points to be visualized.
    * @return true/false on success/failure.
    */
    bool set_visualized_points(1:i32 vis);

    /**
    * Get the parameters of the reconstructed
    * superquadric 
    * @param name of the object of which we want the superquadric
    * @return the 11 parameters (x0, .. x10) of the current superquadric. 
    * In particular, x0, x1, x2 are the three semi-axes lenghts,
    * x3 and x4 are the exponents, responsible for the superquadric shape.
    * x5, x6, x7 are the coordinate of the superquadric center and
    * x8, x9, 10 are the Euler angles, representing the superquadric orientation.
    */
    list<double> get_superq(1:string name);

    /**
    * On/off point cloud filtering
    * @param entry is "yes/no" if you want/do not want to filter points.
    * @return true/false on success/failure.
    */
    bool set_filtering(1:string entry);

    /**
    * Say if filtering is on or not.
    * @return yes/no string if filtering is on/off.
    */
    string get_filtering();

    /**
    * Set the desired tolerance value of the optimization algorithm
    * @param desired_tol is the stop tolerance of the optimization algorithm.
    * @return true/false on success/failure.
    */
    bool set_tol(1:double desired_tol);
    
    /**
    * Get the desired tolerance value of the optimization algorithm
    * @return tolerance value.
    */
    double get_tol();

    /**
    * Set the maximum time acceptable for running the optimization algorithm.
    * @param max_time is the maximum time acceptable for running the optimization algorithm.
    * @return true/false on success/failure.
    */
    bool set_max_time(1:double max_time);
    
    /**
    * Get the maximum time for running the optimization algorithm.
    * @return maximum time.
    */
    double get_max_time();

    /**
    * Get the advanced parameters of the module. The user must pay attention
    * in changing them.
    * @return the Property including all the advanced parameter values.
    */
    Property get_advanced_options();

    /**
    * Set the advanced parameters of the module. The user must pay attention
    * in changing them.
    * You can set the advanced parameters typing: 
    * command: ((filter_radius_advanced <radius-value>) (filter_nnThreshold_advanced <nnThreshold-value>))
    * @return true/false on success/failure
    */
    bool set_advanced_options(1:Property options);

    /**
    * Set what you want to show on the yarpview
    *@param plot can be: "superq", "points", "filtered-points"
    *@return true/false on success/failure
    */
    bool set_plot(1:string plot);

    /**
    * Get what is shown on the yarpview
    *@return  "superq", "points" or "filtered-points"
    */
    string get_plot();
}


