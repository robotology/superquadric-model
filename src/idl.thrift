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
* superquadricModel_IDL
*
* IDL Interface to \ref superquadric-model services.
*/

service superquadricModel_IDL
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
    Property get_superq(1:string name, 2:bool filtered_or_not);

    /**
    * On/off point cloud filtering
    * @param entry is "on/off" if you want/do not want to filter points.
    * @return true/false on success/failure.
    */
    bool set_filtering(1:string entry);

    /**
    * Say if filtering is on or not.
    * @return on/off string if filtering is on/off.
    */
    string get_filtering();

    /**
    * On/off superquadric filtering
    * @param entry is "on/off" if you want/do not want to filter points.
    * @return true/false on success/failure.
    */
    bool set_filtering_superq(1:string entry);

    /**
    * Say if superquadric filtering is on or not.
    * @return on/off string if filtering is on/off.
    */
    string get_filtering_superq();

    /**
    * Set if you want to save the acquired point cloud
    *@param entry can be: "on" or "off"
    *@return true/false on success/failure
    */
    bool set_save_points(1:string entry);

    /**
    * Set if you are saving the acquired point cloud
    *@return  "on" or "off"
    */
    string get_save_points();

    /**
    * Get the advanced parameters of the module. The user must pay attention
    * in changing them.
    * @return the Property including all the advanced parameter values.
    */
    Property get_advanced_options(1: string field);

    /**
    * Set the advanced parameters of the module. The user must pay attention
    * in changing them.
    * You can set the advanced parameters typing: 
    * command: ((filter_radius_advanced <radius-value>) (filter_nnThreshold_advanced <nnThreshold-value>))
    * @return true/false on success/failure
    */
    bool set_advanced_options(1:Property options, 2: string field);

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

    /**
    * Set the step used to downsample the points to be show
    * @param step must be a positive value
    * @return true/false on success/failure
    */
    bool set_visualized_points_step(1:i32 step);

    /** Get the step used to downsample the points to be show
    * @return the step value
    */
    i32 get_visualized_points_step();

    /**
    * Set fixed_window value on or off
    *@param entry can be: "yes", "no"
    *@return true
    */
    bool set_fixed_window(1:string entry);

    /**
    * Get if the window value is fixed or not
    *@return  true/false
    */
    bool get_fixed_window();

    /**
    * Set if the one-shot mode must be enabled
    *@param entry can be "on" or "off"
    *@return  true/false
    */
    bool set_one_shot_mode(1:string entry);

    /**
    * Get if the one-shot mode is enabled
    *@return  true/false
    */
    bool get_one_shot_mode();

    /**
    * Set if visualization is enabled
    *@return  true/false
    */
    bool set_visualization_on(1:string e)

    /**
    * Get if visualization is enabled
    *@return  true/false
    */
    string get_visualization_on()



}


