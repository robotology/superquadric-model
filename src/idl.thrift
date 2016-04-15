# Copyright: (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Giulia Vezzani
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# idl.thrift
/**
* superquadricDetection_IDL
*
* IDL Interface to \ref superquadric-detection services.
*/

service superquadricDetection_IDL
{
    /**
    * Set the name of the object
    * to be detected and modeled
    * @param name of the object
    * @return true/false is the object is known/unknown
    */
    bool object_name(1:string entry="");

    /**
    * Say to wait for a click on the
    * camera, without using the name object
    * return true/false on success/failure.
    */
    bool seed_point();

    /**
    * Return the name of the object that is
    * being detected and modeled
    * @return the name of the object
    */
    string which_object();

    /**
    * Say which method is set for getting
    * blob
    * @return "name" if you have to type the name
    * "point" if you have to click on the camera
    */
    string name_or_not();

    /**
    * Get downsampling value
    * @return int value for the downsampling
    */
    i32 get_downsampling();

    /**
    * Set downsampling value
    * @return true/false for success/failure
    */
    bool set_downsampling(1: i32 d);

    /**
    * Get RGB values
    * @return a list with the RGB values of the
    * superquadric and blob visualization
    */
    list<i32> get_rgb()

    /**
    * Set RGB values
    * @return set the RGB values of the superquadric
    * and blob visualization
    * @return true/false on success/failure
    */
    bool set_rgb(1:i32 r, 2:i32 g, 3:i32 b)

    /**
    * Get the eye used for projection of the
    * 3D points on the superquadric surface
    * to the 2D pixels
    * @return the name of the eye used
    */
    string get_eye()

    /**
    * Set the eye used for projection of the
    * 3D points on the superquadric surface
    * to the 2D pixels
    * @return true/false on success/failure
    */
    bool set_eye(1:string eye="")

    /**
    * Get the maximum number of points used
    * for the superquadric reconstruction
    * @return the maximum number of the points
    */
    i32 get_max_num_points()

    /**
    * Set the maximum number of points used
    * for the superquadric reconstruction
    * @return true/false on success/failure
    */
    bool set_max_num_points(1:i32 max)

    /**
    * Get max number of visualized points
    * on superquadric surface
    * @return the number of the visualized points
    */
    i32 get_visualized_points()

    /**
    * Set max number of visualized points
    * on superquadric surface
    * @return true/false on success/failure
    */
    bool set_visualized_points(1:i32 vis)

}

