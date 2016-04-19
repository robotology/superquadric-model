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
    * @param entry is the name of the object
    * @return true/false is the object is known/unknown
    */
    bool set_object_name(1:string entry="");

    /**
    * Get the seed point of the object
    * without using the name object
    * @param x_pixel is the x pixel value of the object seed point
    * @param y_pixel is the x pixel value of the object seed point
    * return true/false on success/failure.
    */
    bool seed_point(1:i32 x_pixel, 2:i32 y_pixel);

    /**
    * Return the name of the object that is
    * being detected and modeled
    * @return the name of the object
    */
    string get_object_name();

    /**
    * Say which method is set for getting
    * blob
    * @return "name" if you have to type the name
    * "point" if you have to click on the camera
    */
    string get_method();

    /**
    * Get downsampling value
    * @return int value for the downsampling
    */
    i32 get_downsampling();

    /**
    * Set downsampling value
    * @param d is the downsampling value
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
    * Set the RGB values of the superquadric
    * and blob visualization
    * @param r is red value in [0,255] 
    * @param g is green value in [0,255]
    * @param b is blue value in [0,255]  
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
    * @param eye is a string "left" or "right" for selecting the left or right eye
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
    * @param max is the number of points for superquadric reconstruction
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
    * @param vis is the number of points to be visualized
    * @return true/false on success/failure
    */
    bool set_visualized_points(1:i32 vis)

}

