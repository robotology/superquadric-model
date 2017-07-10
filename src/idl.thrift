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

struct Vector
{
} (
   yarp.name = "yarp::sig::Vector"
   yarp.includefile="yarp/sig/Vector.h"
  )

struct Bottle
{
} (
   yarp.name = "yarp::os::Bottle"
   yarp.includefile="yarp/os/Bottle.h"
  )

/**
* superquadricModel_IDL
*
* IDL Interface to \ref superquadric-model services.
*/

service superquadricModel_IDL
{
    /**
    * Set the tag for storing files.
    * @param entry is the tag that will be used in file names.
    * @return true.
    */
    bool set_tag_file(1:string entry);

    /**
    * Return the tag used for storing files.
    * @return the tag name.
    */
    string get_tag_file();

    /**
    * Get the parameters of the reconstructed superquadric.
    * @param point_cloud is the 3D point cloud of the object we want to model with the superquadric, 
    * for instance: ((100.0 102.0) (100.0 103.0) ... ).
    * @return the 12 parameters (x0, .. x11) of the current superquadric. 
    * In particular, the parameters are grouped in a Property as follows: "dimensions" (x0, x1, x2) 
    * are the three semi-axes lenghts; "exponents" (x3 and x4) are the exponents,
    * responsible for the superquadric shape; "center"(x5, x6, x7) contains the coordinate of
    * the superquadric center; and "orientation" (x8, x9, 10, x11) is the axis-angle representation
    * obtained from the Euler angles.
    */
    Property get_superq(1:list<Vector> point_cloud);

     /**
    * Get the parameters of the reconstructed and filtered superquadric.
    * In particular, the parameters are grouped in a Property as follows: "dimensions" (x0, x1, x2) 
    * are the three semi-axes lenghts; "exponents" (x3 and x4) are the exponents,
    * responsible for the superquadric shape; "center"(x5, x6, x7) contains the coordinate of
    * the superquadric center; and "orientation" (x8, x9, 10, x11) is the axis-angle representation
    * obtained from the Euler angles.
    */
    Property get_superq_filtered();

    /*
    *Reset the median filter (useful when the object changes)
    */    
    bool reset_filter();

    /*
    *Send point cloud and execute one superquadric reconstruction (useful for filtering with one-shot mode)
    */    
    bool send_point_clouds(1:list<Vector> point_cloud);

    /**
    * On/off point cloud filtering.
    * @param entry is "on/off" if you want/do not want to filter points.
    * @return true/false on success/failure.
    */
    bool set_points_filtering(1:string entry);

    /**
    * Say if points filtering is on or not.
    * @return on/off string if points filtering is on/off.
    */
    string get_points_filtering();

    /**
    * On/off superquadric filtering.
    * @param entry is "on/off" if you want/do not want to filter the estimated superquadric.
    * @return true/false on success/failure.
    */
    bool set_superq_filtering(1:string entry);

    /**
    * Say if superquadric filtering is on or not.
    * @return on/off string if superquadeic filtering is on/off.
    */
    string get_superq_filtering();

    /**
    * Set if you want to save the acquired point cloud.
    *@param entry can be: "on" or "off".
    *@return true/false on success/failure.
    */
    bool set_save_points(1:string entry);

    /**
    * Set if you are saving the acquired point cloud.
    *@return  "on" or "off".
    */
    string get_save_points();

    /**
    * Get the  parameters of the module. The user must pay attention
    * in changing them.
    * @param field can be "points_filter", "superq_filter", "optimization", "visualization" or "statistics".
    * depending on which parameters we are interested in.
    * @return the Property including all the  parameter values.
    */
    Property get_options(1: string field);

    /**
    * Set the  parameters of the module. The user must pay attention
    * in changing them.
    * @param options is a Property containing the parameters the user want to change.
    * @param field is a string specifying which can of parameter we are going to change.
    * Field can be: "points_filter", "superq_filter", "optimization" or "visualization".
    * You can set the  parameters typing: 
    * command:  set_options ((filter_radius <radius-value>) (filter_nnThreshold <nnThreshold-value>)) points_filter.
    * @return true/false on success/failure.
    */
    bool set_options(1:Property options, 2: string field);

    /**
    * Set if the visualization has to be enabled.
    *@return  true/false on success/failure.
    */
    bool set_visualization(1:string e)

    /**
    * Get if visualization is enabled.
    *@return "on" or "off".
    */
    string get_visualization()

    /**
    * Set object class for speeding up superquadric reconstruction
    *@param a string with the name of the class
    *@return return true
    */
    bool set_object_class(1:string objclass)



}


