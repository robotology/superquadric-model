// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_superquadricDetection_IDL
#define YARP_THRIFT_GENERATOR_superquadricDetection_IDL

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class superquadricDetection_IDL;


/**
 * superquadricDetection_IDL
 * IDL Interface to \ref superquadric-detection services.
 */
class superquadricDetection_IDL : public yarp::os::Wire {
public:
  superquadricDetection_IDL();
  /**
   * Set the name of the object
   * to be detected and modeled
   * @param name of the object
   * @return true/false is the object is known/unknown
   */
  virtual bool object_name(const std::string& entry = "");
  /**
   * Say to wait for a click on the
   * camera, without using the name object
   * return true/false on success/failure.
   */
  virtual bool seed_point();
  /**
   * Return the name of the object that is
   * being detected and modeled
   * @return the name of the object
   */
  virtual std::string which_object();
  /**
   * Say which method is set for getting
   * blob
   * @return "name" if you have to type the name
   * "point" if you have to click on the camera
   */
  virtual std::string name_or_not();
  /**
   * Get downsampling value
   * @return int value for the downsampling
   */
  virtual int32_t get_downsampling();
  /**
   * Set downsampling value
   * @return true/false for success/failure
   */
  virtual bool set_downsampling(const int32_t d);
  /**
   * Get RGB values
   * @return a list with the RGB values of the
   * superquadric and blob visualization
   */
  virtual std::vector<int32_t>  get_rgb();
  /**
   * Set RGB values
   * @return set the RGB values of the superquadric
   * and blob visualization
   * @return true/false on success/failure
   */
  virtual bool set_rgb(const int32_t r, const int32_t g, const int32_t b);
  /**
   * Get the eye used for projection of the
   * 3D points on the superquadric surface
   * to the 2D pixels
   * @return the name of the eye used
   */
  virtual std::string get_eye();
  /**
   * Set the eye used for projection of the
   * 3D points on the superquadric surface
   * to the 2D pixels
   * @return true/false on success/failure
   */
  virtual bool set_eye(const std::string& eye = "");
  /**
   * Get the maximum number of points used
   * for the superquadric reconstruction
   * @return the maximum number of the points
   */
  virtual int32_t get_max_num_points();
  /**
   * Set the maximum number of points used
   * for the superquadric reconstruction
   * @return true/false on success/failure
   */
  virtual bool set_max_num_points(const int32_t max);
  /**
   * Get max number of visualized points
   * on superquadric surface
   * @return the number of the visualized points
   */
  virtual int32_t get_visualized_points();
  /**
   * Set max number of visualized points
   * on superquadric surface
   * @return true/false on success/failure
   */
  virtual bool set_visualized_points(const int32_t vis);
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
