// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_superquadricDetection_IDL
#define YARP_THRIFT_GENERATOR_superquadricDetection_IDL

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <yarp/os/Property.h>

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
   * to be detected and modeled.
   * @param entry is the name of the object.
   * @return true/false is the object is known/unknown.
   */
  virtual bool set_object_name(const std::string& entry);
  /**
   * Get the seed point of the object
   * without using the name object.
   * @param x_pixel is the x pixel value of the object seed point.
   * @param y_pixel is the x pixel value of the object seed point.
   * return true/false on success/failure.
   */
  virtual bool set_seed_point(const int32_t x_pixel, const int32_t y_pixel);
  /**
   * Return the name of the object that is
   * being detected and modeled.
   * @return the name of the object.
   */
  virtual std::string get_object_name();
  /**
   * Say which method is set for getting
   * blob.
   * @return "name" if you have to type the name
   * "point" if you have to click on the camera.
   */
  virtual std::string get_method();
  /**
   * Get RGB values
   * @return a list with the RGB values of the
   * superquadric and blob visualization.
   */
  virtual std::vector<int32_t>  get_color();
  /**
   * Set the RGB values of the superquadric
   * and blob visualization.
   * @param r is red value in [0,255].
   * @param g is green value in [0,255].
   * @param b is blue value in [0,255].
   * @return true/false on success/failure.
   */
  virtual bool set_color(const int32_t r, const int32_t g, const int32_t b);
  /**
   * Get the eye used for projection of the
   * 3D points on the superquadric surface
   * to the 2D pixels.
   * @return the name of the eye used.
   */
  virtual std::string get_eye();
  /**
   * Set the eye used for projection of the
   * 3D points on the superquadric surface
   * to the 2D pixels.
   * @param eye is a string "left" or "right" for selecting the left or right eye.
   * @return true/false on success/failure.
   */
  virtual bool set_eye(const std::string& eye);
  /**
   * Get the maximum number of points used
   * for the superquadric reconstruction.
   * @return the maximum number of the points.
   */
  virtual int32_t get_optimizer_points();
  /**
   * Set the maximum number of points used
   * for the superquadric reconstruction.
   * @param max is the number of points for superquadric reconstruction
   * @return true/false on success/failur.
   */
  virtual bool set_optimizer_points(const int32_t max);
  /**
   * Get max number of visualized points
   * on superquadric surface.
   * @return the number of the visualized points.
   */
  virtual int32_t get_visualized_points();
  /**
   * Set max number of visualized points
   * on superquadric surface.
   * @param vis is the number of points to be visualized.
   * @return true/false on success/failure.
   */
  virtual bool set_visualized_points(const int32_t vis);
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
  virtual std::vector<double>  get_superq(const std::string& name);
  /**
   * On/off point cloud filtering
   * @param entry is "on/off" if you want/do not want to filter points.
   * @return true/false on success/failure.
   */
  virtual bool set_filtering(const std::string& entry);
  /**
   * Say if filtering is on or not.
   * @return on/off string if filtering is on/off.
   */
  virtual std::string get_filtering();
  /**
   * Set the desired tolerance value of the optimization algorithm
   * @param desired_tol is the stop tolerance of the optimization algorithm.
   * @return true/false on success/failure.
   */
  virtual bool set_tol(const double desired_tol);
  /**
   * Get the desired tolerance value of the optimization algorithm
   * @return tolerance value.
   */
  virtual double get_tol();
  /**
   * Set the maximum time acceptable for running the optimization algorithm.
   * @param max_time is the maximum time acceptable for running the optimization algorithm.
   * @return true/false on success/failure.
   */
  virtual bool set_max_time(const double max_time);
  /**
   * Get the maximum time for running the optimization algorithm.
   * @return maximum time.
   */
  virtual double get_max_time();
  /**
   * Get the advanced parameters of the module. The user must pay attention
   * in changing them.
   * @return the Property including all the advanced parameter values.
   */
  virtual yarp::os::Property get_advanced_options();
  /**
   * Set the advanced parameters of the module. The user must pay attention
   * in changing them.
   * You can set the advanced parameters typing:
   * command: ((filter_radius_advanced <radius-value>) (filter_nnThreshold_advanced <nnThreshold-value>))
   * @return true/false on success/failure
   */
  virtual bool set_advanced_options(const yarp::os::Property& options);
  /**
   *  Set what you want to show on the yarpview
   * @param plot can be: "superq", "points", "filtered-points"
   * @return true/false on success/failure
   */
  virtual bool set_plot(const std::string& plot);
  /**
   *  Get what is shown on the yarpview
   * @return  "superq", "points" or "filtered-points"
   */
  virtual std::string get_plot();
  /**
   * Set the step used to downsample the points to be show
   * @param step must be a positive value
   * @return true/false on success/failure
   */
  virtual bool set_visualized_points_step(const int32_t step);
  /**
   * Get the step used to downsample the points to be show
   * @return the step value
   */
  virtual int32_t get_visualized_points_step();
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
