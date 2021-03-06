// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_superquadricModel_IDL
#define YARP_THRIFT_GENERATOR_superquadricModel_IDL

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>

class superquadricModel_IDL;


/**
 * superquadricModel_IDL
 * IDL Interface to \ref superquadric-model services.
 */
class superquadricModel_IDL : public yarp::os::Wire {
public:
  superquadricModel_IDL();
  /**
   * Set the tag for storing files.
   * @param entry is the tag that will be used in file names.
   * @return true.
   */
  virtual bool set_tag_file(const std::string& entry);
  /**
   * Return the tag used for storing files.
   * @return the tag name.
   */
  virtual std::string get_tag_file();
  /**
   * Get the parameters of the reconstructed superquadric.
   * @param point_cloud is the 3D point cloud of the object we want to model with the superquadric,
   * for instance: ((100.0 102.0) (100.0 103.0) ... ).
   * @param filtered_or_not is a bool variable specifing if we want the superquadric
   * to be filtered (true/1) or not (false/0).
   * @param reset_or_not is a bool variable specifing if we want to reset the
   * superquadric filtered (if enabled) or not.
   * @return the 12 parameters (x0, .. x11) of the current superquadric.
   * In particular, the parameters are grouped in a Property as follows: "dimensions" (x0, x1, x2)
   * are the three semi-axes lenghts; "exponents" (x3 and x4) are the exponents,
   * responsible for the superquadric shape; "center"(x5, x6, x7) contains the coordinate of
   * the superquadric center; and "orientation" (x8, x9, 10, x11) is the axis-angle representation
   * obtained from the Euler angles.
   */
  virtual yarp::os::Property get_superq(const std::vector<yarp::sig::Vector> & point_cloud, const bool filtered_or_not, const bool reset_or_not);
  /**
   * On/off point cloud filtering.
   * @param entry is "on/off" if you want/do not want to filter points.
   * @return true/false on success/failure.
   */
  virtual bool set_points_filtering(const std::string& entry);
  /**
   * Say if points filtering is on or not.
   * @return on/off string if points filtering is on/off.
   */
  virtual std::string get_points_filtering();
  /**
   * On/off superquadric filtering.
   * @param entry is "on/off" if you want/do not want to filter the estimated superquadric.
   * @return true/false on success/failure.
   */
  virtual bool set_superq_filtering(const std::string& entry);
  /**
   * Say if superquadric filtering is on or not.
   * @return on/off string if superquadeic filtering is on/off.
   */
  virtual std::string get_superq_filtering();
  /**
   *  Set if you want to save the acquired point cloud.
   * @param entry can be: "on" or "off".
   * @return true/false on success/failure.
   */
  virtual bool set_save_points(const std::string& entry);
  /**
   *  Set if you are saving the acquired point cloud.
   * @return  "on" or "off".
   */
  virtual std::string get_save_points();
  /**
   * Get the  parameters of the module. The user must pay attention
   * in changing them.
   * @param field can be "points_filter", "superq_filter", "optimization", "visualization" or "statistics".
   * depending on which parameters we are interested in.
   * @return the Property including all the  parameter values.
   */
  virtual yarp::os::Property get_options(const std::string& field);
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
  virtual bool set_options(const yarp::os::Property& options, const std::string& field);
  /**
   *  Set if the visualization has to be enabled.
   * @return  true/false on success/failure.
   */
  virtual bool set_visualization(const std::string& e);
  /**
   *  Get if visualization is enabled.
   * @return "on" or "off".
   */
  virtual std::string get_visualization();
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
