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
struct Bottle
{
} (
   yarp.name = "yarp::os::Bottle"
   yarp.includefile="yarp/os/Bottle.h"
  )

/**
* testingModule_IDL
*
* IDL Interface to \ref testing-module services.
*/

service testingModule_IDL
{

    /* Send the current 2D blob
    *@return a bottle containing all the 2D points of the blob
    */
    Bottle  get_blob();

    /* Set the name of the object we want to model
    *@param object_name is the name of the object, memorizied by the OPC
    *@return true
    */
    bool set_object_name(1:string object_name)

    /* Set the streaming mode on or off
    *@param entry can be "on" or "off"
    *@return true/false on success/failure
    */
    bool  set_streaming_mode(1:string entry)

}


