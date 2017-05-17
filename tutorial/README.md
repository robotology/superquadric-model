# superquadric-model tutorial
## Dependency
This example module relies on

- [IOL](https://github.com/robotology/iol)

for  the blob extraction process.

## How to compile
```
mkdir build; cd build
ccmake ..
make install
```
## How to communicate with the superquadric-model
The user can provide the 2D blob and ask the estimated superquadric in two modes:

1. **streaming**
2. **one-shot**

**Streaming**: In this case,  the user should send the 2D blob to the module, for example:
```
  blobPort.open("/testing-module/blob:o");
  
  Bottle &blob=blobPort.prepare();
  Bottle &b1=blob.addList();
  
  for (size_t i=0; i<blob_points.size(); i++)
  {
      Bottle &b=b1.addList();
      b.addDouble(blob_points[i].x); b.addDouble(blob_points[i].y);
  }

blobPort.write();
```
connecting to the  corresponding buffered port of the superquadric-model:
```
yarp connect /testing-module/blob:o /superquadric-model/blob:i
```
The estimated superquadric can be read from another buffered port 
```
yarp connect /superquadric-model/superq:o /testing-module/superq:i
```
in the format:
```
(dimensions (x0 x1 x2)) (exponents (x3 x4)) (center (x5 x6 x7)) (orientation (x8 x9 x10 x11))
```
where:
 - **dimensions** are the semi-axes lenghts of the superquadric;
 - **exponents** are the responsible for the superquadric shape;
 - **center** are the coordinates of center of the superquadric;
 - **orientation** is the axis-angle orientation (derived from the Euler angles: x8, x9, x10 mentioned before).
 
**One-shot**: In this mode, the user can ask a simple query to the superquadric-model just sending a specific 2D blob and asking for the corresponding estimated superquadric. In this case, the port to use in the `rpc` port:
```
yarp connect /testing-module/superq:rpc /superquadric-model/rpc
```
An example of code is the following:
```
Bottle cmd, reply;
cmd.addString("get_superq");

Bottle &in1=cmd.addList();

for (size_t i=0; i<blob_points.size(); i++)
{
    Bottle &in=in1.addList();
    in.addDouble(blob_points[i].x);                        
    in.addDouble(blob_points[i].y);
}
//0 is for getting the estimated superquadric, 1 is for getting the filtered estimated superquadric
cmd.addInt(0);

superqRpc.write(cmd, reply);
```
## How to run the `superquadric-model` + `tutorial`

If you want to test the `superquadric-model` code without writing your own code, you can use the this `tutorial`, following the following steps:

1. Launch the basic modules of the iCub (`yarprobotinterface`, `iKinGazeCtrl`, `iKinCartesianSolver` - both for right and left arm-, `wholeBodyDynamics`, `gravityCompensator`, `imuFilter`) and the cameras.
2. Launch the [IOL](https://github.com/robotology/iol) modules. [This xml file](https://github.com/robotology/iol/blob/master/app/scripts/iol.xml.template) contains all the required modules. 
3. Launche the `superquadric-model` and a `yarpviewer`. You can use [this xml file](https://github.com/robotology/superquadric-model/blob/master/app/scripts/superquadric-model.xml.template).
4. Launch the `tutorial` code with [this xml](https://github.com/robotology/superquadric-model/blob/master/tutorial/app/script/testing-module.xml.template).
5. Connect everything.

You can now play with the `/testing-module/rpc` port:
- setting the name of the object you want to model
```
yarp rpc /testing-module/rpc
>>set_object_name object
```
- setting if you want the streaming mode or not
```
set_streaming_mode on
```

Enjoy! :smiley:
