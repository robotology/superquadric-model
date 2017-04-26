# superquadric-model tutorial
## Dependency
This example module relies on

- [IOL](https://github.com/robotology/iol)

for  the blob extraction process.

## How to compile
```
cd superquadric-model/example
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
((dimensions x0 x1 x2) (exponents x3 x4) (center x5 x6 x7) (orientation x8 x9 x10 x11))
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
