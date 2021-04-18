
#Addon Installation
To install this blender addon you need to install `protobuf` and `grpcio` to the blender python installation.
This is an example of how this can be done.

Open the script tab in blender and run this:

```
import subprocess
import sys
import os
import site

print("python executable:", sys.executable)

# upgrade pip
subprocess.call([sys.executable, "-m", "ensurepip"])
subprocess.call([sys.executable, "-m", "pip", "install", "--upgrade", "pip"])

# install required packages
subprocess.call([sys.executable, "-m", "pip", "install", "protobuf"])
subprocess.call([sys.executable, "-m", "pip", "install", "grpcio"])

print("copy toxicblend_pb2.py toxicblend_pb2_grpc.py to this path:")
print(site.getsitepackages())
```
If you run into permission problems you will have to run the pip commands on the blender python executable with raised privileges (e.g. sudo)


You will also have to copy the `toxicblend_pb2.py` and `toxicblend_pb2_grpc.py` files to the
site packages of the same python environment.

```
# example MacOS:
# cp toxicblend_pb2.py toxicblend_pb2_grpc.py /Applications/Blender.app/Contents/Resources/2.92/python/lib/python3.7/site-packages
```