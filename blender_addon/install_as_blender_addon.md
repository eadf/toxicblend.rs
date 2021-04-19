
# Addon Installation

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

# install the pip packages protobuf and grpcio
subprocess.call([sys.executable, "-m", "pip", "install", "protobuf"])
subprocess.call([sys.executable, "-m", "pip", "install", "grpcio"])

print("copy or link toxicblend_pb2.py and toxicblend_pb2_grpc.py to this path:")
print(site.getsitepackages())
```
If you run into permission problems, you will have to run the pip commands on the blender python executable with raised privileges (e.g. sudo)


You will also have to copy the `toxicblend_pb2.py` and `toxicblend_pb2_grpc.py` files to one of the
site-package directories of the blender python environment. Soft links works fine as well.

```
# example MacOS:
$ cd toxicblend.rs
$ cp blender_addon/toxicblend_pb2.py blender_addon/toxicblend_pb2_grpc.py /Applications/Blender.app/Contents/Resources/2.92/python/lib/python3.7/site-packages
```

Then you need to install the addon itself. `Blender->Preferences->Addons->Install..` select the `blender_addon/toxicblend_meshtools.py` file
and click 'Install Add-on'.

Don't forget to activate the addon in the list.

