
# Addon Installation

To install these blender addons you need to install the pip packages `toxicblend`, `protobuf` and `grpcio` to the blender python installation.
This is an example of how this can be done.

**MacOs warning**: It seems that whenever something is installed to the blender builtin Python packages the Blender installation will forever be marked as "damaged" on MacOs.
You can still run Blender, but it needs to be run from the command line:
```sh
$ /Applications/Blender.app/Contents/MacOS/Blender
```

Run blender from the console so that you can see the console output. Then open the script tab in blender and run this:

```python
import subprocess
import sys
import os
import site

print("this is the path to the python executable: ", sys.executable)

# upgrade pip
assert(subprocess.call([sys.executable, "-m", "ensurepip"]) == 0)
assert(subprocess.call([sys.executable, "-m", "pip", "install", "--upgrade", "pip"]) == 0)

# install the pip package toxicblend + the dependencies wheel, protobuf and grpcio
assert( subprocess.call([sys.executable, "-m", "pip", "install", "--upgrade", "wheel"]) == 0)
assert( subprocess.call([sys.executable, "-m", "pip", "install", "--upgrade", "toxicblend"]) == 0)

# check that the installed toxicblend version is 0.2.0 
assert(subprocess.call([sys.executable, "-m", "pip", "list"]) == 0)
```
Note that you can't install [grpcio](https://github.com/grpc/grpc) on Blender MacOS Arm 2.93.4 (grpcio does not contain the Arm binaries, and Blender is missing Python.h). 
But the x64 version of Blender can install grpcio.

If you run into permission problems, you will have to run the pip commands on the blender python executable with raised privileges (e.g. sudo)
```
sudo <path to the blender built in python>/python3.7m -m pip install --upgrade toxicblend
```

Then you need to install the addons themselves. `Blender->Preferences->Addons->Install..` select the `blender_addon/toxicblend_meshtools.py` file
and click 'Install Add-on'. Same thing for the `blender_addon/toxicblend_metavolume.py` and `blender_addon/toxicblend_object_add.py` files.

Don't forget to enable the addons in the Blender addon list.

# Update

Usually you do not need to update the python pip package, the addons checks the version of the toxicblend pip package and will display an error message if it needs to be updated.

The addon-files needs to be updated for each version: remove them from the Blender addon screen and re-install them.


# Uninstall
From blender script tab:
```python
import subprocess
import sys

# uninstall the pip package toxicblend 
assert(subprocess.call([sys.executable, "-m", "pip", "uninstall", "-y", "toxicblend"])==0)
# uninstall the dependencies protobuf and grpcio if not required by other addons
assert(subprocess.call([sys.executable, "-m", "pip", "uninstall", "-y", "protobuf", "grpcio"])==0)
```

or from a terminal (try without sudo first):

```sh
sudo <path to the blender built in python>/python3.7m -m pip uninstall toxicblend 
# and maybe uninstall the dependencies protobuf and grpcio too
sudo <path to the blender built in python>/python3.7m -m pip uninstall protobuf grpcio
```
You will have to do the normal blender-addon removal of the scripts as well.
