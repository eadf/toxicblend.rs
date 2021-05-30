
# Addon Installation

To install these blender addons you need to install the pip packages `toxicblend`, `protobuf` and `grpcio` to the blender python installation.
This is an example of how this can be done.

Run blender from the console so that you can see the console output. Then open the script tab in blender and run this:

Blender MacOS Arm 2.93.0 Beta does not seem to work as it does not (yet) contain Python.h, so grpcio can't be installed.

```
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
If you run into permission problems, you will have to run the pip commands on the blender python executable with raised privileges (e.g. sudo)
```
sudo <path to the blender built in python>/python3.7m -m pip install --upgrade toxicblend
```

Then you need to install the addons themselves. `Blender->Preferences->Addons->Install..` select the `blender_addon/toxicblend_meshtools.py` file
and click 'Install Add-on'. Same thing for the `blender_addon/toxicblend_metavolume.py` and `blender_addon/toxicblend_object_add.py` files.

Don't forget to enable the addons in the Blender addon list.

# Uninstall
From blender script tab:
```
import subprocess
import sys

# uninstall the pip package toxicblend 
assert(subprocess.call([sys.executable, "-m", "pip", "uninstall", "-y", "toxicblend"])==0)
# uninstall the dependencies protobuf and grpcio if not required by other addons
assert(subprocess.call([sys.executable, "-m", "pip", "uninstall", "-y", "protobuf", "grpcio"])==0)
```

or from a terminal (try without sudo first):

```
sudo <path to the blender built in python>/python3.7m -m pip uninstall toxicblend 
# and maybe uninstall the dependencies protobuf and grpcio too
sudo <path to the blender built in python>/python3.7m -m pip uninstall protobuf grpcio
```
You will have to do the normal blender-addon removal of the scripts as well.
