
# Addon Installation

To install these blender addons you need to install the pip packages `toxicblend`, `protobuf` and `grpcio` to the blender python installation.
This is an example of how this can be done.

Run blender from the console so that you can see the console output. Then open the script tab in blender and run this:

```
import subprocess
import sys
import os
import site

print("this is the path to the python executable: ", sys.executable)

# upgrade pip
subprocess.call([sys.executable, "-m", "ensurepip"])
subprocess.call([sys.executable, "-m", "pip", "install", "--upgrade", "pip"])

# install the pip package toxicblend + the dependencies protobuf and grpcio
subprocess.call([sys.executable, "-m", "pip", "install", "--upgrade", "toxicblend"])

# check that the installed toxicblend version is 0.2.0 
subprocess.call([sys.executable, "-m", "pip", "list"])
```
If you run into permission problems, you will have to run the pip commands on the blender python executable with raised privileges (e.g. sudo)
```
sudo <path to the blender built in python>/python3.7m -m pip install --upgrade toxicblend
```

Then you need to install the addons themselves. `Blender->Preferences->Addons->Install..` select the `blender_addon/toxicblend_meshtools.py` file
and click 'Install Add-on'. Same thing for the `blender_addon/toxicblend_metavolume.py` and `blender_addon/toxicblend_lsystem.py` files.

Don't forget to enable the addons in the Blender addon list.

# Uninstall
From blender script tab:
```
import subprocess
import sys

# uninstall the pip package toxicblend 
subprocess.call([sys.executable, "-m", "pip", "uninstall", "-y", "toxicblend"])
# uninstall the dependencies protobuf and grpcio if not required by other addons
subprocess.call([sys.executable, "-m", "pip", "uninstall", "-y", "protobuf", "grpcio"])
```

or from a terminal (try without sudo first):

```
sudo <path to the blender built in python>/python3.7m -m pip uninstall toxicblend 
# and maybe uninstall the dependencies protobuf and grpcio too
sudo <path to the blender built in python>/python3.7m -m pip uninstall protobuf grpcio
```
You will have to do the normal blender-addon removal of the scripts as well.
