
# Addon Installation

To install this blender addon you need to install `protobuf` and `grpcio` to the blender python installation.
This is an example of how this can be done.

Run blender from the console so that you can see the console output. Then open the script tab in blender and run this:

```
import subprocess
import sys
import os
import site

print("python executable:", sys.executable)

# upgrade pip
subprocess.call([sys.executable, "-m", "ensurepip"])
subprocess.call([sys.executable, "-m", "pip", "install", "--upgrade", "pip"])

# install the pip package toxicblend + the dependencies protobuf and grpcio
subprocess.call([sys.executable, "-m", "pip", "install", "--upgrade", "toxicblend"])

## uninstall the pip package toxicblend 
# subprocess.call([sys.executable, "-m", "pip", "uninstall", "-y", "toxicblend"])
## uninstall the dependencies protobuf and grpcio
# subprocess.call([sys.executable, "-m", "pip", "uninstall", "-y", "protobuf", "grpcio"])

```
If you run into permission problems, you will have to run the pip commands on the blender python executable with raised privileges (e.g. sudo)
```
sudo /Applications/Blender.app/Contents/Resources/2.92/python/bin/python3.7m -m pip install --upgrade toxicblend
```

To uninstall, you use this command:
```
sudo <path to the blender built in python executable>/python3.7m -m pip uninstall toxicblend 
# and maybe uninstall the dependencies protobuf and grpcio too
sudo <path to the blender built in python executable>python3.7m -m pip uninstall protobuf grpcio
```

Then you need to install the addon itself. `Blender->Preferences->Addons->Install..` select the `blender_addon/toxicblend_meshtools.py` file
and click 'Install Add-on'. Same thing for the `blender_addon/toxicblend_metavolume.py` file.

Don't forget to enable the addon in the list.

