
# The fastMavlink Library: Code Generation #

As common with MAVLink, the code is created from the MAVLink XML definition files using a 'code generator', which is nothing else than a Python script. 

The main driver is the fmav_gen.py script located in this folder. It may however not be the most convenient script to use, and fastMavlink provides two further scripts which you may find more useful, and which also may serve as examples for how to use fmav_gen.py. These are:

* fmav_gui.py: This script, which is located in the root folder, runs a GUI based on tkinter. This might be the easiest approach.

* fmav_generate_c_library.py: This script, which is located in the tools folder, is what I use to recreate the C code in the c_library folder. I also use copies of it with adapted path and file names (which you can easily do in the top section of the script) in order to create the fastMavlink C code for my other projects. It makes it easy to also use home-grown dialects or for development, and is obviously also suitable for automated build systems.  

The MAVLink XML definition files are not part of fastMavlink and are not included. Therefore, you need to host them somewhere else on your system, and make the code generator be aware of the location. 

A good and common approach could be to clone or submodule both the fastMavlink and mavlink repos into the same folder, like xyzpath/fastMavlink and xyzpath/mavlink. When using the GUI fmav_gui.py, you simply browse to the location. With (a copy of) fmav_generate_c_library.py you simply specify the location in the respective variable (the script should be self explaning).


