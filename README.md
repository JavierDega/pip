# PiP
Open Source 2D Physics Lib

Get started:
Linux:
Linux (Ubuntu):
install dependencies:
sudo apt-get install:
libglfw3, libglfw3-dev, libmesa-dev, libglu1-dev, libglew-dev, clang


Dev Notes:
-Generating too many bodies (Probably past multibody pool size) results in eventual crash on object creation
-OrientedBoxes collide generating too much reactionary impulse (bounce further away)
-Obb-obb collision is quite generic and unoptimized in that it doesnt know when its clipping against one OBB's frame of reference, thus not needing to do dot projections to know its minimum and maximum extent along one axis, rather using its raw extents