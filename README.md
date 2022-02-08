# PiP
## Intro / Summary
PiP is an C++ open source 2d physics lib.
I built this during my first couple of years working as a game developer to continue studying into game physics, which I liked quite a bit in uni. I met a friend who sparked some conversations about floating point determinism and networking physics, so we had the idea of making this engine work on an alternate decimal mode, using fixed point decimal representations (Upping the scale of your numbers by N to represent decimals with bigger integers), for which he provided [this fixedpoint library](https://gitlab.com/DixieDev/fixed-point-lib).
This project almost got stale for a while, specially given I had to spend a lot of time dealing with compilation and dev issues to keep the library working for Linux and Windows (The fixed point library it uses is written in Rust with C bindings, and theres other dependencies). Regardless, after over 2 years of development, it works and runs as intended. Its compiled with clang on both operating systems and IDEs, to keep the workflow easier.  

It is quite simple and should provide a good introduction point for newbies wanting to learn about game physics or those that want to implement third party code to deal with simple collisions into their game.  

A lot of debug information is visible and editable in the testbed to experiment, using imgui (Check Usage Guide for knowing what to edit); You can also do some actions like instancing and deleting objects in the testbed.  

The library is distributed as a static .lib  

## Technical Summary:  

- Circle, Capsule, and OrientedBox collisions using newtonian physics and impulse based collision response, meaning velocities are computed instantly on collision to simulate a sudden change in momentum.  
- Object sleep optimization is used to stop sleeping bodies from processin intersect with one another, PIP_SLEEP_DELTA defines the minimum threshold in their positions /  rotations to be brought to sleep.
- A static / kinetic friction model is used, where the friction coefficient for a static object is above the one for a kinetic or moving body, thus helping with bringing objects to sleep and making it harder for them to exit this mode.
- It uses dynamic quad trees as a space subdivision scheme to avoid (o)N^2 complexity scaling (Things getting exponentially slower with more rigidbodies), the space is divided in AABB boxes that further subdivide themselves when needed. 
- The library can be recompiled with or without fixedpoint by changing PIP_USE_FIXEDPOINT, and most numerical operations are defined in PipMath to account for both modes.
- Handles are used to keep track of your rigidbodies and edit their data or destroy them when needed.  
- A memory allocator interface is provided including a DefaultAllocator implementation that uses a multibody, dense linear pool.  
- Imgui used for the user interface
- Unit tests implemented using catch  

## Usage Guide:  

First, you need to instance a Solver, passing the timestep frequency (Smaller numbers will result in physics engine using more resources, physics looking more 'Continuous' or smoother), an allocator (a DefaultAllocator if none is provided), and defining the playspace of the engine as an AABB; Objects outside this space won't process collision, but they won't be deleted by the engine, they will remain in memory unless deleted.  

All client communication can be done through the Solver, to make things.  
You can instance bodies through the Solver's interface for every shape, providing a handle reference and the body's properties. You will also receive an int for error code checking. This handle pattern is there to replace using pointers that may go invalid. It is an index into a list and its generational index to check the object on that list is the same as the one you had. For the properties, its worth checking in Rigidbody.h for safe ranges on things like m_e (coefficient of restitution), kinetic friction, etc.  

From there on, just call Solver::Step() on your update loop passing delta time as an argument. The solver will step as many times as its timestep indicates, and store leftover delta time in its accumulator to keep things balanced. If delta time between frames grows too much (Say the physics takes too long or else), the Solver will process up to 0.2f seconds in each Step() call, to avoid a spiral of death. You will notice that the physics processing will appear to run in slow motion, and look choppy (As multiple physics frames are stepped through before rendering).  

If you need to edit a body's properties after creation, do Solver::GetAllocator(), and you can retrieve it with the handle provided on creation through BaseAllocator::GetBody(), alternatively, you can do GetBodyAt() to retrieve from an index on a list, this is however unreliable when using the DefaultAllocator as objects are moved around on deletion (Swap and pop), and the whole concept of handles implies one idx could have a different body at different points in time.  
Its worth noting that a lot of rigidbody properties are not designed to be edited at runtime, see Rigidbody.h for a description of when properties should be edited. As a guide, the ones editable at runtime in the testbed through the rigidbody editor should be safe to do so (Mainly the velocities). If you need to edit positions directly from game code, just make sure you're not physics objects inside one another, to avoid degenerate cases in collision detection.  

When the game logic needs to delete a rigidbody, it's done through handles using Solver::DestroyBody(). You can use Solver::DestroyAllBodies() to empty the scene from rigidbodies.  

You wouldn't normally be doing this, but if you delete an object at runtime, the manifold list is going to hold invalid references until the next Solver::Step() when they're
cleared, this is worth knowing in case you're deleting and then rendering manifold data or something in between Step() calls.  

## Get started (win/linux):
- Download a release  
- Include headers in 'include' folder  
- Link against 'pip.lib'  
- Done  

The 'Testbed' binary is there so you can quickly check the engine demos. The executable will generate 'imgui.ini' when running to store GUI window settings.  

## Compiling / Building from source:  
You will need to build from source in order to change PIP_USE_FIXEDPOINT  

### Linux (Ubuntu):  
Install dependencies using 'sudo apt-get install', 'snap install' or your package manager of choice:  
libglfw3, libglfw3-dev, libmesa-dev, libglu1-dev, libglew-dev, clang, cmake  

Visual Studio Code (Recommended, tested):  
- Download source  
- Install 'CMake Tools' extension  
- Scan for kits should show you the clang installation from llvm (something like Clang_platform_pc-linux-gnu)  
- Compile and run  

### Windows:
This has been tested and developed using Visual Studio's Clang installation. You can add this from Visual Studio Installer:  
- Download source  
- Install Visual Studio (tested on vs2019, 2022). In the installer, make sure to install 'Desktop development with C++', and within it, install 'Clang in C++ Tools for Windows'  
- Install CMake and add to PATH  

#### Visual Studio Code (Recommented, tested)  
- Install 'CMake Tools' extension  
- 'CMake: Scan for kits' on the vscode command palette should show you the Visual Studio installation of clang (something like Clang X.X.X for MSVC X.X.X (Visual Studio Release)  
- Build testbed  

#### Visual Studio  (2019, 2022 recommended)
- Generate the binaries for your Visual studio version (Only tested on 2019, 2022) using CMake (in something like pip/build)  
- Open with visual studio, in order to run on clang, you need to go to Project properties (for Testbed, pip and imgui), General, platform toolset = LLVM(clang-cl)  
- Build testbed  
