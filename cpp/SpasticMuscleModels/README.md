### Creating a CMake-Visual Studio Project to modify the Spastic Muscle Models
Based on: https://simtk-confluence.stanford.edu:8443/display/OpenSim33/Step-by-Step+Example
1. First, download Cmake from https://cmake.org/download/. For Windows, download the latest release .msi file (https://github.com/Kitware/CMake/releases/download/v3.17.3/cmake-3.17.3-win64-x64.msi). Install it checking the option *Add CMake to the system path*.
2. Open *CMake (cmake-gui)*.
3. Fill the fields:
   - *Where is the source code*: Select the folder *SpasticMillardMuscleModel* or *SpasticThelenMuscleModel*, depending on which muscle you wish to modify. To work with spasticity, the Millard muscle works best with small muscle activations.
   - *Where to build the binaries*: For example, for the Millard muscle, *...\roboespas\cpp\SpasticMuscleModels\SpasticMillardMuscleModel\build\*.
4. Click *Configure*:
   - Select your version of Visual Studio. Check the version before, versions are not related with years, for example version 15 is from 2017. 
   - Select x64 platform.
   - Click *Finish* and wait for CMake to configure.
5. Click *Generate*. 
6. Open the project in Visual Studio by clicking the button next to *Generate*. You may close CMake now. 
7. To compile, select the *Release* configuration, and *x64* platform in the top part of VisualStudio. Then click *Compile\Compile solution*. 
8. If the compilation was correct, a new *.dll* file should have appeared in *...roboespas\cpp\SpasticMuscleModels\SpasticMillardMuscleModel\build\Release*. It should have appeared a new *.dll* called *SpasticMillardMuscleModel.dll*.
9. To add the spastic muscle as a plugin, copy the *.dll* file mentioned before inside *<OpenSim installation folder>\Plugins\*. Restart OpenSim.
10. To use this plugin, launch OpenSim and click *Tools\User Plugins\SpasticMillardMuscleModel.dll*. You should do this before opening the model in OpenSim to be able to use this muscle.
11. 





