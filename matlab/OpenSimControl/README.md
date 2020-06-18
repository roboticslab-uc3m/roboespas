### How to configure OpenSim and MATLAB
1. Download OpenSim
    - Go to https://opensim.stanford.edu/ and click *Download*
    - Click *Download Links* -> *OpenSim-4.1-win64.exe*
    - If you don’t already have a SimTK account, create it by clicking Create new account. Fill up the fields and wait until you receive an Account Registration e-mail. This may take up to **72 hours**.
    - Log in your SimTK account, agree the License Agreement and download the file.
    - Follow the installation steps, it is recommended to be installed in *C:\OpenSim 4.1*, but you can install it wherever you prefer.
2. Configure MATLAB to use OpenSim (from https://simtk-confluence.stanford.edu:8443/display/OpenSim33/Scripting+with+Matlab)
    - In MATLAB, go to *<OpenSim installation folder>\Resources\Code\Matlab*
    - Run the script *configureOpenSim.m*
    - Add to path *<OpenSim installation folder>\bin* folder. To do this, type in the Windows start menu *Path*, and the first search result should be something similar to *Edit environment variables*. Inside, click *Environment variables*, and edit the *System variable* named ```PATH```. Add at the end the *<OpenSim installation dir>\bin*. Save and exit. Restart MATLAB.
    - To check if the folder was correctly added to the path, type in MATLAB: ```model = org.opensim.modeling.Model();``` and check it works.
3. To use this model, download the folders inside the *AddToOpenSimDir* Drive folder (https://drive.google.com/drive/u/4/folders/1dpf1CuUMkGFJbfxdy6ACZUnbttgPLTjY) and copy them inside *<OpenSim installation folder>*.
   - The folder *Geometry* already exists, and there may be some downloaded files already present there, do not overwrite them, just copy the new ones.
4. Now, download the model and the trajectories, found inside the *AddToOpenSimControlDir* Drive folder (https://drive.google.com/drive/u/4/folders/1srC4AXj5f4QFgC-2gBhiMFmkBGDkVZFl). *ROBOESPAS_FLEXION* contain everything necessary to run the model, and *TrayectoriasGrabadas* contain some example trajectories. Both of them should be placed at the same level as this *README*, the folder *MatlabCode* and the folder *SpasticMusclesCpp*. Do not change the names of these folders as they are added to the .gitignore file.
5. Modify the path to *<OpenSim installation folder>* in line 44 of the file *Main.m* inside the *MatlabCode\Principales* folder if it's not *C:\OpenSim ...*. If you prefer, discomment the lines below that path to ask for the installation path everytime you run the *Main.m* file.
6. Add to MATLAB path the whole *OpenSimControl* folder, by right clicking this folder in the MATLAB *Current folder* explorer and selecting *Add to Path* > *Selected folders and subfolders*.
7. Execute the file *Main.m*. 

## Common errors
- Sometimes, MATLAB should be run as Administrator to run the *Main.m*.