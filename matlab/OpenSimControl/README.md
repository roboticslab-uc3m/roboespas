### Configure OpenSim and MATLAB to run Main.m
1. Download OpenSim
    - Go to https://opensim.stanford.edu/ and click *Download*
    - Click *Download Links* -> *OpenSim-4.1-win64.exe*
    - If you don’t already have a SimTK account, create it by clicking Create new account. Fill up the fields and wait until you receive an Account Registration e-mail. This may take up to **72 hours**.
    -	Log in your SimTK account (twice), and then agree the License Agreement and Download
2. Configure MATLAB to use OpenSim (from https://simtk-confluence.stanford.edu:8443/display/OpenSim33/Scripting+with+Matlab)
    -	In MATLAB, go to the folder where you installed OpenSim -> *Resources* -> *Code* -> *Matlab*
    -	Run *configureOpenSim.m* 
    - Add to path the OpenSim bin folder. First type in the Windows start menu *Path*, then click the first option available and a *System configuration* window should open. Click *Environment variables*, and edit the *System variable* named ```PATH```. Add at the end the *\bin* folder inside your OpenSim installation path. Save and exit.  Restart MATLAB.
    - To check if the folder was correctly added to the path, type in MATLAB: ```model = org.opensim.modeling.Model();``` and check it doesn't return any errors.
3. Download the folders inside the *AddToOpenSimDir* Drive folder (https://drive.google.com/drive/u/4/folders/1dpf1CuUMkGFJbfxdy6ACZUnbttgPLTjY) and copy them in the OpenSim installation directory: 
   - The folder *Geometry* already exist, and there may be some downloaded files already present there, do not overwrite them, just copy the new ones.
4. Download the folders inside the *AddToOpenSimControlDir* Drive folder (https://drive.google.com/drive/u/4/folders/1srC4AXj5f4QFgC-2gBhiMFmkBGDkVZFl). These folders are the model directory called *ROBOESPAS_FLEXION* and some example trajectories called *TrayectoriasGrabadas* and should be placed at the same level of this *README* and the folder *MatlabCode*. Do not change the names of the folders as they are added to the .gitignore file.
5. Modify the paths in the file *Main.m* inside the *MatlabCode* folder: 
   - Line 54: *path_to_OpenSimControl\TrayectoriasGrabadas\Test-1707\iiwa1707*
   - Line 74: *path_to_OpenSimControl\TrayectoriasGrabadas\Test-1707\TrayectoriasKinect1707\Sin fuerza*
   - Line 68: *path_to_OpenSimControl\ROBOESPAS_FLEXION*
   - Line 76: *path_to_OpenSimControl\TrayectoriasGrabadas\Test-1707\TrayectoriasKinect1707\Con fuerza*
6. Add to MATLAB path the whole *OpenSimControl* folder, by right clicking this folder in the MATLAB *Current folder* explorer and selecting *Add to Path* > *Selected folders and subfolders*
7. Execute the file *Main.m*.
