# Configuration

Simulink Real-Time uses a temp directory for data archiving. It is currently set
to `C:\Users\roboespas\Documents\GitHub\roboespas\RoboespasApp\MATLAB\FTSensor\FTSensor_tmp`

If the directory structure is changed it will ned to be modified, either through
the GUI:
```
ReadFT.slx>Toolbar>Code>"External Mode Control Panel">"Data Archiving">Directory
```
or via script
```matlab
set_param(model, 'ExtModeArchiveDirName', 'c:\your_directory')
```