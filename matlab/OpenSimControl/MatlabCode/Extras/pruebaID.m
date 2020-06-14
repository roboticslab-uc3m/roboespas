clear all
close all
clc

%Open the directory where .osim model is located
cd('D:\User\escrit\Universidad\Master\2\TFM\MODELO\Arm_THELEN_4M')

import org.opensim.modeling.*

model = Model('Arm_Thelen_4M.osim')

tool=InverseDynamicsTool()
tool.setModel(model);
tool.setStartTime(1);
tool.setEndTime(4000);
% tool.setMarkerDataFileName('Lab.trc')
tool.setCoordinatesFileName('Movimiento.mot')
tool.setOutputGenForceFileName('IDoutput.sto')
tool.run();
