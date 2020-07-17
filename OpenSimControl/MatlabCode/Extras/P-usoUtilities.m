%PRUEBA PARA APRENDER A USAR LAS UTILITIES

clear all
close all
clc

data.time = [0;1;2;3] 
% data.positions = [1 2 3;4 5 6;7 8 9;1 2 3]
data.X = [1;4;7;10]
data.Y = [2;5;8;11]
data.Z = [3;6;9;12]

ttable = osimTableFromStruct(data);

% %% Write to file using the TRCFileAdapter
% outputPath = fullfile('D:\User\escrit\Universidad\Master\2\TFM\MATLAB CODES', strrep('pruebaTRC.trc', '.trc', '_fixed.trc'))
% trcTool=org.opensim.modeling.TRCFileAdapter();
% trcTool.write(ttable,outputPath)
% % display(['TRC File Written to ' outputPath ]);
% 






%% no funciona
TRCFileFixer("D:\User\escrit\Universidad\Master\2\TFM\MATLAB CODES\pruebaTRC.trc")





