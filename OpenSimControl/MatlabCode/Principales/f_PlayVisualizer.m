function [Message] = f_PlayVisualizer(CD,model,MOT)

cd(CD)
import org.opensim.modeling.*
%Inputs:
% model = Model(modelStr);
    model.setUseVisualizer(true);
    state = model.initSystem();
    storage = org.opensim.modeling.Storage(MOT);
%Main:
    numCoords = model.getNumCoordinates();
    q = Vector(numCoords, 0);
%     timeSeriesTable = STOFileAdapter.read("Movimiento.mot");
    sviz = model.updVisualizer().updSimbodyVisualizer();
        sviz.setShowSimTime(true);
        sviz.setShowFrameNumber(true);
        sviz.setShowShadows(true);
        sviz.setBackgroundTypeByInt(2);%2
            color=org.opensim.modeling.Vec3(131/255, 215/255, 209/255);
        sviz.setBackgroundColor(color);
    silo = model.updVisualizer().updInputSilo();
        silo.clear();
        % Get the next key press.
            while ~silo.isAnyUserInput()
                pause(0.01);
            end
            Message='Pulse una tecla para iniciar el movimiento'
     %Bucle
for i = 0:10:storage.getSize()-1 %motion loop. Un bucle un frame
    dat = storage.getStateVector(i).getData(); %array de coordenadas para cada tiempo
%     for j = 0:numCoords-1 %bucle coordenadas modelo
%         if (j >=10 && j<=11) || (j >=14 && j<=16)
%              mcoord = (dat.get(j))*(pi/180); %paso dat.get(j) a radianes
%         else 
%              mcoord = dat.get(j); %en estas coordenadas no se pasan a radianes
%         end 
%         q.set(j, mcoord);
%     end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% REPRESENTO SOLO LAS MISMAS COORD QUE SALEN EN OS
for j = 10:17 %bucle coordenadas modelo
        if (j >=10 && j<=11) || (j >=13 && j<=17)
             mcoord = (dat.get(j))*(pi/180); %paso dat.get(j) a radianes
        else 
             mcoord = dat.get(j); %en estas coordenadas no se pasan a radianes
        end 
        if j~=12
        q.set(j, mcoord);
        end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    state.setQ(q);
%     pause(0.2)
    model.getVisualizer().show(state); %MUESTRA CADA ESTADO CADA VEZ
end

end

