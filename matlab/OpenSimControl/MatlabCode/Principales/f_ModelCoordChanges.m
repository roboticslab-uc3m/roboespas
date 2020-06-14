function [model] = f_ModelCoordChanges(CD_model,MODELO)
% Leo y modifico los valores de las coordenadas articulares
import org.opensim.modeling.*
cd(CD_model)
model = Model(MODELO);

%Coordenadas (todas)
    CoordSet= model.getCoordinateSet();
        elv_angle=CoordSet.get('elv_angle');
        shoulder_elv=CoordSet.get('shoulder_elv');
        shoulder_rot=CoordSet.get('shoulder_rot');
        elbow_flexion=CoordSet.get('elbow_flexion');
        pro_sup=CoordSet.get('pro_sup');
        deviation=CoordSet.get('deviation');
        flexion=CoordSet.get('flexion');
    %Coordenadas Iniciales:
        elv_angle.setDefaultValue(deg2rad(70));
        shoulder_elv.setDefaultValue(deg2rad(10));
        shoulder_rot.setDefaultValue(deg2rad(-5.3));
        pro_sup.setDefaultValue(deg2rad(-90));
        elbow_flexion.setDefaultValue(deg2rad(0));
        deviation.setDefaultValue(deg2rad(0));
        flexion.setDefaultValue(deg2rad(0));
        
     %Prescribed:
%         "Flag indicating whether or not the values of the coordinates should "
%           "be prescribed according to the function above. 
%           It is ignored if the" "no prescribed function is specified."
        Pres=true;
        
        elv_angle.set_prescribed(Pres);
        shoulder_elv.set_prescribed(Pres);
        shoulder_rot.set_prescribed(Pres);
        pro_sup.set_prescribed(Pres);
        elbow_flexion.set_prescribed(Pres);
        deviation.set_prescribed(Pres);
        flexion.set_prescribed(Pres);
        
     % Free to satisfy constraints
%      FtSC=true;
%         elv_angle.set_is_free_to_satisfy_constraints(FtSC);
%         shoulder_elv.set_is_free_to_satisfy_constraints(FtSC);
%         shoulder_rot.set_is_free_to_satisfy_constraints(FtSC);
%         pro_sup.set_is_free_to_satisfy_constraints(FtSC);
%         elbow_flexion.set_is_free_to_satisfy_constraints(FtSC);
%         deviation.set_is_free_to_satisfy_constraints(FtSC);
%         flexion.set_is_free_to_satisfy_constraints(FtSC);
        
    % Clamped: si se tienen que mantener dentro de los limites:
        elv_angle.set_clamped(true);
        shoulder_elv.set_clamped(true);
        shoulder_rot.set_clamped(true);
        pro_sup.set_clamped(true);
        elbow_flexion.set_clamped(true);
        deviation.set_clamped(true);
        flexion.set_clamped(true);
    %Bloqueos:
        elv_angle.set_locked(0);
        shoulder_elv.set_locked(0);
        shoulder_rot.set_locked(0);
        elbow_flexion.set_locked(0);
        pro_sup.set_locked(1);
        deviation.set_locked(1);
        flexion.set_locked(1);
%          elbow_flexion.getPropertyByName('locked') 
end

