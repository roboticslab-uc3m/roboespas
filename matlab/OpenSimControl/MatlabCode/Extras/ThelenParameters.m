% Obtención y modificación de los parámetros propios de un músculo
% Thelen2003

clear all
close all
clc
import org.opensim.modeling.*
MODELO= 'Arm_Thelen_4M_OS40.osim';
model = Model(MODELO);
mcl = Thelen2003Muscle.safeDownCast( model.getMuscles().get('DELT1') );

optimal_force=mcl.get_optimal_force();
max_isometric_force=mcl.get_max_isometric_force();
optimal_fiber_length=mcl.get_optimal_fiber_length();
tendon_slack_length=mcl.get_tendon_slack_length();
pennation_angle_at_optimal=mcl.get_pennation_angle_at_optimal();
max_contraction_velocity=mcl.get_max_contraction_velocity();
activation_time_constant=mcl.get_activation_time_constant();
deactivation_time_constant=mcl.get_deactivation_time_constant();
FmaxTendonStrain=mcl.get_FmaxTendonStrain();
FmaxMuscleStrain=mcl.get_FmaxMuscleStrain();
KshapeActive=mcl.get_KshapeActive();
KshapePassive=mcl.get_KshapePassive();
Af=mcl.get_Af();
Flen=mcl.get_Flen();

%Modificar valores: 
mcl.set_optimal_force();
mcl.set_max_isometric_force();
mcl.set_optimal_fiber_length();
mcl.set_tendon_slack_length();
mcl.set_pennation_angle_at_optimal();
mcl.set_max_contraction_velocity();
mcl.set_activation_time_constant();
mcl.set_deactivation_time_constant();
mcl.set_FmaxTendonStrain();
mcl.set_FmaxMuscleStrain();
mcl.set_KshapeActive();
mcl.set_KshapePassive();
mcl.set_Af();
mcl.set_Flen();

