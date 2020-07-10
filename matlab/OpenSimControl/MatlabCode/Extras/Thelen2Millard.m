% Conversor Thelen to Millard

clear all
close all
clc
import org.opensim.modeling.*
thelenModel = Model('Arm_Flexion_Thelen.osim');
thelenMuscleSet=thelenModel.getMuscles;
numMuscles=thelenMuscleSet.getSize;
    for i=0:numMuscles-1
    thelenMuscle = thelenMuscleSet.get(i);
    MIF = thelenMuscle.getMaxIsometricForce()
    OFL=thelenMuscle.getOptimalFiberLength()
    TSL=thelenMuscle.get_tendon_slack_length()
    PA=thelenMuscle.getPennationAngleAtOptimalFiberLength()
    name=thelenMuscle.getName
    millardMuscle = org.opensim.modeling.Millard2012EquilibriumMuscle('BRAm',MIF,OFL,TSL,PA);
     muscle = org.opensim.modeling.Muscle(1,true);
    millardMuscle.safeDownCast(muscle);
    millardMuscleSet.adoptAndAppend(millardMuscle);
    thelenMuscleSet.remove(thelenMuscle);
    end
millardMuscleSet.getSize
thelenMuscleSet.getSize
    