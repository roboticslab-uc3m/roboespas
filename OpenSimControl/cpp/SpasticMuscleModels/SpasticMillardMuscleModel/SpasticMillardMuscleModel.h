#pragma once

#ifndef OPENSIM_SPASTIC_MILLARD_MUSCLE_H_
#define OPENSIM_SPASTIC_MILLARD_MUSCLE_H_
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  SpasticMillardMuscleModel.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Loan, Ajay Seth                                           *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied    *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */


 // INCLUDE
#include <OpenSim/OpenSim.h>
#include <OpenSim/Common/Storage.h> //
#include "osimPluginDLL.h"

namespace OpenSim {

	//=============================================================================
	//=============================================================================
	/**
	 */
	class OSIMPLUGIN_API SpasticMillardMuscleModel : public Millard2012EquilibriumMuscle
	{
		OpenSim_DECLARE_CONCRETE_OBJECT(SpasticMillardMuscleModel, Millard2012EquilibriumMuscle);
	public:
		//=============================================================================
		// PROPERTIES
		//=============================================================================
		OpenSim_DECLARE_PROPERTY(gain_factor, double,
			"Gain factor by which the muscle fiber lengthening velocity will be multiplied, "
			"to induce a spasticity response in excitation");
		OpenSim_DECLARE_PROPERTY(threshold_value, double,
			"Fiber velocity threshold, "
			"above which a spasticity response in excitation is evoked");
		OpenSim_DECLARE_PROPERTY(time_delay, double,
			"Delay between stretch and spasticity response "
			"representative of stretch reflex delay. Default 0.030 s");

	
	public:
		//=============================================================================
		// METHODS
		//=============================================================================
			//-------------------------------------------------------------------------
			// CONSTRUCTION
			//-------------------------------------------------------------------------
		SpasticMillardMuscleModel();
		SpasticMillardMuscleModel(const std::string &name, double maxIsometricForce,
			double optimalFiberLength, double tendonSlackLength,
			double pennationAngle, double gainFactor,
			double thresholdValue, double timeDelay);

		// employs the default destructor, copy constructor and copy assignment 
		// that are automatically supplied by the compiler if none are defined

		//-------------------------------------------------------------------------
		// GET & SET Properties
		//-------------------------------------------------------------------------
		/** Gain, threshold and delay are properties of this muscle */
		double getGainFactor() const { return get_gain_factor(); }
			void setGainFactor(double aGainFactor);
		double getThresholdValue() const { return get_threshold_value(); }
			void setThresholdValue(double aThresholdValue);
		double getTimeDelay() const { return get_time_delay(); }
			void setTimeDelay(double aTimeDelay);

		//-------------------------------------------------------------------------
		// GET & SET STATES and their derivatives
		//-------------------------------------------------------------------------
		

	protected:
		// Model Component Interface
		/** add new dynamical states to the multibody system corresponding
			to this muscle */
		//void extendAddToSystem(SimTK::MultibodySystem& system) const override;
		/** initialize muscle state variables from properties. For example, any
			properties that contain default state values */
		//void extendInitStateFromProperties(SimTK::State& s) const override;
		/** use the current values in the state to update any properties such as
			default values for state variables */
		//void extendSetPropertiesFromState(const SimTK::State& s) override;

		//-------------------------------------------------------------------------
		// COMPUTATIONS
		//-------------------------------------------------------------------------
		/** Compute the derivatives for state variables added by this muscle */
		//void computeStateVariableDerivatives(const SimTK::State& s) const;  //AQUI
		//void SpasticMuscleModelPlugin::computeMuscleExcitation(SimTK::State& s);// const override;
		//void SpasticMuscleModelPlugin::computeControls(const SimTK::State& s, SimTK::Vector &controls) const;
		void computeStateVariableDerivatives(const SimTK::State& s) const override;
	private:
		double SpasticMillardMuscleModel::applySpasticEffect(const SimTK::State& s /*, double excitation*/) const;
		/** construct the new properties and set their default values */
		void constructProperties();
		mutable std::vector<double> fiberVelocities;
		mutable std::vector<double> stampsFiberVelocities;
		//=============================================================================
	};  // END of class SpasticMillardMuscleModel
	//=============================================================================
	//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_SPASTIC_MUSCLE_H_


