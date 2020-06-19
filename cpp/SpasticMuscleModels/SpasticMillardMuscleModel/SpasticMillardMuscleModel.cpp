
/* -------------------------------------------------------------------------- *
 *                      OpenSim:  SpasticMillardMuscleModel.cpp                         *
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

 //=============================================================================
 // INCLUDES
 //=============================================================================
#include "SpasticMillardMuscleModel.h"
#include <OpenSim/OpenSim.h>
#include <fstream>
#include <stdio.h>


//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using namespace SimTK;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/*
 * Default constructor
 */
SpasticMillardMuscleModel::SpasticMillardMuscleModel()
{
	constructProperties();
}
/*const string SpasticMillardMuscleModel::
STATE_ACTIVATION_NAME = "activation";
const string SpasticMillardMuscleModel::
STATE_FIBER_LENGTH_NAME = "fiber_length";*/
//_____________________________________________________________________________
/*
 * Constructor.
 */
SpasticMillardMuscleModel::SpasticMillardMuscleModel(const std::string &name,
	double maxIsometricForce, double optimalFiberLength,
	double tendonSlackLength, double pennationAngle,
	double gainFactor, double thresholdValue, double timeDelay) :
	Super(name, maxIsometricForce, optimalFiberLength, tendonSlackLength,
		pennationAngle)
{
	//name, maxIsometricForce, optimalFiberLength, tendonSlackLenfth, pennationAngle -> Specific parameters for each muscle
	//gainFactor -> Constant multiplying fiber velocity in the past obtaining muscle excitation in present
	//thresholdValue -> Velocity limit to consider spasticity
	//timeDelay -> Time passed from the moment the velocity passes the threshold value to the moment the muscle sends the new modified excitation
	constructProperties();
	setGainFactor(gainFactor);
	setThresholdValue(thresholdValue);
	setTimeDelay(timeDelay);
	std::cout << "Created spastic muscle" << std::endl;
}

//_____________________________________________________________________________
/*
 * Construct and initialize properties.
 * All properties are added to the property set. Once added, they can be
 * read in and written to files.
 */
void SpasticMillardMuscleModel::constructProperties()
{
	setAuthors("ROBOESPAS");
	constructProperty_gain_factor(2);
	constructProperty_threshold_value(0.1);
	constructProperty_time_delay(0.03);
}

//--------------------------------------------------------------------------
// GET & SET Properties
//--------------------------------------------------------------------------
void SpasticMillardMuscleModel::setGainFactor(double aGainFactor)
{
	set_gain_factor(aGainFactor);
}

void SpasticMillardMuscleModel::setThresholdValue(double aThresholdValue)
{
	set_threshold_value(aThresholdValue);
}

void SpasticMillardMuscleModel::setTimeDelay(double aTimeDelay)
{
	set_time_delay(aTimeDelay);
}

//=============================================================================
// COMPUTATION
//=============================================================================


double SpasticMillardMuscleModel::applySpasticEffect(const SimTK::State& s) const
{
	double currentTime = s.getTime();
	//cout << "DENTRO de applySpasticEffect en el musculo \n";
	SimTK_ASSERT1(getNumStateVariables() == 2,
		"SpasticMillardMuscleModel: Expected 2 state variables"
		" but encountered  %f.", getNumStateVariables());		
	double spasExcitation = getExcitation(s);
	double gainFactor = get_gain_factor();
	double thresholdValue = get_threshold_value();
	double timeDelay = get_time_delay();
	FiberVelocityInfo fvi;
	calcFiberVelocityInfo(s, fvi);
	double fv = fvi.normFiberVelocity;
	if (stampsFiberVelocities.empty())
	{
		//If the vector is empty, just save a new fiber velocity + stamp
		fiberVelocities.push_back(fv);
		stampsFiberVelocities.push_back(currentTime);
	}
	else
	{
		//If it is not empty, check if the stamp already exists
		if (currentTime > stampsFiberVelocities[stampsFiberVelocities.size() - 1])
		{
			//If there is a new stamp, save it
			fiberVelocities.push_back(fv);
			stampsFiberVelocities.push_back(currentTime);
		}
		else if (currentTime = stampsFiberVelocities[stampsFiberVelocities.size() - 1])
		{
			//If there is a repeated stamp, replace the fiber velocity
			fiberVelocities[stampsFiberVelocities.size() - 1] = fv;
			stampsFiberVelocities[stampsFiberVelocities.size() - 1] = currentTime;
		}
		//Delete the fiberVelocities recorded more than timeDelay seconds ago in simulation
		double saved_time = stampsFiberVelocities[stampsFiberVelocities.size() - 1] - stampsFiberVelocities[0];
		while (saved_time > timeDelay)
		{
			stampsFiberVelocities.erase(stampsFiberVelocities.begin());
			fiberVelocities.erase(fiberVelocities.begin());
			saved_time = stampsFiberVelocities[stampsFiberVelocities.size() - 1] - stampsFiberVelocities[0];
		}
	}
	//Get the one at the beginning (timeDelay seconds ago)
	double pastFV = fiberVelocities[0];

	//Apply spasticity if fiberVelocity is higher than the threshold value, and if it has passed enough time since the beginning to apply spasticity
	double actualTime = s.getTime();
	if ((pastFV > thresholdValue) && (actualTime > timeDelay))
	{
		//cout << "EXISTE ESPASTICIDAD, pastNFV= " << pastFV << "At Time = " << actualTime << " en: " << this->getName() << endl;//
		spasExcitation = pastFV * gainFactor + spasExcitation;
	}
	return spasExcitation;
}

void SpasticMillardMuscleModel::computeStateVariableDerivatives(const SimTK::State& s) const
{

	// Activation dynamics if not ignored
	if (!get_ignore_activation_dynamics()) {
		double adot = 0;
		// if not disabled or overridden then compute its derivative
		if (appliesForce(s) && !isActuationOverridden(s)) 
		{
			double excitation = applySpasticEffect(s);   
			adot = getActivationModel().calcDerivative(getActivation(s), excitation);
		}
		setStateVariableDerivativeValue(s, "activation", adot);
	}

	// Fiber length is the next state (if it is a state at all)
	if (!get_ignore_tendon_compliance()) {
		double ldot = 0;
		// if not disabled or overridden then compute its derivative
		if (appliesForce(s) && !isActuationOverridden(s)) {
			ldot = getFiberVelocity(s);
		}
		setStateVariableDerivativeValue(s, "fiber_length", ldot);
	}
}

