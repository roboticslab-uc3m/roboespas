
/* -------------------------------------------------------------------------- *
 *                      OpenSim:  SpasticThelenMuscleModel.cpp                         *
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
#include "SpasticThelenMuscleModel.h"
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
SpasticThelenMuscleModel::SpasticThelenMuscleModel()
{
	constructProperties();
}

//_____________________________________________________________________________
/*
 * Constructor.
 */
SpasticThelenMuscleModel::SpasticThelenMuscleModel(const std::string &name,
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
void SpasticThelenMuscleModel::constructProperties()
{
	setAuthors("ROBOESPAS");
	constructProperty_gain_factor(2);
	constructProperty_threshold_value(0.1);
	constructProperty_time_delay(0.03);
}

//--------------------------------------------------------------------------
// GET & SET Properties
//--------------------------------------------------------------------------
void SpasticThelenMuscleModel::setGainFactor(double aGainFactor)
{
	set_gain_factor(aGainFactor);
}

void SpasticThelenMuscleModel::setThresholdValue(double aThresholdValue)
{
	set_threshold_value(aThresholdValue);
}

void SpasticThelenMuscleModel::setTimeDelay(double aTimeDelay)
{
	set_time_delay(aTimeDelay);
}

//=============================================================================
// COMPUTATION
//=============================================================================

double SpasticThelenMuscleModel::applySpasticEffect(const SimTK::State& s/*, double excitation*/) const //11
{
	
	SimTK_ASSERT1(getNumStateVariables() == 2,
		"SpasticThelenMuscleModel: Expected 2 state variables"
		" but encountered  %f.", getNumStateVariables());

	double spasExcitation = getExcitation(s); 
	double gainFactor = get_gain_factor();
	double thresholdValue = get_threshold_value();
	double timeDelay = get_time_delay();
	double maxSizeFiberVelocities = timeDelay * 1000 / 2;

	FiberVelocityInfo fvi;
	calcFiberVelocityInfo(s, fvi); //*fvi
	double fv = fvi.normFiberVelocity;
	//Add new fiber velocity
	fiberVelocities.push_back(fv);
	//Get the one at the beginning
	double pastFV = fiberVelocities[0];
	//Delete this one if there are already too much velocities in the vector
	if (fiberVelocities.size() > maxSizeFiberVelocities)
	{
		fiberVelocities.erase(fiberVelocities.begin());
	}

	double actualTime = s.getTime();
	if ((pastFV > thresholdValue) && (actualTime > timeDelay))
	{
		//cout << "EXISTE ESPASTICIDAD, pastNFV= " << pastFV << "At Time = " << actualTime << " en: " << this->getName() << endl;//
		spasExcitation = pastFV * gainFactor + spasExcitation; 
	}

	return spasExcitation;
}
double SpasticThelenMuscleModel::calcActivationRate(const SimTK::State& s) const
{
	//std::cout << "calcActivationRate \n";
	try
	{
		double excitation = applySpasticEffect(s/*, excitation*/);
		double activation = getActivation(s);
		double dadt = getActivationModel().calcDerivative(activation, excitation);
		return dadt;
	}
	catch (const std::exception& ex)
	{
		std::cout << ex.what() << std::endl;
		return 1;
	}
	catch (...)
	{
		std::cout << "UNRECOGNIZED EXCEPTION" << std::endl;
		return 1;
	}
}




