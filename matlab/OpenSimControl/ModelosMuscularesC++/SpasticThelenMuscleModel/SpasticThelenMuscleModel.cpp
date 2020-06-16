
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
//#include <iostream.h>
#include <stdio.h>
//#include "apstring.cpp"


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
/*
// Define new states and their derivatives in the underlying system
void SpasticThelenMuscleModel::extendAddToSystem(SimTK::MultibodySystem& system) const
{
	// Allow Thelen2003Muscle to add its states, before extending
	Super::extendAddToSystem(system);

	// Now add the states necessary to implement the spastic behavior
	//addStateVariable("fiber_velocity");
	//addStateVariable("muscle_excitation");
	// and their corresponding derivatives
	addCacheVariable("fiber_velocity_deriv", 0.0, SimTK::Stage::Dynamics);
	addCacheVariable("muscle_excitation_deriv", 0.0, SimTK::Stage::Dynamics);
}
*/


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



// Initiate Fiber Velocity Analysis  
//_fiberVelocityAnalysis = new FiberVelocityAnalysis(_model, getActuatorSet()[0].getName());
//_model->addAnalysis(_fiberVelocityAnalysis);

//=============================================================================
// COMPUTATION
//=============================================================================

double SpasticThelenMuscleModel::updatePreviousFiberVelocities(double fvi) const
{
	char buffer[MAX_PATH];
	GetModuleFileName(NULL, buffer, MAX_PATH);
	string::size_type pos = string(buffer).find_last_of("\\/");
	string ExePath = string(buffer).substr(0, pos);

	string	muscleName = this->getName();
	double pastFV = 0;
	std::ifstream ifs;
	std::ofstream ofs;

	////// MEJORAR EN EL FUTURO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	std::string previousFiberVelocitiesFileStr = ExePath + "/" + muscleName + "_previousFiberVelocities.txt";
	std::string tempPFVFileStr = ExePath + "/" + muscleName + "_tempPFV.txt";
	

	const char *previousFiberVelocitiesFile = previousFiberVelocitiesFileStr.c_str();
	const char *tempPFVFile = tempPFVFileStr.c_str();
	

	ifs.open(previousFiberVelocitiesFileStr, std::ifstream::in);
	ofs.open(tempPFVFileStr, std::ofstream::out);
	if (!ifs.is_open() || !ofs.is_open())
	{
		// Los archivos ifs de cada m�sculo se crean desde matlab al iniciar la simulaci�n, cuando se llega aqui ya tienen que exisitir
		cout << "Could not open previousFiberVelocities file\n";
		return 0;
	}
	
	else
	{
		// Count number of lines in the file y voy rellenando Lista
		int  n = 0;
		double Lista[31];
		std::string line;
		// Actualizo la lista y cuento el numero de valores presentes en ifs
		while (getline(ifs, line))
		{
			//cout << n << "-> " << line << endl;
			Lista[n] = atof(line.c_str());
			n = n + 1;
		}
		//cout << "Count=" << n << endl;
		if (n < 30)
		{
			//cout << "Smaller than 30, fvi= ";
			//cout << fvi << endl;
			// Relleno el file con la lista, que voy actualizando
			for (int i = 0; i <= n - 1; i = i + 1)
			{
				ofs << Lista[i];
				ofs << endl;
			}
			// WRITE NEW LINE AT THE END OF THE LIST
			ofs << fvi;
			ofs << std::endl;
		}
		else // La lista ya est� llena
		{
			//cout << "Bigger than 30, pastFV=" << Lista[0] << endl;
			//cout << "fvi:" << fvi << endl;
			// Introduzco nuevo valor fvi
			Lista[30] = fvi;
			// Obtengo el primer valor de la lista (pastFV)
			pastFV = Lista[0];

			// Relleno el file con la lista, que voy actualizando
			for (int i = 0; i <= n - 1; i = i + 1)
			{
				Lista[i] = Lista[i + 1];
				ofs << Lista[i];
				ofs << endl;
			}
		}

		ifs.close();
		ofs.close();

		if (ifs.is_open())
		{
			cout << "ifs not closed \n";
			ifs.close();
		}
		if (ofs.is_open())
		{
			cout << "ofs not closed \n";
			ofs.close();
		}
		if (!ofs.is_open())
		{
			// delete the original file
			remove(previousFiberVelocitiesFile); //ifs

			// rename old to new
			while (rename(tempPFVFile, previousFiberVelocitiesFile) != 0) //ifs=ofs
			{
				cout << "Error renaming file";
				//rename("tempPFV.txt", "previousFiberVelocities.txt");
			}		
		}

		return pastFV;
	}

}

double SpasticThelenMuscleModel::applySpasticEffect(const SimTK::State& s/*, double excitation*/) const //11
{
	
	//cout << "DENTRO de applySpasticEffect en el musculo \n";
	// Allow Super to assign any state derivative values for states it allocated
	//Super::computeStateVariableDerivatives(s);

	// int nd = getNumStateVariables();

	SimTK_ASSERT1(getNumStateVariables() == 2,
		"SpasticThelenMuscleModel: Expected 2 state variables"
		" but encountered  %f.", getNumStateVariables());


	double spasExcitation = getExcitation(s); //excitation; // getExcitation(s);      //11
	double gainFactor = get_gain_factor();
	double thresholdValue = get_threshold_value();
	double timeDelay = get_time_delay();
	//cout << "spasExcitation= " << spasExcitation << endl;
	FiberVelocityInfo fvi;
	calcFiberVelocityInfo(s, fvi); //*fvi
	double fv = fvi.normFiberVelocity;
	double pastFV = updatePreviousFiberVelocities(fv);
	//cout << "pastFV= " << pastFV << endl << endl;
	
	double actualTime = s.getTime();
	if ((pastFV > thresholdValue) && (actualTime > timeDelay))
	{
		//cout << "EXISTE ESPASTICIDAD, pastNFV= " << pastFV << "At Time = " << actualTime << " en: " << this->getName() << endl;//
		spasExcitation = pastFV * gainFactor + spasExcitation; 
		// Add spasticity control value to existing controls
			//setExcitation(s, spasExcitation);
		// Add spasticity control value to existing controls
		//SimTK::Vector actControls(1, spasExcitation); // first make sure it's a vector
		//this->addInControls(actControls, controls);
	}

	return spasExcitation;
}
double SpasticThelenMuscleModel::calcActivationRate(const SimTK::State& s) const
{
	//std::cout << "calcActivationRate \n";
	try
	{
		
		/*double tiempoEjecucion = s.getTime();                  //11
		//std::cout << tiempoEjecucion << std::endl;
			double excitation = getExcitation(s);
	
		if (tiempoEjecucion > 0.02)
		{*/
			//std::cout << "calculando activacion t>0.02 \n";
		double excitation = applySpasticEffect(s/*, excitation*/);    //11
		//}
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




