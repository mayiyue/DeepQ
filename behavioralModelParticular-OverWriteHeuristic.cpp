#include "behavioralModelParticular.h"
#include "simVehicleParticular.h"
#include "AKIProxie.h"
#include "ANGConProxie.h"
#include <stdio.h>
#include <fstream>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <cstring>
#include <vector>
#include <array>


using namespace std;
#define Tolerancia 0.01
#define VISIBLELIMIT 500
#define SPEED_CRITICAL 60 // using for Asymmetric MOBIL
#define DBL_MAX 1.7976931348623158e+308 
#define DATAPATH "D:\\working\\AIMSUN CODE\\Dutch_network&coding\\data\\"


bool useIDM = true;
bool useHeuristicLaneChangeModel = true;
bool useMOBILinNormal = false;
bool useAsymmetricMOBIL = false;        //Symmetric MOBIL is default

double const penetrationOfACC = 0;
double const lowLimitOfACCTimeGap = 0.5;
double const upLimitOfACCTimeGap = 0.7;

behavioralModelParticular::behavioralModelParticular() : A2BehavioralModel()
{
	const unsigned short *randomSeedString = AKIConvertFromAsciiString("GKReplication::randomSeedAtt");
	seed = ANGConnGetAttributeValueInt(ANGConnGetAttribute(randomSeedString), ANGConnGetReplicationId());
	const unsigned short *param0 = AKIConvertFromAsciiString("GKExperiment::p_distance");
	p_distance = ANGConnGetAttributeValueDouble(ANGConnGetAttribute(param0), ANGConnGetExperimentId());

	isOnRamp = false;
	isSegmRamp = false;
}

behavioralModelParticular::~behavioralModelParticular()
{
}

simVehicleParticular * behavioralModelParticular::arrivalNewVehicle(void *handlerVehicle, unsigned short idHandler, bool isFictitiousVeh) {

	simVehicleParticular * res = new simVehicleParticular(handlerVehicle, idHandler, isFictitiousVeh);
	if (!isFictitiousVeh)
	{

		if (AKIGetRandomNumber() < penetrationOfACC)
		{
			res->SetIsACC(true);
			res->SetTimeGapOfACC(generateTimeGapOfACC(lowLimitOfACCTimeGap, upLimitOfACCTimeGap));
		}
	}
	return res;
}

void behavioralModelParticular::removedVehicle(void *handlerVehicle, unsigned short idHandler, A2SimVehicle * a2simVeh)
{
}

bool behavioralModelParticular::evaluateCarFollowing(A2SimVehicle *vehicle, double &newpos, double &newspeed)
{
	return false;
}


bool behavioralModelParticular::evaluateLaneChanging(A2SimVehicle *vehicle, int threadId)
{
	double currStep = AKIGetCurrentSimulationTime();
	double currTime = currStep;

	int direction = 0, targetLane;// actually, targetLane is useless,(by Wang Long)

	int id_veh = vehicle->getId();
	int numSect = vehicle->getIdCurrentSection();
	double currSpeed = vehicle->getSpeed(0);
	int numLane = vehicle->getIdCurrentLane();
	int maxLanes = vehicle->getNumberOfLanesInCurrentSection();


	simVehicleParticular* vehicleTemp = (simVehicleParticular*)vehicle;
	bool isACC = vehicleTemp->getIsACC();
	if (id_veh == 7) isACC = true;  // vehicle 7 is test vehicle, which is always ACC


	/******* Heuristic Lane Change Model by Georgia ***********/
	if (useHeuristicLaneChangeModel)
	{

		double currentVehPosition = vehicle->getPosition(0);
		double XPosTargetlane = vehicle->getPosition(0);
		A2SimVehicle* pVehDw = NULL;
		A2SimVehicle *pVehUp = NULL;
		double ShiftUp = 0, ShiftDw = 0;

		vehicle->getUpDown(direction, XPosTargetlane, pVehUp, ShiftUp, pVehDw, ShiftDw);


		// in the on-ramp, define a behaviour, otherwise NO LANE-CHANGINGS
		// 404 and 423 are on-ramp sections, and 404 is the first on-ramp, 423 is second.   
		if ((numSect == 404) || (numSect == 423))
		{
			if (vehicle->IsLaneOnRamp(0))  //  vehicle in acceleration lane
			{
				double avgSpeedInMainLane = 0, distanceToMainLaneUp = 0, distanceToMainLaneDown = 0;
				if (pVehUp != NULL)
				{
					avgSpeedInMainLane = pVehUp->getSpeed(0);
					distanceToMainLaneUp = currentVehPosition - pVehUp->getPosition(0);
				}
				else
				{
					avgSpeedInMainLane = 120 / 3.6; // 120 km/h -> m/s
					distanceToMainLaneUp = 100;     // m
				}
				if (pVehDw != NULL)
				{
					distanceToMainLaneDown = pVehDw->getPosition(0) - currentVehPosition;
				}
				else
				{
					distanceToMainLaneDown = 100;
				}

				double SpeedDifferenceWithMainLane = (avgSpeedInMainLane - currSpeed);


				// thresholds as linear functions of the position within the section

				double thresholdCurrentSpeed = (-currentVehPosition / 3.8 + 88.25) / 3.6;
				if (currentVehPosition > 320)
					thresholdCurrentSpeed = 1.12207602339181;



				double	thresholdSpeedDifference = (currentVehPosition*2.5 / 5.8 - 30) / 3.6;
				if (currentVehPosition > 300)
					thresholdSpeedDifference = 27.586206896551726;



				double thresholdSpace = -0.1 * currentVehPosition + 29.5;
				if (currentVehPosition > 255)
					thresholdSpace = 3;



				// To determine change the lane from acceleration to main lane or not.
				if (
					(
						currentVehPosition > 30
						&& currSpeed > thresholdCurrentSpeed
						&& SpeedDifferenceWithMainLane <= thresholdSpeedDifference
						&& distanceToMainLaneUp > thresholdSpace
						&& distanceToMainLaneDown > thresholdSpace
						)
					||
					(
						currentVehPosition > 50
						&& currSpeed <0.1
						&& SpeedDifferenceWithMainLane <= thresholdSpeedDifference
						&& distanceToMainLaneUp >thresholdSpace
						&& distanceToMainLaneDown > thresholdSpace
						/*it indicates that the merging area includeing main lane and accele
						ration lane is congested, but the lane changeing space are still exist.*/
						)
					)
				{
					direction = -1;

					double XPosTargetlane = vehicle->getPosition(0);
					A2SimVehicle* pVehDw = NULL;
					A2SimVehicle *pVehUp = NULL;
					double ShiftUp = 0, ShiftDw = 0;
					vehicle->getUpDown(direction, XPosTargetlane, pVehUp, ShiftUp, pVehDw, ShiftDw);

					isOnRamp = true;
					bool GapAcceptable = vehicle->isGapAcceptable(direction, XPosTargetlane, pVehUp, ShiftUp, pVehDw, ShiftDw);
					isOnRamp = false;
					if (GapAcceptable)
					{
						// vehicle->assignAcceptedGap(direction, XPosTargetlane, pVehUp, ShiftUp, pVehDw, ShiftDw, threadId); 
						// it's a little confusing compare with Aimsun example.
						// So I annotate this line.
						vehicle->applyLaneChanging(direction, threadId);
						return true;
					}
				}

			}

			//if current lane is not acceleration lane of on-ramp, but main lane of on-ramp merging area.
			else if (numLane == 2 || numLane == 3)
			{

				if (currSpeed > 20 / 3.6) // 20m/s
				{
					return false; // don't use Heuristic lane-changing model
				}
				else
				{
					double	coefficientOfCurrentSpeed = 0.8; //	12.1
					double	coefficientOfLeaderSpeed = 0.9;  //	14.8

					double averageSpeedDwAndUp = 0.0;
					int count = 0;

					if (pVehDw != NULL)
					{
						averageSpeedDwAndUp += pVehDw->getSpeed(0);
						count++;
					}
					if (pVehUp != NULL)
					{
						averageSpeedDwAndUp += pVehUp->getSpeed(0);
						count++;
					}

					if (count > 0)
					{
						averageSpeedDwAndUp = averageSpeedDwAndUp / (double)count;
					}
					// if pVehDw and pVehUp doesn't exist nextSpeed=0;


					double tempShift = 0;
					double speedLeader = vehicle->getLeader(tempShift)->getSpeed(0);


					if ((averageSpeedDwAndUp > coefficientOfCurrentSpeed * currSpeed) && (averageSpeedDwAndUp > coefficientOfLeaderSpeed * speedLeader))
						// I can not understand the attempt of this module. by wanglong 
						// in case nextspeed is zero lanechanging is not applied
					{
						direction = -1;
						bool GapAcceptable = vehicle->isGapAcceptable(direction, XPosTargetlane, pVehUp, ShiftUp, pVehDw, ShiftDw);
						if (GapAcceptable)
						{
							isSegmRamp = true;
							vehicle->assignAcceptedGap(direction, XPosTargetlane, pVehUp, ShiftUp, pVehDw, ShiftDw, threadId);
							//vehicle->applyLaneChanging(direction, threadId);
							isSegmRamp = false;
							return true;
						}
					}
				}

			}

		}

		// 395 and 386 are Lane-Drop Sections, and the 386 is the section in upstream of 395.
		else if (numSect == 395 || numSect == 386)
		{
			double avgSpeedInMainLane = 0, distanceToMainLaneUp = 0, distanceToMainLaneDown = 0;
			if (pVehUp != NULL)
			{
				avgSpeedInMainLane = pVehUp->getSpeed(0);
				distanceToMainLaneUp = currentVehPosition - pVehUp->getPosition(0);
			}
			else
			{
				avgSpeedInMainLane = 120 / 3.6; // 120 km/h -> m/s
				distanceToMainLaneUp = 100;     // m
			}
			if (pVehDw != NULL)
			{
				distanceToMainLaneDown = pVehDw->getPosition(0) - currentVehPosition;
			}
			else
			{
				distanceToMainLaneDown = 100;
			}

			double SpeedDifferenceWithMainLane = (avgSpeedInMainLane - currSpeed);
			double thresholdCurrentSpeed = 0, thresholdSpeedDifference = 0, thresholdSpace = 0;


			// thresholds as linear functions of the position within the section
			if (numSect == 395 && numLane == 3)
			{
				thresholdCurrentSpeed = (-currentVehPosition / 1 + 58.25) / 3.6;
				if (currentVehPosition > 50)
					thresholdCurrentSpeed = 2.29166666666667;


				thresholdSpeedDifference = (currentVehPosition*2.5 / 5.8 - 3) / 3.6;
				if (currentVehPosition > 50)
					thresholdSpeedDifference = 5.15325670498084;


				thresholdSpace = -0.1 * currentVehPosition + 29.5;
				if (currentVehPosition > 70)
					thresholdSpace = 1;


				if (
					(currentVehPosition > 5
						&& currSpeed > thresholdCurrentSpeed
						&& SpeedDifferenceWithMainLane <= thresholdSpeedDifference
						&& distanceToMainLaneUp > thresholdSpace
						&& distanceToMainLaneDown > thresholdSpace)
					||
					(currentVehPosition > 5
						&& currSpeed <0.1
						&& SpeedDifferenceWithMainLane <= thresholdSpeedDifference
						&& distanceToMainLaneUp >thresholdSpace
						&& distanceToMainLaneDown > thresholdSpace)
					)

				{
					direction = 1;
					isOnRamp = true;
					bool GapAcceptable = vehicle->isGapAcceptable(direction, XPosTargetlane, pVehUp, ShiftUp, pVehDw, ShiftDw);
					isOnRamp = false;
					if (GapAcceptable)
					{
						//vehicle->assignAcceptedGap(direction, XPosTargetlane, pVehUp, ShiftUp, pVehDw, ShiftDw, threadId);
						vehicle->applyLaneChanging(direction, threadId);
						return true;
					}
				}

			}
			else if (numSect == 386 && numLane == 4)
			{


				thresholdSpeedDifference = (-currentVehPosition / 2.3 + 100.25) / 3.6;
				if (currentVehPosition > 200)
					thresholdSpeedDifference = 3.69263285024155;


				SpeedDifferenceWithMainLane = (currentVehPosition*2.5 / 4.8 - 2) / 3.6;
				if (currentVehPosition > 200)
					SpeedDifferenceWithMainLane = 28.3796296296296;


				thresholdSpace = -0.1 * currentVehPosition + 29.5;
				if (currentVehPosition > 200)
					thresholdSpace = 5;



				if (
					(currentVehPosition > 5
						&& currSpeed > thresholdSpeedDifference
						&& SpeedDifferenceWithMainLane <= SpeedDifferenceWithMainLane
						&& distanceToMainLaneUp > thresholdSpace
						&& distanceToMainLaneDown > thresholdSpace)
					||
					(currentVehPosition > 5
						&& currSpeed <0.1
						&& SpeedDifferenceWithMainLane <= SpeedDifferenceWithMainLane
						&& distanceToMainLaneUp >thresholdSpace
						&& distanceToMainLaneDown > thresholdSpace))
				{
					direction = 1;
					isOnRamp = true;
					bool GapAcceptable = vehicle->isGapAcceptable(direction, XPosTargetlane, pVehUp, ShiftUp, pVehDw, ShiftDw);
					isOnRamp = false;
					if (GapAcceptable)
					{

						//	vehicle->assignAcceptedGap(direction, XPosTargetlane, pVehUp, ShiftUp, pVehDw, ShiftDw, threadId);
						vehicle->applyLaneChanging(direction, threadId);
						return true;
					}

				}

			}
			else if (numSect == 395 && numLane == 2)
			{

				if (currSpeed > 20 / 3.6) // 20m/s
				{
					return false; // don't use Heuristic lane-changing model
				}
				else
				{
					double	coefficientOfCurrentSpeed = 0.8;
					double	coefficientOfLeaderSpeed = 1.0;

					double averageSpeedDwAndUp = 0.0;
					int count = 0;

					if (pVehDw != NULL)
					{
						averageSpeedDwAndUp += pVehDw->getSpeed(0);
						count++;
					}
					if (pVehUp != NULL)
					{
						averageSpeedDwAndUp += pVehUp->getSpeed(0);
						count++;
					}

					if (count > 0)
					{
						averageSpeedDwAndUp = averageSpeedDwAndUp / (double)count;
					}
					// if pVehDw and pVehUp doesn't exist nextSpeed=0;


					double tempShift = 0;
					double speedLeader = vehicle->getLeader(tempShift)->getSpeed(0);


					if ((averageSpeedDwAndUp > coefficientOfCurrentSpeed * currSpeed) && (averageSpeedDwAndUp > coefficientOfLeaderSpeed * speedLeader))
						// I can not understand the attempt of this module. by wanglong 
						// in case nextspeed is zero lanechanging is not applied
					{
						direction = -1;
						bool GapAcceptable = vehicle->isGapAcceptable(direction, XPosTargetlane, pVehUp, ShiftUp, pVehDw, ShiftDw);
						if (GapAcceptable)
						{
							isSegmRamp = true;
							vehicle->assignAcceptedGap(direction, XPosTargetlane, pVehUp, ShiftUp, pVehDw, ShiftDw, threadId);
							//vehicle->applyLaneChanging(direction, threadId);
							isSegmRamp = false;
							return true;
						}
					}
				}

			}
			else if ((numSect == 386) && (numLane == 3))
			{

				if (currSpeed > 20 / 3.6) // 20m/s
				{
					return false; // don't use Heuristic lane-changing model
				}
				else
				{
					double	coefficientOfCurrentSpeed = 1.1;
					double	coefficientOfLeaderSpeed = 1.3;

					double averageSpeedDwAndUp = 0.0;
					int count = 0;

					if (pVehDw != NULL)
					{
						averageSpeedDwAndUp += pVehDw->getSpeed(0);
						count++;
					}
					if (pVehUp != NULL)
					{
						averageSpeedDwAndUp += pVehUp->getSpeed(0);
						count++;
					}

					if (count > 0)
					{
						averageSpeedDwAndUp = averageSpeedDwAndUp / (double)count;
					}
					// if pVehDw and pVehUp doesn't exist nextSpeed=0;


					double tempShift = 0;
					double speedLeader = vehicle->getLeader(tempShift)->getSpeed(0);


					if ((averageSpeedDwAndUp > coefficientOfCurrentSpeed * currSpeed) && (averageSpeedDwAndUp > coefficientOfLeaderSpeed * speedLeader))
						// I can not understand the attempt of this module. by wanglong 
						// in case nextspeed is zero lanechanging is not applied
					{
						direction = -1;
						bool GapAcceptable = vehicle->isGapAcceptable(direction, XPosTargetlane, pVehUp, ShiftUp, pVehDw, ShiftDw);
						if (GapAcceptable)
						{
							isSegmRamp = true;
							vehicle->assignAcceptedGap(direction, XPosTargetlane, pVehUp, ShiftUp, pVehDw, ShiftDw, threadId);
							//vehicle->applyLaneChanging(direction, threadId);
							isSegmRamp = false;
							return true;
						}
					}
				}

			}

		}

	}
	/************ otherwise, normal lane change *************/
	if (isACC && useMOBILinNormal)
	{
		direction = MOBILDirection(vehicle);
		vehicle->applyLaneChanging(direction, threadId);
		return true;
	}
	else
	{
		// use default Gipps lane change model
		return false;

	}

}


bool behavioralModelParticular::avoidCollision(A2SimVehicle *vehicle, A2SimVehicle *vehiclePre, double ShiftPre)
{
	return false;
}

bool behavioralModelParticular::isVehicleGivingWay(A2SimVehicle *vehicleGiveWay, A2SimVehicle *vehiclePrio, yieldInfo *givewayInfo, int &Yield)
{
	return false;
}

double behavioralModelParticular::computeMinimumGap(A2SimVehicle *vehicleUp, A2SimVehicle *vehicleDown, double Xup, double Vup, double Xdw, double Vdw, double Gap, bool ImprudentCase, bool VehicleIspVehDw)
{
	double GapMin = 0;
	if (useHeuristicLaneChangeModel)
	{
		if (isOnRamp)
			return 0.0;

		if (isSegmRamp)
			return 10.0;
	}
	if (useIDM)
	{
		GapMin = getIDMDesiredGap((simVehicleParticular*)vehicleUp, (simVehicleParticular*)vehicleDown, Vup, Vdw, Gap);
	}
	else
	{
		GapMin = getGippsMinimumGap((simVehicleParticular*)vehicleUp, (simVehicleParticular*)vehicleDown, Xup, Vup, Xdw, Vdw, Gap, ImprudentCase, VehicleIspVehDw);
	}
	return GapMin;
}

// for IDM implementation
double behavioralModelParticular::computeCarFollowingAccelerationComponentSpeed(A2SimVehicle *vehicle, double VelActual, double VelDeseada, double RestoCiclo)
{
	double VelPropia = 0;

	//if not inflow section
	if (
		(useIDM)
		&& vehicle->getIdCurrentSection() != 671
		&& vehicle->getIdCurrentSection() != 664
		&& vehicle->getIdCurrentSection() != 670
		&& vehicle->getIdCurrentSection() != 928
		&& vehicle->getIdCurrentSection() != 932

		&& vehicle->getIdCurrentSection() != 556// only used in capacity test
		)
	{
		VelPropia = getIDMAccelerationSpeed((simVehicleParticular*)vehicle, VelActual, VelDeseada, RestoCiclo);
	}
	else
	{
		VelPropia = getGippsAccelerationSpeed((simVehicleParticular*)vehicle, VelActual, VelDeseada, RestoCiclo);
	}
	return VelPropia;
}

double behavioralModelParticular::computeCarFollowingDecelerationComponentSpeed(A2SimVehicle *vehicle, double Shift, A2SimVehicle *vehicleLeader, double ShiftLeader, bool controlDecelMax, bool aside, int time)
{
	double VelImpuesta = 0;
	if ((useIDM)
		&& vehicle->getIdCurrentSection() != 671
		&& vehicle->getIdCurrentSection() != 664
		&& vehicle->getIdCurrentSection() != 670
		&& vehicle->getIdCurrentSection() != 928
		&& vehicle->getIdCurrentSection() != 932

		&& vehicle->getIdCurrentSection() != 556// only used in capacity test
		) {
		VelImpuesta = getIDMDecelerationSpeed((simVehicleParticular*)vehicle, Shift, (simVehicleParticular*)vehicleLeader, ShiftLeader);
	}
	else
	{
		VelImpuesta = getGippsDecelerationSpeed((simVehicleParticular*)vehicle, Shift, (simVehicleParticular*)vehicleLeader, ShiftLeader, controlDecelMax, aside, time);
	}
	return VelImpuesta;
}

//Gipps
double behavioralModelParticular::getGippsAccelerationSpeed(simVehicleParticular *vehicle, double VelActual, double VelDeseada, double RestoCiclo)
{
	// Calcula l'acceleracio tenint en compte les pendents
	double X = VelActual / VelDeseada;
	int numSect = vehicle->getIdCurrentSection();
	double a = vehicle->getAcceleration();
	int id = vehicle->getId();
	double VelPropia = min(VelDeseada, VelActual + 2.5 * vehicle->getAcceleration() * RestoCiclo * (1.0 - X)* sqrt(0.025 + X));
	if (VelPropia < VelActual)
	{
		VelPropia = max(VelPropia, max(0., VelActual + (0.5 * vehicle->getDeceleration() * RestoCiclo)));
	}
	return VelPropia;
}

double behavioralModelParticular::getGippsDecelerationSpeed(simVehicleParticular *vehicle, double Shift, simVehicleParticular *leader, double ShiftLeader, bool controlDecelMax, bool aside, int time)
{
	int id = vehicle->getId();
	int idl = leader->getId();

	double PosAnterior, VelAnterior, PosAnteriorLeader, VelAnteriorLeader;
	double GapAnterior = vehicle->getGap(Shift, leader, ShiftLeader, PosAnterior, VelAnterior, PosAnteriorLeader, VelAnteriorLeader, time);
	double RT = vehicle->getReactionTime();
	double DecelEstimada = 0;
	if (VelAnteriorLeader > Tolerancia)
	{
		DecelEstimada = vehicle->getEstimationOfLeadersDeceleration(leader, VelAnteriorLeader);
	}
	double bn = vehicle->getDeceleration();
	double bnTau = bn*RT;
	double VelImpuesta = bnTau;
	double factorSqrt = 0;
	if (VelAnteriorLeader < Tolerancia)
	{
		factorSqrt = (bnTau* bnTau) - vehicle->getDeceleration() * (2.0 * GapAnterior - (VelAnterior * RT));
	}
	else if (VelAnteriorLeader < DBL_MAX)
	{
		factorSqrt = (bnTau * bnTau) - vehicle->getDeceleration() * (2.0 * GapAnterior - (VelAnterior * RT) - ((VelAnteriorLeader * VelAnteriorLeader) / DecelEstimada));
	}
	else {
		VelImpuesta = DBL_MAX;
	}
	if (factorSqrt > 0)
	{
		VelImpuesta = bnTau + (double)sqrt(factorSqrt);
	}

	if (aside)
	{
		//SpeedAnterior imposed by Car-Following on side lane > Tolerancia to avoid RT at stop
		int id = vehicle->getId();
		int idl = leader->getId();
		double GapMin = getGippsMinimumGap(vehicle, leader, PosAnterior, VelAnterior, PosAnteriorLeader, VelAnteriorLeader, GapAnterior, false, false);
		GapMin = GapAnterior - GapMin;
		if (GapMin < 0)
		{
			double Distance2Obstacle = DBL_MAX;
			if (vehicle->getObstacleType() != eNone)
			{
				Distance2Obstacle = vehicle->getDistance2Obstacle() / abs(vehicle->getNbLaneChanges2ReachNextValidLane());
				Distance2Obstacle = max(0., Distance2Obstacle - max(VelAnteriorLeader*RT, leader->getLength()));
			}
			double minimo = Distance2Obstacle;
			double AdaptationDistance = max(vehicle->getFreeFlowSpeed()*RT, vehicle->getLength());
			if (vehicle->getObstacleType() == eOnRamp)
			{
				minimo = min(minimo, 3 * AdaptationDistance);
			}
			else
			{
				minimo = min(minimo, AdaptationDistance);
			}
			double maximo = max(VelAnteriorLeader, Tolerancia);
			double expParam = 0.5*(1. - VelAnterior / maximo*(1 - (GapMin) / minimo));
			double expValue = (float)exp(expParam);//la función exp en 32/64 bits retorna el mismo valor usándola de este modo
			VelImpuesta = VelAnterior*expValue;
		}
	}
	if (controlDecelMax)
	{
		double VelMin = 0;
		if (aside)
		{
			VelMin = max(0., VelAnterior + vehicle->getDeceleration()*RT);
		}
		else
		{
			VelMin = max(0., VelAnterior + vehicle->getDecelerationMax()*RT);
		}
		if (VelImpuesta < VelMin)
		{
			VelImpuesta = VelMin;
		}
	}
	return VelImpuesta;
}

double behavioralModelParticular::getGippsMinimumGap(simVehicleParticular* pVehUp, simVehicleParticular* pVehDw, double Xup, double Vup, double Xdw, double Vdw, double Gap, bool ImprudentCase, bool VehicleIspVehDw)
{
	double DecelFactorUp = 1;
	if (VehicleIspVehDw)
	{
		DecelFactorUp = pVehDw->getDecelerationVariationFactor(ImprudentCase);
	}
	else
	{
		DecelFactorUp = pVehUp->getDecelerationVariationFactor(ImprudentCase);
	}
	double tau = pVehUp->getReactionTime();
	double GapMin = 0;
	if (Vdw < 0.01)
	{
		GapMin = max(0., 0.5*Vup*tau + max(0., -Vup*Vup / (2 * pVehUp->getDeceleration()) + DecelFactorUp*(1. - 0.5*DecelFactorUp)*pVehUp->getDeceleration()*tau*tau + (1 - DecelFactorUp)*Vup*tau));
		if (DecelFactorUp > -Vup / (pVehUp->getDeceleration())*tau)
		{
			GapMin = max(0., 0.5*Vup*tau);
		}
	}
	else
	{
		double DecelEstimada = pVehUp->getEstimationOfLeadersDeceleration(pVehDw, Vdw);
		GapMin = max(0., (Vdw*Vdw) / (2 * DecelEstimada) + 0.5*Vup*tau + max(0., -Vup*Vup / (2 * pVehUp->getDeceleration()) + DecelFactorUp*(1. - 0.5*DecelFactorUp)*pVehUp->getDeceleration()*tau*tau + (1 - DecelFactorUp)*Vup*tau));
		if (DecelFactorUp > -Vup / (pVehUp->getDeceleration())*tau)
		{
			GapMin = max(0., (Vdw*Vdw) / (2 * DecelEstimada) + 0.5*Vup*tau);
		}
	}
	return GapMin;
}

//IDM

//this method only used in computeCarFollowingAccelerationComponentSpeed() 
double behavioralModelParticular::getIDMAccelerationSpeed(simVehicleParticular *vehicle, double VelActual, double VelDeseada, double RestoCiclo)
{
	double X = VelActual / VelDeseada;
	double a = vehicle->getAcceleration();
	double acceleration = max(vehicle->getDeceleration(), vehicle->getAcceleration()*(1. - pow(X, 4)));
	double speed = max(0., VelActual + acceleration * vehicle->getReactionTime());
	return speed;
}

//this method only used in computeCarFollowingDecelerationComponentSpeed()
double behavioralModelParticular::getIDMDecelerationSpeed(simVehicleParticular *vehicle, double Shift, simVehicleParticular *leader, double ShiftLeader)
{
	bool useGeorgia = true;
	if (useGeorgia)
	{
		int id = vehicle->getId();
		int idl = leader->getId();
		double sh = ShiftLeader;

		simVehicleParticular *tempVeh;
		int idT;


		double a = vehicle->getAcceleration();
		double b = vehicle->getDeceleration();
		double VelAnterior, PosAnterior, VelAnteriorLeader, PosAnteriorLeader;


		double GapAnterior = vehicle->getGap(Shift, leader, ShiftLeader, PosAnterior, VelAnterior, PosAnteriorLeader, VelAnteriorLeader);
		double diff = VelAnterior - VelAnteriorLeader;


		simVehicleParticular *newLeader;
		simVehicleParticular *newVehicle = vehicle;

		double DesiredGap = getIDMDesiredGap(vehicle, leader, VelAnterior, VelAnteriorLeader, GapAnterior);

		double X = VelAnterior / vehicle->getFreeFlowSpeed();
		double bkin = (VelAnterior*VelAnterior) / (2 * GapAnterior);
		double acceleration;

		acceleration = max(b, a*(1 - pow(X, 4) - (DesiredGap / GapAnterior)*(DesiredGap / GapAnterior)));

		double speed = max(0., VelAnterior + acceleration * vehicle->getReactionTime());
		return speed;

	}
	double acceleration = 0;
	double currentSpeed = 0;

	currentSpeed = vehicle->getSpeed(vehicle->isUpdated());
	acceleration = get_IDM_acceleration(vehicle, leader);

	double speed = max(0., currentSpeed + acceleration * vehicle->getReactionTime());
	return speed;
}

double behavioralModelParticular::get_IDM_acceleration(simVehicleParticular*vehicle, simVehicleParticular*leader)
{
	if (vehicle == NULL) return 0;
	double acceleration = 0;
	double shift_temp = 0;
	double  shiftLeader = 0;

	double a = vehicle->getAcceleration();
	double b = vehicle->getDeceleration();
	double currentSpeed, pos_cur, speed_leader, pos_leader;
	double gapToLeader = vehicle->getGap(shift_temp, leader, shiftLeader, pos_cur, currentSpeed, pos_leader, speed_leader);
	currentSpeed = vehicle->getSpeed(vehicle->isUpdated());
	double desiredGap = 0;
	desiredGap = getIDMDesiredGap(vehicle, leader, currentSpeed, speed_leader, gapToLeader);
	double X = vehicle->getSpeed(vehicle->isUpdated()) / vehicle->getFreeFlowSpeed();
	if (leader != NULL) acceleration = max(vehicle->getDecelerationMax(), a*(1 - pow(X, 4) - (desiredGap / gapToLeader)*(desiredGap / gapToLeader)));
	if (leader == NULL) acceleration = max(vehicle->getDecelerationMax(), a*(1 - pow(X, 4)));



	// for Asymmetric MOBIL
	A2SimVehicle *pVehLeftDw = NULL;
	A2SimVehicle *pVehLeftUp = NULL;
	double shiftUpLeft = 0, shiftDwLeft = 0;
	double speed_leader_left = 0;
	double acceleration_europe = 0;
	double ac = 0, ac_tilde = 0;

	int curLane = vehicle->getIdCurrentLane();
	int numSect = vehicle->getIdCurrentSection();
	int maxLanes = vehicle->getNumberOfLanesInCurrentSection();
	//double speed_average_lane = get_speed_average_bylane(vehicle);
	if (useAsymmetric((simVehicleParticular*)vehicle) && (maxLanes - curLane == 1))// only for middle lane
	{
		//get left lane follower and leader
		vehicle->getUpDown(-1, vehicle->getPosition(vehicle->isUpdated()), pVehLeftUp, shiftUpLeft, pVehLeftDw, shiftDwLeft);
		if (shiftDwLeft > VISIBLELIMIT) pVehLeftDw = NULL;
		ac = acceleration;

		if (pVehLeftDw != NULL)
		{
			speed_leader_left = pVehLeftDw->getSpeed(pVehLeftDw->isUpdated());
			desiredGap = getIDMDesiredGap(vehicle, (simVehicleParticular*)pVehLeftUp, currentSpeed, speed_leader_left, shiftDwLeft);
			ac_tilde = max(b, a*(1 - pow(X, 4) - (desiredGap / shiftDwLeft)*(desiredGap / shiftDwLeft)));
		}
		else
		{
			ac_tilde = max(b, a*(1 - pow(X, 4)));
		}



		if (currentSpeed > speed_leader_left && speed_leader_left > SPEED_CRITICAL)
			acceleration_europe = min(ac, ac_tilde);
		else
			acceleration_europe = ac;
		return acceleration_europe;
	}


	return acceleration;
}
double behavioralModelParticular::get_IDM_acceleration(A2SimVehicle*vehicle_p, A2SimVehicle*leader_p)
{

	simVehicleParticular *vehicle = (simVehicleParticular*)vehicle_p;
	simVehicleParticular *leader = (simVehicleParticular*)leader_p;

	if (vehicle_p == NULL) return 0;
	double acceleration = 0;
	double shift_temp = 0;
	double  shiftLeader = 0;

	double a = vehicle->getAcceleration();
	double b = vehicle->getDeceleration();
	double currentSpeed, pos_cur, speed_leader, pos_leader;
	double gapToLeader = vehicle->getGap(shift_temp, leader, shiftLeader, pos_cur, currentSpeed, pos_leader, speed_leader);
	currentSpeed = vehicle->getSpeed(vehicle->isUpdated());
	double desiredGap = 0;
	desiredGap = getIDMDesiredGap(vehicle, leader, currentSpeed, speed_leader, gapToLeader);
	double X = vehicle->getSpeed(vehicle->isUpdated()) / vehicle->getFreeFlowSpeed();
	if (leader != NULL) acceleration = max(vehicle->getDecelerationMax(), a*(1 - pow(X, 4) - (desiredGap / gapToLeader)*(desiredGap / gapToLeader)));
	if (leader == NULL) acceleration = max(vehicle->getDecelerationMax(), a*(1 - pow(X, 4)));


	// for Asymmetric MOBIL
	A2SimVehicle *pVehLeftDw = NULL;
	A2SimVehicle *pVehLeftUp = NULL;
	double shiftUpLeft = 0, shiftDwLeft = 0;
	double speed_leader_left = 0;
	double acceleration_europe = 0;
	double ac = 0, ac_tilde = 0;

	int curLane = vehicle->getIdCurrentLane();
	int numSect = vehicle->getIdCurrentSection();
	int maxLanes = vehicle->getNumberOfLanesInCurrentSection();
	//double speed_average_lane = get_speed_average_bylane(vehicle);
	if (useAsymmetric((simVehicleParticular*)vehicle) && (maxLanes - curLane == 1))// only for middle lane
	{
		//get left lane follower and leader
		vehicle->getUpDown(-1, vehicle->getPosition(vehicle->isUpdated()), pVehLeftUp, shiftUpLeft, pVehLeftDw, shiftDwLeft);
		double gap_to_leader_left = vehicle->getGap(shift_temp, pVehLeftDw, shiftLeader, pos_cur, currentSpeed, pos_leader, speed_leader);
		if (shiftDwLeft > VISIBLELIMIT) pVehLeftDw = NULL;
		ac = acceleration;

		if (pVehLeftDw != NULL)
		{
			speed_leader_left = pVehLeftDw->getSpeed(pVehLeftDw->isUpdated());
			desiredGap = getIDMDesiredGap(vehicle, (simVehicleParticular*)pVehLeftUp, currentSpeed, speed_leader_left, shiftDwLeft);
			ac_tilde = max(b, a*(1 - pow(X, 4) - (desiredGap / gap_to_leader_left)*(desiredGap / gap_to_leader_left)));
		}
		else
		{
			ac_tilde = max(b, a*(1 - pow(X, 4)));
		}



		if (currentSpeed > speed_leader_left && speed_leader_left > SPEED_CRITICAL)
			acceleration_europe = min(ac, ac_tilde);
		else
			acceleration_europe = ac;
		return acceleration_europe;
	}


	return acceleration;
}

//DesiredGap
double behavioralModelParticular::getIDMDesiredGap(simVehicleParticular* pVehCur, simVehicleParticular* pVehDw, double speed_current, double speed_leader, double gap_to_leader)
{
	if (pVehCur == NULL || pVehDw == NULL) return 0;

	bool isACC = pVehCur->getIsACC();

	double timeGap = pVehCur->getMinimumHeadway();
	if (isACC)
	{
		timeGap = pVehCur->getACCTimeGap();
	}

	double a = pVehCur->getAcceleration();
	double b = -pVehCur->getDeceleration();
	double desiredGap = pVehCur->getMinimumDistanceInterVeh() + max(0., speed_current*timeGap + speed_current*(speed_current - speed_leader) / (2 * sqrt(a*b))); // replace with 2 * sqrt
	return desiredGap;
}

double behavioralModelParticular::max(double a, double b)
{
	if (a > b) return a;
	else return b;
}

double behavioralModelParticular::min(double a, double b)
{
	if (a > b) return b;
	else return a;
}

double behavioralModelParticular::generateTimeGapOfACC(double lowLimit, double upLimit)
{
	double timeGap = AKIGetRandomNumber() * (upLimit - lowLimit) + lowLimit;
	return timeGap;
}

double behavioralModelParticular::generateGaussianNoise(double mu, double sigma)
{
	const double epsilon = std::numeric_limits<double>::min();
	const double two_pi = 2.0*3.14159265358979323846;

	static double z0, z1;
	static bool generate;
	generate = !generate;

	if (!generate)
		return z1 * sigma + mu;

	double u1, u2;
	do
	{
		u1 = rand() * (1.0 / RAND_MAX);
		u2 = rand() * (1.0 / RAND_MAX);
	} while (u1 <= epsilon);

	z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);
	z1 = sqrt(-2.0 * log(u1)) * sin(two_pi * u2);
	return z0 * sigma + mu;
}
//double behavioralModelParticular::get_speed_average_bylane(simVehicleParticular * vehicle)
//{
//
//	int lane_current = vehicle->getIdCurrentLane();
//	int section_current = vehicle->getIdCurrentSection();
//	
//	return vehicle->getLaneDensity(lane_current);
//
//}

//determining by speed, section and lane   
bool behavioralModelParticular::useAsymmetric(simVehicleParticular * vehicle)
{
	int curlane = vehicle->getIdCurrentLane();
	int maxlanes = vehicle->getNumberOfLanesInCurrentSection();
	int section_current = vehicle->getIdCurrentSection();
	double speed_current = vehicle->getSpeed(vehicle->isUpdated()) / 1000 * 3600;	// m/s -> km/h
	if (
		// global ON-OFF
		useAsymmetricMOBIL
		// lane
		&& (maxlanes - curlane == 1
			|| maxlanes - curlane == 0)
		// speed
		&& speed_current > SPEED_CRITICAL
		// ramp
		&& section_current != 396
		&& section_current != 401
		&& section_current != 415
		&& section_current != 582
		// inflow section
		&& section_current != 928
		&& section_current != 932
		)
		return true;
	else
		return false;
}

// 给出MOBIL 决定的换道方向 ，default: politeness factor p: 0.5, a_threshold: 1, bsafe: -5
int behavioralModelParticular::MOBILDirection(A2SimVehicle *vehicle, double politeness_factor, double a_threshold, double bsafe)
{


	double a_bias = a_threshold*1.1;	// Asymmetric MOBIL

	int curLane = vehicle->getIdCurrentLane();
	int numSect = vehicle->getIdCurrentSection();
	int maxLanes = vehicle->getNumberOfLanesInCurrentSection();


	int direction = 0;

	double profit_to_left = 0, profit_to_right = 0;



	double	ac_Left_LC = 0;			//ac*	//turn left
	double	ac_Left_NOLC = 0;		//ac	//turn left
	double	an_Left_LC = 0;			//an*	//turn left
	double	an_Left_NOLC = 0;		//an	//turn left
	double	ao_Left_LC = 0;			//ao*	//turn left
	double	ao_Left_NOLC = 0;		//ao	//turn left


	double	ac_Right_LC = 0;			//ac*	//turn Right
	double	ac_Right_NOLC = 0;			//ac	//turn Right
	double	an_Right_LC = 0;			//an*	//turn Right
	double	an_Right_NOLC = 0;			//an	//turn Right
	double	ao_Right_LC = 0;			//ao*	//turn Right
	double	ao_Right_NOLC = 0;			//ao	//turn right

	double XPosTargetlaneLeft = 0;
	double XPosTargetlaneRight = 0;

	A2SimVehicle *pVehLeftDw = NULL;
	A2SimVehicle *pVehLeftUp = NULL;
	A2SimVehicle *pVehRightDw = NULL;
	A2SimVehicle *pVehRightUp = NULL;
	A2SimVehicle *pVehCurUp = NULL;
	A2SimVehicle *pVehCurDw = NULL;

	double shiftCurUp = 0, shiftCurDw = 0;
	double ShiftUpLeft = 0, ShiftDwLeft = 0;
	double ShiftUpRight = 0, ShiftDwRight = 0;


	/*******************************************************************/
	/**********************MOBIL CRITERION*****************************/


	//get current lane follower and leader
	vehicle->getUpDown(0, vehicle->getPosition(0), pVehCurUp, shiftCurUp, pVehCurDw, shiftCurDw);

	//get left lane follower and leader
	vehicle->getUpDown(-1, vehicle->getPosition(0), pVehLeftUp, ShiftUpLeft, pVehLeftDw, ShiftDwLeft);

	//get right lane follower and leader
	vehicle->getUpDown(1, vehicle->getPosition(0), pVehRightUp, ShiftUpRight, pVehRightDw, ShiftDwRight);


	/*visible limit, if a vehicle is not visible, it can not influence current vehicle */
	double gapLeftToUp, gapLeftToDw;
	gapLeftToUp = VISIBLELIMIT;
	gapLeftToDw = VISIBLELIMIT;
	if (shiftCurUp > VISIBLELIMIT) pVehCurUp = NULL;
	if (shiftCurDw > VISIBLELIMIT) pVehCurDw = NULL;

	if (ShiftUpLeft > VISIBLELIMIT) pVehLeftUp = NULL;
	if (ShiftDwLeft > VISIBLELIMIT) pVehLeftDw = NULL;

	if (ShiftUpRight > VISIBLELIMIT) pVehRightUp = NULL;
	if (ShiftDwRight > VISIBLELIMIT) pVehRightDw = NULL;




	/**********************LEFT Calculation*********************************/
	if (curLane < maxLanes)
	{
		ac_Left_LC = get_IDM_acceleration(vehicle, pVehLeftDw);//ac*

		ac_Left_NOLC = get_IDM_acceleration(vehicle, pVehCurDw); // ac

		an_Left_LC = get_IDM_acceleration(pVehLeftUp, vehicle);	//an*

		an_Left_NOLC = get_IDM_acceleration(pVehLeftUp, pVehLeftDw);//an

		ao_Left_LC = get_IDM_acceleration(pVehCurUp, pVehCurDw); //a0*

		ao_Left_NOLC = get_IDM_acceleration(pVehCurUp, vehicle);  //a0
	}
	/**********************RIGHT Calculation*********************************/
	if (curLane > 1)
	{
		ac_Right_LC = get_IDM_acceleration(vehicle, pVehRightDw);  //ac*

		ac_Right_NOLC = get_IDM_acceleration(vehicle, pVehCurDw);    // ac

		an_Right_LC = get_IDM_acceleration(pVehRightUp, vehicle); //an*

		an_Right_NOLC = get_IDM_acceleration(pVehRightUp, pVehRightDw);  //an

		ao_Right_LC = get_IDM_acceleration(pVehCurUp, pVehCurDw); //a0*

		ao_Right_NOLC = get_IDM_acceleration(pVehCurUp, vehicle);  //a0
	}



	/****************Special Section:Lane drop  || on ramp******************/

	/*

	if (numSect == 396 && numLane == 3 && pVehCurDw == NULL)// lane drop
	{
		double	distToLaneDrop = 100 - vehicle->getPosition(0);

		simVehicleParticular* vehicleTemp = (simVehicleParticular*)vehicle;

		double timeGap = vehicleTemp->GetACCTimeGap();
		double speed = vehicle->getSpeed(0);
		double a = vehicle->getAcceleration();
		double b = -vehicle->getDeceleration();
		double desiredGap = vehicle->getMinimumDistanceInterVeh() + max(0., speed*timeGap + speed*(speed - 0) / (2 * sqrt(a*b))); // replace with 2 * sqrt
		double speedComponent = vehicle->getSpeed(0) / vehicle->getFreeFlowSpeed();
		double distanceComponent = distToLaneDrop / desiredGap;
		ac_Right_NOLC = a*(1 - pow(speedComponent, 4) - pow(distanceComponent, 2));
		ao_Left_LC = 0;
	}

	*/

	/************************ calculating lane change profits **************/
	// profits to LEFT
	if (useAsymmetric((simVehicleParticular*)vehicle) && (maxLanes - curLane == 1))
	{
		profit_to_left
			=
			ac_Left_LC - ac_Left_NOLC
			+
			politeness_factor*(an_Left_LC - an_Left_NOLC);//LEFT
	}
	else
	{
		profit_to_left
			=
			ac_Left_LC - ac_Left_NOLC
			+
			politeness_factor*(an_Left_LC - an_Left_NOLC + ao_Left_LC - ao_Left_NOLC);//LEFT
	}
	// profits to RIGHT
	if (useAsymmetric((simVehicleParticular*)vehicle) && (maxLanes - curLane == 0))
	{
		profit_to_right
			=
			ac_Right_LC - ac_Right_NOLC
			+
			politeness_factor*(ao_Right_LC - ao_Right_NOLC);//Right
	}
	else
	{
		profit_to_right
			=
			ac_Right_LC - ac_Right_NOLC
			+
			politeness_factor*(an_Right_LC - an_Right_NOLC + ao_Right_LC - ao_Right_NOLC);//Right
	}




	/************************Determining direction****************************/

	//非对称式，	只有在最左侧车道车辆考虑向右、第二左车道车辆考虑向左 两种情况才使用非对称换道

	if (useAsymmetric((simVehicleParticular*)vehicle))
	{
		if ((maxLanes - curLane == 1))	//asymmetric left
		{
			if ((profit_to_left > a_threshold + a_bias))
				direction = -1;
			else if (profit_to_right > a_threshold)//symmetric right
				direction = 1;
			else
				direction = 0;
		}
		else if (maxLanes - curLane == 0)	//asymmetric right
		{
			if (profit_to_right > a_threshold - a_bias)
				direction = 1;
			else
				direction = 0;
		}
		else
			direction = 0;
	}
	else //对称式
	{
		if ((profit_to_left > a_threshold) && (profit_to_right > a_threshold) && (profit_to_left > 0) && (profit_to_right > 0))
		{
			if (profit_to_right > profit_to_left)
				direction = 1;

			else if (profit_to_right < profit_to_left)
				direction = -1;
		}
		else if ((profit_to_left < a_threshold) && (profit_to_right > a_threshold))
			direction = 1;
		else if ((profit_to_left > a_threshold) && (profit_to_right < a_threshold))
			direction = -1;
		else if (a_threshold <= 0)//force lane change
		{
			if (profit_to_left >= a_threshold)
				direction = -1;
			if (profit_to_right >= a_threshold)
				direction = 1;
		}
		else
			direction = 0;
	}

	//safe criterion
	if (an_Left_LC < bsafe && direction == -1) direction = 0;
	if (an_Right_LC < bsafe && direction == 1) direction = 0;
	//不能向入匝车道变道
	if ((numSect == 404 || numSect == 423 || numSect == 1238) && (curLane == 2) && (direction == 1))
	{
		direction = 0;
	}






	/************ TEST OUTPUT ************/
	bool needTest = false;
	if (needTest)
	{
		double currentTime = AKIGetCurrentSimulationTime();
		int id_veh = vehicle->getId();//vehicle id
		string laneChangeTestName = "LCaccelerationTEST.txt";
		string laneChangeTestNamePath;
		laneChangeTestNamePath = DATAPATH + laneChangeTestName;
		ofstream laneChangeTest;
		if (currentTime < 20)
		{
			laneChangeTest.open(laneChangeTestNamePath, ios::trunc);
			laneChangeTest << "TIME" << currentTime << endl;
			laneChangeTest.close();
		}

		if (currentTime > 6 * 60 + 40 && id_veh == 37)
		{



			laneChangeTest.open(laneChangeTestNamePath, ios::app);

			int vehLeftDwID = 0;
			int vehLeftUpID = 0;
			int vehCurDwID = 0;
			int vehCurUpID = 0;
			int vehRightUpID = 0;
			int vehRightDwID = 0;

			if (pVehLeftDw != NULL)vehLeftDwID = pVehLeftDw->getId();
			if (pVehLeftUp != NULL)vehLeftUpID = pVehLeftUp->getId();
			if (pVehCurDw != NULL)vehCurDwID = pVehCurDw->getId();
			if (pVehCurUp != NULL)vehCurUpID = pVehCurUp->getId();
			if (pVehRightDw != NULL)vehRightDwID = pVehRightDw->getId();
			if (pVehRightUp != NULL)vehRightUpID = pVehRightUp->getId();


			laneChangeTest
				<< "LEFT TIME" << currentTime << "\n"
				<< "ac~=" << ac_Left_LC << ","
				<< "ac=" << ac_Left_NOLC << ","
				<< "an~=" << an_Left_LC << ","
				<< "an=" << an_Left_NOLC << ","
				<< "ao~=" << ao_Left_LC << ","
				<< "ao=" << ao_Left_NOLC << "\n"
				<< "Right TIME" << currentTime << "\n"
				<< "ac~=" << ac_Right_LC << ","
				<< "ac=" << ac_Right_NOLC << ","
				<< "an~=" << an_Right_LC << ","
				<< "an=" << an_Right_NOLC << ","
				<< "ao~=" << ao_Right_LC << ","
				<< "ao=" << ao_Right_NOLC << "\n"
				<< "ID=" << id_veh << ","
				<< "CurrentSection=" << numSect << ","
				<< " vehLeftDwID=" << vehLeftDwID << ","
				<< "vehLeftUpID=" << vehLeftUpID << ","
				<< "vehCurDwID=" << vehCurDwID << ","
				<< "vehCurUpID=" << vehCurUpID << ","
				<< "vehRightUpID=" << vehRightUpID << ","
				<< " vehRightDwID=" << vehRightDwID << "\n"
				<< "curLane=" << curLane << ","
				<< "maxLanes=" << maxLanes << ","
				<< "current_speed=" << vehicle->getSpeed(vehicle->isUpdated()) << ","
				<< "useAsymmetric？" << useAsymmetric((simVehicleParticular*)vehicle) << "\n"
				<< "向左增益" << profit_to_left << ","
				<< "向右增益" << profit_to_right << ","
				<< "direction=" << direction << "\n\n" << endl;


			laneChangeTest.close();
		}
	}


	return direction;
}


