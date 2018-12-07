//-*-Mode: C++;-*-
#ifndef _behavioralModelParticular_h_
#define _behavioralModelParticular_h_

#include "A2BehavioralModelUtil.h"
#include "A2BehavioralModel.h"
#include "simVehicleParticular.h"



enum Type {eNone, eReservedLane, eTurning, eNotAllowedInfluence, eNotAllowed, ePTStopInfluence, eOnRamp, eLaneClosureInfluence, eIncidentInfluence, eLaneClosure, eIncident, ePTStop};



class A2BEHAVIORALEXPORT behavioralModelParticular: public A2BehavioralModel
{
private:
	int seed;
	double p_distance;

	double max(double a, double b);
	double min(double a, double b);
	bool isOnRamp, isSegmRamp;

public:
	
	
	behavioralModelParticular();
	~behavioralModelParticular();
	
	simVehicleParticular * arrivalNewVehicle( void *handlerVehicle, unsigned short idHandler, bool isFictitiousVeh);
	virtual void removedVehicle( void *handlerVehicle, unsigned short idHandler, A2SimVehicle * a2simVeh );
	bool evaluateCarFollowing(A2SimVehicle *vehicle, double &newpos, double &newspeed);
	bool evaluateLaneChanging(A2SimVehicle *vehicle, int threadId);
	bool isVehicleGivingWay( A2SimVehicle *vehicleGiveWay, A2SimVehicle *vehiclePrio, yieldInfo *givewayInfo, int &Yield);
	double computeCarFollowingAccelerationComponentSpeed(A2SimVehicle *vehicle,double VelActual,double VelDeseada, double RestoCiclo);
	double computeCarFollowingDecelerationComponentSpeed (A2SimVehicle *vehicle,double Shift,A2SimVehicle *vehicleLeader,double ShiftLeader,bool controlDecelMax=false, bool aside=false,int time=1);
	double computeMinimumGap(A2SimVehicle *vehicleUp,A2SimVehicle *vehicleDown,double Xup,double Vup,double Xdw,double Vdw,double Gap,bool ImprudentCase=false, bool VehicleIspVehDw=false);

	bool avoidCollision(A2SimVehicle *vehicle, A2SimVehicle *vehiclePre, double ShiftPre);

	double generateGaussianNoise(double mu, double sigma);
	//double get_speed_average_bylane(simVehicleParticular * vehicle);
	double generateTimeGapOfACC(double downLimit, double upLimit);



	bool useAsymmetric(simVehicleParticular * vehicle);

	int MOBILDirection(A2SimVehicle * vehicle, double p = 0.5, double a_threshold = 1, double bsafe = -5);

	double getGippsAccelerationSpeed(simVehicleParticular *vehicle,double VelActual,double VelDeseada, double RestoCiclo);
	double getGippsDecelerationSpeed(simVehicleParticular *vehicle,double Shift,simVehicleParticular *leader,double ShiftLeader,bool controlDecelMax,bool aside,int time);
	double getGippsMinimumGap(simVehicleParticular* pVehUp,simVehicleParticular* pVehDw,double Xup,double Vup,double Xdw,double Vdw,double Gap,bool ImprudentCase,bool VehicleIspVehDw);

	double getIDMAccelerationSpeed(simVehicleParticular *vehicle,double VelActual,double VelDeseada, double RestoCiclo);
	double getIDMDecelerationSpeed(simVehicleParticular *vehicle,double Shift,simVehicleParticular *leader,double ShiftLeader);
	
	
	double getIDMDesiredGap(simVehicleParticular* pVehUp,simVehicleParticular* pVehDw,double VelAnterior,double VelAnteriorLeader,double GapAnterior);

	double get_IDM_acceleration(simVehicleParticular*vehicle, simVehicleParticular*leader);

	double get_IDM_acceleration(A2SimVehicle * vehicle_p, A2SimVehicle * leader_p);
	


	double getModifiedThreshold(A2SimVehicle * vehicle, int targetLane);

	int getQLearningDecisionAction(A2SimVehicle * vehicle);

	int getStateID_QLearning(A2SimVehicle * vehicle);

	int getMaxQValueAction(int stateID, A2SimVehicle * vehicle);

	int getAvailableActionRandomly_Qlearning(int stateID, A2SimVehicle * vehicle);

	
	void getLeadersAccelerationsDistributionDifference(A2SimVehicle * currentVehicle, double & diff_left_mean, double & diff_left_sd, double & diff_right_mean, double & diff_right_sd);

	

};

#endif


