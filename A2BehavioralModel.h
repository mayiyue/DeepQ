//-*-Mode: C++;-*-
#ifndef _A2BehavioralModel_h_
#define _A2BehavioralModel_h_

#include "A2BehavioralModelUtil.h"
#include "A2SimVehicle.h"

//---- A2BehavioralModel -----------------------------------------------------------

/*! The objective of this class is to update all the vehicles every simulation step.

	The user must rewrite its two virtual methods as:
	- The updateVehicle method will be called by Aimsun Micro for every simulation step.
	  No order of call is guaranty between different updates.
	- The arrivalNewVehicle will create a new vehicle. Usually this method will return
	  not an instance of A2SimVehicle but an instance of a subclass of this class.
*/
class A2BEHAVIORALEXPORT A2BehavioralModel
{
public:

	A2BehavioralModel();
	virtual ~A2BehavioralModel();

	virtual double getSimStep();
	virtual bool evaluateCarFollowing( A2SimVehicle *vehicle, double &newpos, double &newspeed) = 0;
	virtual bool evaluateLaneChanging( A2SimVehicle *vehicle,int threadId ) = 0;
	virtual bool avoidCollision(A2SimVehicle *vehicle,A2SimVehicle *vehiclePre,double ShiftPre)=0;
	virtual bool isVehicleGivingWay(A2SimVehicle *vehicleGiveWay, A2SimVehicle *vehiclePrioritary, yieldInfo *givewayInfo, int &Yield)=0;
	virtual double computeCarFollowingAccelerationComponentSpeed (A2SimVehicle *vehicle,double VelActual,double VelDeseada, double RestoCiclo)=0;
	virtual double computeCarFollowingDecelerationComponentSpeed (A2SimVehicle *vehicle,double Shift,A2SimVehicle *vehicleLeader,double ShiftLeader,bool controlDecelMax=false, bool aside=false,int time=1)=0;
	virtual double computeMinimumGap(A2SimVehicle *vehicleUp,A2SimVehicle *vehicleDown,double Xup,double Vup,double Xdw,double Vdw,double Gap,bool ImprudentCase=false,bool VehicleIspVehDw=false)=0;

	/*! Creates a new vehicle.
		*/
	virtual A2SimVehicle * arrivalNewVehicle( void *handlerVehicle, unsigned short idHandler, bool isFictitiousVeh) = 0;

	/*! It is called whenever a vehicle exits the system.
		*/
	virtual void removedVehicle( void *handlerVehicle, unsigned short idHandler, A2SimVehicle * a2simVeh ) = 0;
};

#endif
