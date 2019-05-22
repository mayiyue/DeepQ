#include "simVehicleParticular.h"

simVehicleParticular:: simVehicleParticular ( void *handlerVehicle, unsigned short idhandler,bool isFictitiousVeh ) : A2SimVehicle( handlerVehicle, idhandler,isFictitiousVeh )
{
	isSmartVehicle = false;
	isCACC = false;

	
}
simVehicleParticular::~simVehicleParticular ()
{
}


const double simVehicleParticular::getACCModeTimeGap() const
{
	return aCCModeTimeGap;
}
const bool simVehicleParticular::getIsCACC() const
{
	return isCACC;
}
void simVehicleParticular::setTimeGapOfACC(double setValue)
{
	aCCModeTimeGap = setValue;
}
void simVehicleParticular::setIsCACC(bool setValue)
{
	isCACC = setValue;
}


const bool simVehicleParticular::getIsSmartVehicle() const
{
	return isSmartVehicle;
}
void simVehicleParticular::setIsSmartVehicle(bool setValue)
{
	isSmartVehicle = setValue;
}




const int simVehicleParticular::getPlatoonPositionCACC() const
{
	return platoonPositionCACC;
}
void simVehicleParticular::setCACCPlatoonPosition(int setValue)
{
	platoonPositionCACC = setValue;
}



const double simVehicleParticular::getPreT_CACC() const
{
	return preT_CACC;
}
void simVehicleParticular::setPreT_CACC(double setValue)
{
	preT_CACC = setValue;
}
