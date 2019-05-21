#include "simVehicleParticular.h"

simVehicleParticular:: simVehicleParticular ( void *handlerVehicle, unsigned short idhandler,bool isFictitiousVeh ) : A2SimVehicle( handlerVehicle, idhandler,isFictitiousVeh )
{
	isSmartVehicle = false;
	isACC = false;

	
}
simVehicleParticular::~simVehicleParticular ()
{
}


const double simVehicleParticular::getACCTimeGap() const
{
	return aCCTimeGap;
}
const bool simVehicleParticular::getIsACC() const
{
	return isACC;
}
void simVehicleParticular::setTimeGapOfACC(double setValue)
{
	aCCTimeGap = setValue;
}
void simVehicleParticular::setIsACC(bool setValue)
{
	isACC = setValue;
}


const bool simVehicleParticular::getIsSmartVehicle() const
{
	return isSmartVehicle;
}
void simVehicleParticular::setIsSmartVehicle(bool setValue)
{
	isSmartVehicle = setValue;
}




const int simVehicleParticular::getPlatoonPosition() const
{
	return platoonPosition;
}
void simVehicleParticular::setPlatoonPosition(int setValue)
{
	platoonPosition = setValue;
}



const double simVehicleParticular::getPreT_CACC() const
{
	return preT_CACC;
}
void simVehicleParticular::setPreT_CACC(double setValue)
{
	preT_CACC = setValue;
}
