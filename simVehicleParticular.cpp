#include "simVehicleParticular.h"

simVehicleParticular:: simVehicleParticular ( void *handlerVehicle, unsigned short idhandler,bool isFictitiousVeh ) : A2SimVehicle( handlerVehicle, idhandler,isFictitiousVeh )
{
	isOptimizedVehicle = false;
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


const bool simVehicleParticular::getIsOptimizedVehicle() const
{
	return isOptimizedVehicle;
}
void simVehicleParticular::setIsOptimizedVehicle(bool setValue)
{
	isOptimizedVehicle = setValue;
}
