//-*-Mode: C++;-*-
#ifndef _simVehicleParticular_h_
#define _simVehicleParticular_h_

#include "A2BehavioralModelUtil.h"
#include "A2SimVehicle.h"

class A2BEHAVIORALEXPORT simVehicleParticular: public A2SimVehicle
{
private:
	float newAttribute;

	bool isACC;
	
	bool isSmartVehicle;
	double aCCTimeGap;
public:
	
	simVehicleParticular ( void *handlerVehicle, unsigned short idhandler,bool isFictitiousVeh );
	~ simVehicleParticular ();

	void setIsACC(bool setValue);
	const double getACCTimeGap() const;
	const bool getIsACC() const;
	void setTimeGapOfACC(double setValue);

	const bool getIsOptimizedVehicle() const;
	void setIsOptimizedVehicle(bool setValue);



	//const bool GetIsACC() const;
	//void SetIsACC(bool isACC);

};

#endif
