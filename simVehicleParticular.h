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

	const bool getIsSmartVehicle() const;
	void setIsSmartVehicle(bool setValue);


	void setPlatoonPosition(int setValue);
	void setPreT_CACC(double setValue);

	int const platoonMaxSize = 5;
	int platoonPosition;
	const int getPlatoonPosition()const;

	const double getPreT_CACC()const;
	double preT_CACC;




};

#endif
