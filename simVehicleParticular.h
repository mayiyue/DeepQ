//-*-Mode: C++;-*-
#ifndef _simVehicleParticular_h_
#define _simVehicleParticular_h_

#include "A2BehavioralModelUtil.h"
#include "A2SimVehicle.h"

class A2BEHAVIORALEXPORT simVehicleParticular: public A2SimVehicle
{
private:
	float newAttribute;

	bool isCACC;
	
	bool isSmartVehicle;
	double aCCModeTimeGap;
public:
	
	simVehicleParticular ( void *handlerVehicle, unsigned short idhandler,bool isFictitiousVeh );
	~ simVehicleParticular ();

	void setIsCACC(bool setValue);
	const double getACCModeTimeGap() const;
	const bool getIsCACC() const;
	void setTimeGapOfACC(double setValue);

	const bool getIsSmartVehicle() const;
	void setIsSmartVehicle(bool setValue);


	void setCACCPlatoonPosition(int setValue);
	void setPreT_CACC(double setValue);

	int const platoonMaxSizeCACC = 5;
	int platoonPositionCACC;
	const int getPlatoonPositionCACC()const;

	const double getPreT_CACC()const;
	double preT_CACC;




};

#endif
