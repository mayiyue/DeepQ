//-*-Mode: C++;-*-
#ifndef _A2BehavioralModelCreator_h_
#define _A2BehavioralModelCreator_h_

#include "A2BehavioralModelUtil.h"

class A2BehavioralModel;
class A2BehavioralModelCreator;

typedef A2BehavioralModelCreator * (*A2BehavioralModelFactory)();

//---- A2BehavioralModelCreator -----------------------------------------------------------

/*! This class will create a behavioural model class. The developer will subclass it to return a new model.
*/
class A2BEHAVIORALEXPORT A2BehavioralModelCreator
{
public:
	A2BehavioralModelCreator();
	virtual ~A2BehavioralModelCreator();

	/*! Creates a new behavioural model that will be used in the whole network.
	*/
	virtual A2BehavioralModel * newModel() = 0;
};

#endif
