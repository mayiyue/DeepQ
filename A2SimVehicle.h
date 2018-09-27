//-*-Mode: C++;-*-
#ifndef _A2SimVehicle_h_
#define _A2SimVehicle_h_

#include "A2BehavioralModelUtil.h"

typedef A2BEHAVIORALEXPORT struct yieldInfo{
	double distance2ConfVehiclePrio;
	double distance2ConfVehicleGiveWay;
	bool isVehiclePrioWithinVisibility;
	bool isVehiclePrioRealAndReachingConflict;
	bool isVehicleGiveWayComingNext;
	bool isVehiclePrioAfectedByStop;
	bool isVehiclePrioAfectedByYellowBox;
	bool isVehiclePrioAfectedByGiveWay;
	bool isVehiclePrioAffectedByCoop;
	bool isVehiclePrioPrioritaryBasedOnWaitingTime;
	bool isVehiclePrioComingNext;
	bool isVehiclePrioLeaderOfVehicleGiveWay;
	double passingTimeVehiclePrio;
	double leavingTimeVehiclePrio;
	double passingTimeVehicleGiveWay;
	double leavingTimeVehicleGiveWay;
	double safetyMargin;
} yieldInfo;

//---- A2SimVehicle -----------------------------------------------------------

/*! This class contains all the relevant vehicle data, method to access it and methods to update the vehicle every simulation step.

	The developer will rewrite this class to include extra information in the vehicle (if required) and to
	implement the update logic on it.

	See A2BehavioralModel::updateVehicle and A2BehavioralModel::arrivalNewVehicle
*/
class A2BEHAVIORALEXPORT A2SimVehicle
{
public:
	A2SimVehicle( void *handlerVehicle, unsigned short idhandler, bool isFictitiousVeh );
	virtual ~A2SimVehicle();

	/*!
		Internal function to be used only by the microsimulator
	*/
	void setHandlerVehicle( void *handlerVehicle );
	void * getHandlerVehicle() {return handlerVehicle;};

	//FUNCTIONS RELATIVE TO THE VEHICLE´S ATTRIBUTES:

	//Whether vehicle is real or fictitious (lane-changing shadow, incidents, giveway, stop, semafore)
	bool isFictitious() const;

	bool isTrafficLight() const;

	//Id of Vehicle, 0 if Fictitious and >0 otherwise
	int getId() const;

	//Id of the vehicle type
	unsigned int getVehType() const;

	//Maximum acceleration of the vehicle as specified in the vehicle type considering local variations
	double getAcceleration( ) const;

	//Normal deceleration of the vehicle as defined in the vehicle type considering local variations
	double getDeceleration( ) const;

	//Maximum deceleration of the vehicle as defined in the vehicle type
	double getDecelerationMax( ) const;

	//Estimation of the leader´s deceleration used in the Gipps Model
	double getEstimationOfLeadersDeceleration(A2SimVehicle *leader, double VelPre2Consider) const;

	//Reaction time expressed in number of simulation steps considering local variations
	unsigned int getReactionSteps() const;

	//Reaction time in seconds considering local variations
	double getReactionTime() const;

	//ReactionTime at Stop considering local variations
	double getReactionTimeAtStop() const;

	//ReactionTime at Traffic Light considering local variations
	double getReactionTimeAtTrafficLight() const;

	//Length of the vehicle as defined in the vehicle type
    double getLength() const;

	//Minimum gap in front of the vehicle as defined in the vehicle type
	double getMinimumDistanceInterVeh() const;

	//Minimum gap in front of the vehicle considering minimum safety distance
	//and corrected for the vehicle type of the leader (0 if traffic light or incident)
	double getMinimumDistanceInterVeh(A2SimVehicle *leader) const;

	//Maximum Desired Speed of the vehicle for the current lane
    double getFreeFlowSpeed() const;

	//Sensitivity Factor to Leader´s deceleration as defined in the vehicle type
	double getSensitivityFactor() const;

	//Minimum Headway (also know as Gap) in font of the vehicle as defined in the vehicle type
	double getMinimumHeadway() const;

	//Returns true if the vehicle has already been updated, false otherwise
	bool isUpdated() const;

	//Returns the position of the vehicle at time (t – state * simulationStep) (units: m) from the begining of the current A2KSection.
	//A2KSections are the sections used by the micro simulator. All positions used in the micro model refer to the A2KSections!
	double getPosition( const unsigned int state) const;

	//Returns the position of the vehicle at time (t – state * simulationStep) (units: m) from the begining of the current GKSection.
	//The GKSections are the sections provided by the GUI. If they contain segment points, they may be cut into different A2KSections at these segment points by the model.
	double getPositionInGKSection( const unsigned int state) const;

	//Returns the position in targetlane, equivalent to X in current lane;
	double getPositionInTargetlane( double X,int targetLane) const;

	//Returns the speed of the vehicle at time (t – state * simulationStep) (units: m/s)*/
	double getSpeed( const unsigned int state) const;

	//Returns the deceleration factor accepted by the vehicle for imprudent lane changing case
	double getDecelerationVariationFactor(const bool ImprudentCase)const;

	// It returns the world coordinates of the middle point of a vehicle front bumper (xfront, yfront)
	// and the world coordinates of the middle point of a vehicle rear bumper (xback, yback).
	void getCoordinates( double &xfront, double &yfront, double &xback, double &yback) const;

	//Get the position of the vehicle at time (t - state * simulationStep), taking the reference vehicle vehReference(units: m).
	//The vehicle and reference vehicle do not need to be neither on the same section, nor on the same lane,
	//although the output does not take the difference in length across the sections but only along them.
	//If not located in the same section, it returns the shortest distance between the two vehicle´s fronts.
	double getPositionReferenceVeh( const unsigned int state, A2SimVehicle *vehReference, const unsigned stateVehRef) const;

	//Returns the Gap between vehUp and vehDown, their position and speeds at time t=t-RTup, t if time=0 or t+dt if time=-1
	// gap等于两车车头距离相减，再减去车长，再减去MBDV (Minimum Distance between Vehicle) of this vehicle 
	double getGap(double Shift,A2SimVehicle *vehDown,double ShiftDw,double &Position, double &Speed, double &PosDw,double &SpeedDw,int time=1)const;


	//FUNCTIONS RELATIVE TO THE VEHICLE´S PATH:

	//Returns the index of the vehicle´s current lane, 1 being the rightmost lane of the section.
	unsigned int getIdCurrentLane() const;

	//Returns the index of the vehicle´s current section, or next section if vehicle is in Node
	unsigned int getIdCurrentSection() const;

	//Returns the index of the vehicle´s next section
	unsigned int getIdNextSection() const;

	//Returns the index of the vehicle´s next junction, or current junction if vehicle is in a Node
	unsigned int getIdNextJunction() const;

	//Returns the index of the vehicle´s next turning, or current turning if vehicle is in a Node
	unsigned int getIdNextTurning() const;

	//Returns the number of lanes of the vehicle´s current section not taking side lanes into account
	int getNumberOfMainLanesInCurrentSection() const;

	//Returns the total number of lanes of the vehicle´s current section
	int getNumberOfLanesInCurrentSection() const;

	//Returns the number of lanes of the vehicle´s next section not taking side lanes into account
	int getNumberOfMainLanesInNextSection() const;

	//Returns the total number of lanes of the vehicle´s next section
	int getNumberOfLanesInNextSection() const;

	//Returns true if targetlane (-1 is left, 0 current and 1 is right) is an entrance side lane
	bool IsLaneOnRamp(int targetlane) const;

	//Returns true if the current lane is inside a node.
	bool isCurrentLaneInNode() const;

	//Returns the density inside the targetlane (-1 is left, 0 is current, 1 is right) in veh/m
	double getDensity(const int targetLane) const;

	//Returns the density inside the lane (1 is the rightmost lane of the section) in veh/m
	double getLaneDensity(const int lane) const;

	//Returns the number of vehicles with speed=0 inside the lane (1 is the rightmost lane of the section) in veh/m
	int getNumberOfVehiclesStoppedInLane(const int lane) const;

	//Returns the average speed of the first maximumNbVehs vehicles located within maximumDist (meters) inside the targetlane (-1 is left, 0 is current, 1 is right) in m/s
	double getAverageSpeedAHead(const int targetLane, const double maximumDist, const int maximumNbVehs) const;

	//Returns the average speed of the first maximumNbVehs vehicles located within maximumDist (meters) inside the lane (1 is the rightmost lane of the section) in m/s
	double getAverageLaneSpeedAHead(const int ordenCarril, double maximumDist, int maximumNbVehs) const;

	//Sets the vehicle´s next section (needs to be called before entering the node)
	void setNextSection(int idNextSection) const;

	//Sets the vehicle´s next lane and next section (needs to be called before entering the node)
	void setTargetLaneInDownStreamSection(int idNextSection, int nextTargetLane) const;

	//Gets the type of Obstacle present in the current lane:
	//enum Type {eNone, eReservedLane, eTurning, eNotAllowedInfluence, eNotAllowed, ePTStopInfluence, eOnRamp, eLaneClosureInfluence, eIncidentInfluence, eLaneClosure, eIncident, ePTStop};
	int getObstacleType() const;

	//Gets the type of Obstacle present in the lane (1 is the rightmost lane):
	int getObstacleTypeInLane(const int lane) const;

	//Gets the distance to the next Obstacle present in the current lane:
	double getDistance2Obstacle() const;

	//Gets the distance to the next Obstacle present in the lane (1 is the rightmost lane):
	double getDistance2ObstacleInLane(const int lane) const;

	//Gets the distance of influence to the next Obstacle present in the lane (1 is the rightmost lane):
	double getDistanceOfInfluenceOfObstacleInLane(const int ordenCarril) const;

	//Gets the number of lane changes to be made to reach a valid lane (>0 means right)
	int getNbLaneChanges2ReachNextValidLane() const;


	//FUNCTIONS RELATIVE TO SURROUNDING VEHICLES:

	//Returns the current leader (can be fictitious) and its offset
	//Shift is the offset between the beginning of the section of the vehicle and the beginning of section of the leader
	//The leader being downstream it is >=0.
	A2SimVehicle * getLeader(double &Shift) const;

	//Returns the current leader (not fictitious) and its offset
	A2SimVehicle * getRealLeader(double &Shift) const;

	//Returns the leader reflected by a nexo or a conflicto
	A2SimVehicle * getReflectedVehicle() const;

	//Returns the current follower (can be fictitious) and its offset
	//Shift is the offset between the beginning of the section of the vehicle and the beginning of section of the follower
	//The follower being upstream it is <=0.
	A2SimVehicle * getFollower(double &Shift) const;

	//Returns the current follower (not fictitious) and its offset
	A2SimVehicle * getRealFollower(double &Shift) const;

	//Returns the current up and down vehicles (can be fictitious) and their offsets in the targetlane (-1 is left, 0 current and 1 is right)
	void getUpDown(int targetlane, double XPosInObj, A2SimVehicle *&vehUp, double &shiftUp, A2SimVehicle *&vehDown, double &shiftDown ) const;

	//Returns the current up and down vehicles (can be fictitious) and their offsets (not fictitious) in the targetlane (-1 is left, 0 current and 1 is right)
	void getRealUpDown(int targetlane, double XPosInObj,  A2SimVehicle *&vehUp, double &shiftUp,  A2SimVehicle *&vehDown, double &shiftDown ) const;


	//FUNCTION RELATIVE TO THE CAR-FOLLOWING METHOD:

	//Applies the Default Aimsun Car Following Model for this vehicle
	bool applyAimsunCarFollowingModel()const;

	//Computes the Default Aimsun Car Following Speed for this vehicle
	double getAimsunCarFollowingSpeed() const;

	//Computes the Default Aimsun Gipps Acceleration Speed Component for this vehicle
	double getAccelerationComponentGippsModelSpeed() const;
	double getAccelerationComponentGippsModelSpeed(double VelActual,double VelDeseada,double RestoCiclo) const;

	//Computes the Default Aimsun Gipps Deceleration Speed Component for this vehicle
	//imposed by a standing Obstacle located at Distance2Obstacle from the vehicle.
	double getDecelerationComponentGippsModelSpeed(double Distance2Obstacle) const;

	//Computes the Default Aimsun Gipps Deceleration Speed Component for this vehicle
	//imposed by the vehicle Leader with offset ShiftPre.
	//controlDecelMax enforces the speed to be compatible with braking capabilities
	//aside allows to use a softer deceleration if the Leadr is in a different lane
	//time (normally sets to 1, can be set 0 if future (+RT) predictions are needed
	double getDecelerationComponentGippsModelSpeed(const A2SimVehicle *Leader,double ShiftPre,bool controlDecelMax,bool aside,int time) const;


	//FUNCTION RELATIVE TO THE LANE-CHANGING METHOD:

	//Applies the Default Aimsun Lane Changing Model for this vehicle
	bool applyAimsunLaneChangingModel()const;

	//Returns whether lane changing is possible toward the targetlane (-1 is left, 0 current and 1 is right)
	//Checks for: lack of solid line, vehicle not currently changing lane
	bool isLaneChangingPossible(int targetLane) const;

	//Puts the vehicle in the list of vehicles to change lane, letting Aimsun decide
	//whether it is possible in terms of priority  with respect to other vehicles willing to change lane.
	void assignAcceptedGap(int targetlane,double XposInpEntObj,const A2SimVehicle *pVehUp,double ShiftUp,const A2SimVehicle *pVehDw,double ShiftDw, int threadId) const;

	//Returns whether the gap between pVehUp and pVehDw is acceptable  accrding to the CarFollowingAccelerationComponentSpeed and CarFollowingDecelerationComponentSpeed.
	bool isGapAcceptable(int targetlane,double XposInpEntObj,const A2SimVehicle *pVehUp,double ShiftUp,const A2SimVehicle *pVehDw,double ShiftDw)const;

	//Performs the Lane Changing to targetlane between pVehUp and pVehDw
	void applyLaneChanging(const int targetlane, int threadId)const;

	//Looks for a new target gap for cooperation
	void targetNewGap(int targetlane, double XPosInpEntObj, A2SimVehicle *pVehUpReal, double ShiftUp, A2SimVehicle *vehDownReal, double ShiftDw, int threadId)const;

	//Puts the vehicle in the list of vehicles to target the gap between vehUp and vehDown
	//(vehUp will cooperate depending on the cooperation parameters), letting Aimsun decide
	//whether it is possible in terms of priority  with respect to other vehicles willing to target the same gap.
	void assignNewTargetGap(double XPosInpEntObj, const A2SimVehicle *vehUpReal, double ShiftUp, const A2SimVehicle *vehDownReal, double ShiftDw, int threadId) const;

	//Forces the current vehicle to cooperate with veh2Yield
	void setAsCourtesyVehicle(const A2SimVehicle *veh2Yield) const;


private:
	void 	*handlerVehicle;
	unsigned short	idhandler;
};

#endif

