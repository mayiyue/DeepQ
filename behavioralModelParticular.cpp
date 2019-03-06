/*
可以改进的地方：
1. 把建立文件指针功能做成一个函数，提高复用性 2018/3/23 21.33
2. 把optimized vehicle data 做成一个类，将数据与功能组织在一起，便于管理 2018-9-20 13:41:35
2018-9-27 9:58:44已完成 VVVV 3. 使用Git管理代码，同时建立完善的日志，记录想法变迁等内容。 2018-9-20 13:42:28
4. 写一个debug类或者一些debug函数，用于调试，摆脱传统的注释、运行部分 2018-9-20 13:55:25
5. 2018-11-7 12:16:30 OPTvehicle数据结构中 pathlength有耦合
*/

#include "behavioralModelParticular.h"
#include "simVehicleParticular.h"
#include "AKIProxie.h"
#include "ANGConProxie.h"
#include <stdio.h>
#include <fstream>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <cstring>
#include <vector>
#include <map>
#include <array>
#include <list>

using namespace std;
#define Tolerancia 0.01
#define SPEED_CRITICAL 60 // using for Asymmetric MOBIL
#define DBL_MAX 1.7976931348623158e+308 
#define DATAPATH "D:\\working\\WorkingInEnhancedAIMSUNPlatform\\LaneChanging\\Data\\"
#define AFTPATH "D:\\Aimsun 8.1\\"

// controllers on-off
bool const needTestMsg = false;
bool const useIDM = true;
bool const useHeuristicLaneChangeModel = true;
bool const useMOBIL = false;
bool const useAsymmetricMOBIL = false;        // Symmetric MOBIL is default
bool const useIterationOptimization = false; // when it's value is true, the optimized vehicle will be selected in the entry section to re-experience the traffic condition over and over
bool const useQLearning = true;
bool const useOutSideInPut_SmartVehiclePenetrationRate = false;


// triggers
bool hasRanInThisTimeSetp = false; // for some codes that need be run only one time in one simulation step
bool hasReadSmartVehiclePenetrationRate = false;
bool hasInputQTable = false;
bool haveOutPutFunctionsRan = false;
bool haveInPutFunctionsRan = false;
bool hasOutPutFilesInitiated = false;


// for IDM, (C)ACC
double const penetrationOfACC = 0;
double const lowLimitOfACCTimeGap = 0.5;
double const upLimitOfACCTimeGap = 0.7;


// for Optimizing Working
double  penetrationOfSmartVehicles = 0; // if useOutSideInPut_SmartVehiclePenetrationRate = true, it will be read from a outside file.

// a new vehicle entry into the network will be setted as optimized vehicle 
// when the previous optimized vehicle is in the exit section. 
int optimizedExperienceTimes = 1;
int optimizedVehIDSequence[100]; //maximun of array size will not over the maximum of iteration
int optimiazedVehID = 4000;//4000 4121 5070 // work only when   useIterationOptimization = false




// q-learning data structure
struct
{
	float epsilon;
	float learning_rate_r;
	float alpha;

	// 6^10*3 float ， 0.72 GB memory cost
	// a2	口	口    Left 
	// a3	a1	口    Current
	// a4	口	口	  Right
	// [deta_a1L][deta_a1R][deta_a2L][deta_a3][deta_a4R][deta_μL][deta_sigmaL][deta_μR][deta_sigmaR][Actions] 10 Dimensions
	// [3] Actions:0 current 1 left 2 right
	float q_table[6][6][6][6][6][6][6][6][6][3];

	unsigned int stateID_last[16000]; // stateID_last[vehicle ID]
	unsigned int action_last[16000]; // action_last[vehicle ID]

}q_LearningParameters = { (float)0.95,(float)0.8,(float)0.01,{ (float)0 },{0},{0} };





struct {
	bool isExist;
	double existTime;
}haveOptimizedVeh = { false,0 };


struct {
	int entrySection;
	int exitSection;;
	int entryLane;
	double entryTime;
	double exitTime;
	double totalTravelTime;
	double totalTravelPathLength;
	int preSectionID;

	bool isSmartVehicle; // TEST for SV demand
}allVehicleODInfoDataSet[16000]; // total vehicle number entry in the network is about 15600

struct TrajectoryData {
	double time;
	double coordinateX;
	double coordinateY;
	double pathLengthPerSecond;
	double speed;
};
struct VehiclePathInfo {


	struct LaneChangeDetail {
		double occurrenceTime; // occurrence time of lanechanging
		int occurrenceSection;
		double occurrencePositionInSection; // position refers to current section
		double occurrenceCoordinationX;
		double occurrenceCoordinationY;
		int occurenceVehID;
		int preLane; // using absolute network lane ID
		int folLane; // using absolute network lane ID
	}laneChangeDetailSet[10000]; // the maximum times of lanechanging for one vehicle ID

	struct SectionPath {
		double entrySectionTime;
		int sectionID;
	}sectionPathInfo[23];// the sections are 22 in total, so the path section number will not over the 22


	double totalTravelPathLength; // m
	double pathLengthBySection; // record the total length of sections that the vehicle had passed, m
	int totalLaneChangeTimes;
	double totalTravelTime; // sec
	double entryTime, exitTime; // sec
	int entryLaneInitialSection;


	list <TrajectoryData> trajectoryDataSet;
	//	double	acceleration[36000]; // 4 hours = 14400 sec = 36000 steps

	//temp variation for judging whether vehicle states is changed
	int	preSectionID;
	int currSectionID;
	int preLane; // using absolute network lane ID

}optVehDataSet = { 0 };


//map sectionID->parameter
map<int, double> parameterSet = {
	{ 363,0 },
	{ 364,0 },
	{ 370,0 },
	{ 387,0 },
	{ 1022,0 },
	{ 952,0 },
	{ 949,0 },
	{ 386,0 },
	{ 395,0 },
	{ 935,0 },
	{ 404,0 },
	{ 974,0 },
	{ 982,0 },
	{ 967,0 },
	{ 406,0 },
	{ 414,0 },
	{ 423,0 },
	{ 986,0 },
	{ 990,0 },
	{ 994,0 },
	{ 998,0 },
	{ 1002,0 },
};



list <int> impactedVehIDSet;


// sectionSequence[sequence]=sectionID
int const sequenceSectionID[23] = { 0,363,364,370,387,1022,952,949,386,395,935,404,974,982,967,406,414,423,986,990,994,998,1002 };



/*************************** Function Declaration (custom) **************************/
/*Method, Smart Demand from a outside file*/
/*
int smartVehicleNum_interval_temp = 0;
int smartVehicleNum_entrySection_temp[5] = { 0 };
int readSmartVehicleDemandRumTimes = 0;
int smartVehicleDemand[5][240] = { 0 };
void readSmartVehicleDemand()
{
	ifstream readSmartVehicleDemandFile;
	readSmartVehicleDemandFile.open("D:\\working\\WorkingInEnhancedAIMSUNPlatform\\LaneChanging\\Data\\InputData\\SmartVehicleDemand.dat", ios::in);
	for (int inputSection = 0; inputSection <= 5 - 1; ++inputSection)
	{
		for (int timeInterval = 0; timeInterval <= (240 / 30) - 1; ++timeInterval)
		{
			readSmartVehicleDemandFile >> smartVehicleDemand[inputSection][timeInterval];
		}
	}


}
*/

/*Method, Smart Demand (penetration rate) from a outside file*/
void readSmartVehiclePenetrationRate()
{
	ifstream readSmartVehicleDemandFile;
	readSmartVehicleDemandFile.open("D:\\working\\WorkingInEnhancedAIMSUNPlatform\\LaneChanging\\Data\\InputData\\SmartVehiclePenetrationRate.dat", ios::in);

	readSmartVehicleDemandFile >> penetrationOfSmartVehicles;
	readSmartVehicleDemandFile.close();

}

int behavioralModelParticular::getQLearningDecisionAction(A2SimVehicle* vehicle)
{

	int action = 0;
	unsigned int stateID = getStateID_QLearning(vehicle);
	if (AKIGetRandomNumber() < q_LearningParameters.epsilon)
	{
		action = getMaxQValueAction(stateID, vehicle);
	}
	//else
	//{
	//	action = getAvailableActionRandomly_Qlearning(stateID, vehicle);
	//}



	return action;
}

int convertQActionToDirection(int Qaction)
{
	int direction = 0;
	switch (Qaction)
	{
	case 0: direction = 0; break;
	case 1: direction = -1; break;
	case 2: direction = 1; break;
	default: direction = 0;
		break;
	}
	return direction;
}

unsigned int behavioralModelParticular::getStateID_QLearning(A2SimVehicle* vehicle)
{

	int curLane = vehicle->getIdCurrentLane();
	int numSect = vehicle->getIdCurrentSection();
	int maxLanes = vehicle->getNumberOfLanesInCurrentSection();

	double	ac_Left_LC = 0;			//ac*	//turn left
	double	ac_Left_NOLC = 0;		//ac	//turn left
	double	an_Left_LC = 0;			//an*	//turn left
	double	an_Left_NOLC = 0;		//an	//turn left
	double	ao_Left_LC = 0;			//ao*	//turn left
	double	ao_Left_NOLC = 0;		//ao	//turn left


	double	ac_Right_LC = 0;			//ac*	//turn Right
	double	ac_Right_NOLC = 0;			//ac	//turn Right
	double	an_Right_LC = 0;			//an*	//turn Right
	double	an_Right_NOLC = 0;			//an	//turn Right
	double	ao_Right_LC = 0;			//ao*	//turn Right
	double	ao_Right_NOLC = 0;			//ao	//turn right


	double XPosTargetlaneLeft = 0;
	double XPosTargetlaneRight = 0;

	A2SimVehicle *pVehLeftDw = NULL;
	A2SimVehicle *pVehLeftUp = NULL;
	A2SimVehicle *pVehRightDw = NULL;
	A2SimVehicle *pVehRightUp = NULL;
	A2SimVehicle *pVehCurUp = NULL;
	A2SimVehicle *pVehCurDw = NULL;

	double shiftCurUp = 0, shiftCurDw = 0;
	double ShiftUpLeft = 0, ShiftDwLeft = 0;
	double ShiftUpRight = 0, ShiftDwRight = 0;


	/*******************************************************************/
	/**********************MOBIL CRITERION*****************************/


	//get current lane follower and leader
	vehicle->getUpDown(0, vehicle->getPosition(0), pVehCurUp, shiftCurUp, pVehCurDw, shiftCurDw);

	//get left lane follower and leader
	vehicle->getUpDown(-1, vehicle->getPosition(0), pVehLeftUp, ShiftUpLeft, pVehLeftDw, ShiftDwLeft);

	//get right lane follower and leader
	vehicle->getUpDown(1, vehicle->getPosition(0), pVehRightUp, ShiftUpRight, pVehRightDw, ShiftDwRight);


	/**********************LEFT Calculation*********************************/
	if (curLane < maxLanes)
	{
		ac_Left_LC = get_IDM_acceleration(vehicle, pVehLeftDw);//ac*

		ac_Left_NOLC = get_IDM_acceleration(vehicle, pVehCurDw); // ac

		an_Left_LC = get_IDM_acceleration(pVehLeftUp, vehicle);	//an*

		an_Left_NOLC = get_IDM_acceleration(pVehLeftUp, pVehLeftDw);//an

		ao_Left_LC = get_IDM_acceleration(pVehCurUp, pVehCurDw); //a0*

		ao_Left_NOLC = get_IDM_acceleration(pVehCurUp, vehicle);  //a0
	}
	/**********************RIGHT Calculation*********************************/
	if (curLane > 1)
	{
		ac_Right_LC = get_IDM_acceleration(vehicle, pVehRightDw);  //ac*

		ac_Right_NOLC = get_IDM_acceleration(vehicle, pVehCurDw);    // ac

		an_Right_LC = get_IDM_acceleration(pVehRightUp, vehicle); //an*

		an_Right_NOLC = get_IDM_acceleration(pVehRightUp, pVehRightDw);  //an

		ao_Right_LC = get_IDM_acceleration(pVehCurUp, pVehCurDw); //a0*

		ao_Right_NOLC = get_IDM_acceleration(pVehCurUp, vehicle);  //a0
	}


	double diff_ac_left = 0;
	double diff_ao = 0;
	double diff_an_left = 0;
	double diff_ac_right = 0;
	double diff_an_right = 0;


	double diff_mean_left = 0;
	double diff_mean_right = 0;

	double diff_sd_left = 0;
	double diff_sd_right = 0;


	diff_ac_left = ac_Left_LC - ac_Left_NOLC;
	diff_ao = ao_Left_LC - ao_Left_NOLC; // ao_Left_LC=ao_Right_LC, so there is only one diff_ao
	diff_an_left = an_Left_LC - an_Left_NOLC;
	diff_ac_right = ac_Right_LC - ac_Right_NOLC;
	diff_an_right = an_Right_LC - an_Right_NOLC;
	getLeadersAccelerationsDistributionDifference(vehicle, diff_mean_left, diff_sd_left, diff_mean_right, diff_sd_right);

	int  state_diff_ac_left = 0;
	int  state_diff_ao = 0;
	int  state_diff_an_left = 0;
	int  state_diff_ac_right = 0;
	int  state_diff_an_right = 0;


	int state_diff_mean_left = 0;
	int state_diff_mean_right = 0;


	int state_diff_sd_left = 0;
	int state_diff_sd_right = 0;


	state_diff_ac_left = getDiscretedState_Qlearning(diff_ac_left);
	state_diff_ao = getDiscretedState_Qlearning(diff_ao);
	state_diff_an_left = getDiscretedState_Qlearning(diff_an_left);

	state_diff_ac_right = getDiscretedState_Qlearning(diff_ac_right);

	state_diff_an_right = getDiscretedState_Qlearning(diff_an_right);

	state_diff_mean_left = getDiscretedState_Qlearning(diff_mean_left);
	state_diff_mean_right = getDiscretedState_Qlearning(diff_mean_right);


	state_diff_sd_left = getDiscretedState_Qlearning(diff_sd_left);
	state_diff_sd_right = getDiscretedState_Qlearning(diff_sd_right);


	unsigned int stateID = 0;

	stateID =
		state_diff_ac_left *(int)pow(10, 8)
		+ state_diff_ac_right* (int)pow(10, 7)
		+ state_diff_an_left  *(int)pow(10, 6)
		+ state_diff_ao * (int)pow(10, 5)
		+ state_diff_an_right* (int)pow(10, 4)
		+ state_diff_mean_left *(int)pow(10, 3)
		+ state_diff_sd_left *(int)pow(10, 2)
		+ state_diff_mean_right *(int)pow(10, 1)
		+ state_diff_sd_right * (int)pow(10, 0);

	return stateID;
}

//0~5, 6 states for each 
int behavioralModelParticular::getDiscretedState_Qlearning(double diff_value)
{
	int state = 0;
	if (diff_value <= -1)
		state = 0;
	else if (diff_value > -1 && diff_value <= -0.5)
		state = 1;
	else if (diff_value > -0.5 && diff_value <= 0)
		state = 2;
	else if (diff_value > 0 && diff_value <= 0.5)
		state = 3;
	else if (diff_value > 0.5&& diff_value <= 1)
		state = 4;
	else if (diff_value > 1)
		state = 5;

	return state;
}

int behavioralModelParticular::getMaxQValueAction(unsigned int stateID, A2SimVehicle * vehicle)
{
	int action = 0;
	int stateCode[10] = { 0 };
	int curlane = vehicle->getIdCurrentLane();
	int maxlane = vehicle->getNumberOfLanesInCurrentSection();

	A2SimVehicle *pVehLeftDw = NULL;
	A2SimVehicle *pVehLeftUp = NULL;
	A2SimVehicle *pVehRightDw = NULL;
	A2SimVehicle *pVehRightUp = NULL;
	A2SimVehicle *pVehCurUp = NULL;
	A2SimVehicle *pVehCurDw = NULL;

	double shiftCurUp = 0, shiftCurDw = 0;
	double ShiftUpLeft = 0, ShiftDwLeft = 0;
	double ShiftUpRight = 0, ShiftDwRight = 0;


	/*******************************************************************/

	//get current lane follower and leader
	vehicle->getUpDown(0, vehicle->getPosition(0), pVehCurUp, shiftCurUp, pVehCurDw, shiftCurDw);

	//get left lane follower and leader
	vehicle->getUpDown(-1, vehicle->getPosition(0), pVehLeftUp, ShiftUpLeft, pVehLeftDw, ShiftDwLeft);

	//get right lane follower and leader
	vehicle->getUpDown(1, vehicle->getPosition(0), pVehRightUp, ShiftUpRight, pVehRightDw, ShiftDwRight);





	int currentSection = vehicle->getIdCurrentSection();
	int vehID = vehicle->getId();

	for (int i = 0; i <= 8; ++i)
	{
		stateCode[i] = stateID / int(pow(10, i)) % 10;
	}

	float currentDirectionQuality = q_LearningParameters.q_table[stateCode[8]][stateCode[7]][stateCode[6]][stateCode[5]][stateCode[4]][stateCode[3]][stateCode[2]][stateCode[1]][stateCode[0]][0];
	float leftDirectionqQuality = q_LearningParameters.q_table[stateCode[8]][stateCode[7]][stateCode[6]][stateCode[5]][stateCode[4]][stateCode[3]][stateCode[2]][stateCode[1]][stateCode[0]][1];
	float rightDirectionqQuality = q_LearningParameters.q_table[stateCode[8]][stateCode[7]][stateCode[6]][stateCode[5]][stateCode[4]][stateCode[3]][stateCode[2]][stateCode[1]][stateCode[0]][2];
	if ((curlane < maxlane)// left is possible
		&& (leftDirectionqQuality > currentDirectionQuality&&leftDirectionqQuality >= rightDirectionqQuality)

		)
	{
		return 1; // turn left
	}
	else if ((curlane > 1) // right is possible
		&& (rightDirectionqQuality > currentDirectionQuality&&rightDirectionqQuality > leftDirectionqQuality)

		)
	{
		return 2; // turn right
	}
	else
	{
		return 0; // do not change lane
	}

}

int behavioralModelParticular::getAvailableActionRandomly_Qlearning(unsigned int stateID, A2SimVehicle* vehicle)
{

	int curLane = vehicle->getIdCurrentLane();

	int maxLanes = vehicle->getNumberOfLanesInCurrentSection();


	A2SimVehicle *pVehLeftDw = NULL;
	A2SimVehicle *pVehLeftUp = NULL;
	A2SimVehicle *pVehRightDw = NULL;
	A2SimVehicle *pVehRightUp = NULL;
	A2SimVehicle *pVehCurUp = NULL;
	A2SimVehicle *pVehCurDw = NULL;

	double shiftCurUp = 0, shiftCurDw = 0;
	double ShiftUpLeft = 0, ShiftDwLeft = 0;
	double ShiftUpRight = 0, ShiftDwRight = 0;


	/*******************************************************************/

	//get current lane follower and leader
	vehicle->getUpDown(0, vehicle->getPosition(0), pVehCurUp, shiftCurUp, pVehCurDw, shiftCurDw);

	//get left lane follower and leader
	vehicle->getUpDown(-1, vehicle->getPosition(0), pVehLeftUp, ShiftUpLeft, pVehLeftDw, ShiftDwLeft);

	//get right lane follower and leader
	vehicle->getUpDown(1, vehicle->getPosition(0), pVehRightUp, ShiftUpRight, pVehRightDw, ShiftDwRight);








	double randNUM = AKIGetRandomNumber();

	// the vehicle can change to left or right
	if (curLane <= maxLanes - 1 && curLane > 1)
	{
		if (randNUM < 0.33333

			)
		{
			return 2; // right
		}
		else if (randNUM < 0.66666

			)
		{
			return 1; // left
		}
		else
		{
			return 0; // current
		}
	}
	//  the vehicle can only change to left 
	else if (curLane == 1 && maxLanes > 1)
	{
		if (randNUM < 0.5

			)
		{
			return 1; // left
		}
		else
		{
			return 0; // current
		}
	}
	//  the vehicle can only change to right 
	else if (curLane == maxLanes&&maxLanes != 1)
	{
		if (randNUM < 0.5

			)
		{
			return 2; // right
		}
		else
		{
			return 0; // current
		}
	}
	else if (curLane == maxLanes&&maxLanes == 1) // can not change lane
	{
		return 0; // current
	}
	//  the vehicle cannot change lane
	else
	{
		return 0;
	}
}

float rewardQLearning(unsigned int stateID, int action)
{
	float reward = 0;
	int stateCode[10] = { 0 };
	for (int i = 0; i <= 8; ++i)
	{
		stateCode[i] = stateID / int(pow(10, i)) % 10;

	}
	if (action == 1)
	{
		reward = (float)stateCode[8]; // state_diff_ac_left
	}
	else if (action == 2)
	{
		reward = (float)stateCode[9]; // state_diff_ac_right
	}
	else // action == 0
	{
		reward = 0; //it may be get_IDM_acceleration(vehicle, pVehCurDw);
	}
	return reward;
}
float maxQActionValueForState(unsigned int stateID)
{
	int stateCode[10] = { 0 };
	for (int i = 0; i <= 8; ++i)
	{
		stateCode[i] = stateID / int(pow(10, i)) % 10;
	}
	float actionQValues[3] = {
		q_LearningParameters.q_table[stateCode[8]][stateCode[7]][stateCode[6]][stateCode[5]][stateCode[4]][stateCode[3]][stateCode[2]][stateCode[1]][stateCode[0]][0],
		q_LearningParameters.q_table[stateCode[8]][stateCode[7]][stateCode[6]][stateCode[5]][stateCode[4]][stateCode[3]][stateCode[2]][stateCode[1]][stateCode[0]][1],
		q_LearningParameters.q_table[stateCode[8]][stateCode[7]][stateCode[6]][stateCode[5]][stateCode[4]][stateCode[3]][stateCode[2]][stateCode[1]][stateCode[0]][2]
	};

	float max = actionQValues[0];
	if (max < actionQValues[1])
		max = actionQValues[1];
	if (max < actionQValues[2])
		max = actionQValues[2];
	return max;

}

void updateQTable(unsigned int stateID, int action, int next_stateID)
{
	int stateCode[10] = { 0 };
	for (int i = 0; i <= 8; ++i)
	{
		stateCode[i] = stateID / int(pow(10, i)) % 10;

	}

	/*
	如果在当前时刻更新Q表，当前时刻的action应用后下一时刻的state不一定是当前时刻“预测”的state，应该用预测state还是下一时刻实际的state作为next_state?
	应该在哪一时刻更新Q表？
	如果apply之后AIMSUN并不能完整的执行，那么这一部分必须要反映到Qlearning里
	*/
	//getNextStateID_Qlearning(vehicle);





	q_LearningParameters.q_table[stateCode[8]][stateCode[7]][stateCode[6]][stateCode[5]][stateCode[4]][stateCode[3]][stateCode[2]][stateCode[1]][stateCode[0]][action]
		=
		q_LearningParameters.q_table[stateCode[8]][stateCode[7]][stateCode[6]][stateCode[5]][stateCode[4]][stateCode[3]][stateCode[2]][stateCode[1]][stateCode[0]][action]
		+ q_LearningParameters.alpha*(
			rewardQLearning(stateID, action)
			+ q_LearningParameters.learning_rate_r * maxQActionValueForState(next_stateID)
			- q_LearningParameters.q_table[stateCode[8]][stateCode[7]][stateCode[6]][stateCode[5]][stateCode[4]][stateCode[3]][stateCode[2]][stateCode[1]][stateCode[0]][action]
			);


}

// main sections mean that they are not turn, node, on/off-ramp or inputflow section 
bool isMainSection(int sectionID) //all sections on the mainroad
{
	switch (sectionID)
	{
	case	363:
	case	364:
	case	370:
	case	387:
	case	1022:
	case	952:
	case	949:
	case	386:
	case	395:
	case	935:
	case	404:
	case	974:
	case	982:
	case	967:
	case	406:
	case	414:
	case	423:
	case	986:
	case	990:
	case	994:
	case	998:
	case	1002:
		return true;

	default:
		return false;

	}
}


/*
Special Sections' lane 1 is acceleration lane or off-ramp lane
In NetWorkAbsoluteLane, the lane ID of accelaeration lane and off-ramp lane is 0
*/
int getNetWorkAbsoluteLaneID(int sectionID, int curSectionLane)
{

	switch (sectionID)
	{
	case	386:
	case	404:
	case	406:
	case	423:
		return --curSectionLane;
	case 671:
		return 3;
	case 664:
		return 2;
	case 670:
		return 1;
	case 928:
		return 0;
	case 932:
		return 0;
	default:
		return curSectionLane;
	}

}

bool isExitSection(int sectionID)
{
	switch (sectionID)
	{
	case 396:
	case 415:
	case 1002:
		return true;
	default:
		return false;
	}

}

// if it's not a entry section, return -1, else return sequence 
int getEntrySectionSequence(int sectionID)
{
	switch (sectionID)
	{
	case 671: return 1;
	case 664: return 2;
	case 670: return 3;
	case 928: return 4;
	case 932: return 5;
	default:
		return -1;
	}

}

// this test function should be integrated into outPutControlGroupVehiclesODInfo()
bool isControlGroupVehicle(int referenceVehicleID, int testVehicleID)
{

	if (// 1. time window comparison
		allVehicleODInfoDataSet[testVehicleID].entryTime > allVehicleODInfoDataSet[referenceVehicleID].entryTime - 30
		&& allVehicleODInfoDataSet[testVehicleID].entryTime < allVehicleODInfoDataSet[referenceVehicleID].entryTime + 30
		// 2. OD comparison
		&& allVehicleODInfoDataSet[testVehicleID].entrySection == allVehicleODInfoDataSet[referenceVehicleID].entrySection
		&&allVehicleODInfoDataSet[testVehicleID].entryLane == allVehicleODInfoDataSet[referenceVehicleID].entryLane
		&&allVehicleODInfoDataSet[testVehicleID].exitSection == allVehicleODInfoDataSet[referenceVehicleID].exitSection
		)
	{
		return true;
	}
	else
		return false;

}

// record travel time, path length and OD
void recordAllVehicleODInfo(A2SimVehicle *vehicle)
{


	int vehID = vehicle->getId();
	double currTime = AKIGetCurrentSimulationTime(); // seconds
	int currSectionID = vehicle->getIdCurrentSection();


	// record time infomation
	if (allVehicleODInfoDataSet[vehID].entryTime == 0)
		allVehicleODInfoDataSet[vehID].entryTime = currTime;
	if (allVehicleODInfoDataSet[vehID].exitTime < currTime)
		allVehicleODInfoDataSet[vehID].exitTime = currTime;
	if (allVehicleODInfoDataSet[vehID].exitTime - allVehicleODInfoDataSet[vehID].entryTime > allVehicleODInfoDataSet[vehID].totalTravelTime)
		allVehicleODInfoDataSet[vehID].totalTravelTime = allVehicleODInfoDataSet[vehID].exitTime - allVehicleODInfoDataSet[vehID].entryTime;



	// record path length section by section
	if (currSectionID != allVehicleODInfoDataSet[vehID].preSectionID)
	{
		A2KSectionInf sectionInfo;
		sectionInfo = AKIInfNetGetSectionANGInf(currSectionID);

		allVehicleODInfoDataSet[vehID].totalTravelPathLength += sectionInfo.length;
		allVehicleODInfoDataSet[vehID].preSectionID = currSectionID;
	}


	// record OD 
	if (getEntrySectionSequence(currSectionID) != -1)
	{
		allVehicleODInfoDataSet[vehID].entrySection = currSectionID;
		allVehicleODInfoDataSet[vehID].entryLane = vehicle->getIdCurrentLane(); // since the lane changing cannot happened in the input sections which have only one lane. 
		allVehicleODInfoDataSet[vehID].isSmartVehicle = ((simVehicleParticular*)vehicle)->getIsSmartVehicle();
	}
	if (isExitSection(currSectionID))
	{
		allVehicleODInfoDataSet[vehID].exitSection = currSectionID;

	}

}



void recordOptVehicleTravelTime(double currTime)
{
	if (optVehDataSet.entryTime == 0)
		optVehDataSet.entryTime = currTime;
	if (optVehDataSet.exitTime < currTime)
		optVehDataSet.exitTime = currTime;
	if (optVehDataSet.exitTime - optVehDataSet.entryTime > optVehDataSet.totalTravelTime)
		optVehDataSet.totalTravelTime = optVehDataSet.exitTime - optVehDataSet.entryTime;

}
void recordOptVehiclePathLength(A2SimVehicle * vehicle)
{
	int currentSectionID = vehicle->getIdCurrentSection();
	// record path length each time step
	if (getEntrySectionSequence(currentSectionID) != -1)
	{
		optVehDataSet.totalTravelPathLength = vehicle->getPosition(0);
		optVehDataSet.preSectionID = currentSectionID;
	}
	else if (currentSectionID != optVehDataSet.preSectionID)
	{
		A2KSectionInf preSectionInfo;
		preSectionInfo = AKIInfNetGetSectionANGInf(optVehDataSet.preSectionID);

		optVehDataSet.pathLengthBySection += preSectionInfo.length;

		optVehDataSet.preSectionID = currentSectionID;
	}
	else
	{
		optVehDataSet.totalTravelPathLength = optVehDataSet.pathLengthBySection + vehicle->getPosition(0);
	}
}
void recordOptVehiclLaneChangingInfo(A2SimVehicle *vehicle)
{
	int currAbsoluteLaneID = getNetWorkAbsoluteLaneID(vehicle->getIdCurrentSection(), vehicle->getNumberOfLanesInCurrentSection());
	if (optVehDataSet.preLane != currAbsoluteLaneID)
	{
		++optVehDataSet.totalLaneChangeTimes;
		optVehDataSet.laneChangeDetailSet[optVehDataSet.totalLaneChangeTimes].occurenceVehID = vehicle->getId();
		optVehDataSet.laneChangeDetailSet[optVehDataSet.totalLaneChangeTimes].occurrenceTime = AKIGetCurrentSimulationTime();
		optVehDataSet.laneChangeDetailSet[optVehDataSet.totalLaneChangeTimes].occurrencePositionInSection = vehicle->getPosition(0);
		optVehDataSet.laneChangeDetailSet[optVehDataSet.totalLaneChangeTimes].occurrenceSection = vehicle->getIdCurrentSection();

		double temp_xback, temp_back;
		vehicle->getCoordinates(optVehDataSet.laneChangeDetailSet[optVehDataSet.totalLaneChangeTimes].occurrenceCoordinationX, optVehDataSet.laneChangeDetailSet[optVehDataSet.totalLaneChangeTimes].occurrenceCoordinationY, temp_xback, temp_back);

		optVehDataSet.laneChangeDetailSet[optVehDataSet.totalLaneChangeTimes].preLane = optVehDataSet.preLane;
		optVehDataSet.laneChangeDetailSet[optVehDataSet.totalLaneChangeTimes].folLane = currAbsoluteLaneID;
		optVehDataSet.preLane = currAbsoluteLaneID;
	}
}
void behavioralModelParticular::getLeadersAccelerationsDistributionDifference(A2SimVehicle * currentVehicle, double &diff_mean_left, double &diff_sd_left, double &diff_mean_right, double &diff_sd_right)
{




	A2SimVehicle * leaders_right[12] = { 0 };
	A2SimVehicle * leaders_current[12] = { 0 };
	A2SimVehicle * leaders_left[12] = { 0 };

	double accelerations_right[12] = { 0 };
	double accelerations_current[12] = { 0 };
	double accelerations_left[12] = { 0 };

	double sum_acceleration_right = 0;
	double sum_acceleration_current = 0;
	double sum_acceleration_left = 0;

	double mean_right = 0;
	double mean_current = 0;
	double mean_left = 0;

	double standardDeviation_right = 0;
	double standardDeviation_current = 0;
	double standardDeviation_left = 0;





	A2SimVehicle *pVehLeftDw = NULL;
	A2SimVehicle *pVehLeftUp = NULL;
	A2SimVehicle *pVehRightDw = NULL;
	A2SimVehicle *pVehRightUp = NULL;
	A2SimVehicle *pVehCurUp = NULL;
	A2SimVehicle *pVehCurDw = NULL;

	double shiftCurUp = 0, shiftCurDw = 0;
	double ShiftUpLeft = 0, ShiftDwLeft = 0;
	double ShiftUpRight = 0, ShiftDwRight = 0;

	leaders_current[0] = currentVehicle;
	leaders_left[0] = currentVehicle;
	leaders_right[0] = currentVehicle;


	//get the first leader in current lane
	currentVehicle->getUpDown(0, currentVehicle->getPosition(0), pVehCurUp, shiftCurUp, pVehCurDw, shiftCurDw);

	//get the first leader in left lane
	currentVehicle->getUpDown(-1, currentVehicle->getPosition(0), pVehLeftUp, ShiftUpLeft, pVehLeftDw, ShiftDwLeft);

	//get the first leader in right lane
	currentVehicle->getUpDown(1, currentVehicle->getPosition(0), pVehRightUp, ShiftUpRight, pVehRightDw, ShiftDwRight);

	leaders_current[1] = pVehCurDw;
	leaders_right[1] = pVehRightDw;
	leaders_left[1] = pVehLeftDw;




	// 11 leaders for each lane, 1-11, 10 accelerations 
	for (int i = 1; i <= 10; ++i)
	{


		A2SimVehicle *pVehLeftDw = NULL;
		A2SimVehicle *pVehLeftUp = NULL;
		A2SimVehicle *pVehRightDw = NULL;
		A2SimVehicle *pVehRightUp = NULL;
		A2SimVehicle *pVehCurUp = NULL;
		A2SimVehicle *pVehCurDw = NULL;

		double shiftCurUp = 0, shiftCurDw = 0;
		double ShiftUpLeft = 0, ShiftDwLeft = 0;
		double ShiftUpRight = 0, ShiftDwRight = 0;


		// if the leader does not exist, the pointer will be NULL, the acceleration will be 0
		// get the next leader in current lane, 
		if (leaders_current[i] != NULL)
		{
			leaders_current[i]->getUpDown(0, leaders_current[i]->getPosition(0), pVehCurUp, shiftCurUp, pVehCurDw, shiftCurDw);
		}
		// get the next leader in left lane
		if (leaders_left[i] != NULL)
		{
			leaders_left[i]->getUpDown(-1, leaders_left[i]->getPosition(0), pVehLeftUp, ShiftUpLeft, pVehLeftDw, ShiftDwLeft);
		}
		// get the next leader in right lane
		if (leaders_right[i] != NULL)
		{
			leaders_right[i]->getUpDown(1, leaders_right[i]->getPosition(0), pVehRightUp, ShiftUpRight, pVehRightDw, ShiftDwRight);
		}


		if (pVehCurDw != NULL)
		{
			leaders_current[i + 1] = pVehCurDw;
			accelerations_current[i] = get_IDM_acceleration(leaders_current[i], leaders_current[i + 1]);
		}

		if (pVehLeftDw != NULL)
		{
			leaders_left[i + 1] = pVehLeftDw;
			accelerations_left[i] = get_IDM_acceleration(leaders_left[i], leaders_left[i + 1]);
		}

		if (pVehRightDw != NULL)
		{
			leaders_right[i + 1] = pVehRightDw;
			accelerations_right[i] = get_IDM_acceleration(leaders_right[i], leaders_right[i + 1]);
		}



	}
	for (int i = 1; i <= 10; ++i)
	{
		sum_acceleration_current += accelerations_current[i];
		sum_acceleration_left += accelerations_left[i];
		sum_acceleration_right += accelerations_right[i];
	}

	mean_current = sum_acceleration_current / 10;
	mean_left = sum_acceleration_left / 10;
	mean_right = sum_acceleration_right / 10;


	double deviationSUM_current = 0;
	double deviationSUM_left = 0;
	double deviationSUM_right = 0;

	for (int i = 1; i <= 10; ++i)
	{
		deviationSUM_current += (accelerations_current[i] - mean_current)*(accelerations_current[i] - mean_current);
		deviationSUM_right += (accelerations_right[i] - mean_right)*(accelerations_right[i] - mean_right);
		deviationSUM_left += (accelerations_left[i] - mean_left)*(accelerations_left[i] - mean_left);
	}


	standardDeviation_current = sqrt(0.1 * deviationSUM_current);
	standardDeviation_right = sqrt(0.1 * deviationSUM_right);
	standardDeviation_left = sqrt(0.1 * deviationSUM_left);



	diff_sd_right = standardDeviation_right - standardDeviation_current;
	diff_sd_left = standardDeviation_left - standardDeviation_current;

	diff_mean_right = mean_right - mean_current;
	diff_mean_left = mean_left - mean_current;




	//string outPutSDInfoFullPath;
	//string outPutSDInfoFileName = "ACCELERATION_SDInfo.dat";
	//outPutSDInfoFullPath = DATAPATH + outPutSDInfoFileName;
	//ofstream outPutSDInfo;
	//outPutSDInfo.open(outPutSDInfoFullPath, ios::app);

	//outPutSDInfo
	//	<< diff_mean_left << "\t"
	//	<< diff_sd_left << "\n"
	//	<< diff_mean_right << "\t"
	//	<< diff_sd_right << endl;


	//outPutSDInfo.close();


}

void recordOptVehiclTrajectory(A2SimVehicle *vehicle, double currentTime, int currSectionID)
{


	TrajectoryData trajectoryDataPoint;
	trajectoryDataPoint.time = currentTime;
	trajectoryDataPoint.speed = vehicle->getSpeed(0);
	trajectoryDataPoint.pathLengthPerSecond = optVehDataSet.totalTravelPathLength;


	/*if (needTestMsg && currentTime > 5740 && currentTime < 5760)
	{
		char msg[200];
		sprintf_s(msg, "%f,%f,%f,%d,%d", optVehDataSet.totalTravelPathLength, vehicle->getPosition(0), sectionInfo.length, sectionInfo.id, currSectionID);
		AKIPrintString(msg);
	}*/

	double xback_temp = 0;
	double yback_temp = 0;
	vehicle->getCoordinates(trajectoryDataPoint.coordinateX, trajectoryDataPoint.coordinateY, xback_temp, yback_temp);
	optVehDataSet.trajectoryDataSet.push_back(trajectoryDataPoint);
}


void inputParameterSetFromAFT()
{
	if (!haveInPutFunctionsRan)
	{

		string parameterFullPath;
		string parameterFileName = "parameters1.dat";
		parameterFullPath = AFTPATH + parameterFileName;
		ifstream parameterFile;
		parameterFile.open(parameterFullPath, ios::in);

		//for (std::map<int, double>::iterator iter = parameterSet.begin(); iter != parameterSet.end(); ++iter)
		//	{
		//		parameterFile >> iter->second;
		//	}

		parameterFile >> parameterSet[363]; // optimizedPolitenessFactor--P
		parameterFile >> parameterSet[364]; // optimizedThreshold--T
		parameterFile.close();


		/*	char msg[150];
			sprintf_s(msg, "SmartVehID=%d MOBIL Parameters loaded. (P,T)=(%f,%f)", optimiazedVehID, parameterSet[363], parameterSet[364]);
			AKIPrintString(msg);*/

		haveInPutFunctionsRan = true;
	}

}


void inputQTable()
{
	if (!hasInputQTable)
	{
		string inputQTableFullPath;
		string inputQTableFileName = "QTable.dat";
		inputQTableFullPath = DATAPATH + inputQTableFileName;
		ifstream inputQTableFile;
		inputQTableFile.open(inputQTableFullPath, ios::in);
		if (inputQTableFile.is_open())
		{
			for (int deta_a1L = 0; deta_a1L <= 5; ++deta_a1L)
			{
				for (int deta_a1R = 0; deta_a1R <= 5; ++deta_a1R)
				{
					for (int deta_a2L = 0; deta_a2L <= 5; ++deta_a2L)
					{
						for (int deta_a3 = 0; deta_a3 <= 5; ++deta_a3)
						{
							for (int deta_a4R = 0; deta_a4R <= 5; ++deta_a4R)
							{
								for (int deta_miuL = 0; deta_miuL <= 5; ++deta_miuL)
								{
									for (int deta_sigmaL = 0; deta_sigmaL <= 5; ++deta_sigmaL)
									{
										for (int deta_miuR = 0; deta_miuR <= 5; ++deta_miuR)
										{
											for (int deta_sigmaR = 0; deta_sigmaR <= 5; ++deta_sigmaR)
											{
												for (int actions = 0; actions <= 2; ++actions)
												{
													inputQTableFile >>
														q_LearningParameters.q_table[deta_a1L][deta_a1R][deta_a2L][deta_a3][deta_a4R][deta_miuL][deta_sigmaL][deta_miuR][deta_sigmaR][actions];
												}
											}
										}
									}
								}
							}
						}
					}
				}
			}
		}

		inputQTableFile.close();

		hasInputQTable = true;
	}
}
void outPutQTable()
{
	string outPutQTableFullPath;
	string outPutQTableFileName = "QTable.dat";
	outPutQTableFullPath = DATAPATH + outPutQTableFileName;
	ofstream outPutQTable;
	outPutQTable.open(outPutQTableFullPath, ios::trunc);

	// 6^10*3 float ， 0.72 GB memory cost
	// a2	口	口    Left 
	// a3	a1	口    Current
	// a4	口	口	  Right
	// [deta_a1L][deta_a1R][deta_a2L][deta_a3][deta_a4R][deta_miuL][deta_sigmaL][deta_miuR][deta_sigmaR][actions] 10 Dimensions
	// [3] Actions:0 current 1 left 2 right
	for (int deta_a1L = 0; deta_a1L <= 5; ++deta_a1L)
	{
		for (int deta_a1R = 0; deta_a1R <= 5; ++deta_a1R)
		{
			for (int deta_a2L = 0; deta_a2L <= 5; ++deta_a2L)
			{
				for (int deta_a3 = 0; deta_a3 <= 5; ++deta_a3)
				{
					for (int deta_a4R = 0; deta_a4R <= 5; ++deta_a4R)
					{
						for (int deta_miuL = 0; deta_miuL <= 5; ++deta_miuL)
						{
							for (int deta_sigmaL = 0; deta_sigmaL <= 5; ++deta_sigmaL)
							{
								for (int deta_miuR = 0; deta_miuR <= 5; ++deta_miuR)
								{
									for (int deta_sigmaR = 0; deta_sigmaR <= 5; ++deta_sigmaR)
									{
										for (int actions = 0; actions <= 2; ++actions)
										{
											outPutQTable
												<< q_LearningParameters.q_table[deta_a1L][deta_a1R][deta_a2L][deta_a3][deta_a4R][deta_miuL][deta_sigmaL][deta_miuR][deta_sigmaR][actions] << "\t";
										}
										outPutQTable << endl;
									}
								}
							}
						}
					}
				}
			}
		}
	}

	outPutQTable.close();

}

//filter control Group Vehicles from allVehicleODInfo and output
void outPutControlGroupVehiclesODInfo()
{
	string outPutControlGroupVehicleODInfoFullPath;
	string outPutControlGroupVehicleODInfoFileName = "ControlGroupVehicleODInfo.dat";
	outPutControlGroupVehicleODInfoFullPath = DATAPATH + outPutControlGroupVehicleODInfoFileName;
	ofstream outPutControlGroupVehicleODInfo;
	outPutControlGroupVehicleODInfo.open(outPutControlGroupVehicleODInfoFullPath, ios::app);

	outPutControlGroupVehicleODInfo
		<< "VehicleID" << "\t"
		<< "entrySection" << "\t"
		<< "exitSection" << "\t"
		<< "entryTime" << "\t"
		<< "exitTime" << "\t"
		<< "totalTravelPathLength" << "\t"
		<< "totalTravelTime" << "\t"
		<< "Average Travel Time" << endl;


	for (auto vehID = 1; vehID < 16000; ++vehID)
	{
		if (isControlGroupVehicle(optimiazedVehID, vehID))
		{
			outPutControlGroupVehicleODInfo
				<< vehID << "\t"
				<< allVehicleODInfoDataSet[vehID].entrySection << "\t"
				<< allVehicleODInfoDataSet[vehID].exitSection << "\t"
				<< allVehicleODInfoDataSet[vehID].entryTime << "\t"
				<< allVehicleODInfoDataSet[vehID].totalTravelPathLength << "\t"
				<< allVehicleODInfoDataSet[vehID].totalTravelTime << "\t"
				<< allVehicleODInfoDataSet[vehID].totalTravelTime / allVehicleODInfoDataSet[vehID].totalTravelPathLength * 1000 << endl;
		}
	}
	outPutControlGroupVehicleODInfo << endl;

	outPutControlGroupVehicleODInfo.close();

}


void outPutAllVehicleODInfo()
{
	string outPutAllVehicleODInfoFullPath;
	string outPutAllVehicleODInfoFileName = "AllVehicleODInfo.dat";
	outPutAllVehicleODInfoFullPath = DATAPATH + outPutAllVehicleODInfoFileName;
	ofstream outPutAllVehicleODInfo;
	outPutAllVehicleODInfo.open(outPutAllVehicleODInfoFullPath, ios::app);

	outPutAllVehicleODInfo
		<< "VehicleID" << "\t"
		<< "entrySection" << "\t"
		<< "exitSection" << "\t"
		<< "entryTime" << "\t"
		<< "exitTime" << "\t"
		<< "totalTravelPathLength" << "\t"
		<< "totalTravelTime" << "\t"
		<< "Average Travel Time" << "\t"
		<< "Smart Vehicle Identify"
		<< endl;
	for (auto vehID = 1; vehID < 16000; ++vehID)
	{
		outPutAllVehicleODInfo
			<< vehID << "\t"
			<< allVehicleODInfoDataSet[vehID].entrySection << "\t"
			<< allVehicleODInfoDataSet[vehID].exitSection << "\t"
			<< allVehicleODInfoDataSet[vehID].entryTime << "\t"
			<< allVehicleODInfoDataSet[vehID].totalTravelPathLength << "\t"
			<< allVehicleODInfoDataSet[vehID].totalTravelTime << "\t"
			<< allVehicleODInfoDataSet[vehID].totalTravelTime / allVehicleODInfoDataSet[vehID].totalTravelPathLength * 1000 << "\t"
			<< allVehicleODInfoDataSet[vehID].isSmartVehicle
			<< endl;

	}
	outPutAllVehicleODInfo << endl;

	outPutAllVehicleODInfo.close();

}


void outPutImpactedVehicleIDSet()
{
	string outPutImpactedVehIDFullPath;
	string outPutImpactedVehIDFileName = "ImpactedVehicleIDSet.dat";
	outPutImpactedVehIDFullPath = DATAPATH + outPutImpactedVehIDFileName;
	ofstream outPutImpactedVehID;
	outPutImpactedVehID.open(outPutImpactedVehIDFullPath, ios::trunc);
	for (auto impactedVehIDIter = impactedVehIDSet.begin(); impactedVehIDIter != impactedVehIDSet.end(); ++impactedVehIDIter)
	{
		outPutImpactedVehID << *impactedVehIDIter << "\t";
	}
	outPutImpactedVehID.close();

}


void outPutOptVehTrajectoryDataSet()
{
	string outPutOptVehTrajectoryFullPath;
	string outPutOptVehTrajectoryFileName = "OptVehTrajectoryDataSet.dat";
	outPutOptVehTrajectoryFullPath = DATAPATH + outPutOptVehTrajectoryFileName;
	ofstream outPutOptVehTrajectory;
	outPutOptVehTrajectory.open(outPutOptVehTrajectoryFullPath, ios::app);

	outPutOptVehTrajectory
		<< "Time" << "\t"
		<< "Position" << "\t"
		<< "Speed" << "\t"
		<< "PathLength"
		<< endl;


	for (auto optVehTrajectoryIter : optVehDataSet.trajectoryDataSet)
	{
		outPutOptVehTrajectory
			<< optVehTrajectoryIter.time << "\t"
			<< sqrt(optVehTrajectoryIter.coordinateX*optVehTrajectoryIter.coordinateX + optVehTrajectoryIter.coordinateY*optVehTrajectoryIter.coordinateY) << "\t"
			<< optVehTrajectoryIter.speed << "\t"
			<< optVehTrajectoryIter.pathLengthPerSecond
			<< endl;

	}
	outPutOptVehTrajectory.close();

}


void outPutOptVehLaneChangingDetials()
{

	string outPutLaneChangingFullPath;
	string outPutLaneChangingFileName = "OptVehLaneChangingDetails.dat";
	outPutLaneChangingFullPath = DATAPATH + outPutLaneChangingFileName;
	ofstream outPutLaneChanging;
	outPutLaneChanging.open(outPutLaneChangingFullPath, ios::app);
	outPutLaneChanging
		<< "TotalLaneChangingTimes\t" << optVehDataSet.totalLaneChangeTimes << "\n"
		<< "VehicleID" << "\t"
		<< "OccurrenceTime" << "\t"
		<< "OccurrencePositionInSection" << "\t"
		<< "OccurrenceSection" << "\t"
		<< "OccurrencePositionXY" << "\t"
		<< "PreviousLane" << "\t"
		<< "FollowingLane"
		<< endl;
	int iter;
	for (iter = 1; iter <= optVehDataSet.totalLaneChangeTimes; ++iter)
	{
		outPutLaneChanging
			<< optVehDataSet.laneChangeDetailSet[iter].occurenceVehID << "\t"
			<< optVehDataSet.laneChangeDetailSet[iter].occurrenceTime << "\t"
			<< optVehDataSet.laneChangeDetailSet[iter].occurrencePositionInSection << "\t"
			<< optVehDataSet.laneChangeDetailSet[iter].occurrenceSection << "\t"
			<< sqrt(
				pow(optVehDataSet.laneChangeDetailSet[iter].occurrenceCoordinationX, 2)
				+
				pow(optVehDataSet.laneChangeDetailSet[iter].occurrenceCoordinationY, 2)) << "\t"
			<< optVehDataSet.laneChangeDetailSet[iter].preLane << "\t"
			<< optVehDataSet.laneChangeDetailSet[iter].folLane << "\t"
			<< endl;
		if (iter == optVehDataSet.totalLaneChangeTimes)
			outPutLaneChanging << endl;
	}

	outPutLaneChanging.close();

}



void outPutOptVehPerformance()
{
	string outPutPerformanceFullPath;
	string outPutPerformanceFileName = "Performance.txt";
	outPutPerformanceFullPath = DATAPATH + outPutPerformanceFileName;
	ofstream outPutPerformance;
	outPutPerformance.open(outPutPerformanceFullPath, ios::trunc);
	outPutPerformance
		<< optVehDataSet.totalTravelTime / optVehDataSet.totalTravelPathLength * 1000 << endl;
	outPutPerformance.close();

}

// iteration vehicle id, OD and time information about entry and exit
void outPutOptVehData()
{
	string outPutDataFullPath;
	string outPutDataFileName = "OptimizedVehicleData.txt";
	outPutDataFullPath = DATAPATH + outPutDataFileName;
	ofstream outPutData;
	outPutData.open(outPutDataFullPath, ios::app);

	for (int num = 1; num <= optimizedExperienceTimes; ++num)
	{

		if (num == 1)
			outPutData << "IterationTimes" << "\t" << optimizedExperienceTimes << "\n";

		outPutData << optimizedVehIDSequence[num] << "\t";

		if (num == optimizedExperienceTimes)
			outPutData << endl;
	}

	outPutData
		//<<optimiazedVehID << "\t"
		<< optVehDataSet.entryTime << "\t"
		<< optVehDataSet.exitTime << "\t"
		<< optVehDataSet.totalTravelTime << "\t"
		<< optVehDataSet.totalTravelPathLength << "\t"
		<< optVehDataSet.totalTravelTime / optVehDataSet.totalTravelPathLength * 1000 << endl;
	outPutData.close();

}


//TEST, for investigating the relation between lane changing times and politeness factor p 
/*
void processLCProfitData(
	A2SimVehicle *vehicle,
	double ac_Right_profit,
	double ac_Left_profit,
	double af_Right_profit,
	double af_Left_profit,
	int curLane,
	int maxLanes,

	double	ac_Left_LC,			//ac*	//turn left
	double	ac_Left_NOLC,		//ac	//turn left
	double	an_Left_LC,			//an*	//turn left
	double	an_Left_NOLC,		//an	//turn left
	double	ao_Left_LC,			//ao*	//turn left
	double	ao_Left_NOLC,		//ao	//turn left


	double	ac_Right_LC,			//ac*	//turn Right
	double	ac_Right_NOLC,			//ac	//turn Right
	double	an_Right_LC,			//an*	//turn Right
	double	an_Right_NOLC,			//an	//turn Right
	double	ao_Right_LC,			//ao*	//turn Right
	double	ao_Right_NOLC, 			//ao	//turn right
	double  distance_to_left_leader,
	double  distance_to_right_leader
)
{
	double currTime = AKIGetCurrentSimulationTime();

	string outPutLCprofitFullPath;
	string outPutLCprofitFileName = "LCprofit.txt";
	outPutLCprofitFullPath = DATAPATH + outPutLCprofitFileName;
	ofstream outPutLCprofit;

	outPutLCprofit.open(outPutLCprofitFullPath, ios::app);
	if (lcProfitOpenRunTime == 0)
	{
		outPutLCprofit << "SIMULATION START" << endl;
		lcProfitOpenRunTime = 1;
	}
	if (curLane < maxLanes && curLane > 1) //turn left && right are both possible lane=2
	{
		outPutLCprofit
			<< vehicle->getId() << "\t"
			<< currTime << "\t"
			<< vehicle->getIdCurrentSection() << "\t"
			<< vehicle->getIdCurrentLane() << "\t"
			<< ac_Left_profit << "\t"
			<< ac_Right_profit << "\t"
			<< af_Left_profit << "\t"
			<< af_Right_profit << "\t"

			<< ac_Left_LC << "\t"			//ac*	//turn left
			<< ac_Left_NOLC << "\t"		//ac	//turn left
			<< an_Left_LC << "\t"			//an*	//turn left
			<< an_Left_NOLC << "\t"		//an	//turn left
			<< ao_Left_LC << "\t"			//ao*	//turn left
			<< ao_Left_NOLC << "\t"		//ao	//turn left

			<< ac_Right_LC << "\t"			//ac*	//turn Right
			<< ac_Right_NOLC << "\t"			//ac	//turn Right
			<< an_Right_LC << "\t"			//an*	//turn Right
			<< an_Right_NOLC << "\t"			//an	//turn Right
			<< ao_Right_LC << "\t"			//ao*	//turn Right
			<< ao_Right_NOLC << "\t"			//ao	//turn right
			<< distance_to_left_leader << "\t"
			<< distance_to_right_leader << endl;

	}
	else if (curLane < maxLanes) // only turn left is possible, lane=1
	{
		outPutLCprofit
			<< vehicle->getId() << "\t"
			<< currTime << "\t"
			<< vehicle->getIdCurrentSection() << "\t"
			<< vehicle->getIdCurrentLane() << "\t"
			<< ac_Left_profit << "\t"
			<< "NULL" << "\t"
			<< af_Left_profit << "\t"
			<< "NULL" << "\t"

			<< ac_Left_LC << "\t"			//ac*	//turn left
			<< ac_Left_NOLC << "\t"		//ac	//turn left
			<< an_Left_LC << "\t"			//an*	//turn left
			<< an_Left_NOLC << "\t"		//an	//turn left
			<< ao_Left_LC << "\t"			//ao*	//turn left
			<< ao_Left_NOLC << "\t"		//ao	//turn left

			<< "NULL" << "\t"			//ac*	//turn Right
			<< "NULL" << "\t"			//ac	//turn Right
			<< "NULL" << "\t"			//an*	//turn Right
			<< "NULL" << "\t"			//an	//turn Right
			<< "NULL" << "\t"			//ao*	//turn Right
			<< "NULL" << "\t"		//ao	//turn right
			<< distance_to_left_leader << "\t"
			<< distance_to_right_leader << endl;

	}
	else if (curLane > 1) // only turn right is possible, lane=3
	{
		outPutLCprofit
			<< vehicle->getId() << "\t"
			<< currTime << "\t"
			<< vehicle->getIdCurrentSection() << "\t"
			<< vehicle->getIdCurrentLane() << "\t"
			<< "NULL" << "\t"
			<< ac_Right_profit << "\t"
			<< "NULL" << "\t"
			<< af_Right_profit << "\t"

			<< "NULL" << "\t"			//ac*	//turn left
			<< "NULL" << "\t"		//ac	//turn left
			<< "NULL" << "\t"			//an*	//turn left
			<< "NULL" << "\t"		//an	//turn left
			<< "NULL" << "\t"			//ao*	//turn left
			<< "NULL" << "\t"		//ao	//turn left

			<< ac_Right_LC << "\t"			//ac*	//turn Right
			<< ac_Right_NOLC << "\t"			//ac	//turn Right
			<< an_Right_LC << "\t"			//an*	//turn Right
			<< an_Right_NOLC << "\t"			//an	//turn Right
			<< ao_Right_LC << "\t"			//ao*	//turn Right
			<< ao_Right_NOLC << "\t"		//ao	//turn right
			<< distance_to_left_leader << "\t"
			<< distance_to_right_leader << endl;
	}
	else // lane changing is limited by road geometry and is impossible, lane = 0
	{
		outPutLCprofit
			<< vehicle->getId() << "\t"
			<< currTime << "\t"
			<< vehicle->getIdCurrentSection() << "\t"
			<< vehicle->getIdCurrentLane() << "\t"
			<< "NULL" << "\t"
			<< "NULL" << "\t"
			<< "NULL" << "\t"
			<< "NULL" << "\t"

			<< "NULL" << "\t"			//ac*	//turn left
			<< "NULL" << "\t"		//ac	//turn left
			<< "NULL" << "\t"			//an*	//turn left
			<< "NULL" << "\t"		//an	//turn left
			<< "NULL" << "\t"			//ao*	//turn left
			<< "NULL" << "\t"		//ao	//turn left

			<< "NULL" << "\t"			//ac*	//turn Right
			<< "NULL" << "\t"			//ac	//turn Right
			<< "NULL" << "\t"			//an*	//turn Right
			<< "NULL" << "\t"			//an	//turn Right
			<< "NULL" << "\t"			//ao*	//turn Right
			<< "NULL" << "\t"		//ao	//turn right
			<< distance_to_left_leader << "\t"
			<< distance_to_right_leader << endl;
	}



	outPutLCprofit.close();





}

*/


/*
// For farsighted Lane Changing based on the MOBIL in VXV enviroment. by Wang Long
double getFutureA(A2SimVehicle *pVehCur, double laneDensity, double laneSpeed)
{
	double futrueA = 0;

	simVehicleParticular* vehicleTemp = (simVehicleParticular*)pVehCur;
	bool isACC = vehicleTemp->getIsACC();

	double timeGap = pVehCur->getMinimumHeadway();
	if (isACC)
	{
		timeGap = vehicleTemp->getACCTimeGap();
	}

	double a = pVehCur->getAcceleration();
	double b = -pVehCur->getDeceleration();
	double speed_current = pVehCur->getSpeed(0); // m/s
	double speed_leader = laneSpeed;
	double gapToLeader = (1.0 / laneDensity);


	double desiredGap =
		pVehCur->getMinimumDistanceInterVeh()
		+
		max(0., speed_current*timeGap + speed_current*(speed_current - speed_leader) / (2 * sqrt(a*b))); // replace with 2 * sqrt



	double X = pVehCur->getSpeed(0) / pVehCur->getFreeFlowSpeed();
	futrueA = max(pVehCur->getDecelerationMax(), a*(1 - pow(X, 4) - (desiredGap / gapToLeader)*(desiredGap / gapToLeader)));


	return futrueA;




}

// For farsighted Lane Changing based on the MOBIL in VXV enviroment.
// computing modified threshold of MOBIL according far-sight
double behavioralModelParticular::getModifiedThreshold(A2SimVehicle* vehicle, int targetLane)
{
	double modifiedValue[5][5] = { 0 };
	int sightDistance = 1;
	int currLane = vehicle->getIdCurrentLane();
	int currSectionID = vehicle->getIdCurrentSection();
	int currAbsLaneID = getNetWorkAbsoluteLaneID(currSectionID, currLane);

	// get Sequence of current section
	int currSequence = 0;
	for (auto seq : sequenceSectionID)
	{
		if (sequenceSectionID[seq] == currSectionID)
		{
			currSequence = seq;
			break;
		}
	}
	int farSectionID = 0; //section num
	if (currSequence <= 22 - sightDistance)
	{
		farSectionID = sequenceSectionID[currSequence + sightDistance];
	}


	double laneDensity[5] = { 0 };
	double laneSpeed[5] = { 0 };
	int allSectionNum = AKIInfNetNbSectionsANG();
	for (int iter_section = 0; iter_section < allSectionNum; iter_section++)
	{
		int sectionID = AKIInfNetGetSectionANGId(iter_section);
		if (sectionID == farSectionID)
		{
			int allVehNumInSection = AKIVehStateGetNbVehiclesSection(sectionID, true);
			int vehNumOfLane[5] = { 0 };
			double vehSpeedSum[5] = { 0 };
			for (int iter_veh = 0; iter_veh < allVehNumInSection; iter_veh++)
			{
				InfVeh currentVeh = AKIVehStateGetVehicleInfSection(sectionID, iter_veh);
				int currentVehID = currentVeh.idVeh;
				int currentLane = currentVeh.numberLane;
				int absLane = getNetWorkAbsoluteLaneID(sectionID,currentLane);
				vehNumOfLane[absLane]++;
				vehSpeedSum[absLane] += currentVeh.CurrentSpeed;
			}
			A2KSectionInf currentSectionInfo;
			currentSectionInfo = AKIInfNetGetSectionANGInf(sectionID);

			if (currentSectionInfo.length != 0)
			{
				laneDensity[1] = 1000 * vehNumOfLane[1] / currentSectionInfo.length;
				laneDensity[2] = 1000 * vehNumOfLane[2] / currentSectionInfo.length;
				laneDensity[3] = 1000 * vehNumOfLane[3] / currentSectionInfo.length;
				laneDensity[4] = 1000 * vehNumOfLane[4] / currentSectionInfo.length;
			}
			for (int iter_lane = 0; iter_lane <= 4; iter_lane++)
			{
				if (vehNumOfLane[iter_lane]!=0)
				{
					laneSpeed[iter_lane] = vehSpeedSum[iter_lane] / vehNumOfLane[iter_lane];
				}
			}

		}

	}

	// current lane TO target lane 1~4 TO 1~4

	A2SimVehicle *pVehCurUp = NULL;
	A2SimVehicle *pVehCurDw = NULL;
	double shiftCurUp = 0, shiftCurDw = 0;
	vehicle->getUpDown(0, vehicle->getPosition(0), pVehCurUp, shiftCurUp, pVehCurDw, shiftCurDw);

	for (int iter_lane = 1; iter_lane <= 4; iter_lane++)
	{
		if (laneDensity[iter_lane] != 0)
		{

				modifiedValue[currLane][iter_lane] =
				getFutureA(vehicle, laneDensity[iter_lane],laneSpeed[iter_lane])
				-
				get_IDM_acceleration(vehicle, pVehCurDw);
		}
		else
		{
			modifiedValue[currLane][iter_lane]= get_IDM_acceleration(vehicle, NULL)- get_IDM_acceleration(vehicle, pVehCurDw);
		}
	}





	return modifiedValue[currLane][targetLane];







}
*/




//function for debug, test 2018-9-20 14:03:20
void debug_needDebugMessage(char* message, double value)
{
	char msg[200];
	sprintf_s(msg, "xxxxxxxxx", value);
	AKIPrintString(msg);

}



/***************************Build-in Function  **************************/
behavioralModelParticular::behavioralModelParticular() : A2BehavioralModel()
{
	const unsigned short *randomSeedString = AKIConvertFromAsciiString("GKReplication::randomSeedAtt");
	seed = ANGConnGetAttributeValueInt(ANGConnGetAttribute(randomSeedString), ANGConnGetReplicationId());
	const unsigned short *param0 = AKIConvertFromAsciiString("GKExperiment::p_distance");
	p_distance = ANGConnGetAttributeValueDouble(ANGConnGetAttribute(param0), ANGConnGetExperimentId());

	isOnRamp = false;
	isSegmRamp = false;
}

behavioralModelParticular::~behavioralModelParticular()
{
}

simVehicleParticular * behavioralModelParticular::arrivalNewVehicle(void *handlerVehicle, unsigned short idHandler, bool isFictitiousVeh)
{

	simVehicleParticular * newVehicle = new simVehicleParticular(handlerVehicle, idHandler, isFictitiousVeh);
	if (!isFictitiousVeh)
	{
		// for ACC
		if (AKIGetRandomNumber() < penetrationOfACC)
		{
			newVehicle->setIsACC(true);
			newVehicle->setTimeGapOfACC(generateTimeGapOfACC(lowLimitOfACCTimeGap, upLimitOfACCTimeGap));
		}


		/*Smart Vehicle Demand, Penetration Rate */


		// method 1, using the discreted deamnd to denote smart vehicles
		/*
		if (readSmartVehicleDemandRumTimes == 0)
		{
			readSmartVehicleDemand();

			readSmartVehicleDemandRumTimes = 1;
		}

		int timeIntervalSequence = -1;
		int inputFlowSectionSequence = -1;
		if (timeIntervalSequence != floor(AKIGetCurrentSimulationTime() / timeInverval))
		{
			timeIntervalSequence = floor(AKIGetCurrentSimulationTime() / timeInverval);
		}

		if (inputFlowSectionSequence != getEntrySectionSequence(newVehicle->getIdCurrentSection()))
		{
			inputFlowSectionSequence = getEntrySectionSequence(newVehicle->getIdCurrentSection());
		}

		if (inputFlowSectionSequence != -1 && smartVehicleNum_entrySection_temp[inputFlowSectionSequence] < smartVehicleDemand[inputFlowSectionSequence][timeIntervalSequence])
		{
			newVehicle->setIsSmartVehicle(true);
			smartVehicleNum_entrySection_temp[inputFlowSectionSequence]++;
		}
		else if (inputFlowSectionSequence != -1 && smartVehicleNum_entrySection_temp[inputFlowSectionSequence] >= smartVehicleDemand[inputFlowSectionSequence][timeIntervalSequence])
		{
			smartVehicleNum_entrySection_temp[inputFlowSectionSequence] = 0;
		}
	}

	if (needTestMsg)
	{

		string outPutDataFullPath;
		string outPutDataFileName = "SmartVehicleDemandTEST.txt";
		outPutDataFullPath = DATAPATH + outPutDataFileName;
		ofstream outPutData;
		outPutData.open(outPutDataFullPath, ios::app);

		outPutData
			<< newVehicle->getId() << "\t"
			<< newVehicle->getIdCurrentSection() << "\t"
			<< AKIGetCurrentSimulationTime() << endl;
		outPutData.close();
	}
	*/

	// method 2, using randomly denoting
		if (useOutSideInPut_SmartVehiclePenetrationRate && (!hasReadSmartVehiclePenetrationRate))
		{
			readSmartVehiclePenetrationRate();
			hasReadSmartVehiclePenetrationRate = true;
		}


		if (AKIGetRandomNumber() < penetrationOfSmartVehicles) // penetrationOfSmartVehicles, default value = 0
		{
			newVehicle->setIsSmartVehicle(true);
		}
	}

	return newVehicle;
}
void behavioralModelParticular::removedVehicle(void *handlerVehicle, unsigned short idHandler, A2SimVehicle * a2simVeh)
{
}

bool behavioralModelParticular::evaluateCarFollowing(A2SimVehicle *vehicle, double &newpos, double &newspeed)
{
	return false;
}


bool behavioralModelParticular::evaluateLaneChanging(A2SimVehicle *vehicle, int threadId)
{
	double currStep = AKIGetCurrentSimulationTime();
	double currTime = currStep;
	int currSectionID = vehicle->getIdCurrentSection();
	int direction = 0, targetLane;// actually, targetLane is useless,(WL)
	int currLane = vehicle->getIdCurrentLane();
	int vehID = vehicle->getId();
	int numSect = vehicle->getIdCurrentSection();
	double currSpeed = vehicle->getSpeed(0);
	int maxLanes = vehicle->getNumberOfLanesInCurrentSection();


	simVehicleParticular* vehicle_particular_Temp = (simVehicleParticular*)vehicle; //evaluateLaneChanging使用的是A2SimVehicle类，但arrivalNewVehicle产生的是它的子类
	//bool isACC = vehicleTemp->getIsACC();
	//bool isOptimizedVehicle = false; //vehicleTemp->getIsOptimizedVehicle();



	if (useIterationOptimization && haveOptimizedVeh.isExist == false && (getEntrySectionSequence(currSectionID) != -1))
	{
		optimiazedVehID = vehID;
		haveOptimizedVeh.isExist = true;
	}


	if (vehicle_particular_Temp->getIsSmartVehicle() || vehID == optimiazedVehID)
	{

		//inputParameterSetFromAFT();// input parameters to  <map>parameterSet, it will be ran only once
		if (useQLearning) 
		{
			inputQTable();
		}
		recordOptVehicleTravelTime(currTime);
		recordOptVehiclePathLength(vehicle);
		recordOptVehiclLaneChangingInfo(vehicle);
		recordOptVehiclTrajectory(vehicle,currTime,currSectionID);

		/******TEST for locating special position*********/
		/*if (needTestMsg &&
			(currSectionID == 949
				|| currSectionID == 386
				|| currSectionID == 967
				|| currSectionID == 395
				|| currSectionID == 935
				|| currSectionID == 395
				|| currSectionID == 935
				|| currSectionID == 404
				|| currSectionID == 967
				|| currSectionID == 406
				|| currSectionID == 414
				|| currSectionID == 986
				))
		{
			char msg[200];
			sprintf_s(msg, "Now Section %d PathLength=%f", currSectionID, optVehDataSet.totalTravelPathLength);
			AKIPrintString(msg);
		}*/
		/******TEST*********/


		// if this travel is ending, then select one vehicle as optimized vehilce in the entry sections
		if (useIterationOptimization && isExitSection(currSectionID) && haveOptimizedVeh.isExist == true)
		{
			haveOptimizedVeh.isExist = false;
			optimizedExperienceTimes++;
			optimizedVehIDSequence[optimizedExperienceTimes] = vehID;

			//char msg[150];
			//sprintf_s(msg, "Now MOBIL vehicle ID is %d, exited", vehID);
			//AKIPrintString(msg);
		}
	}


	recordAllVehicleODInfo(vehicle);



	// OUTPUT data at the end time of simulation, (sec) 4 hours equals 14400 seconds
	if ((!haveOutPutFunctionsRan) && currTime > 14399)
	{
		outPutOptVehData();

		//outPutOptVehPerformance();

	    outPutOptVehTrajectoryDataSet();

		outPutOptVehLaneChangingDetials();

		//outPutAllVehicleODInfo();

		outPutControlGroupVehiclesODInfo();
		
		//outPutQTable();


		haveOutPutFunctionsRan = true;
	}




	/******* Heuristic Lane Changing Model by Georgia ***********/
	if (useHeuristicLaneChangeModel)
	{

		//// in the on-ramp, define a behaviour, otherwise NO LANE-CHANGINGS
		if ((numSect == 404) || (numSect == 423))
		{
			if (vehicle->IsLaneOnRamp(0))
			{
				double pos = vehicle->getPosition(0);

				vehID;

				isOnRamp = true;


				currSpeed;
				direction = -1;
				targetLane = currLane - direction;

				double XPosTargetlane = vehicle->getPositionInTargetlane(vehicle->getPosition(0), targetLane);

				A2SimVehicle* pVehDw = NULL;
				A2SimVehicle *pVehUp = NULL;
				double ShiftUp = 0, ShiftDw = 0;
				vehicle->getUpDown(direction, XPosTargetlane, pVehUp, ShiftUp, pVehDw, ShiftDw);

				double avgSpeed, distUp, distDown;

				if (pVehUp != NULL)
				{
					avgSpeed = pVehUp->getSpeed(0);
					distUp = pos - pVehUp->getPosition(0);
				}
				else
				{
					avgSpeed = 120 / 3.6;
					distUp = 100;
				}

				if (pVehDw != NULL)
				{
					distDown = pVehDw->getPosition(0) - pos;
				}
				else
				{
					distDown = 100;
				}

				double diffSpeed = (avgSpeed - currSpeed);


				//	// thresholds as linear functions of the position within the section

				double thresCurrSpeed = (-pos / 3.8 + 88.25) / 3.6;


				if (pos > 320)
					thresCurrSpeed = 1.12207602339181;

				double thresDiffSpeed;


				thresDiffSpeed = (pos*2.5 / 5.8 - 30) / 3.6;

				if (pos > 300)
					thresDiffSpeed = 27.586206896551726;



				double thresDist = -0.1 * pos + 29.5;

				if (pos > 255)
					thresDist = 3;


				if (
					(pos > 30
						&& currSpeed > thresCurrSpeed
						&& diffSpeed <= thresDiffSpeed
						&& distUp > thresDist
						&& distDown > thresDist)
					||
					(pos > 50
						&& currSpeed <0.1
						&& diffSpeed <= thresDiffSpeed
						&& distUp >thresDist
						&& distDown > thresDist)
					)

				{
					direction = -1;
					targetLane = currLane - direction;

					double XPosTargetlane = vehicle->getPositionInTargetlane(vehicle->getPosition(0), targetLane);

					A2SimVehicle* pVehDw = NULL;
					A2SimVehicle *pVehUp = NULL;
					double ShiftUp = 0, ShiftDw = 0;
					vehicle->getUpDown(direction, XPosTargetlane, pVehUp, ShiftUp, pVehDw, ShiftDw);

					bool GapAcceptable = vehicle->isGapAcceptable(targetLane, XPosTargetlane, pVehUp, ShiftUp, pVehDw, ShiftDw);
					if (GapAcceptable) {

						vehicle->assignAcceptedGap(targetLane, XPosTargetlane, pVehUp, ShiftUp, pVehDw, ShiftDw, threadId);
						vehicle->applyLaneChanging(direction, threadId);

						return true;
					}

				}

				isOnRamp = false;

			}



			else if (currLane == 2)
			{

				currTime;
				if (currSpeed > 20 / 3.6)
					return false;

				double thres, thresLead;


				direction = -1;
				thres = 0.8;//12.1
				thresLead = 0.9;//14.8



				targetLane = currLane - direction;

				int vid = vehicle->getId();


				double pos = vehicle->getPosition(0);

				double XPosTargetlane = vehicle->getPositionInTargetlane(vehicle->getPosition(0), targetLane);//it cannot be computed that is why it crashes

				A2SimVehicle* pVehDw = NULL;
				A2SimVehicle *pVehUp = NULL;
				double ShiftUp = 0, ShiftDw = 0;
				vehicle->getUpDown(direction, XPosTargetlane, pVehUp, ShiftUp, pVehDw, ShiftDw);

				double nextSpeed = 0.0;
				int count = 0;

				if (pVehDw != NULL)
				{
					nextSpeed += pVehDw->getSpeed(0);
					count++;
				}
				if (pVehUp != NULL)
				{
					nextSpeed += pVehUp->getSpeed(0);
					count++;
				}

				count;
				if (count > 0)

				{
					nextSpeed = nextSpeed / (double)count;
				}


				nextSpeed;
				double temp;


				double speedLeader = vehicle->getLeader(temp)->getSpeed(0);

				if (nextSpeed > thres * currSpeed && nextSpeed > thresLead * speedLeader)//in case nextspeed is zero lanechanging is not applied

				{
					isSegmRamp = true;

					double XPosTargetlane = vehicle->getPositionInTargetlane(vehicle->getPosition(0), targetLane);

					A2SimVehicle* pVehDw = NULL;
					A2SimVehicle *pVehUp = NULL;
					double ShiftUp = 0, ShiftDw = 0;
					vehicle->getUpDown(direction, XPosTargetlane, pVehUp, ShiftUp, pVehDw, ShiftDw);

					bool GapAcceptable = vehicle->isGapAcceptable(targetLane, XPosTargetlane, pVehUp, ShiftUp, pVehDw, ShiftDw);
					if (GapAcceptable) {

						vehicle->assignAcceptedGap(targetLane, XPosTargetlane, pVehUp, ShiftUp, pVehDw, ShiftDw, threadId);
						vehicle->applyLaneChanging(direction, threadId);

						return true;
					}

					isSegmRamp = false;
				}
			}
			return true;
		}


		if (numSect == 395 || numSect == 386)//lane-drop section and upstream
		{
			double pos = vehicle->getPosition(0);

			vehID;

			isOnRamp = true;


			currSpeed;
			direction = 1;
			targetLane = currLane - direction;

			//double XPosTargetlane = vehicle->getPositionInTargetlane(vehicle->getPosition(0), targetLane);
			double XPosTargetlane = pos;
			A2SimVehicle* pVehDw = NULL;
			A2SimVehicle *pVehUp = NULL;
			double ShiftUp = 0, ShiftDw = 0;
			vehicle->getUpDown(direction, pos, pVehUp, ShiftUp, pVehDw, ShiftDw);

			double avgSpeed, distUp, distDown;

			if (pVehUp != NULL)
			{
				avgSpeed = pVehUp->getSpeed(0);
				distUp = pos - pVehUp->getPosition(0);
			}
			else
			{
				avgSpeed = 120 / 3.6;
				distUp = 100;
			}

			if (pVehDw != NULL)
			{
				distDown = pVehDw->getPosition(0) - pos;
			}
			else
			{
				distDown = 100;
			}

			double diffSpeed = (avgSpeed - currSpeed);


			//	// thresholds as linear functions of the position within the section
			double thresCurrSpeed;
			double thresDiffSpeed;
			double thresDist;

			if (numSect == 395)
			{
				if (currLane == 3)
				{
					thresCurrSpeed = (-pos / 1 + 58.25) / 3.6;


					if (pos > 50)
						thresCurrSpeed = 2.29166666666667;




					thresDiffSpeed = (pos*2.5 / 5.8 - 3) / 3.6;

					if (pos > 50)
						thresDiffSpeed = 5.15325670498084;



					thresDist = -0.1 * pos + 29.5;

					if (pos > 70)
						thresDist = 1;

					if ((pos > 5 && currSpeed > thresCurrSpeed && diffSpeed <= thresDiffSpeed && distUp > thresDist && distDown > thresDist) || (pos > 5 && currSpeed <0.1 && diffSpeed <= thresDiffSpeed && distUp >thresDist && distDown > thresDist))

					{
						direction = 1;
						targetLane = currLane - direction;


						double pos = vehicle->getPosition(0);
						//double XPosTargetlane = vehicle->getPositionInTargetlane(vehicle->getPosition(0), targetLane);
						double XPosTargetlane = pos;


						A2SimVehicle* pVehDw = NULL;
						A2SimVehicle *pVehUp = NULL;
						double ShiftUp = 0, ShiftDw = 0;
						vehicle->getUpDown(direction, pos, pVehUp, ShiftUp, pVehDw, ShiftDw);

						bool GapAcceptable = vehicle->isGapAcceptable(targetLane, XPosTargetlane, pVehUp, ShiftUp, pVehDw, ShiftDw);
						if (GapAcceptable) {

							vehicle->assignAcceptedGap(targetLane, XPosTargetlane, pVehUp, ShiftUp, pVehDw, ShiftDw, threadId);
							vehicle->applyLaneChanging(direction, threadId);

							return true;
						}
					}
				}
			}
			else
			{
				if (currLane == 4)
				{
					thresCurrSpeed = (-pos / 2.3 + 100.25) / 3.6;


					if (pos > 200)
						thresCurrSpeed = 3.69263285024155;



					thresDiffSpeed = (pos*2.5 / 4.8 - 2) / 3.6;

					if (pos > 200)
						thresDiffSpeed = 28.3796296296296;



					thresDist = -0.1 * pos + 29.5;

					if (pos > 200)
						thresDist = 5;

					if ((pos > 5 && currSpeed > thresCurrSpeed && diffSpeed <= thresDiffSpeed && distUp > thresDist && distDown > thresDist) || (pos > 5 && currSpeed <0.1 && diffSpeed <= thresDiffSpeed && distUp >thresDist && distDown > thresDist))

					{
						direction = 1;
						targetLane = currLane - direction;


						double pos = vehicle->getPosition(0);
						//double XPosTargetlane = vehicle->getPositionInTargetlane(vehicle->getPosition(0), targetLane);
						double XPosTargetlane = pos;


						A2SimVehicle* pVehDw = NULL;
						A2SimVehicle *pVehUp = NULL;
						double ShiftUp = 0, ShiftDw = 0;
						vehicle->getUpDown(direction, pos, pVehUp, ShiftUp, pVehDw, ShiftDw);

						bool GapAcceptable = vehicle->isGapAcceptable(targetLane, XPosTargetlane, pVehUp, ShiftUp, pVehDw, ShiftDw);
						if (GapAcceptable) {

							vehicle->assignAcceptedGap(targetLane, XPosTargetlane, pVehUp, ShiftUp, pVehDw, ShiftDw, threadId);
							vehicle->applyLaneChanging(direction, threadId);

							return true;
						}
					}
				}
			}

			isOnRamp = false;

		}
		if (numSect == 395)
		{
			if (currLane == 2)
			{
				if (currSpeed > 20 / 3.6)
					return false;

				direction = 1;
				targetLane = currLane - direction;

				double XPosTargetlane = vehicle->getPositionInTargetlane(vehicle->getPosition(0), targetLane);

				A2SimVehicle* pVehDw = NULL;
				A2SimVehicle *pVehUp = NULL;
				double ShiftUp = 0, ShiftDw = 0;
				vehicle->getUpDown(direction, XPosTargetlane, pVehUp, ShiftUp, pVehDw, ShiftDw);

				double nextSpeed = 0.0;
				int count = 0;

				if (pVehDw != NULL)
				{
					nextSpeed += pVehDw->getSpeed(0);
					count++;
				}
				if (pVehUp != NULL)
				{
					nextSpeed += pVehUp->getSpeed(0);
					count++;
				}

				if (count > 0)
					nextSpeed = nextSpeed / (double)count;

				double temp;
				double speedLeader = vehicle->getLeader(temp)->getSpeed(0);

				double thres, thresLead;


				thres = 0.8;
				thresLead = 1.0;


				if (nextSpeed > thres * currSpeed && nextSpeed > thresLead * speedLeader)
				{
					isSegmRamp = true;

					double XPosTargetlane = vehicle->getPositionInTargetlane(vehicle->getPosition(0), targetLane);
					//Define whether a lane changing attempt is made or not

					A2SimVehicle* pVehDw = NULL;
					A2SimVehicle *pVehUp = NULL;
					double ShiftUp = 0, ShiftDw = 0;
					vehicle->getUpDown(direction, XPosTargetlane, pVehUp, ShiftUp, pVehDw, ShiftDw);

					bool GapAcceptable = vehicle->isGapAcceptable(targetLane, XPosTargetlane, pVehUp, ShiftUp, pVehDw, ShiftDw);
					if (GapAcceptable)
					{
						vehicle->assignAcceptedGap(targetLane, XPosTargetlane, (const simVehicleParticular*)pVehUp, ShiftUp, (const simVehicleParticular*)pVehDw, ShiftDw, threadId);
						vehicle->applyLaneChanging(direction, threadId);
					}

					isSegmRamp = false;
				}

				return true;
			}
		}
		if (numSect == 386)
		{
			if (currLane == 3)
			{
				if (currSpeed > 20 / 3.6)
					return false;

				direction = 1;
				targetLane = currLane - direction;

				double XPosTargetlane = vehicle->getPositionInTargetlane(vehicle->getPosition(0), targetLane);

				A2SimVehicle* pVehDw = NULL;
				A2SimVehicle *pVehUp = NULL;
				double ShiftUp = 0, ShiftDw = 0;
				vehicle->getUpDown(direction, XPosTargetlane, pVehUp, ShiftUp, pVehDw, ShiftDw);

				double nextSpeed = 0.0;
				int count = 0;

				if (pVehDw != NULL)
				{
					nextSpeed += pVehDw->getSpeed(0);
					count++;
				}
				if (pVehUp != NULL)
				{
					nextSpeed += pVehUp->getSpeed(0);
					count++;
				}

				if (count > 0)
					nextSpeed = nextSpeed / (double)count;

				double temp;
				double speedLeader = vehicle->getLeader(temp)->getSpeed(0);

				double thres, thresLead;


				thres = 1.1;
				thresLead = 1.3;


				if (nextSpeed > thres * currSpeed && nextSpeed > thresLead * speedLeader)
				{
					isSegmRamp = true;

					double XPosTargetlane = vehicle->getPositionInTargetlane(vehicle->getPosition(0), targetLane);
					//Define whether a lane changing attempt is made or not

					A2SimVehicle* pVehDw = NULL;
					A2SimVehicle *pVehUp = NULL;
					double ShiftUp = 0, ShiftDw = 0;
					vehicle->getUpDown(direction, XPosTargetlane, pVehUp, ShiftUp, pVehDw, ShiftDw);

					bool GapAcceptable = vehicle->isGapAcceptable(targetLane, XPosTargetlane, pVehUp, ShiftUp, pVehDw, ShiftDw);
					if (GapAcceptable)
					{
						vehicle->assignAcceptedGap(targetLane, XPosTargetlane, (const simVehicleParticular*)pVehUp, ShiftUp, (const simVehicleParticular*)pVehDw, ShiftDw, threadId);
						vehicle->applyLaneChanging(direction, threadId);
					}

					isSegmRamp = false;
				}

				return true;
			}
		}

	}
	/************ otherwise, normal lane changing *************/
	if (vehicle_particular_Temp->getIsSmartVehicle() && useMOBIL)
	{



		//optimizedThreshold = parameterSet[currSectionID];
		double optimizedPolitenessFactor = parameterSet[363];
		double optimizedThreshold = parameterSet[364];

		direction = MOBILDirection(vehicle, optimizedPolitenessFactor, optimizedThreshold);

		// control OD, force the optimized vehicle to experience the whole freeway, it cannot exit the freeway though off-ramp
		/*if ((currSectionID == 386 && currLane == 2 && direction == 1)
			|| (currSectionID == 406 && currLane == 2 && direction == 1))
		{
			direction = 0;
		}*/


		//direction = MOBILDirection(vehicle, 1, 0.5);
		//direction = MOBILDirection(vehicle, 1, 0);

		//double farSightThreshold = getModifiedThreshold(vehicle, currLane - direction);
		//int farSightDirection = MOBILDirection(vehicle, 1, 0.5+farSightThreshold);

		// block Turn To OffRamp
		//if ((currSectionID == 386 || currSectionID == 406) && currLane == 2 && direction == 1)
		//	direction = 0;

		vehicle->applyLaneChanging(direction, threadId); // this function includes the gap acceptance. so if direction is not acceptable, aimsun will consider it
		return true;
	}
	else if (vehID == optimiazedVehID || vehicle_particular_Temp->getIsSmartVehicle() && useQLearning)
	{
		unsigned int stateID = getStateID_QLearning(vehicle);

		updateQTable(q_LearningParameters.stateID_last[vehID], q_LearningParameters.action_last[vehID], stateID);

		int action = getQLearningDecisionAction(vehicle);
		direction = convertQActionToDirection(action);

		/************ TEST OUTPUT ************/
		if (needTestMsg&&direction!=0)
		{

			string laneChangeTestName = "TEST_LCacceleration.txt";
			string laneChangeTestNamePath;
			laneChangeTestNamePath = DATAPATH + laneChangeTestName;
			ofstream laneChangeTest;
			if (!hasOutPutFilesInitiated)
			{
				laneChangeTest.open(laneChangeTestNamePath, ios::trunc);
				laneChangeTest << "TIME" << currTime << endl;
				laneChangeTest.close();
				hasOutPutFilesInitiated = true;
			}

			int stateCode[10] = { 0 };
			for (int i = 0; i <= 8; ++i)
			{
				stateCode[i] = stateID / int(pow(10, i)) % 10;
			}

			float currentDirectionQuality = q_LearningParameters.q_table[stateCode[8]][stateCode[7]][stateCode[6]][stateCode[5]][stateCode[4]][stateCode[3]][stateCode[2]][stateCode[1]][stateCode[0]][0];
			float leftDirectionqQuality = q_LearningParameters.q_table[stateCode[8]][stateCode[7]][stateCode[6]][stateCode[5]][stateCode[4]][stateCode[3]][stateCode[2]][stateCode[1]][stateCode[0]][1];
			float rightDirectionqQuality = q_LearningParameters.q_table[stateCode[8]][stateCode[7]][stateCode[6]][stateCode[5]][stateCode[4]][stateCode[3]][stateCode[2]][stateCode[1]][stateCode[0]][2];


			laneChangeTest.open(laneChangeTestNamePath, ios::app);


			laneChangeTest
				<< "TIME" << currTime << "\n"
				<< "StateID"<<stateID<<"\n"
				<< "currentDirectionQuality = " << currentDirectionQuality << "\n"
				<< "leftDirectionqQuality = " << leftDirectionqQuality << "\n"
				<< "rightDirectionqQuality = " << rightDirectionqQuality << "\n"
				<< "ID=" << vehID << "\n"
				<< "CurrentSection=" << numSect << "\n"
				<< "curLane=" << currLane << "\n"
				<< "maxLanes=" << maxLanes << "\n"
				<< "current_speed=" << vehicle->getSpeed(vehicle->isUpdated()) << "\n"
				<< "direction=" << direction << "\n\n" << endl;


			laneChangeTest.close();

		}
		/************ TEST OUTPUT ************/


		vehicle->applyLaneChanging(direction, threadId);

		q_LearningParameters.action_last[vehID] = action;
		q_LearningParameters.stateID_last[vehID] = stateID;


		return true;
	}
	else
	{
		// use default Gipps lane change model
		return false;

	}

}


bool behavioralModelParticular::avoidCollision(A2SimVehicle *vehicle, A2SimVehicle *vehiclePre, double ShiftPre)
{
	return false;
}

bool behavioralModelParticular::isVehicleGivingWay(A2SimVehicle *vehicleGiveWay, A2SimVehicle *vehiclePrio, yieldInfo *givewayInfo, int &Yield)
{
	return false;
}

double behavioralModelParticular::computeMinimumGap(A2SimVehicle *vehicleUp, A2SimVehicle *vehicleDown, double Xup, double Vup, double Xdw, double Vdw, double Gap, bool ImprudentCase, bool VehicleIspVehDw)
{
	double GapMin = 0;
	if (useHeuristicLaneChangeModel)
	{
		if (isOnRamp)
			return 0.0;

		if (isSegmRamp)
			return 10.0;
	}
	if (useIDM)
	{
		GapMin = getIDMDesiredGap((simVehicleParticular*)vehicleUp, (simVehicleParticular*)vehicleDown, Vup, Vdw, Gap);
	}
	else
	{
		GapMin = getGippsMinimumGap((simVehicleParticular*)vehicleUp, (simVehicleParticular*)vehicleDown, Xup, Vup, Xdw, Vdw, Gap, ImprudentCase, VehicleIspVehDw);
	}
	return GapMin;
}

// for IDM implementation
double behavioralModelParticular::computeCarFollowingAccelerationComponentSpeed(A2SimVehicle *vehicle, double VelActual, double VelDeseada, double RestoCiclo)
{
	double VelPropia = 0;

	//if not inflow section, or it will influence the calculation of demand
	if ((useIDM)
		&& getEntrySectionSequence(vehicle->getIdCurrentSection()) == -1
		//&& vehicle->getIdCurrentSection() != 556// only used in capacity test work
		)
	{
		VelPropia = getIDMAccelerationSpeed((simVehicleParticular*)vehicle, VelActual, VelDeseada, RestoCiclo);
	}
	else
	{
		VelPropia = getGippsAccelerationSpeed((simVehicleParticular*)vehicle, VelActual, VelDeseada, RestoCiclo);
	}
	return VelPropia;
}

double behavioralModelParticular::computeCarFollowingDecelerationComponentSpeed(A2SimVehicle *vehicle, double Shift, A2SimVehicle *vehicleLeader, double ShiftLeader, bool controlDecelMax, bool aside, int time)
{
	double VelImpuesta = 0;
	if ((useIDM)
		&& getEntrySectionSequence(vehicle->getIdCurrentSection()) == -1
		) {
		VelImpuesta = getIDMDecelerationSpeed((simVehicleParticular*)vehicle, Shift, (simVehicleParticular*)vehicleLeader, ShiftLeader);
	}
	else
	{
		VelImpuesta = getGippsDecelerationSpeed((simVehicleParticular*)vehicle, Shift, (simVehicleParticular*)vehicleLeader, ShiftLeader, controlDecelMax, aside, time);
	}
	return VelImpuesta;
}

//Gipps
double behavioralModelParticular::getGippsAccelerationSpeed(simVehicleParticular *vehicle, double VelActual, double VelDeseada, double RestoCiclo)
{
	// Calcula l'acceleracio tenint en compte les pendents
	double X = VelActual / VelDeseada;
	int numSect = vehicle->getIdCurrentSection();
	double a = vehicle->getAcceleration();
	int id = vehicle->getId();
	double VelPropia = min(VelDeseada, VelActual + 2.5 * vehicle->getAcceleration() * RestoCiclo * (1.0 - X)* sqrt(0.025 + X));
	if (VelPropia < VelActual)
	{
		VelPropia = max(VelPropia, max(0., VelActual + (0.5 * vehicle->getDeceleration() * RestoCiclo)));
	}
	return VelPropia;
}

double behavioralModelParticular::getGippsDecelerationSpeed(simVehicleParticular *vehicle, double Shift, simVehicleParticular *leader, double ShiftLeader, bool controlDecelMax, bool aside, int time)
{
	int id = vehicle->getId();
	int idl = leader->getId();

	double PosAnterior, VelAnterior, PosAnteriorLeader, VelAnteriorLeader;
	double GapAnterior = vehicle->getGap(Shift, leader, ShiftLeader, PosAnterior, VelAnterior, PosAnteriorLeader, VelAnteriorLeader, time);
	double RT = vehicle->getReactionTime();
	double DecelEstimada = 0;
	if (VelAnteriorLeader > Tolerancia)
	{
		DecelEstimada = vehicle->getEstimationOfLeadersDeceleration(leader, VelAnteriorLeader);
	}
	double bn = vehicle->getDeceleration();
	double bnTau = bn*RT;
	double VelImpuesta = bnTau;
	double factorSqrt = 0;
	if (VelAnteriorLeader < Tolerancia)
	{
		factorSqrt = (bnTau* bnTau) - vehicle->getDeceleration() * (2.0 * GapAnterior - (VelAnterior * RT));
	}
	else if (VelAnteriorLeader < DBL_MAX)
	{
		factorSqrt = (bnTau * bnTau) - vehicle->getDeceleration() * (2.0 * GapAnterior - (VelAnterior * RT) - ((VelAnteriorLeader * VelAnteriorLeader) / DecelEstimada));
	}
	else {
		VelImpuesta = DBL_MAX;
	}
	if (factorSqrt > 0)
	{
		VelImpuesta = bnTau + (double)sqrt(factorSqrt);
	}

	if (aside)
	{
		//SpeedAnterior imposed by Car-Following on side lane > Tolerancia to avoid RT at stop
		int id = vehicle->getId();
		int idl = leader->getId();
		double GapMin = getGippsMinimumGap(vehicle, leader, PosAnterior, VelAnterior, PosAnteriorLeader, VelAnteriorLeader, GapAnterior, false, false);
		GapMin = GapAnterior - GapMin;
		if (GapMin < 0)
		{
			double Distance2Obstacle = DBL_MAX;
			if (vehicle->getObstacleType() != eNone)
			{
				Distance2Obstacle = vehicle->getDistance2Obstacle() / abs(vehicle->getNbLaneChanges2ReachNextValidLane());
				Distance2Obstacle = max(0., Distance2Obstacle - max(VelAnteriorLeader*RT, leader->getLength()));
			}
			double minimo = Distance2Obstacle;
			double AdaptationDistance = max(vehicle->getFreeFlowSpeed()*RT, vehicle->getLength());
			if (vehicle->getObstacleType() == eOnRamp)
			{
				minimo = min(minimo, 3 * AdaptationDistance);
			}
			else
			{
				minimo = min(minimo, AdaptationDistance);
			}
			double maximo = max(VelAnteriorLeader, Tolerancia);
			double expParam = 0.5*(1. - VelAnterior / maximo*(1 - (GapMin) / minimo));
			double expValue = (float)exp(expParam);//la función exp en 32/64 bits retorna el mismo valor usándola de este modo
			VelImpuesta = VelAnterior*expValue;
		}
	}
	if (controlDecelMax)
	{
		double VelMin = 0;
		if (aside)
		{
			VelMin = max(0., VelAnterior + vehicle->getDeceleration()*RT);
		}
		else
		{
			VelMin = max(0., VelAnterior + vehicle->getDecelerationMax()*RT);
		}
		if (VelImpuesta < VelMin)
		{
			VelImpuesta = VelMin;
		}
	}
	return VelImpuesta;
}

double behavioralModelParticular::getGippsMinimumGap(simVehicleParticular* pVehUp, simVehicleParticular* pVehDw, double Xup, double Vup, double Xdw, double Vdw, double Gap, bool ImprudentCase, bool VehicleIspVehDw)
{
	double DecelFactorUp = 1;
	if (VehicleIspVehDw)
	{
		DecelFactorUp = pVehDw->getDecelerationVariationFactor(ImprudentCase);
	}
	else
	{
		DecelFactorUp = pVehUp->getDecelerationVariationFactor(ImprudentCase);
	}
	double tau = pVehUp->getReactionTime();
	double GapMin = 0;
	if (Vdw < 0.01)
	{
		GapMin = max(0., 0.5*Vup*tau + max(0., -Vup*Vup / (2 * pVehUp->getDeceleration()) + DecelFactorUp*(1. - 0.5*DecelFactorUp)*pVehUp->getDeceleration()*tau*tau + (1 - DecelFactorUp)*Vup*tau));
		if (DecelFactorUp > -Vup / (pVehUp->getDeceleration())*tau)
		{
			GapMin = max(0., 0.5*Vup*tau);
		}
	}
	else
	{
		double DecelEstimada = pVehUp->getEstimationOfLeadersDeceleration(pVehDw, Vdw);
		GapMin = max(0., (Vdw*Vdw) / (2 * DecelEstimada) + 0.5*Vup*tau + max(0., -Vup*Vup / (2 * pVehUp->getDeceleration()) + DecelFactorUp*(1. - 0.5*DecelFactorUp)*pVehUp->getDeceleration()*tau*tau + (1 - DecelFactorUp)*Vup*tau));
		if (DecelFactorUp > -Vup / (pVehUp->getDeceleration())*tau)
		{
			GapMin = max(0., (Vdw*Vdw) / (2 * DecelEstimada) + 0.5*Vup*tau);
		}
	}
	return GapMin;
}

//IDM

//this method only used in computeCarFollowingAccelerationComponentSpeed() 
double behavioralModelParticular::getIDMAccelerationSpeed(simVehicleParticular *vehicle, double VelActual, double VelDeseada, double RestoCiclo)
{
	double X = VelActual / VelDeseada;
	double a = vehicle->getAcceleration();
	double acceleration = max(vehicle->getDeceleration(), vehicle->getAcceleration()*(1. - pow(X, 4)));
	double speed = max(0., VelActual + acceleration * vehicle->getReactionTime());
	return speed;
}

//this method only used in computeCarFollowingDecelerationComponentSpeed()
double behavioralModelParticular::getIDMDecelerationSpeed(simVehicleParticular *vehicle, double Shift, simVehicleParticular *leader, double ShiftLeader)
{
	bool useGeorgia = true;
	if (useGeorgia)
	{
		int vehID = vehicle->getId();



		//// Test for understanding "Shift'
		//A2SimVehicle *pVehCurUp = NULL;
		//double temp_shift_to_cur_follower = 0;
		//pVehCurUp = vehicle->getFollower(temp_shift_to_cur_follower);
		//if (pVehCurUp != NULL)
		//{
		//	A2SimVehicle * temp_up = NULL;
		//	double temp_shift_up = 0;
		//	A2SimVehicle * temp_dw = NULL;
		//	double temp_shift_dw = 0;
		//	pVehCurUp->getUpDown(0, pVehCurUp->getPosition(pVehCurUp->isUpdated()), temp_up, temp_shift_up, temp_dw, temp_shift_dw);
		//	char msg[200];
		//	sprintf_s(msg, "ID is %d, postion=%f, shift=%f,temp_shift_dw=%f,shift", vehicle->getId(), vehicle->getPosition(0), Shift, temp_shift_dw);
		//	AKIPrintString(msg);
		//}
		//A2SimVehicle *pVehCurDw = NULL;
		//double temp_shift_to_cur_leader = 0;
		//pVehCurUp = vehicle->getFollower(temp_shift_to_cur_leader);
		//if (pVehCurDw != NULL)
		//{
		//	A2SimVehicle * temp_up = NULL;
		//	double temp_shift_up = 0;
		//	A2SimVehicle * temp_dw = NULL;
		//	double temp_shift_dw = 0;
		//	pVehCurDw->getUpDown(0, pVehCurDw->getPosition(pVehCurDw->isUpdated()), temp_up, temp_shift_up, temp_dw, temp_shift_dw);
		//	char msg[200];
		//	sprintf_s(msg, "ID is %d, postion=%f, shift=%f,temp_shift_up=%f,shift", vehicle->getId(),vehicle->getPosition(0), Shift, temp_shift_up);
		//	AKIPrintString(msg);
		//}

		/* START---Test for understanding "getgap" */

	/*	double currTime = AKIGetCurrentSimulationTime();
		if (Shift>0.2&&vehID>1)
		{





			A2SimVehicle *pVehLeftUp = NULL;
			A2SimVehicle *pVehRightUp = NULL;
			double shiftLeftUp = 0;
			double shiftRightUp = 0;

			A2SimVehicle *pVehLeftDown = NULL;
			A2SimVehicle *pVehRightDown = NULL;
			double shiftLeftDown = 0;
			double shiftRightDown = 0;

			double currentSpeed, pos_cur;
			double  speed_left_leader, pos_left_leader;
			double  speed_right_leader, pos_right_leader;




			double currPosition = vehicle->getPosition(0);
			double shift = 0;


			vehicle->getUpDown(-1, vehicle->getPosition(vehicle->isUpdated()), pVehLeftUp, shiftLeftUp, pVehLeftDown, shiftLeftDown);
			vehicle->getUpDown(1, vehicle->getPosition(vehicle->isUpdated()), pVehRightUp, shiftRightUp, pVehRightDown, shiftRightDown);



			double gapToLeftLeader = vehicle->getGap(Shift, pVehLeftDown, shiftLeftDown, pos_cur, currentSpeed, pos_left_leader, speed_left_leader);
			double gapToRightLeader = vehicle->getGap(Shift, pVehRightDown, shiftRightDown, pos_cur, currentSpeed, pos_right_leader, speed_right_leader);

			if (pVehLeftDown != NULL&& pVehRightDown != NULL)
			{
				char msg[500];
				sprintf_s(msg,
					"Time=%f,ID is %d,pos_cur=%f,shift=%f,LeftLeader=%d,posLL=%f,shiftLeftDown=%f,RightLeader=%d,posRL=%f,shiftRightDown=%f,GapToLL=%f,GapToRL=%f",
					currTime,
					vehicle->getId(),
					pos_cur,
					Shift,
					pVehLeftDown->getId(),
					pos_left_leader,
					shiftLeftDown,
					pVehRightDown->getId(),
					pos_right_leader,
					shiftRightDown,
					gapToLeftLeader,
					gapToRightLeader
				);
				AKIPrintString(msg);
			}
		}*/
		/* END Test for understanding "getgap" */







		double a = vehicle->getAcceleration();
		double b = vehicle->getDeceleration();
		double VelAnterior, PosAnterior, VelAnteriorLeader, PosAnteriorLeader;


		double GapAnterior = vehicle->getGap(Shift, leader, ShiftLeader, PosAnterior, VelAnterior, PosAnteriorLeader, VelAnteriorLeader);
		double diff = VelAnterior - VelAnteriorLeader;


		simVehicleParticular *newVehicle = vehicle;

		double DesiredGap = getIDMDesiredGap(vehicle, leader, VelAnterior, VelAnteriorLeader, GapAnterior);

		double X = VelAnterior / vehicle->getFreeFlowSpeed();
		double bkin = (VelAnterior*VelAnterior) / (2 * GapAnterior);
		double acceleration;

		acceleration = max(b, a*(1 - pow(X, 4) - (DesiredGap / GapAnterior)*(DesiredGap / GapAnterior)));

		double speed = max(0., VelAnterior + acceleration * vehicle->getReactionTime());
		return speed;

	}
	double acceleration = 0;
	double currentSpeed = 0;

	currentSpeed = vehicle->getSpeed(vehicle->isUpdated());
	acceleration = get_IDM_acceleration(vehicle, leader);

	double speed = max(0., currentSpeed + acceleration * vehicle->getReactionTime());
	return speed;
}

double behavioralModelParticular::get_IDM_acceleration(simVehicleParticular*vehicle, simVehicleParticular*leader)
{
	if (vehicle == NULL) return 0;
	double acceleration = 0;
	double shift_temp = 0;
	double  shiftLeader = 0;

	double a = vehicle->getAcceleration();
	double b = vehicle->getDeceleration();
	double currentSpeed, pos_cur, speed_leader, pos_leader;
	double gapToLeader = vehicle->getGap(shift_temp, leader, shiftLeader, pos_cur, currentSpeed, pos_leader, speed_leader);
	currentSpeed = vehicle->getSpeed(vehicle->isUpdated());
	double desiredGap = 0;
	desiredGap = getIDMDesiredGap(vehicle, leader, currentSpeed, speed_leader, gapToLeader);
	double X = vehicle->getSpeed(vehicle->isUpdated()) / vehicle->getFreeFlowSpeed();
	if (leader != NULL) acceleration = max(vehicle->getDecelerationMax(), a*(1 - pow(X, 4) - (desiredGap / gapToLeader)*(desiredGap / gapToLeader)));
	if (leader == NULL) acceleration = max(vehicle->getDecelerationMax(), a*(1 - pow(X, 4)));



	// for Asymmetric MOBIL
	A2SimVehicle *pVehLeftDw = NULL;
	A2SimVehicle *pVehLeftUp = NULL;
	double shiftUpLeft = 0, shiftDwLeft = 0;
	double speed_leader_left = 0;
	double acceleration_europe = 0;
	double ac = 0, ac_tilde = 0;

	int curLane = vehicle->getIdCurrentLane();
	int numSect = vehicle->getIdCurrentSection();
	int maxLanes = vehicle->getNumberOfLanesInCurrentSection();
	//double speed_average_lane = get_speed_average_bylane(vehicle);
	if (useAsymmetric((simVehicleParticular*)vehicle) && (maxLanes - curLane == 1))// only for middle lane
	{
		//get left lane follower and leader
		vehicle->getUpDown(-1, vehicle->getPosition(vehicle->isUpdated()), pVehLeftUp, shiftUpLeft, pVehLeftDw, shiftDwLeft);

		ac = acceleration;

		if (pVehLeftDw != NULL)
		{
			speed_leader_left = pVehLeftDw->getSpeed(pVehLeftDw->isUpdated());
			desiredGap = getIDMDesiredGap(vehicle, (simVehicleParticular*)pVehLeftUp, currentSpeed, speed_leader_left, shiftDwLeft);
			ac_tilde = max(b, a*(1 - pow(X, 4) - (desiredGap / shiftDwLeft)*(desiredGap / shiftDwLeft)));
		}
		else
		{
			ac_tilde = max(b, a*(1 - pow(X, 4)));
		}



		if (currentSpeed > speed_leader_left && speed_leader_left > SPEED_CRITICAL)
			acceleration_europe = min(ac, ac_tilde);
		else
			acceleration_europe = ac;
		return acceleration_europe;
	}


	return acceleration;
}
double behavioralModelParticular::get_IDM_acceleration(A2SimVehicle*vehicle_p, A2SimVehicle*leader_p)
{

	simVehicleParticular *vehicle = (simVehicleParticular*)vehicle_p;
	simVehicleParticular *leader = (simVehicleParticular*)leader_p;

	if (vehicle_p == NULL) return 0;
	double acceleration = 0;
	double shift_temp = 0;
	double  shiftLeader = 0;

	double a = vehicle->getAcceleration();
	double b = vehicle->getDeceleration();
	double currentSpeed, pos_cur, speed_leader, pos_leader;




	double gapToLeader = vehicle->getGap(shift_temp, leader, shiftLeader, pos_cur, currentSpeed, pos_leader, speed_leader);




	currentSpeed = vehicle->getSpeed(vehicle->isUpdated());
	double desiredGap = 0;
	desiredGap = getIDMDesiredGap(vehicle, leader, currentSpeed, speed_leader, gapToLeader);
	double X = vehicle->getSpeed(vehicle->isUpdated()) / vehicle->getFreeFlowSpeed();
	if (leader != NULL) acceleration = max(vehicle->getDecelerationMax(), a*(1 - pow(X, 4) - (desiredGap / gapToLeader)*(desiredGap / gapToLeader)));
	if (leader == NULL) acceleration = max(vehicle->getDecelerationMax(), a*(1 - pow(X, 4)));


	// for Asymmetric MOBIL
	A2SimVehicle *pVehLeftDw = NULL;
	A2SimVehicle *pVehLeftUp = NULL;
	double shiftUpLeft = 0, shiftDwLeft = 0;
	double speed_leader_left = 0;
	double acceleration_europe = 0;
	double ac = 0, ac_tilde = 0;

	int curLane = vehicle->getIdCurrentLane();
	int numSect = vehicle->getIdCurrentSection();
	int maxLanes = vehicle->getNumberOfLanesInCurrentSection();
	//double speed_average_lane = get_speed_average_bylane(vehicle);
	if (useAsymmetric((simVehicleParticular*)vehicle) && (maxLanes - curLane == 1))// only for middle lane
	{
		//get left lane follower and leader
		vehicle->getUpDown(-1, vehicle->getPosition(vehicle->isUpdated()), pVehLeftUp, shiftUpLeft, pVehLeftDw, shiftDwLeft);
		double gap_to_leader_left = vehicle->getGap(shift_temp, pVehLeftDw, shiftLeader, pos_cur, currentSpeed, pos_leader, speed_leader);

		ac = acceleration;

		if (pVehLeftDw != NULL)
		{
			speed_leader_left = pVehLeftDw->getSpeed(pVehLeftDw->isUpdated());
			desiredGap = getIDMDesiredGap(vehicle, (simVehicleParticular*)pVehLeftUp, currentSpeed, speed_leader_left, shiftDwLeft);
			ac_tilde = max(b, a*(1 - pow(X, 4) - (desiredGap / gap_to_leader_left)*(desiredGap / gap_to_leader_left)));
		}
		else
		{
			ac_tilde = max(b, a*(1 - pow(X, 4)));
		}



		if (currentSpeed > speed_leader_left && speed_leader_left > SPEED_CRITICAL)
			acceleration_europe = min(ac, ac_tilde);
		else
			acceleration_europe = ac;
		return acceleration_europe;
	}


	return acceleration;
}

//DesiredGap
double behavioralModelParticular::getIDMDesiredGap(simVehicleParticular* pVehCur, simVehicleParticular* pVehDw, double speed_current, double speed_leader, double gap_to_leader)
{
	if (pVehCur == NULL || pVehDw == NULL) return 0;

	bool isACC = pVehCur->getIsACC();

	double timeGap = pVehCur->getMinimumHeadway();
	if (isACC)
	{
		timeGap = pVehCur->getACCTimeGap();
	}

	// With regard to Optimized Lane Changing Strategy, longitudinal driving characteristics should be identical.


	double a = pVehCur->getAcceleration();
	double b = -pVehCur->getDeceleration();
	double desiredGap = pVehCur->getMinimumDistanceInterVeh() + max(0., speed_current*timeGap + speed_current*(speed_current - speed_leader) / (2 * sqrt(a*b))); // replace with 2 * sqrt
	return desiredGap;
}

double behavioralModelParticular::max(double a, double b)
{
	if (a > b) return a;
	else return b;
}

double behavioralModelParticular::min(double a, double b)
{
	if (a > b) return b;
	else return a;
}

double behavioralModelParticular::generateTimeGapOfACC(double lowLimit, double upLimit)
{
	double timeGap = AKIGetRandomNumber() * (upLimit - lowLimit) + lowLimit;
	return timeGap;
}

double behavioralModelParticular::generateGaussianNoise(double mu, double sigma)
{
	const double epsilon = std::numeric_limits<double>::min();
	const double two_pi = 2.0*3.14159265358979323846;

	static double z0, z1;
	static bool generate;
	generate = !generate;

	if (!generate)
		return z1 * sigma + mu;

	double u1, u2;
	do
	{
		u1 = rand() * (1.0 / RAND_MAX);
		u2 = rand() * (1.0 / RAND_MAX);
	} while (u1 <= epsilon);

	z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);
	z1 = sqrt(-2.0 * log(u1)) * sin(two_pi * u2);
	return z0 * sigma + mu;
}
//double behavioralModelParticular::get_speed_average_bylane(simVehicleParticular * vehicle)
//{
//
//	int lane_current = vehicle->getIdCurrentLane();
//	int section_current = vehicle->getIdCurrentSection();
//	
//	return vehicle->getLaneDensity(lane_current);
//
//}

//determining by speed, section and lane , a on-off controller: useAsymmetricMOBIL 
bool behavioralModelParticular::useAsymmetric(simVehicleParticular * vehicle)
{
	int curlane = vehicle->getIdCurrentLane();
	int maxlanes = vehicle->getNumberOfLanesInCurrentSection();
	int section_current = vehicle->getIdCurrentSection();
	double speed_current = vehicle->getSpeed(vehicle->isUpdated()) / 1000 * 3600;	// m/s -> km/h
	if (
		// global ON-OFF
		useAsymmetricMOBIL
		// lane
		&& (maxlanes - curlane == 1
			|| maxlanes - curlane == 0)
		// speed
		&& speed_current > SPEED_CRITICAL
		// ramp
		&& section_current != 396
		&& section_current != 401
		&& section_current != 415
		&& section_current != 582
		// inflow section
		&& section_current != 928
		&& section_current != 932
		)
		return true;
	else
		return false;
}

// 给出MOBIL 决定的换道方向 ，default: politeness factor p: 0.5, a_threshold: 1 m/s^2, bsafe: -5 m/s^2
int behavioralModelParticular::MOBILDirection(A2SimVehicle *vehicle, double politeness_factor, double a_threshold, double bsafe)
{


	double a_bias = a_threshold*1.1;	// Asymmetric MOBIL

	int curLane = vehicle->getIdCurrentLane();
	int numSect = vehicle->getIdCurrentSection();
	int maxLanes = vehicle->getNumberOfLanesInCurrentSection();


	int direction = 0;

	double profit_to_left = 0, profit_to_right = 0;



	double	ac_Left_LC = 0;			//ac*	//turn left
	double	ac_Left_NOLC = 0;		//ac	//turn left
	double	an_Left_LC = 0;			//an*	//turn left
	double	an_Left_NOLC = 0;		//an	//turn left
	double	ao_Left_LC = 0;			//ao*	//turn left
	double	ao_Left_NOLC = 0;		//ao	//turn left


	double	ac_Right_LC = 0;			//ac*	//turn Right
	double	ac_Right_NOLC = 0;			//ac	//turn Right
	double	an_Right_LC = 0;			//an*	//turn Right
	double	an_Right_NOLC = 0;			//an	//turn Right
	double	ao_Right_LC = 0;			//ao*	//turn Right
	double	ao_Right_NOLC = 0;			//ao	//turn right

	double XPosTargetlaneLeft = 0;
	double XPosTargetlaneRight = 0;

	A2SimVehicle *pVehLeftDw = NULL;
	A2SimVehicle *pVehLeftUp = NULL;
	A2SimVehicle *pVehRightDw = NULL;
	A2SimVehicle *pVehRightUp = NULL;
	A2SimVehicle *pVehCurUp = NULL;
	A2SimVehicle *pVehCurDw = NULL;

	double shiftCurUp = 0, shiftCurDw = 0;
	double ShiftUpLeft = 0, ShiftDwLeft = 0;
	double ShiftUpRight = 0, ShiftDwRight = 0;


	/*******************************************************************/
	/**********************MOBIL CRITERION*****************************/


	//get current lane follower and leader
	vehicle->getUpDown(0, vehicle->getPosition(0), pVehCurUp, shiftCurUp, pVehCurDw, shiftCurDw);

	//get left lane follower and leader
	vehicle->getUpDown(-1, vehicle->getPosition(0), pVehLeftUp, ShiftUpLeft, pVehLeftDw, ShiftDwLeft);

	//get right lane follower and leader
	vehicle->getUpDown(1, vehicle->getPosition(0), pVehRightUp, ShiftUpRight, pVehRightDw, ShiftDwRight);


	/**********************LEFT Calculation*********************************/
	if (curLane < maxLanes)
	{
		ac_Left_LC = get_IDM_acceleration(vehicle, pVehLeftDw);//ac*

		ac_Left_NOLC = get_IDM_acceleration(vehicle, pVehCurDw); // ac

		an_Left_LC = get_IDM_acceleration(pVehLeftUp, vehicle);	//an*

		an_Left_NOLC = get_IDM_acceleration(pVehLeftUp, pVehLeftDw);//an

		ao_Left_LC = get_IDM_acceleration(pVehCurUp, pVehCurDw); //a0*

		ao_Left_NOLC = get_IDM_acceleration(pVehCurUp, vehicle);  //a0
	}
	/**********************RIGHT Calculation*********************************/
	if (curLane > 1)
	{
		ac_Right_LC = get_IDM_acceleration(vehicle, pVehRightDw);  //ac*

		ac_Right_NOLC = get_IDM_acceleration(vehicle, pVehCurDw);    // ac

		an_Right_LC = get_IDM_acceleration(pVehRightUp, vehicle); //an*

		an_Right_NOLC = get_IDM_acceleration(pVehRightUp, pVehRightDw);  //an

		ao_Right_LC = get_IDM_acceleration(pVehCurUp, pVehCurDw); //a0*

		ao_Right_NOLC = get_IDM_acceleration(pVehCurUp, vehicle);  //a0
	}



	/****************Special Section:Lane drop  || on ramp******************/

	/*

	if (numSect == 396 && numLane == 3 && pVehCurDw == NULL)// lane drop
	{
		double	distToLaneDrop = 100 - vehicle->getPosition(0);

		simVehicleParticular* vehicleTemp = (simVehicleParticular*)vehicle;

		double timeGap = vehicleTemp->GetACCTimeGap();
		double speed = vehicle->getSpeed(0);
		double a = vehicle->getAcceleration();
		double b = -vehicle->getDeceleration();
		double desiredGap = vehicle->getMinimumDistanceInterVeh() + max(0., speed*timeGap + speed*(speed - 0) / (2 * sqrt(a*b))); // replace with 2 * sqrt
		double speedComponent = vehicle->getSpeed(0) / vehicle->getFreeFlowSpeed();
		double distanceComponent = distToLaneDrop / desiredGap;
		ac_Right_NOLC = a*(1 - pow(speedComponent, 4) - pow(distanceComponent, 2));
		ao_Left_LC = 0;
	}

	*/

	/************************ calculating lane change profits **************/
	// profits to LEFT
	if (useAsymmetric((simVehicleParticular*)vehicle) && (maxLanes - curLane == 1))
	{
		profit_to_left
			=
			ac_Left_LC - ac_Left_NOLC
			+
			politeness_factor*(an_Left_LC - an_Left_NOLC);//LEFT
	}
	else
	{
		profit_to_left
			=
			ac_Left_LC - ac_Left_NOLC
			+
			politeness_factor*(an_Left_LC - an_Left_NOLC + ao_Left_LC - ao_Left_NOLC);//LEFT
	}
	// profits to RIGHT
	if (useAsymmetric((simVehicleParticular*)vehicle) && (maxLanes - curLane == 0))
	{
		profit_to_right
			=
			ac_Right_LC - ac_Right_NOLC
			+
			politeness_factor*(ao_Right_LC - ao_Right_NOLC);//Right
	}
	else
	{
		profit_to_right
			=
			ac_Right_LC - ac_Right_NOLC
			+
			politeness_factor*(an_Right_LC - an_Right_NOLC + ao_Right_LC - ao_Right_NOLC);//Right
	}



	/************************Determining direction****************************/

	//非对称式，	只有在最左侧车道车辆考虑向右、第二左车道车辆考虑向左 两种情况才使用非对称换道

	if (useAsymmetric((simVehicleParticular*)vehicle))
	{
		if ((maxLanes - curLane == 1))	//asymmetric left
		{
			if ((profit_to_left > a_threshold + a_bias))
				direction = -1;
			else if (profit_to_right > a_threshold)//symmetric right
				direction = 1;
			else
				direction = 0;
		}
		else if (maxLanes - curLane == 0)	//asymmetric right
		{
			if (profit_to_right > a_threshold - a_bias)
				direction = 1;
			else
				direction = 0;
		}
		else
			direction = 0;
	}
	else //对称式
	{
		if ((profit_to_left > a_threshold) && (profit_to_right > a_threshold) && (profit_to_left > 0) && (profit_to_right > 0))
		{
			if (profit_to_right > profit_to_left)
				direction = 1;

			else if (profit_to_right < profit_to_left)
				direction = -1;
		}
		else if ((profit_to_left < a_threshold) && (profit_to_right > a_threshold))
			direction = 1;
		else if ((profit_to_left > a_threshold) && (profit_to_right < a_threshold))
			direction = -1;
		else if (a_threshold <= 0)// this situation can be used for force lane change
		{
			if (profit_to_left >= a_threshold)
				direction = -1;
			if (profit_to_right >= a_threshold)
				direction = 1;
		}
		else
			direction = 0;
	}

	//safe criterion
	if (an_Left_LC < bsafe && direction == -1) direction = 0;
	if (an_Right_LC < bsafe && direction == 1) direction = 0;
	//不能向入匝车道变道
	if ((numSect == 404 || numSect == 423) && (curLane == 2) && (direction == 1))
	{
		direction = 0;
	}










	/*****   record the ac_profit and the af_profit   *****/
	/*
	double ditance_to_left_leader = 0;
	double ditance_to_right_leader = 0;
	double ditance_to_cur_leader = 0;

	if (pVehLeftDw != NULL)
	{
		double shift_temp = 0;
		double shiftLeader_temp = 0;
		double pos_cur = vehicle->getPosition(0);
		double currentSpeed = 0;
		double pos_leader = 0;
		double speed_leader = 0;
		ditance_to_left_leader = vehicle->getGap(shift_temp, pVehLeftDw, shiftLeader_temp, pos_cur, currentSpeed, pos_leader, speed_leader);
	}

	if (pVehRightDw != NULL)
	{
		double shift_temp = 0;
		double shiftLeader_temp = 0;
		double pos_cur = vehicle->getPosition(0);
		double currentSpeed = 0;
		double pos_leader = 0;
		double speed_leader = 0;
		ditance_to_right_leader = vehicle->getGap(shift_temp, pVehRightDw, shiftLeader_temp, pos_cur, currentSpeed, pos_leader, speed_leader);
	}

	if (pVehCurDw != NULL)
	{
		double shift_temp = 0;
		double shiftLeader_temp = 0;

		double pos_cur = vehicle->getPosition(0);
		double currentSpeed = 0;
		double pos_leader = 0;
		double speed_leader = 0;
		ditance_to_cur_leader = vehicle->getGap(shift_temp, pVehCurDw, shiftLeader_temp, pos_cur, currentSpeed, pos_leader, speed_leader);
	}

	/*
	processLCProfitData(
		vehicle,
		ac_Right_LC - ac_Right_NOLC,
		ac_Left_LC - ac_Left_NOLC,
		ao_Right_LC - ao_Right_NOLC + an_Right_LC - an_Right_NOLC,
		ao_Left_LC - ao_Left_NOLC + an_Left_LC - an_Left_NOLC,
		curLane,
		maxLanes,

		ac_Left_LC,			//ac*	//turn left
		ac_Left_NOLC,		//ac	//turn left
		an_Left_LC,			//an*	//turn left
		an_Left_NOLC,		//an	//turn left
		ao_Left_LC,			//ao*	//turn left
		ao_Left_NOLC,		//ao	//turn left


		ac_Right_LC,			//ac*	//turn Right
		ac_Right_NOLC,			//ac	//turn Right
		an_Right_LC,			//an*	//turn Right
		an_Right_NOLC,			//an	//turn Right
		ao_Right_LC,			//ao*	//turn Right
		ao_Right_NOLC,			//ao	//turn right
		ditance_to_left_leader,
		ditance_to_right_leader
	);
	*/

	/************ TEST OUTPUT ************/
	/*
	if (needTestMsg)
	{
		double currentTime = AKIGetCurrentSimulationTime();
		int id_veh = vehicle->getId();//vehicle id
		string laneChangeTestName = "TEST_LCacceleration.txt";
		string laneChangeTestNamePath;
		laneChangeTestNamePath = DATAPATH + laneChangeTestName;
		ofstream laneChangeTest;
		if (currentTime < 20)
		{
			laneChangeTest.open(laneChangeTestNamePath, ios::trunc);
			laneChangeTest << "TIME" << currentTime << endl;
			laneChangeTest.close();
		}

		if (currentTime > 311 && id_veh == 40)
		{



			laneChangeTest.open(laneChangeTestNamePath, ios::app);

			int vehLeftDwID = 0;
			int vehLeftUpID = 0;
			int vehCurDwID = 0;
			int vehCurUpID = 0;
			int vehRightUpID = 0;
			int vehRightDwID = 0;


			if (pVehLeftDw != NULL)
			{
				vehLeftDwID = pVehLeftDw->getId();
			}
			if (pVehLeftUp != NULL)
			{
				vehLeftUpID = pVehLeftUp->getId();
			}
			if (pVehCurDw != NULL)
			{
				vehCurDwID = pVehCurDw->getId();
			}
			if (pVehCurUp != NULL)
			{
				vehCurUpID = pVehCurUp->getId();
			}
			if (pVehRightDw != NULL)
			{
				vehRightDwID = pVehRightDw->getId();
			}
			if (pVehRightUp != NULL)
			{
				vehRightUpID = pVehRightUp->getId();
			}


			laneChangeTest
				<< "LEFT TIME" << currentTime << "\n"
				<< "ac~=" << ac_Left_LC << "\n"
				<< "ac=" << ac_Left_NOLC << "\n"
				<< "an~=" << an_Left_LC << "\n"
				<< "an=" << an_Left_NOLC << "\n"
				<< "ao~=" << ao_Left_LC << "\n"
				<< "ao=" << ao_Left_NOLC << "\n"
				<< "Right TIME" << currentTime << "\n"
				<< "ac~=" << ac_Right_LC << "\n"
				<< "ac=" << ac_Right_NOLC << "\n"
				<< "an~=" << an_Right_LC << "\n"
				<< "an=" << an_Right_NOLC << "\n"
				<< "ao~=" << ao_Right_LC << "\n"
				<< "ao=" << ao_Right_NOLC << "\n"
				<< "ID=" << id_veh << "\n"
				<< "CurrentSection=" << numSect << "\n"
				<< "vehLeftDwID=" << vehLeftDwID << "\n"
				<< "Distance to vehLeftDw" << ditance_to_left_leader << "\n"
				<< "vehLeftUpID=" << vehLeftUpID << "\n"
				<< "vehCurDwID=" << vehCurDwID << "\n"
				<< "Distance to vehCurDw" << ditance_to_cur_leader << "\n"
				<< "vehCurUpID=" << vehCurUpID << "\n"
				<< "vehRightUpID=" << vehRightUpID << "\n"
				<< "vehRightDwID=" << vehRightDwID << "\n"
				<< "Distance to vehLeftDw" << ditance_to_left_leader << "\n"
				<< "curLane=" << curLane << "\n"
				<< "maxLanes=" << maxLanes << "\n"
				<< "current_speed=" << vehicle->getSpeed(vehicle->isUpdated()) << "\n"
				<< "useAsymmetric？" << useAsymmetric((simVehicleParticular*)vehicle) << "\n"
				<< "向左增益" << profit_to_left << "\n"
				<< "向右增益" << profit_to_right << "\n"
				<< "direction=" << direction << "\n\n" << endl;


			laneChangeTest.close();
		}
	}
	*/

	return direction;
}
