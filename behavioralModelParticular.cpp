/*
可以改进的地方：
1. 把建立文件指针功能做成一个函数，提高复用性 2018/3/23 21.33
2. 把optimized vehicle data 做成一个类，将数据与功能组织在一起，便于管理 2018-9-20 13:41:35
3. 使用Git管理代码，同时建立完善的日志，记录想法变迁等内容。 2018-9-20 13:42:28
4. 写一个debug类或者一些debug函数，用于调试，摆脱传统的注释、运行部分 2018-9-20 13:55:25
*/

#include "behavioralModelParticular.h"
#include "simVehicleParticular.h"
#include "AKIProxie.h"
#include "ANGConProxie.h"
#include <stdio.h>
#include <fstream>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <cstring>
#include <vector>
#include <map>
#include <array>
#include <list>

using namespace std;
#define Tolerancia 0.01
#define VISIBLELIMIT 500 // for MOBIL  it's unncessary, and it shall be abandoned.
#define SPEED_CRITICAL 60 // using for Asymmetric MOBIL
#define DBL_MAX 1.7976931348623158e+308 
#define DATAPATH "D:\\working\\WorkingInEnhancedAIMSUNPlatform\\LaneChanging\\Data\\"
#define AFTPATH "D:\\Aimsun 8.1\\"

bool useIDM = true;
bool useHeuristicLaneChangeModel = true;
bool useMOBIL = true;
bool useAsymmetricMOBIL = false;        // Symmetric MOBIL is default

bool useIterationOptimization = false; // when it's value is true, the optimized vehicle will be selected in the entry section to re-experience the traffic condition over and over



double const penetrationOfACC = 0;
double const lowLimitOfACCTimeGap = 0.5;
double const upLimitOfACCTimeGap = 0.7;


// for Optimizing Working
double optimizedPolitenessFactor, optimizedThreshold, optimizedSafeFactor; // for MOBIL p , a_th, b_safe

// a new vehicle entry into the network will be setted as optimized vehicle 
// when the previous optimized vehicle is in the exit section. 
int optimizedExperienceTimes = 1;
int optimizedVehIDSequence[100]; //maximun of array size will not over the maximum of iteration
int optimiazedVehID = 4000; // work only when   useIterationOptimization = false

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
}vehicleODInfoDataSet[16000]; // total vehicle number entry in the network is about 15600


struct VehiclePathInfo {


	struct LaneChangeDetail {
		double occurrenceTime; // occurrence time of lanechanging
		int occurrenceSection;
		double occurrencePosition; // position refers to current section
		int occurenceVehID;
		int preLane; // using absolute network lane ID
		int folLane; // using absolute network lane ID
	}laneChangeDetailSet[10000]; // the maximum times of lanechanging  

	struct SectionPath {
		double entrySectionTime;
		int sectionID;
	}sectionPathInfo[22];// the maximum of section is 22, so the path section number will not over the 22


	double totalTravelPathLength; // m
	int totalLaneChangeTimes;
	double totalTravelTime; // sec
	double entryTime, exitTime; // sec
	int entryLane;

	struct {
		double speedValue;
		double time;
	}speedDataSet[9000];
	//	double	acceleration[36000];
	int step;
	//temp variation for judging whether vehicle states is changed
	int	preSectionID;
	int currSectionID;
	int preLane; // using absolute network lane ID

}optVehPathInfo;


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
int outPutRunTimes;
int inPutParaRunTimes;


list <int> controlGroupVehIDSet;


// sectionSequence[sequence]=sectionID
int const sequenceSectionID[23] = { 0,363,364,370,387,1022,952,949,386,395,935,404,974,982,967,406,414,423,986,990,994,998,1002 };




int lcProfitOpenRunTime = 0;
int lcProfitCloseRunTime = 0;

/*************************** Function Declaration (custom) **************************/

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

bool isEntrySection(int sectionID)
{
	switch (sectionID)
	{
	case 671:
	case 664:
	case 670:
	case 928:
	case 932:
		return true;
	default:
		return false;
	}

}

// this test function should be integrated into outPutControlGroupVehiclesODInfo()
bool isControlGroupVehicle(int referenceVehicleID, int testVehicleID)
{

	if (// 1. time window comparison
		vehicleODInfoDataSet[testVehicleID].entryTime > vehicleODInfoDataSet[referenceVehicleID].entryTime - 30
		&& vehicleODInfoDataSet[testVehicleID].entryTime < vehicleODInfoDataSet[referenceVehicleID].entryTime + 30
		// 2. OD comparison
		&& vehicleODInfoDataSet[testVehicleID].entrySection == vehicleODInfoDataSet[referenceVehicleID].entrySection
		&&vehicleODInfoDataSet[testVehicleID].entryLane == vehicleODInfoDataSet[referenceVehicleID].entryLane
		&&vehicleODInfoDataSet[testVehicleID].exitSection == vehicleODInfoDataSet[referenceVehicleID].exitSection
		)
	{
		return true;
	}
	else
		return false;

}

// record travel time, path length and OD
void recordAllVehicleInfo(A2SimVehicle *vehicle)
{


	int vehID = vehicle->getId();
	double currTime = AKIGetCurrentSimulationTime(); // seconds
	int currSectionID = vehicle->getIdCurrentSection();


	// record time infomation
	if (vehicleODInfoDataSet[vehID].entryTime == 0)
		vehicleODInfoDataSet[vehID].entryTime = currTime;
	if (vehicleODInfoDataSet[vehID].exitTime < currTime)
		vehicleODInfoDataSet[vehID].exitTime = currTime;
	if (vehicleODInfoDataSet[vehID].exitTime - vehicleODInfoDataSet[vehID].entryTime > vehicleODInfoDataSet[vehID].totalTravelTime)
		vehicleODInfoDataSet[vehID].totalTravelTime = vehicleODInfoDataSet[vehID].exitTime - vehicleODInfoDataSet[vehID].entryTime;



	// record path length
	if (currSectionID != vehicleODInfoDataSet[vehID].preSectionID)
	{
		A2KSectionInf sectionInfo;
		sectionInfo = AKIInfNetGetSectionANGInf(currSectionID);

		vehicleODInfoDataSet[vehID].totalTravelPathLength += sectionInfo.length;
		vehicleODInfoDataSet[vehID].preSectionID = currSectionID;
	}


	// record OD 
	if (isEntrySection(currSectionID))
	{
		vehicleODInfoDataSet[vehID].entrySection = currSectionID;
		vehicleODInfoDataSet[vehID].entryLane = vehicle->getIdCurrentLane(); // since the lane changing cannot happened in the input sections which have only one lane. 
	}
	if (isExitSection(currSectionID))
		vehicleODInfoDataSet[vehID].exitSection = currSectionID;


}



void recordOptVehicleTravelTime(double currTime)
{
	if (optVehPathInfo.entryTime == 0)
		optVehPathInfo.entryTime = currTime;
	if (optVehPathInfo.exitTime < currTime)
		optVehPathInfo.exitTime = currTime;
	if (optVehPathInfo.exitTime - optVehPathInfo.entryTime > optVehPathInfo.totalTravelTime)
		optVehPathInfo.totalTravelTime = optVehPathInfo.exitTime - optVehPathInfo.entryTime;

}
void recordOptVehiclePathLength(int currSectionID)
{
	// record path length
	if (currSectionID != optVehPathInfo.preSectionID)
	{
		A2KSectionInf sectionInfo;
		sectionInfo = AKIInfNetGetSectionANGInf(currSectionID);

		optVehPathInfo.totalTravelPathLength += sectionInfo.length;
		optVehPathInfo.preSectionID = currSectionID;
	}

}
void recordOptVehiclLaneChangingInfo(A2SimVehicle *vehicle)
{
	int currAbsoluteLaneID = getNetWorkAbsoluteLaneID(vehicle->getIdCurrentSection(), vehicle->getNumberOfLanesInCurrentSection());
	if (optVehPathInfo.preLane != currAbsoluteLaneID)
	{
		++optVehPathInfo.totalLaneChangeTimes;
		optVehPathInfo.laneChangeDetailSet[optVehPathInfo.totalLaneChangeTimes].occurenceVehID = vehicle->getId();
		optVehPathInfo.laneChangeDetailSet[optVehPathInfo.totalLaneChangeTimes].occurrenceTime = AKIGetCurrentSimulationTime();
		optVehPathInfo.laneChangeDetailSet[optVehPathInfo.totalLaneChangeTimes].occurrencePosition = vehicle->getPosition(0);
		optVehPathInfo.laneChangeDetailSet[optVehPathInfo.totalLaneChangeTimes].occurrenceSection = vehicle->getIdCurrentSection();
		optVehPathInfo.laneChangeDetailSet[optVehPathInfo.totalLaneChangeTimes].preLane = optVehPathInfo.preLane;
		optVehPathInfo.laneChangeDetailSet[optVehPathInfo.totalLaneChangeTimes].folLane = currAbsoluteLaneID;
		optVehPathInfo.preLane = currAbsoluteLaneID;
	}
}


void inputParameterSetFromAFT()
{
	if (inPutParaRunTimes == 0)
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


		char msg[150];
		sprintf_s(msg, "MOBIL Parameters loaded. (P,T)=(%f,%f)", parameterSet[363], parameterSet[364]);
		AKIPrintString(msg);

		inPutParaRunTimes = 1;
	}

}


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
				<< vehicleODInfoDataSet[vehID].entrySection << "\t"
				<< vehicleODInfoDataSet[vehID].exitSection << "\t"
				<< vehicleODInfoDataSet[vehID].entryTime << "\t"
				<< vehicleODInfoDataSet[vehID].totalTravelPathLength << "\t"
				<< vehicleODInfoDataSet[vehID].totalTravelTime << "\t"
				<< vehicleODInfoDataSet[vehID].totalTravelTime / vehicleODInfoDataSet[vehID].totalTravelPathLength * 1000 << endl;
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
		<< "Average Travel Time" << endl;
	for (auto vehID = 1; vehID < 16000; ++vehID)
	{
		outPutAllVehicleODInfo
			<< vehID << "\t"
			<< vehicleODInfoDataSet[vehID].entrySection << "\t"
			<< vehicleODInfoDataSet[vehID].exitSection << "\t"
			<< vehicleODInfoDataSet[vehID].entryTime << "\t"
			<< vehicleODInfoDataSet[vehID].totalTravelPathLength << "\t"
			<< vehicleODInfoDataSet[vehID].totalTravelTime << "\t"
			<< vehicleODInfoDataSet[vehID].totalTravelTime / vehicleODInfoDataSet[vehID].totalTravelPathLength * 1000 << endl;

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
	for (auto impactedVehIDIter = controlGroupVehIDSet.begin(); impactedVehIDIter != controlGroupVehIDSet.end(); ++impactedVehIDIter)
	{
		outPutImpactedVehID << *impactedVehIDIter << "\t";
	}
	outPutImpactedVehID.close();

}

void outPutOptVehLaneChangingDetials()
{

	string outPutLaneChangingFullPath;
	string outPutLaneChangingFileName = "OptVehLaneChangingDetails.dat";
	outPutLaneChangingFullPath = DATAPATH + outPutLaneChangingFileName;
	ofstream outPutLaneChanging;
	outPutLaneChanging.open(outPutLaneChangingFullPath, ios::app);
	outPutLaneChanging
		<< "TotalLaneChangingTimes\t" << optVehPathInfo.totalLaneChangeTimes << "\n"
		<< "LaneChangingDetails \n"
		<< "OccurrenceTime" << "\t" << "OccurrencePosition" << "\t" << "OccurrenceSection" << "\t" << "PreviousLane" << "\t" << "FollowingLane"
		<< endl;
	int iter;
	for (iter = 1; iter <= optVehPathInfo.totalLaneChangeTimes; ++iter)
	{
		outPutLaneChanging
			<< optVehPathInfo.laneChangeDetailSet[iter].occurenceVehID << "\t"
			<< optVehPathInfo.laneChangeDetailSet[iter].occurrenceTime << "\t"
			<< optVehPathInfo.laneChangeDetailSet[iter].occurrencePosition << "\t"
			<< optVehPathInfo.laneChangeDetailSet[iter].occurrenceSection << "\t"
			<< optVehPathInfo.laneChangeDetailSet[iter].preLane << "\t"
			<< optVehPathInfo.laneChangeDetailSet[iter].folLane << "\t"
			<< endl;
		if (iter == optVehPathInfo.totalLaneChangeTimes)
			outPutLaneChanging << endl;
	}

	outPutLaneChanging.close();

}

void outPutOptVehSpeed()
{
	string outPutSpeedFullPath;
	string outPutSpeedFileName = "OptimizedVehicleSpeed.dat";
	outPutSpeedFullPath = DATAPATH + outPutSpeedFileName;
	ofstream outPutSpeed;
	outPutSpeed.open(outPutSpeedFullPath, ios::trunc);
	for (int speedIter = 0; optVehPathInfo.speedDataSet[speedIter].time < optVehPathInfo.exitTime; ++speedIter)
	{
		outPutSpeed
			<< optVehPathInfo.speedDataSet[speedIter].time << "\t"
			<< optVehPathInfo.speedDataSet[speedIter].speedValue
			<< endl;
	}
	outPutSpeed.close();
}

void outPutOptVehPerformance()
{
	string outPutPerformanceFullPath;
	string outPutPerformanceFileName = "Performance.txt";
	outPutPerformanceFullPath = DATAPATH + outPutPerformanceFileName;
	ofstream outPutPerformance;
	outPutPerformance.open(outPutPerformanceFullPath, ios::trunc);
	outPutPerformance
		<< optVehPathInfo.totalTravelTime / optVehPathInfo.totalTravelPathLength * 1000 << endl;
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
		<< optVehPathInfo.entryTime << "\t"
		<< optVehPathInfo.exitTime << "\t"
		<< optVehPathInfo.totalTravelTime << "\t"
		<< optVehPathInfo.totalTravelPathLength << "\t"
		<< optVehPathInfo.totalTravelTime / optVehPathInfo.totalTravelPathLength * 1000;
	outPutData.close();

}


// for investigating the relation between lane changing times and politeness factor p 
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




//functions for debug, test 2018-9-20 14:03:20
void needDebugMessage(string message, double value)
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

simVehicleParticular * behavioralModelParticular::arrivalNewVehicle(void *handlerVehicle, unsigned short idHandler, bool isFictitiousVeh) {

	simVehicleParticular * res = new simVehicleParticular(handlerVehicle, idHandler, isFictitiousVeh);
	if (!isFictitiousVeh)
	{

		if (AKIGetRandomNumber() < penetrationOfACC)
		{
			res->setIsACC(true);
			res->setTimeGapOfACC(generateTimeGapOfACC(lowLimitOfACCTimeGap, upLimitOfACCTimeGap));
		}
	}
	return res;
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


	simVehicleParticular* vehicleTemp = (simVehicleParticular*)vehicle; //evaluateLaneChanging使用的是A2SimVehicle类，但arrivalNewVehicle产生的是它的子类
	//bool isACC = vehicleTemp->getIsACC();
	//bool isOptimizedVehicle = false; //vehicleTemp->getIsOptimizedVehicle();



	if (useIterationOptimization && haveOptimizedVeh.isExist == false && isEntrySection(currSectionID))
	{
		optimiazedVehID = vehID;
		haveOptimizedVeh.isExist = true;

	}


	if (vehID == optimiazedVehID)
	{

		inputParameterSetFromAFT();// input parameters to  <map>parameterSet


		recordOptVehicleTravelTime(currTime);
		recordOptVehiclePathLength(currSectionID);



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


	//recordImpactedVehID(vehID, currTime, currSectionID);

	recordAllVehicleInfo(vehicle);

	// OUTPUT data at the end time of simulation, (sec) 4 hours equals 14400 seconds
	if (outPutRunTimes == 0 && currTime > 14399)
	{
		outPutOptVehData();

		outPutOptVehPerformance();

		//	outPutOptVehSpeed();

		outPutOptVehLaneChangingDetials();

		outPutImpactedVehicleIDSet();

		//outPutAllVehicleODInfo();

		outPutControlGroupVehiclesODInfo();


		outPutRunTimes = 1;
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


				if ((pos > 30 && currSpeed > thresCurrSpeed && diffSpeed <= thresDiffSpeed && distUp > thresDist && distDown > thresDist) || (pos > 50 && currSpeed <0.1 && diffSpeed <= thresDiffSpeed && distUp >thresDist && distDown > thresDist))

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
	if (vehID == optimiazedVehID && useMOBIL)
	{

		//optimizedThreshold = parameterSet[currSectionID];
		optimizedPolitenessFactor = parameterSet[363];
		optimizedThreshold = parameterSet[364];

		direction = MOBILDirection(vehicle, optimizedPolitenessFactor, optimizedThreshold);

		// control OD, force the optimized vehicle to experience the whole freeway
		if ((currSectionID == 386 && currLane == 2 && direction == 1)
			|| (currSectionID == 406 && currLane == 2 && direction == 1))
		{
			direction = 0;
		} 


		//direction = MOBILDirection(vehicle, 1, 0.5);
		//direction = MOBILDirection(vehicle, 1, 0);

		//double farSightThreshold = getModifiedThreshold(vehicle, currLane - direction);
		//int farSightDirection = MOBILDirection(vehicle, 1, 0.5+farSightThreshold);

		// block Turn To OffRamp
		//if ((currSectionID == 386 || currSectionID == 406) && currLane == 2 && direction == 1)
		//	direction = 0;

		//if (direction != 0 && currTime > 4203)
		//{
		//	char msg[150];
		//	sprintf_s(msg, "vehicle = %d,current time=%f MOBIL Direction = %d", vehID, currTime, direction);
		//	AKIPrintString(msg);
		//}


		vehicle->applyLaneChanging(direction, threadId); // this function includes the gap acceptance. so if direction is not acceptable, aimsun will consider it
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
	if (
		(useIDM)
		&& vehicle->getIdCurrentSection() != 671
		&& vehicle->getIdCurrentSection() != 664
		&& vehicle->getIdCurrentSection() != 670
		&& vehicle->getIdCurrentSection() != 928
		&& vehicle->getIdCurrentSection() != 932

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
		&& vehicle->getIdCurrentSection() != 671
		&& vehicle->getIdCurrentSection() != 664
		&& vehicle->getIdCurrentSection() != 670
		&& vehicle->getIdCurrentSection() != 928
		&& vehicle->getIdCurrentSection() != 932

		&& vehicle->getIdCurrentSection() != 556// only used in capacity test
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

		simVehicleParticular *tempVeh;


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
		/*END Test for understanding "getgap"*/







		double a = vehicle->getAcceleration();
		double b = vehicle->getDeceleration();
		double VelAnterior, PosAnterior, VelAnteriorLeader, PosAnteriorLeader;


		double GapAnterior = vehicle->getGap(Shift, leader, ShiftLeader, PosAnterior, VelAnterior, PosAnteriorLeader, VelAnteriorLeader);
		double diff = VelAnterior - VelAnteriorLeader;


		simVehicleParticular *newLeader;
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

//determining by speed, section and lane   
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




	//double currTime = AKIGetCurrentSimulationTime();

	//char msg[200];
	//if (pVehLeftUp != NULL)
	//{
	//	sprintf_s(msg, "SV ID is %d, postion=%f, pVehLeftUp = %d, offset=%f ", vehicle->getId(), vehicle->getPosition(0), pVehLeftUp->getId(), pVehLeftUp->getPosition(0));
	//	AKIPrintString(msg);
	//}
	//if (pVehLeftDw != NULL)
	//{
	//	sprintf_s(msg, "SV ID is %d, postion=%f, pVehLeftDw = %d, offset=%f ", vehicle->getId(), vehicle->getPosition(0), pVehLeftDw->getId(), pVehLeftDw->getPosition(0));
	//	AKIPrintString(msg);
	//}
	//if (pVehRightUp != NULL)
	//{
	//	sprintf_s(msg, "SV ID is %d, postion=%f, pVehRightUp = %d, offset=%f ", vehicle->getId(), vehicle->getPosition(0), pVehRightUp->getId(), pVehRightUp->getPosition(0));
	//	AKIPrintString(msg);
	//}
	//if (pVehRightDw != NULL)
	//{
	//	sprintf_s(msg, "SV ID is %d, postion=%f, pVehRightDw = %d, offset=%f ", vehicle->getId(), vehicle->getPosition(0), pVehRightDw->getId(), pVehRightDw->getPosition(0));
	//	AKIPrintString(msg);
	//}
	//


	/*visible limit, if a vehicle is not visible, it can not influence current vehicle
	it is unnecessary, and shall be abandoned.
	*/
	//double gapLeftToUp, gapLeftToDw;
	//gapLeftToUp = VISIBLELIMIT;
	//gapLeftToDw = VISIBLELIMIT;
	//if (shiftCurUp > VISIBLELIMIT) pVehCurUp = NULL;
	//if (shiftCurDw > VISIBLELIMIT) pVehCurDw = NULL;

	//if (ShiftUpLeft > VISIBLELIMIT) pVehLeftUp = NULL;
	//if (ShiftDwLeft > VISIBLELIMIT) pVehLeftDw = NULL;

	//if (ShiftUpRight > VISIBLELIMIT) pVehRightUp = NULL;
	//if (ShiftDwRight > VISIBLELIMIT) pVehRightDw = NULL;



	double testTime = AKIGetCurrentSimulationTime();
	int testID = vehicle->getId();
	if (testTime == 312 && testID == 40)
	{
		testID = 0;
	}
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
	bool needTest = true;
	if (needTest)
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


	return direction;
}
