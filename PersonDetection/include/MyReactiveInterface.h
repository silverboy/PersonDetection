/*
 * MyReactiveInterface.h
 *
 *  Created on: 10/04/2013
 *      Author: jplata
 */




#ifndef MYREACTIVEINTERFACE_H_
#define MYREACTIVEINTERFACE_H_

#include "Aria.h"
#include "Arnl.h"
#include <pthread.h>
#include <sys/types.h>
#include <mrpt/reactivenav/CAbstractReactiveNavigationSystem.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/utils/CObserver.h>


using namespace mrpt::reactivenav;
using namespace mrpt::poses;
using namespace mrpt::gui;



class MyReactiveInterface : public CReactiveInterfaceImplementation {
public:
	MyReactiveInterface(ArRobot* robot,ArSick* laser,CDisplayWindowPlots* plot);
	bool getCurrentPoseAndSpeeds(CPose2D &curPose, float &curV,float &curW);
	bool changeSpeeds( float v, float w );
	bool senseObstacles( mrpt::slam::CSimplePointsMap &obstacles );


private:

	void getCurrentMeasures(CObservation2DRangeScan &laserScan,CPose2D &robotPose);

	void refreshPlot();

	ArRobot* robot;
	ArSick* laser;

	bool  newScan;

	CDisplayWindowPlots* plotWindow;


	CSimplePointsMap laserCurrentMap;
	CSimplePointsMap senseMap;
	CPolygon path;


	pthread_mutex_t m_mutex;


};



#endif /* MYREACTIVEINTERFACE_H_ */
