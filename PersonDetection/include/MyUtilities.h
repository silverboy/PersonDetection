/*
 * MyUtilities.h
 *
 *  Created on: 05/04/2013
 *      Author: jplata
 */

#include <mrpt/base.h>


#include <mrpt/utils/CConfigFile.h>



#include <mrpt/slam/CSimplePointsMap.h>
#include <mrpt/slam/COccupancyGridMap2D.h>
#include <mrpt/gui.h>
#include <mrpt/hwdrivers/CSickLaserSerial.h>




#ifndef MYUTILITIES_H_
#define MYUTILITIES_H_

using namespace mrpt::hwdrivers;
using namespace mrpt::gui;


/**
 * This class tries to connect with a Sick laser on port COM3 and
 * create a simulated laser if
 * connection could not be established
 */

class MyLaser{
public:
	MyLaser();
	void measure(CObservation2DRangeScan &obs,CPose2D robotPose=CPose2D(0,0,0));
	void plot(CObservation2DRangeScan obs);

private:

	bool simulateLaser;
	CSickLaserSerial	laser;
	CDisplayWindowPlots		win;
	COccupancyGridMap2D gridMap;
	CImage mapImage;
	float width,height,res,x_center_pixel,y_center_pixel;



};

#endif /* MYUTILITIES_H_ */
