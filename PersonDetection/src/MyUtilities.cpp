/*
 * MyUtilities.cpp
 *
 *  Created on: 05/04/2013
 *      Author: jplata
 */

#include "../include/MyUtilities.h"

//IMPLEMENTS_GENERIC_SENSOR(CActivMediaRobotBase,mrpt::hwdrivers)

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace mrpt::slam;
using namespace mrpt::hwdrivers;
using namespace std;



MyLaser::MyLaser():win("Laser"){


	simulateLaser=false;

	laser.setSerialPort("ttyS2");

	//laser.setBaudRate(500000);
	laser.setBaudRate(19200);
	laser.setScanFOV(180);
	laser.setScanResolution(50);  // 25=0.25deg, 50=0.5deg, 100=1deg
	//laser.setMillimeterMode(true);

	try{
		cout << "Trying to initialize the laser..." << endl;
		laser.initialize(); // This will raise an exception on error
		cout << "Initialized OK!" << endl;
	}
	catch (std::exception &e){
		cout << "No hardware laser found, using a simulated one with map in map_config.ini" << endl;
		//cerr << e.what() << endl;
		simulateLaser=true;
	}

	if(simulateLaser){

		string iniFile("../map_config.ini");
		ASSERT_(mrpt::system::fileExists(iniFile));
		CConfigFile	cfgFile(iniFile);

		string  bmp=cfgFile.read_string("Params","bitmap_file","",true);
		res=cfgFile.read_float("Params","evaluation_grid_resolution",0.1f,true);
		x_center_pixel=cfgFile.read_float("Params","x_center_pixel",0,true);
		y_center_pixel=cfgFile.read_float("Params","y_center_pixel",0,true);

		cout << bmp << "\t" << res << "\t" << x_center_pixel << "\t" << y_center_pixel << endl;

		gridMap.loadFromBitmapFile(bmp,res,x_center_pixel,y_center_pixel);

		width=gridMap.getXMax()-gridMap.getXMin();
		height=gridMap.getYMax()-gridMap.getYMin();

		gridMap.getAsImage(mapImage);

	}

}

/**
 * Connect to the laser using Aria and Arnl library, in order
 * it could be used in MobileSim
 *
 *
 */

void MyLaser::measure(CObservation2DRangeScan &obs,CPose2D robotPose){


	if(simulateLaser){

		obs.aperture=M_PIf;
		obs.rightToLeft=true;
		obs.maxRange=10.0f;
		obs.stdError=0.003f;

		gridMap.laserScanSimulator(obs,robotPose,0.5f,181,0);
	}
	else{

		bool						thereIsObservation,hardError;


		try
		{
			laser.doProcessSimple( thereIsObservation, obs, hardError );
		}
		catch (std::exception &e)		{
			cerr << e.what() << endl;
			hardError = true;
		}

		if (hardError)
			printf("[TEST] Hardware error=true!!\n");

	}

}

void MyLaser::plot(CObservation2DRangeScan obs){

	if(simulateLaser){

		CSimplePointsMap map;

		map.insertionOptions.minDistBetweenLaserPoints=0;
		map.insertionOptions.also_interpolate=false;
		map.insertionOptions.isPlanarMap=true;

		map.clear();
		map.insertObservation(&obs);


		vector_float	xs,ys,zs;
		map.getAllPoints(xs,ys,zs);


		float width(gridMap.getXMax()-gridMap.getXMin()),height(gridMap.getYMax()-gridMap.getYMin());

		win.image(mapImage,-x_center_pixel*res*1000,-y_center_pixel*res*1000,width*1000,height*1000);

		win.plot(xs*1000,ys*1000,".b3");

	}

	else{


	}





}
