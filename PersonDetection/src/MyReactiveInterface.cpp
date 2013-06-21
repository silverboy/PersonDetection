/*
 * MyReactiveInterface.cpp
 *
 *  Created on: 10/04/2013
 *      Author: jplata
 */

#include "MyReactiveInterface.h"

MyReactiveInterface::MyReactiveInterface(ArRobot* robot,ArSick* laser,CDisplayWindowPlots* plot):
robot(robot),
laser(laser),
newScan(false),
plotWindow(plot){
	// TODO Auto-generated constructor stub
	plotWindow->hold_on();
	pthread_mutex_init(&m_mutex,NULL);

	laserCurrentMap.insertionOptions.addToExistingPointsMap=false;

}

bool MyReactiveInterface::getCurrentPoseAndSpeeds(CPose2D &curPose, float &curV,float &curW){

	cout << "getCurrentPoseAndSpeeds" << endl;

	ArPose pose=robot->getPose();

	curPose.x(pose.getX()*0.001);
	curPose.y(pose.getY()*0.001);
	curPose.phi(DEG2RAD(pose.getTh()));

	curV = robot->getVel() * 0.001;
	curW = DEG2RAD( robot->getRotVel() );
	return true;

}

bool MyReactiveInterface::changeSpeeds(float v, float w){

	cout << "changeSpeeds" << endl;
	robot->setVel( v*1000 );
	robot->setRotVel( RAD2DEG( w ) );
	return true;

}



bool MyReactiveInterface::senseObstacles(CSimplePointsMap &obstacles){

	cout << "senseObstacles" << endl;

	CObservation2DRangeScan laserScan;
	CPose2D rPose;
	CPose3D rPose3D;

	getCurrentMeasures(laserScan,rPose);
	rPose3D=rPose;


	obstacles.insertionOptions.minDistBetweenLaserPoints=0.005f;
	obstacles.insertionOptions.also_interpolate=false;

	obstacles.clear();
	obstacles.insertObservation(&laserScan);

	// Update data for plot thread
	pthread_mutex_lock(&m_mutex);
	path.AddVertex(rPose.x(),rPose.y());
	laserCurrentMap.loadFromRangeScan(laserScan,&rPose3D);
	newScan=true;
	pthread_mutex_unlock(&m_mutex);

	refreshPlot();


	return true;

}

void MyReactiveInterface::getCurrentMeasures(CObservation2DRangeScan &laserScan,CPose2D &robotPose){

	cout << "getCurrentMeasures" << endl;


	// Clear readings
	laserScan.scan.clear();
	laserScan.validRange.clear();
	laserScan.aperture=M_PIf;
	laserScan.rightToLeft=false;


	float dist;
	char valid;
	ArPose punto;


	//Bloquear laser
	laser->lockDevice();

	ArPose pose=robot->getPose();

	vector<ArSensorReading> *readings=laser->getRawReadingsAsVector();

	robotPose.x(pose.getX()*0.001);
	robotPose.y(pose.getY()*0.001);
	robotPose.phi(DEG2RAD(pose.getTh()));



	for(vector<ArSensorReading>::iterator it=readings->begin(); it != readings->end(); it++)
	{

		if(it->getIgnoreThisReading())
		{
			// Establezco 10m, máximo rango del láser
			dist=10000;
			valid=0;
		}
		else
		{
			punto=it->getPose();
			dist=pose.findDistanceTo(punto);
			valid=1;
		}

		laserScan.scan.push_back(dist*0.001);
		laserScan.validRange.push_back(valid);

	}

	laser->unlockDevice();


}

void MyReactiveInterface::refreshPlot(){

	if(newScan){

		plotWindow->clear();

		// Dibujar mapa historico
		vector<float>	xs1,ys1,xs2,ys2,zs;
		senseMap.getAllPoints(xs1,ys1,zs);
		plotWindow->plot(xs1,ys1,".b3");

		// Dibujar scan actual
		laserCurrentMap.getAllPoints(xs2,ys2,zs);
		plotWindow->plot(xs2,ys2,".g3");

		//Dibujar path robot
		vector<double> x,y;
		path.getAllVertices(x,y);
		plotWindow->plot(x,y,"-r1");

		cout << laserCurrentMap.size() << endl;


		pthread_mutex_lock(&m_mutex);
		senseMap.fuseWith(&laserCurrentMap,0.01);
		newScan=false;
		pthread_mutex_unlock(&m_mutex);


		//path.getPlotData(xs,ys);
		//path.AddVertex(rPose.x(),rPose.y());
		//plotWindow.plot(xs,ys,"-r3");

	}
}









