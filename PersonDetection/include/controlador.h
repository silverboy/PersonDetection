

#ifndef CONTROLADOR_H_
#define CONTROLADOR_H_



#include "Detector.h"
#include "MyReactiveInterface.h"
#include "svm.h"
#include <time.h>
#include <pthread.h>




//#include "Aria.h"
#include "Arnl.h"

#include <vector>
#include <string>
#include <iostream>

#include "ArLocalizationTask.h"
#include <ArArgumentBuilder.h>

#include <mrpt/reactivenav/CReactiveNavigationSystem.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/threads.h>
#include <mrpt/utils/CTicTac.h>


#define TIME_TO_GO 20


using namespace std;
using namespace mrpt::slam;
using namespace mrpt::poses;
using namespace mrpt::reactivenav;




class Controlador: public ArASyncTask
{

public:
    enum accion{DETECTAR, IR, SEGUIR, VFF,TEST_PARADO,TEST_MOVIL,
    	TRAYECTORIA, GUARDARMEDIDAS, GUARDARCONTINUO,COUNT};
private:

    // Fields
    ArRobot* robot;
    ArSick* sick;
    ArPose currentPose;
    vector<ArPoseWithTime> medidas;
    vector<double> distancias;
    double max_laser_dist;
    double max_save_dist;
    Detector detector;

    CDisplayWindowPlots winPlot;
    CDisplayWindowPlots clusterPlot;
    CDisplayWindowPlots piernasPlot;
    MyReactiveInterface *interface;
    CReactiveNavigationSystem *reacNavObj;
    CAbstractReactiveNavigationSystem::TNavigationParams navParams;
    pthread_mutex_t nav_mutex;

    // SVM fields
    struct svm_model *model;
    struct svm_node *instancia;

    bool ejecutar;
    accion tarea;
    int data_index,delay;
    char logBuffer[150];


    // Functions
    double anguloEntre(double a, double b);
    bool estaAnguloEntre(double angle,double startAngle,double endAngle);
    bool goToGoal(bool blocking);
    void measure(bool saveToFile);
    void printMedidas();
    ArPose buscarPose(double angle);
    ArPose puntoMedio(ArPose p1, ArPose p2);
    void guardarMedidas();
    void filtrarMedidas(vector<double> *dfiltrada, vector<CPose2D> *puntos);
    void navegar();
    void monitorizarEntorno();
    vector<Cluster> detectarPiernas();


public:
    Controlador(ArRobot* r,int a_inf=100, int a_sup=200, int b=400);
    Controlador(int a_inf=100, int a_sup=200, int b=400);

    void suspender();
    void reanudar();
    bool setAction(accion a);
    bool estaEjecutando();
    void setTarget(double x, double y);
    void setDataIndex(int d);

    void* runThread(void*);
    void setCurrentPose(ArPose pose);

    void setMedidas(vector<ArPoseWithTime> med);
    void setDistancias(vector<double> dist);
    void mostrarVariaciones();
    void getCurrentMeasures(CObservation2DRangeScan &laserScan,CPose2D &robotPose);


};

#endif /* CONTROLADOR_H_ */

