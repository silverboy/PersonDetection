#include "controlador.h"

using namespace std;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::reactivenav;


Controlador::Controlador(int a_inf, int a_sup, int b):
    currentPose(0,0),
    ejecutar(false),
    tarea(DETECTAR),
    data_index(0)
{
}

Controlador::Controlador(ArRobot *r, int a_inf, int a_sup, int b):
    robot(r),
    currentPose(0,0),
    winPlot("Laser"),
    clusterPlot("Cluster"),
    piernasPlot("Piernas"),
    ejecutar(false),
    tarea(DETECTAR),
    data_index(0)
{

	sick=(ArSick*)robot->findLaser(1);
	interface=new MyReactiveInterface(r,sick,&winPlot);

	ASSERT_FILE_EXISTS_("../CONFIG_ReactiveNavigator.ini");
	ASSERT_FILE_EXISTS_("../CONFIG_RobotDescription.ini");
	ASSERT_FILE_EXISTS_("../CONFIG_Measure.ini");

	CConfigFile iniNav("../CONFIG_ReactiveNavigator.ini");
	CConfigFile iniRob("../CONFIG_RobotDescription.ini");
	CConfigFile config("../CONFIG_Measure.ini");


	max_laser_dist=config.read_double("MEASURE_CONFIG","LASER_MAX_DISTANCE",10,false);
	max_save_dist=config.read_double("MEASURE_CONFIG","SAVE_MEASURE_MAX_DISTANCE",10,false);
	detector=Detector(max_save_dist);

	// Cargar SVM
	model=svm_load_model("svm_model");
	instancia=malloc(4*sizeof(struct svm_node));

	if(model == NULL){
		cout << "No pudo cargarse el modelo SVM" << endl;
		exit(0);
	}

	//Configurar plot
	clusterPlot.hold_on();
	piernasPlot.hold_on();



	reacNavObj=new CReactiveNavigationSystem(*interface,false,false);
	reacNavObj->loadConfigFile(iniNav,iniRob);
	reacNavObj->initialize();

	navParams.targetAllowedDistance=0.2f;
	navParams.targetIsRelative=true;

	pthread_mutex_init(&nav_mutex,NULL);

	// Crear hebra de navegacion
	mrpt::system::createThreadFromObjectMethod(this,&Controlador::navegar);


}

void Controlador::setMedidas(vector<ArPoseWithTime> med)
{
    medidas=med;
}

void Controlador::setDistancias(vector<double> dist)
{
    distancias=dist;
}



// Funcion que devuelve el ángulo entre dos ángulos a y b. El ángulo es medido desde a a b en sentido horario.
// El ángulo será negativo si b se encuentra por detrás de a

double Controlador::anguloEntre(double a, double b)
{
    double diferencia=a-b;

    // Si el ángulo resultante es mayor de 180º estoy midiendo por el lado largo
    if(ArMath::fabs(diferencia) > 180)
    {
        if(diferencia < 0)
        {
            diferencia=360+diferencia;
        }
        else
        {
            diferencia=diferencia-360;
        }
    }
    return diferencia;
}

void Controlador::setTarget(double x, double y){
	if(estaEjecutando()){
		navParams.target.x=x;
		navParams.target.y=y;
		reacNavObj->navigate(&navParams);
	}
}



void Controlador::suspender()
{

	pthread_mutex_lock(&nav_mutex);
	reacNavObj->cancel();
	robot->stop();
    ejecutar=false;
    pthread_mutex_unlock(&nav_mutex);
}

void Controlador::reanudar()
{
    ejecutar=true;
}

void Controlador::mostrarVariaciones()
{
    //espia.printVariaciones(medidas,goalPose);
}

bool Controlador::setAction(accion a)
{
    if(a>=0 && a < Controlador::COUNT)
    {
        tarea=a;
        return true;
    }
    else
    {
        return false;
    }
}

void* Controlador::runThread(void*)
{
    double angle;
    clock_t inicio,fin;
    CTicTac temporizador;
    // Run until the thread is requested to end by another thread.
    while(getRunning())
    {
        ArTime timer;

        if(ejecutar)
        {

            switch(tarea)
            {

            case Controlador::DETECTAR:
            {

            	temporizador.Tic();

            	// Medir entorno y filtrar medidas
            	monitorizarEntorno();

            	// Obtener clusteres y determinar cuales son piernas
            	vector<Cluster> piernas=detectarPiernas();


            	vector<CPose2D> personas=detector.buscarPersonas(piernas);
            	cout << "Personas detectadas: " << personas.size() << endl;

            	cout << "Piernas detectadas: " << piernas.size() << endl;
            	detector.printClusters(piernas);

            	vector<double> x,y;
            	x.clear();
            	y.clear();
            	for(int k=0;k < personas.size(); k++){
            		x.push_back(personas[k].x());
            		y.push_back(personas[k].y());
            	}
            	piernasPlot.plot(x,y,".c4");

            	// Si hay personas detectadas nos dirigimos a la más cercana
            	if(!personas.empty()){
            		double t_x=personas[0].x();
            		double t_y=personas[0].y();
            		double dist=personas[0].norm();

            		for(int i=1;i < personas.size(); i++){
            			if(personas[i].norm() < dist){
            				t_x=personas[i].x();
            				t_y=personas[i].y();
            				dist=personas[i].norm();
            			}
            		}

            		cout << "Persona elegida ( " << t_x << " , " << t_y << " )" << endl;
            		setTarget(t_x,t_y);

            	}




            	cout << "tiempo rutina (ms): " << temporizador.Tac()*1000 << endl;

            	ArUtil::sleep(1000);
            }

                break;

            case Controlador::IR:
                /*ArLog::log(ArLog::Normal ,"Entra en ir\n");

                if(!detector.seleccionarPersona(&goalPose,currentPose))
                {
                    setAction(DETECTAR);
                    break;
                }
                else{
                	this->goToGoal(false);

                }
                setAction(DETECTAR);
*/                // Si he conseguido seleccionar la persona objetivo
                // voy hacia ella
//                if(this->goToGoal(true))
//                {
//                    setAction(SEGUIR);
//                	//setAction(DETECTAR);
//                    this->measure(false);
//                    espia.actualizarMedidas(distancias,medidas);
//                    ArLog::log(ArLog::Normal,"Modo seguimiento...");
//                    //ejecutar=false;
//                }
//                else
//                {
//                    setAction(DETECTAR);
//
//                }
                break;
            case Controlador::SEGUIR:


               /* timer.setToNow();
                inicio=clock();

                // Codigo original
                this->measure(false);
                espia.calcularVariaciones(distancias,medidas);
                //espia.printVariaciones(medidas);*/

//                this->measure(false);
//                espia.actualizarMedidas(distancias, medidas);
//                ArUtil::sleep(100);
//                this->measure(false);
//                espia.calcularVariaciones(distancias, medidas);
//                //espia.printVariaciones(medidas);

//                if(espia.calcularVariacionGlobalPerception(medidas,currentPose.findAngleTo(goalPose),goalPose))
//                {
//                    angle=espia.getGoalAngle();
//                    //printMedidas();
//                    espia.printVariaciones(medidas,goalPose);
//                    goalPose=puntoMedio(goalPose, buscarPose(angle));
//                    currentPose.log();
//
//
//                    espia.printGlobalPerception();
//                    sprintf(logBuffer, "Angulo: %f\t GoalPose: ",angle);
//                    ArLog::log(ArLog::Normal ,logBuffer);
//                    goalPose.log();
//
//                    robot->setHeading(angle);
//                    goToGoal(false);
//                    espia.actualizarMedidas(distancias,medidas);
//                    fin=clock();
//
//
//                    sprintf(logBuffer, "Tiempo rutina: %f\n",(double)(fin-inicio)/CLOCKS_PER_SEC);
//                    ArLog::log(ArLog::Normal ,logBuffer);
//                }
//                else{
//                	espia.printVariaciones(medidas,goalPose);
//                	ArLog::log(ArLog::Normal ,"\n");
//
//                }
//
//                //espia.calcularVariaciones(distancias,medidas);
//                //espia.printVariaciones(medidas);
//
//                while(timer.mSecSince() < 200 )
//                {
//                    ArUtil::sleep(50);
//                }

                //ejecutar=false;
                break;

            case Controlador::VFF:
            {
            	cout << "entra en navegacion" << endl;

            	// Deshabilitar key handler
            	ArSyncTask* sensorInterp=robot->findTask("Sensor Interp");
            	ArSyncTask* keyH=sensorInterp->find("ManualKeyHandler");
            	keyH->setState(ArTaskState::SUSPEND);



            	// Pedir coordenadas destino
            	bool relativo;
            	printf ("¿Coordenadas relativas? [0=No 1=Si]: ");
            	cin >> relativo;
            	navParams.targetIsRelative=relativo;
            	double x,y;
            	printf ("Introduzca coordenada x: ");
            	cin >> x;
            	printf ("Introduzca coordenada y: ");
            	cin >> y;
            	printf ("You have entered x: %f y: %f\n",x,y);

            	// Habilitar key Handler
            	keyH->setState(ArTaskState::RESUME);

            	setTarget(x,y);
            	ejecutar=false;

            }

            	break;

            case Controlador::GUARDARMEDIDAS:
            {
            	temporizador.Tic();

            	monitorizarEntorno();

            	vector<Cluster> piernas=detectarPiernas();


            	vector<CPose2D> personas=detector.buscarPersonas(piernas);
            	cout << "Personas detectadas: " << personas.size() << endl;

            	cout << "Piernas detectadas: " << piernas.size() << endl;
            	detector.printClusters(piernas);

            	vector<double> x,y;
            	x.clear();
            	y.clear();
            	for(int k=0;k < personas.size(); k++){
            		x.push_back(personas[k].x());
            		y.push_back(personas[k].y());
            	}
            	piernasPlot.plot(x,y,".c4");

            	cout << "tiempo rutina (ms): " << temporizador.Tac()*1000 << endl;


            	ejecutar=false;


            }
            	break;

            case Controlador::GUARDARCONTINUO:
            {

            	ArUtil::sleep(1000);
            	measure(false);
            	//Filtrar medidas
            	vector<double> dfiltrada;
            	vector<CPose2D> puntos;
            	filtrarMedidas(&dfiltrada,&puntos);

            	// Clusterizar
            	detector.setDistancias(dfiltrada);
            	detector.setPuntos(puntos);
            	vector<Cluster> piernas=detector.clusterizar(0.10,3);
            	vector<CPose2D> *p;
            	vector<double> x,y;

            	for(int i=0;i < piernas.size();i++){
            		p=piernas[i].getPuntos();
            		for(int j=0;j < piernas[i].getNumPuntos();j++){
            			x.push_back(p->at(j).x());
            			y.push_back(p->at(j).y());
            		}
            	}

            	detector.printClusters(piernas);

            	//Representar
            	clusterPlot.clear();
            	clusterPlot.plot(x,y,".b2");

            	// Guardar
            	guardarMedidas();
            }
            	break;


            default:
                break;
            }
            //ejecutar=false;
        }
        ArUtil::sleep(100);
    }
    ArLog::log(ArLog::Normal, "Example thread: requested stop running, ending thread.");
    return NULL;
}

bool Controlador::estaEjecutando(){
	return ejecutar;
}






void Controlador::setCurrentPose(ArPose pose)
{
    currentPose=pose;
}

void Controlador::printMedidas()
{

    for(unsigned int i=0; i < medidas.size(); i++)
    {
        sprintf(logBuffer, "Medida: %d\t Angulo:%.2f\t Distancia:%0.2f\t X:%0.2f\t Y:%0.2f\n",i,
               medidas[i].getTh(),distancias[i],medidas[i].getX(),medidas[i].getY());

        ArLog::log(ArLog::Normal ,logBuffer);

    }
}

void Controlador::navegar(){
	// Esperamos a que la hebra padre comience a ejecutarse
	while(!getRunning()){
		ArUtil::sleep(100);
	}

	while(getRunning()){

		pthread_mutex_lock(&nav_mutex);
    	if(reacNavObj->getCurrentState() == CReactiveNavigationSystem::NAVIGATING){
    		reacNavObj->navigationStep();
    	}
    	pthread_mutex_unlock(&nav_mutex);

    	ArUtil::sleep(10);

	}


}

/**
 * Este método obtiene las medidas del láser, las filtra
 * considerando la distancia máxima de detección (SAVE_MEASURE_MAX_DIST),
 * introduce las distancias y puntos en detector y elimina rectas.
 *
 * También dibuja las rectas detectadas.
 *
 */

void Controlador::monitorizarEntorno(){
	// Medimos con el laser estableciendo distancia maxima
	measure(false);
	vector<double> dfiltrada;
	vector<CPose2D> puntos;
	filtrarMedidas(&dfiltrada,&puntos);

	// Preparamos el detector para clusterizar
	detector.setDistancias(dfiltrada);
	detector.setPuntos(puntos);

	// Eliminamos lineas para facilitar la deteccion
	Eigen::MatrixXf rectas=detector.eliminarRectas(30,181);

	double limits[] = {0,2,-2,2};
	vector<double> limites (limits, limits + 4);

	cout << "Rectas detectadas: " << rectas.rows() << endl;

	// Dibujamos rectas detectadas
	for(int j=0;j < rectas.rows();j++){

		Grafico::dibujarLinea(&winPlot,rectas(j,0),rectas(j,1),limites);

	}

}

/**
 *
 * Obtiene clusteres a partir de las medidas que contiene el detector
 * y determina si son piernas o no usando el clasificador SVM.
 * Dibuja los clusteres y piernas detectados (en ventanas distintas)
 * y devuelve un vector con las piernas detectadas.
 *
 *
 */

vector<Cluster> Controlador::detectarPiernas(){

	// Clusterizar
	vector<Cluster> conjuntos=detector.clusterizar(0.10,3);
	vector<Cluster> piernas;
	vector<CPose2D> *p;
	vector<double> x,y;
	double target;

	string formato[3];
	formato[0]=".r2";
	formato[1]=".b2";
	formato[2]=".g2";

	clusterPlot.clear();
	piernasPlot.clear();



	for(int i=0;i < conjuntos.size();i++){
		p=conjuntos[i].getPuntos();
		for(int j=0;j < conjuntos[i].getNumPuntos();j++){
			x.push_back(p->at(j).x());
			y.push_back(p->at(j).y());
		}
		clusterPlot.plot(x,y,formato[i%3]);


		// Determinar si es pierna o no
		instancia[0].index=1;
		instancia[1].index=2;
		instancia[2].index=3;
		instancia[3].index=-1;

		instancia[0].value=conjuntos[i].getContorno();
		instancia[1].value=conjuntos[i].getAncho();
		instancia[2].value=conjuntos[i].getProfundidad();


		target=svm_predict(model,instancia);

		if(target==1){
			// El clasificador SVM lo reconoce como pierna
			piernas.push_back(conjuntos[i]);
			piernasPlot.plot(x,y,formato[i%3]);
		}
		x.clear();
		y.clear();
	}

	return piernas;
}

void Controlador::setDataIndex(int d){
	data_index=d;
}

/**
 * Cambia coordenadas a metros, si la distancia de un punto es mayor que
 * max_save_dist establece esta como distancia y representa los puntos
 *
 *
 */

void Controlador::filtrarMedidas(vector<double> *dfiltrada, vector<CPose2D> *puntos){


		dfiltrada->clear();
		puntos->clear();

		double dist,angle,p_x,p_y;
		vector<double> x,y;
		CPose2D p;


		for(int i=0; i < distancias.size(); i++)
		{

			dist=distancias[i]*0.001;

			if(dist > max_save_dist){
				dist=max_save_dist;
			}

			dfiltrada->push_back(dist);

			angle=(medidas[i]-currentPose).getTh();
			p_x=dist*ArMath::cos(angle);
			p_y=dist*ArMath::sin(angle);

			p.x(p_x);
			p.y(p_y);
			p.phi(DEG2RAD(angle));

			puntos->push_back(p);
			x.push_back(p_x);
			y.push_back(p_y);

		}

		winPlot.clear();
		winPlot.plot(x,y,".b2");

}




void Controlador::guardarMedidas(){


	FILE* file,*rawFile;

	char* text=(char*) malloc(4);
	sprintf(text,"%d",data_index);


	string outfile=string("laser") + string(text) + string(".dat");
	string rawOutfile=string("raw_laser") + string(text) + string(".dat");



	sprintf(logBuffer,"Medidas guardadas en archivo %s\n",outfile.c_str());
	ArLog::log(ArLog::Normal,logBuffer);

	file=ArUtil::fopen(outfile.data(),"wb");
	rawFile=ArUtil::fopen(rawOutfile.data(),"wb");
	data_index++;


	double dist,dist_raw,angle,x_r,y_r;
	vector<double> x,y;


	for(int i=0; i < distancias.size(); i++)
	{

		dist=distancias[i]*0.001;
		dist_raw=dist;

		if(dist_raw > max_laser_dist){
			dist_raw=max_laser_dist;
		}

		if(dist > max_save_dist){
			dist=max_save_dist;
		}
		angle=(medidas[i]-currentPose).getTh();

		x.push_back(dist*ArMath::cos(angle));
		y.push_back(dist*ArMath::sin(angle));

		x_r=dist_raw*ArMath::cos(angle);
		y_r=dist_raw*ArMath::sin(angle);


		fprintf(file,"i:%d\tAngulo:%.2f\t Distancia:%0.3f\t X:%0.3f\t Y:%0.3f\n",i,angle,dist,x[i],y[i]);
		fprintf(rawFile,"i:%d\tAngulo:%.2f\t Distancia:%0.3f\t X:%0.3f\t Y:%0.3f\n",i,angle,dist_raw,x_r,y_r);
	}



	fclose(file);
	fclose(rawFile);



}







void Controlador::measure(bool saveToFile)
{

    currentPose=robot->getPose();
    //currentPose.log();

    FILE* file;

    if(saveToFile)
    {
        char* text=(char*) malloc(4);
        sprintf(text,"%d",data_index);

        string outfile=string("salida") + string(text) + string(".dat");

        file=ArUtil::fopen(outfile.data(),"wb");
        data_index++;

        fprintf(file,"Posicion X:%0.2f\t Y:%0.2f Th:%0.2f\n",currentPose.getX(),currentPose.getY(),currentPose.getTh());

    }


    medidas.clear();
    distancias.clear();
    ArPose punto;
    double dist,angle;


    //Bloquear laser
    sick->lockDevice();

    vector<ArSensorReading> *readings=sick->getRawReadingsAsVector();

    int i=0;
    for(vector<ArSensorReading>::iterator it=readings->begin(); it != readings->end(); it++)
    {

        if(it->getIgnoreThisReading())
        {
            // Establezco máximo rango del láser leido desde fichero configuracion
            dist=max_laser_dist*1000;
            angle=it->getSensorTh();
            punto=ArPose(dist*ArMath::cos(angle),dist*ArMath::sin(angle),angle);
            punto=punto + currentPose;

        }
        else
        {

            punto=it->getPose();
            angle=currentPose.findAngleTo(punto);
            dist=currentPose.findDistanceTo(punto);
            punto.setTh(angle);
        }
        distancias.push_back(dist);
        medidas.push_back(punto);




        if(saveToFile)
        {
            fprintf(file,"Medida: %d\t Angulo:%.2f\t Distancia:%0.2f\t X:%0.2f\t Y:%0.2f\n",i,angle,dist,punto.getX(),punto.getY());
        }
        i++;

    }


    sick->unlockDevice();

    if(saveToFile)
    {
        fclose(file);
        ArLog::log(ArLog::Normal ,"Lecturas guardadas exitosamente\n");
    }


    //vector<ArPoseWithTime> *readings2;
//    /* Print current buffer of reading positions (maybe filtered) */
//    readings2 = sick->getCurrentBufferAsVector();
//    //printf("%d readings in current buffer\n", (int)readings2->size());


//    if(saveToFile){
//	fprintf(file,"Posicion X:%0.2f\t Y:%0.2f Th:%0.2f\n",currentPose.getX(),currentPose.getY(),currentPose.getTh());
//    }

//    for (int i=0; i < (int)readings2->size(); i++)
//    {
//	punto=readings2->at(i);

//	dist = currentPose.findDistanceTo(punto);
//	distancias.push_back(dist);
//	angle = currentPose.findAngleTo(punto);
//	punto.setTh(angle);
//	medidas.push_back(punto);
//	printf("Medida: %d\t Angulo:%.2f\t Distancia:%0.2f\t X:%0.2f\t Y:%0.2f\n",i,angle,dist,punto.getX(),punto.getY());
//	if(saveToFile){
//	    fprintf(file,"Medida: %d\t Angulo:%.2f\t Distancia:%0.2f\t X:%0.2f\t Y:%0.2f\n",i,angle,dist,punto.getX(),punto.getY());
//	}
//    }



//    double x,y;

//    //ArPose punto;

//    sick->lockDevice();

//    for(int i=-90;i<90;i++){
//	// Obtengo la medida de distancia y angulo
//	dist=sick->currentReadingPolar(i,i+1,&angle);

//	// Obtengo coordenadas del punto usando el laser como referencia
//	x=dist*ArMath::cos(angle);
//	y=dist*ArMath::sin(angle);

//	//Roto los puntos
//	ArMath::pointRotate(&x,&y,-currentPose.getTh());
//	punto.setX(x);
//	punto.setY(y);

//	punto=punto + currentPose;

//	printf("Medida: %d\t Angulo:%.2f\t Angulo:%.2f\t Distancia:%0.2f\t X:%0.2f\t Y:%0.2f\n",i,angle,angle+currentPose.getTh(),dist,punto.getX(),punto.getY());
//    }

//    sick->unlockDevice();

    //obtenerCandidatos();
    //detectarPatas();

}


//********************************************************************
// Funciones anteriores al cambio a MRPT
//*********************************************************************


ArPose Controlador::puntoMedio(ArPose p1, ArPose p2){
    ArPose p=p1+p2;

    p.setX(p.getX()/2);
    p.setY(p.getY()/2);

    return p;
}

ArPose Controlador::buscarPose(double angle)
{

    double endAngle,startAngle,w1,w2;
    ArPose pose,*anterior,*posterior;
    angle=ArMath::subAngle(angle,currentPose.getTh());

    // En el vector el ángulo va descendiendo, las medidas van de 90º a -90º respecto al laser
    // aunque puede haber anomalias y una medida posterior no ser inferior
    for(vector<ArPoseWithTime>::iterator it=medidas.begin(); it != medidas.end()-1; ++it )
    {

        endAngle=ArMath::subAngle(it->getTh(),currentPose.getTh());
        startAngle=ArMath::subAngle((it+1)->getTh(),currentPose.getTh());

        if(startAngle < endAngle){
            // Caso normal
            anterior=&*it;
            posterior=&*(it+1);

        }
        else{
            anterior=&*(it+1);
            posterior=&*it;
            endAngle=startAngle;
            startAngle=ArMath::subAngle(it->getTh(),currentPose.getTh());
        }




        if(ArMath::angleBetween(angle,startAngle,endAngle))
        {
            // Punto medio ponderado considerando la diferencia entre angulos
            w1=(endAngle-angle)/(endAngle-startAngle);
            w2=(angle-startAngle)/(endAngle-startAngle);
            pose=ArPose(anterior->getX()*w2 + posterior->getX()*w1,anterior->getY()*w2 + posterior->getY()*w1,0);

            break;
        }
    }

    return pose;

}



