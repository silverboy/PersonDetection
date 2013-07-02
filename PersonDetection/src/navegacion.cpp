#include "controlador.h"



// Adds key handler callbacks for controlling the gripper
class ControlHandler
{
    Controlador* driver;
    ArRobot* robot;
    bool firstCall;
    bool keyPressedBefore;
    ArFunctorC<ControlHandler> myStartCB;
    ArFunctorC<ControlHandler> myStopCB;
    ArFunctorC<ControlHandler> myPrintCB;
    ArFunctorC<ControlHandler> myHandlerCB;

    void showMenu(){

    	ArLog::log(ArLog::Normal ,"Posicion actual robot:\n");
        robot->getPose().log();

    	cout << "Press the key for your option:" << endl << endl;
    	cout << " flecha arriba     : avanzar" << endl;
    	cout << " flecha abajo      : retroceder" << endl;
    	cout << " flecha izq / drch : girar" << endl;
    	cout << " p   : start" << endl;
    	cout << " s   : stop" << endl;
    	cout << " g   : guardar Medidas" << endl;
    	cout << " c   : guardar Medidas continuamente" << endl;
    	cout << " x   : Quit" << endl;

    }


public:
    ControlHandler(Controlador* d,ArRobot* robot):
        driver(d),
        robot(robot),
        firstCall(true),
        keyPressedBefore(false),
        myStartCB(this, &ControlHandler::start),
        myStopCB(this, &ControlHandler::stop),
        myPrintCB(this, &ControlHandler::print),
        myHandlerCB(this,&ControlHandler::manualControlHandler)
    {
    }

    ArFunctorC<ControlHandler>* getFunctor(){
    	return &myHandlerCB;
    }

    void addKeyHandlers()
    {

        ArKeyHandler *keyHandler = Aria::getKeyHandler();
        if(keyHandler == NULL)
        {
            keyHandler = new ArKeyHandler();
            Aria::setKeyHandler(keyHandler);
            robot->attachKeyHandler(keyHandler);
        }
        //keyHandler->addKeyHandler('g', &myGoCB);
        //keyHandler->addKeyHandler('c', &myGoHomeCB);
        keyHandler->addKeyHandler('p', &myStartCB);
        keyHandler->addKeyHandler('s', &myStopCB);
        keyHandler->addKeyHandler('m', &myPrintCB);
    }

    void manualControlHandler(){


    	if(firstCall){
    		firstCall=false;
    		showMenu();
    	}

    	if(keyPressedBefore && !driver->estaEjecutando()){
    		keyPressedBefore=false;
    		showMenu();
    	}

    	if(mrpt::system::os::kbhit() ){
    		char c = mrpt::system::os::getch();
    		keyPressedBefore=true;


    		switch(c){

    		case '\033':

    			c=mrpt::system::os::getch(); // skip the [
    			cout << "Siguiente caracter" << (int)c << endl;
    			double v;
    			switch(mrpt::system::os::getch()) { // the real value
    			case 'A':
    				// code for arrow up
    				v=robot->getVel();
    				robot->setVel(v+50);
    				break;
    			case 'B':
    				// code for arrow down
    				v=robot->getVel();
    				robot->setVel(v-50);
    				break;
    			case 'C':
    				// code for arrow right
    				v=robot->getRotVel();
    				robot->setRotVel(v-5);
    				break;
    			case 'D':
    				// code for arrow left
    				v=robot->getRotVel();
    				robot->setRotVel(v+5);
    				break;
    			}
    			break;

    			case ' ':
    				robot->stop();
    				break;


    		case 'x':
    			//Aria::shutdown();
    			robot->stopRunning();
    			break;

    		case 'c':
    			guardarContinuo();
    			break;

    		case 'p':
    			start();
    			break;

    		case 'g':
    			guardar();
    			break;

    		case 's':
    			stop();
    			break;
    		}

    	}
    }

    void start()
    {

        driver->setAction(Controlador::DETECTAR);
        driver->reanudar();
    }

    void guardar(){
    	driver->setAction(Controlador::GUARDARMEDIDAS);
    	driver->reanudar();
    }

    void stop()
    {
        driver->suspender();
    }

    void print()
    {
        driver->mostrarVariaciones();
    }
    void guardarContinuo(){
    	driver->setAction(Controlador::GUARDARCONTINUO);
    	driver->reanudar();
    }


};// Fin clase ControlHandler




int main(int argc, char *argv[])
{
    // Initialize location of Aria, Arnl and their args.
    Aria::init();
    Arnl::init();


    // The robot object
    ArRobot robot;

    // Parse the command line arguments.
    ArArgumentParser parser(&argc, argv);

    // Read data_index if exists
    int data_index;
    bool exist_data_index;
    parser.checkParameterArgumentInteger("dataIndex",&data_index,&exist_data_index);


    // Load default arguments for this computer (from /etc/Aria.args, environment
    // variables, and other places)
    parser.loadDefaultArguments();


    // Object that connects to the robot or simulator using program options
    ArRobotConnector robotConnector(&parser, &robot);


    // set up a gyro
    ArAnalogGyro gyro(&robot);

    ArLog::init(ArLog::File,ArLog::Normal,"run.log",false,true,true);

    // Parse arguments for the simple connector.
    if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
    {
        ArLog::log(ArLog::Normal, "\nUsage: %s -map mapfilename\n", argv[0]);
        Aria::logOptions();
        Aria::exit(1);
    }

    // Collision avoidance actions at higher priority
    ArActionAvoidFront avoidAction("avoid",200);
    ArActionLimiterForwards limiterAction("speed limiter near", 150, 500, 150);
    ArActionLimiterForwards limiterFarAction("speed limiter far", 300, 1100, 400);
    ArActionLimiterTableSensor tableLimiterAction;
    //robot.addAction(&tableLimiterAction, 100);
    //robot.addAction(&avoidAction,100);
    //robot.addAction(&limiterAction, 95);
    //robot.addAction(&limiterFarAction, 90);

    // Goto action at lower priority
    ArActionGoto gotoPoseAction("goto");
    //robot.addAction(&gotoPoseAction, 50);
    gotoPoseAction.setCloseDist(750);

    // Stop action at lower priority, so the robot stops if it has no goal
    ArActionStop stopAction("stop");
    //robot.addAction(&stopAction, 40);




    // Connect the robot
    if (!robotConnector.connectRobot())
    {
        ArLog::log(ArLog::Normal, "Could not connect to robot... exiting");
        Aria::exit(3);
    }

    if(!robot.isConnected())
    {
        ArLog::log(ArLog::Terse, "Internal error: robot connector succeeded but ArRobot::isConnected() is false!");
    }




    // Connector for laser rangefinders
    ArLaserConnector laserConnector(&parser, &robot, &robotConnector);


    // Sonar, must be added to the robot, used by teleoperation and wander to
    // detect obstacles, and for localization if SONARNL
    ArSonarDevice sonarDev;

    // Add the sonar to the robot
    robot.addRangeDevice(&sonarDev);

    // Start the robot thread.
    robot.runAsync(true);


    // Connect to the laser(s) if lasers were configured in this robot's parameter
    // file or on the command line, and run laser processing thread if applicable
    // for that laser class.  For the purposes of this demo, add all
    // possible lasers to ArRobot's list rather than just the ones that were
    // connected by this call so when you enter laser mode, you
    // can then interactively choose which laser to use from that list of all
    // lasers mentioned in robot parameters and on command line. Normally,
    // only connected lasers are put in ArRobot's list.
    if (!laserConnector.connectLasers(
                false,  // continue after connection failures
                true,  // add only connected lasers to ArRobot
                true    // add all lasers to ArRobot
            ))
    {
        ArLog::log(ArLog::Normal ,"Could not connect to lasers... exiting\n");
        Aria::exit(2);
    }


    // Puntero a laser
    ArSick* sick=(ArSick*)robot.findLaser(1);



    // Conectamos el laser
    sick->asyncConnect();

    //Esperamos a que esté encendido
    while(!sick->isConnected())
    {
        ArUtil::sleep(100);
    }

    ArLog::log(ArLog::Normal ,"Laser conectado\n");




    // Set up things so data can be logged (only do it with the laser
    // since it can overrun a 9600 serial connection which the sonar is
    // more likely to have)
    ArDataLogger dataLogger(&robot);
    dataLogger.addToConfig(Aria::getConfig());

    // add our logging to the config
    //ArLog::addToConfig(Aria::getConfig());




    // Set up a class that'll put the movement and gyro parameters into ArConfig
    ArRobotConfig robotConfig(&robot);
    robotConfig.addAnalogGyro(&gyro);



    // Add additional range devices to the robot and path planning task.
    // IRs if the robot has them.
    robot.lock();
    ArIRs irs;
    robot.addRangeDevice(&irs);

    // Bumpers.
    ArBumpers bumpers;
    robot.addRangeDevice(&bumpers);


    // cause the sonar to turn off automatically
    // when the robot is stopped, and turn it back on when commands to move
    // are sent. (Note, this should not be done if you need the sonar
    // data to localize, or for other purposes while stopped)
    ArSonarAutoDisabler sonarAutoDisabler(&robot);



    // Read in parameter files.
    Aria::getConfig()->useArgumentParser(&parser);
    if (!Aria::getConfig()->parseFile(Arnl::getTypicalParamFileName()))
    {
        ArLog::log(ArLog::Normal, "Trouble loading configuration file, exiting");
        Aria::exit(5);
    }




    //Configuracion del laser
    sick->setMinDistBetweenCurrent(0);

    robot.enableMotors();
    robot.setAbsoluteMaxTransVel(1000);

    /* Finally, get ready to run the robot: */
    robot.unlock();



    Controlador driver(&robot);

    if(exist_data_index){
    	driver.setDataIndex(data_index);
    }

    driver.runAsync();


    ControlHandler handler(&driver,&robot);
    // Use manual key handler
    //handler.addKeyHandlers(&robot);
    robot.addSensorInterpTask("ManualKeyHandler",50,handler.getFunctor());
    ArLog::log(ArLog::Normal ,"Añadido manejador teclado\n");












//    double x,y,dist,angle;

//    ArPose punto;
//    ArPose origen=robot.getPose();

//    sick->lockDevice();

//    for(int i=-90;i<90;i++){
//	// Obtengo la medida de distancia y angulo
//	dist=sick->currentReadingPolar(i,i+1,&angle);

//	// Obtengo coordenadas del punto usando el laser como referencia
//	x=dist*ArMath::cos(angle);
//	y=dist*ArMath::sin(angle);

//	//Roto los puntos
//	ArMath::pointRotate(&x,&y,-origen.getTh());
//	punto.setX(x);
//	punto.setY(y);

//	punto=punto + origen;

//	printf("Medida: %d\t Angulo:%.2f\t Angulo:%.2f\t Distancia:%0.2f\t X:%0.2f\t Y:%0.2f\n",i,angle,angle+origen.getTh(),dist,punto.getX(),punto.getY());
//    }
//    printf("Medidas adquiridas\n");

//    sick->unlockDevice();


    robot.waitForRunExit();


    //ArUtil::sleep(10000);


    return 0;

}
