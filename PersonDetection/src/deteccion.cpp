#include "deteccion.h"

Edge::Edge(edgeType type, int index, double distance, ArPose pose):
type(type),
index(index),
distance(distance),
point(pose){}

double Edge::getDistance(){
    return distance;
}

int Edge::getIndex(){
    return index;
}

ArPose Edge::getPose(){
    return point;
}

Edge::edgeType Edge::getType(){
    return type;
}

void Edge::print(char *logBuffer){
    std::string tipos[]={"L","R"};

    sprintf(logBuffer, "Type: %s\t  d: %f\t i: %i\t P(x,y,th):",tipos[type].c_str(),distance,index);
    ArLog::log(ArLog::Normal ,logBuffer);
    point.log();

}



Deteccion::Deteccion(int a_inf, int a_sup, int b):
a_inf(a_inf),
a_sup(a_sup),
b(b)
{
}


void Deteccion::saveEdges(int data_index){

    char* text=(char*) malloc(4);
    sprintf(text,"%d",data_index);

    string outfile=string("candidatos") + string(text) + string(".dat");

    FILE* file=ArUtil::fopen(outfile.data(),"wb");
    std::string tipos[]={"L","R"};

    ArPose pose;

    sprintf(logBuffer, "Edges detectados:%u\n",(int)candidatos.size());
    ArLog::log(ArLog::Normal ,logBuffer);

    for(vector<Edge>::iterator it=candidatos.begin(); it != candidatos.end(); ++it){
	it->print(logBuffer);
	pose=it->getPose();
	fprintf(file,"Edge--type: %s\t  distancia: %f\t indice: %i\t Punto(x,y,th):%0.2f\t %0.2f\t %0.2f",
	tipos[it->getType()].c_str(),
	it->getDistance(),it->getIndex(),
	pose.getX(),pose.getY(),pose.getTh());
    }
    fclose(file);
}

void Deteccion::printEdges(){

    Edge* anterior;
    sprintf(logBuffer, "Edges detectados:%u",(int)candidatos.size());
    ArLog::log(ArLog::Normal ,logBuffer);
    for(vector<Edge>::iterator it=candidatos.begin(); it != candidatos.end(); ++it){

	if(it != candidatos.begin()){
	    sprintf(logBuffer, "\nDistancia entre %d y %d : %f\n",
	    anterior->getIndex(),
	    it->getIndex(),
	    anterior->getPose().findDistanceTo(it->getPose()));
	    ArLog::log(ArLog::Normal ,logBuffer);

	}
	it->print(logBuffer);
	anterior=&*it;
    }

}



int Deteccion::findFirstLIndex(int start){

    int first_L_index=candidatos.size();


    for(unsigned int i=start;i < candidatos.size();i++){

	if(candidatos[i].getType() == Edge::L){
	    first_L_index=i;
	    break;
	}
    }

    return first_L_index;

}

bool Deteccion::esPierna(Edge e1,Edge e2){

    bool pierna=false;

    ArPose point1 = e1.getPose();
    ArPose point2 = e2.getPose();

    double dist=point1.findDistanceTo(point2);

    if(dist < a_sup && dist > a_inf){
	pierna=true;
    }

    return pierna;
}

bool Deteccion::esRobot(Edge e1,Edge e2){

    bool robot=false;

    ArPose point1 = e1.getPose();
    ArPose point2 = e2.getPose();

    double dist=point1.findDistanceTo(point2);

    if(dist > 350 && dist < 500 ){
	robot=true;
    }

    return robot;
}


bool Deteccion::esEntrePiernas(Edge e1,Edge e2){

    bool entrePiernas=false;

    ArPose point1 = e1.getPose();
    ArPose point2 = e2.getPose();

    if(point1.findDistanceTo(point2) < b){
	entrePiernas=true;
    }

    return entrePiernas;
}

bool Deteccion::seleccionarPersona(ArPose* goalPose,ArPose currentPose){

    bool persona=false;

    if(!personas.empty()){

	*goalPose=personas[0];
	double dist=currentPose.findDistanceTo(*goalPose);

	for(unsigned int i=1;i < personas.size(); i++){
	    if(currentPose.findDistanceTo(personas[i]) < dist){
		*goalPose=personas[i];
		dist=currentPose.findDistanceTo(*goalPose);
	    }

	}
	persona=true;
    }

    return persona;
}






void Deteccion::detectarPersonas(ArPose currentPose){

    personas.clear();

    // Busco en el vector de candidatos el patrón L,R,L,R

    Edge *first_L;
    unsigned int first_L_index=findFirstLIndex(0);


    while(first_L_index + 3 < candidatos.size()){


	first_L=&candidatos[first_L_index];
	Edge *siguiente=&candidatos[first_L_index+1];

	bool encontrado=false;


	if(siguiente->getType() == Edge::R
	&& esPierna(*first_L,*siguiente)){

	    encontrado=false;
	    unsigned int second_L_index=first_L_index+2;


	    while(!encontrado && second_L_index + 1 < candidatos.size()){

		Edge *siguiente2=&candidatos[second_L_index];

		Edge *siguiente3=&candidatos[second_L_index+1];


		if(siguiente3->getType() == Edge::R && esEntrePiernas(*siguiente,*siguiente2)
		&& esPierna(*siguiente2,*siguiente3)){


		    // He obtenido una persona
		    encontrado=true;

		    ArPose point1 = siguiente->getPose();
		    ArPose point2 = siguiente2->getPose();

		    ArPose medio(point1+point2);
		    medio.setX(medio.getX()/2);
		    medio.setY(medio.getY()/2);

		    medio.setTh(currentPose.findAngleTo(medio));

		    personas.push_back(medio);

		    sprintf(logBuffer, "Persona encontrada: %d  %d\n",siguiente->getIndex(),siguiente2->getIndex());
            ArLog::log(ArLog::Normal ,logBuffer);

		    //Elimino edges implicados
		    candidatos.erase(candidatos.begin()+first_L_index);
		    candidatos.erase(candidatos.begin()+first_L_index +1);
		    candidatos.erase(candidatos.begin()+second_L_index);
		    candidatos.erase(candidatos.begin()+second_L_index+1);

		}
		else{
		    second_L_index=findFirstLIndex(second_L_index+1);

		}
	    }


	}

	if(encontrado){
	    // Busco el primer L desde donde estaba L antes
	    first_L_index=findFirstLIndex(first_L_index);
	}
	else{
	    first_L_index=findFirstLIndex(first_L_index+1);
	}

    }

}

// Busco variaciones grande en la distancia en el vector de medidas mayores de UMBRAL_DIST
void Deteccion::obtenerEdges(vector<double> distancias, vector<ArPoseWithTime> medidas){

    candidatos.clear();
    double actual, siguiente;

    for (int i=0; i < (int)distancias.size()-1; i++)
    {
	actual=distancias.at(i);
	siguiente=distancias.at(i+1);

	if(ArMath::fabs(actual-siguiente) > UMBRAL_DIST){

	    if(actual > siguiente){
		// Me quedo con siguiente que es el mas cercano
		candidatos.push_back(Edge(Edge::L,i+1,siguiente,medidas.at(i+1)));

	    }
	    else{
		// Compruebo si en la iteración anterior ya incluí este elemento
		if(!candidatos.empty()){
		    Edge e=candidatos.back();
		    // Elimino el elemento para incluirlo ahora
		    if(e.getIndex()==i){
			candidatos.pop_back();
		    }
		}
		candidatos.push_back(Edge(Edge::R,i,actual,medidas.at(i)));

	    }

	}

    }
}


void Deteccion::detectarRobot(ArPose currentPose){

    // Para simular una persona y seguirla uso un robot, implemento la detección primero
    // El tracking es equivalente para robot y persona
    // Usare el vector de personas

    personas.clear();

    // Busco en el vector de candidatos el patrón L,R

    Edge *first_L;
    unsigned int first_L_index=findFirstLIndex(0);


    while(first_L_index + 1 < candidatos.size()){


	first_L=&candidatos[first_L_index];
	Edge *siguiente=&candidatos[first_L_index+1];

	if(siguiente->getType() == Edge::R
	&& esRobot(*first_L,*siguiente)){

	    // He obtenido un robot

	    ArPose point1 = first_L->getPose();
	    ArPose point2 = siguiente->getPose();

	    ArPose medio(point1+point2);
	    medio.setX(medio.getX()/2);
	    medio.setY(medio.getY()/2);

	    medio.setTh(currentPose.findAngleTo(medio));

	    personas.push_back(medio);

	    sprintf(logBuffer, "Persona encontrada: %d  %d\n",first_L->getIndex(),siguiente->getIndex());
        ArLog::log(ArLog::Normal ,logBuffer);
	    //Elimino edges implicados
	    candidatos.erase(candidatos.begin()+first_L_index);
	    candidatos.erase(candidatos.begin()+first_L_index +1);
	}
	else{
	    first_L_index=findFirstLIndex(first_L_index+1);

	}
    }

}

void Deteccion::printPersonas(){

    sprintf(logBuffer, "Detectadas %ld personas",personas.size());
    ArLog::log(ArLog::Normal ,logBuffer);
    for(vector<ArPose>::iterator it=personas.begin(); it != personas.end(); ++it){

	it->log();
    }
}
