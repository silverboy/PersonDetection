/*
 * Detector.cpp
 *
 *  Created on: 02/05/2013
 *      Author: jplata
 */

#include "Detector.h"

using namespace mrpt::poses;
using namespace mrpt::utils;

Cluster::Cluster(){
	num_puntos=0;
}


Cluster::Cluster(double contorno, double ancho,double profundidad):contorno(contorno),ancho(ancho),profundidad(profundidad){

}

void Cluster::annadirPunto(CPose2D punto){
	puntos.push_back(punto);
	num_puntos++;
}


void Cluster::clear(){
	puntos.clear();
	num_puntos=0;
}

int Cluster::getNumPuntos(){
	return num_puntos;
}

vector<CPose2D>* Cluster::getPuntos(){
	return &puntos;
}

void Cluster::print(){
	cout << "Numero de puntos: " << num_puntos << endl;
	cout << "Contorno: " << contorno << "\tAncho: " << ancho << "\tProfundidad: " << profundidad
			<<"\tCentro: ( " << centro.x() << " , " << centro.y() << " )" <<  endl;

	for(unsigned int i=0;i < puntos.size();i++){
		printf("X:%f\t Y:%f\n",puntos[i].x(),puntos[i].y());
	}

}

void Cluster::calcularAtributos(){

	// Contorno
	contorno=0;

	// Linea que une el primer y ultimo punto
	TLine2D linea(TPoint2D(puntos[0]),TPoint2D(puntos[getNumPuntos()-1]));

	double d(0),x,y;

	x=puntos.back().x();
	y=puntos.back().y();


	for(int i=0; i < puntos.size()-1; i++){

		contorno+=puntos[i].distanceTo(puntos[i+1]);
		x+=puntos[i].x();
		y+=puntos[i].y();

		if(i > 0){
			if(linea.distance(TPoint2D(puntos[i])) > d){
				d=linea.distance(TPoint2D(puntos[i]));
			}
		}
	}

	x=x/getNumPuntos();
	y=y/getNumPuntos();

	centro=CPose2D(x,y,0);

	// Profundidad
	profundidad=d;

	// Ancho
	ancho=puntos[0].distanceTo(puntos[getNumPuntos()-1]);

}

double Cluster::getAncho(){
	return ancho;
}

double Cluster::getProfundidad(){
	return profundidad;
}

double Cluster::getContorno(){
	return contorno;
}

CPose2D Cluster::getCentro(){
	return centro;
}


Detector::Detector() {
	// TODO Auto-generated constructor stub
	ASSERT_FILE_EXISTS_("../CONFIG_Measure.ini");

	CConfigFile config("../CONFIG_Measure.ini");

	max_dist=config.read_double("MEASURE_CONFIG","SAVE_MEASURE_MAX_DISTANCE",10,false);

}

Detector::Detector(double max_dist):max_dist(max_dist){
}

void Detector::setDistancias(vector<double> d){
	distancias=d;
}

void Detector::setPuntos(vector<CPose2D> p){
	puntos=p;
}

vector<CPose2D>* Detector::getPuntos(){
	return &puntos;
}

void Detector::abrirFichero(char* filename,bool filtrar_distancia){

	FILE* file=fopen(filename,"r");

	int i;
	double d,ang,x,y;


	// Si se indica filtrar distancia solo se almacenan aquellas medidas cuya
	// distancia sea inferior a max_dist
	while(!feof(file)){
		fscanf(file,"i:%d\tAngulo:%lf\t Distancia:%lf\t X:%lf\t Y:%lf\n",&i,&ang,&d,&x,&y);
		if(!filtrar_distancia || d < max_dist){
			distancias.push_back(d);
			angulos.push_back(ang);
			puntos.push_back(CPose2D(x,y,DEG2RAD(ang)));
		}

	}

	fclose(file);

}



vector<Cluster> Detector::clusterizar(float umbral,int min_puntos){

	if(distancias.empty()){
		cout << "No hay datos cargados, llame antes a abrirFichero" << endl;
		return;
	}

	vector<Cluster> piernas;

	Cluster grupo;

	for(int i=0;i < distancias.size()- min_puntos + 1;i++){

		if(distancias[i] < max_dist){


			//Inicio cluster en el punto concreto
			grupo.annadirPunto(puntos[i]);


			// Avanzo por los siguientes puntos
			while(i < distancias.size()-1
					&& distancias[i+1] < max_dist
					&& puntos[i].distanceTo(puntos[i+1]) < umbral){
				// Añadir punto a cluster
				grupo.annadirPunto(puntos[i+1]);
				i++;
			}

			// Cluster terminado, compruebo si tiene el minimo de puntos
			if(grupo.getNumPuntos() >= min_puntos){
				grupo.calcularAtributos();
				piernas.push_back(grupo);
			}

			grupo.clear();

		}


	}

	return piernas;

}


void Detector::printClusters(vector<Cluster> piernas){

	cout << "Clusteres detectados: " << piernas.size() << endl;

	for(int i=0;i < piernas.size();i++){
		piernas[i].print();
	}

}

void Detector::filtrarDatos(){
	// Eliminamos los datos 75 y 76 que son incorrectos en las medidas de entrenamiento desde el perfil 524 en adelante
	distancias.erase(distancias.begin()+75);
	distancias.erase(distancias.begin()+75);
	angulos.erase(angulos.begin()+75);
	angulos.erase(angulos.begin()+75);
	puntos.erase(puntos.begin()+75);
	puntos.erase(puntos.begin()+75);

}

