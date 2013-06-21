#ifndef DETECCION_H
#define DETECCION_H

#include "Aria.h"
#include <vector>
#include <string>
#include <iostream>

#define UMBRAL_DIST 250

/*
  *Union para almacenar la distancia entre dos puntos, contiene los indices de los puntos dentro
  * del vector de candidatos y la distancia entre los puntos
  */

class Edge{

public:
    enum edgeType{L,R};
    Edge(edgeType type, int index, double distance, ArPose pose);
    double getDistance();
    int getIndex();
    ArPose getPose();
    edgeType getType();
    void print(char *logBuffer);
private:
    edgeType type;
    int index;
    double distance;
    ArPose point;
};


using namespace std;

class Deteccion
{
private:
    int a_inf,a_sup,b;
    vector<Edge> candidatos;

    vector<ArPose> personas;
    char logBuffer[50];


    int findFirstLIndex(int start);
    bool esPierna(Edge e1,Edge e2);
    bool esRobot(Edge e1,Edge e2);
    bool esEntrePiernas(Edge e1,Edge e2);


public:
    Deteccion(int a_inf,int a_sup,int b);
    void obtenerEdges(vector<double> distancias, vector<ArPoseWithTime> medidas );
    void detectarPersonas(ArPose currentPose);
    void detectarRobot(ArPose currentPose);
    void printEdges();
    void printPersonas();
    void saveEdges(int data_index);
    bool seleccionarPersona(ArPose *goalPose,ArPose currentPose);




};

#endif // DETECCION_H
