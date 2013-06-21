#ifndef SEGUIMIENTO_H
#define SEGUIMIENTO_H

#include "Aria.h"
#include <vector>

using namespace std;

/*
  *Clase para definir una serie de puntos con caracteristicas similares
  */

class Cluster{

public:
    enum clusterType{P,N};
    Cluster(clusterType type, int index_inicio);
    void incrementar_puntos();
    int getIndex_inicio();
    ArPose getPoint();
    int getNumPoint();
    clusterType getType();
    void calcularCentro();
    //void print();
private:
    clusterType type;
    int index_inicio;
    int n_point;
    ArPose centro;
};




class Seguimiento
{
    public:
        Seguimiento();
        virtual ~Seguimiento();
        void calcularVariaciones(vector<double> distancias, vector<ArPoseWithTime> medidas);
        void actualizarMedidas(vector<double> distancias, vector<ArPoseWithTime> medidas);
        void printVariaciones(vector<ArPoseWithTime> medidas,ArPose goalPose);
        double getGoalAngle();
        bool calcularGlobalPerception(vector<double> distancias,vector<ArPoseWithTime> medidas,double robotAngle);
        bool calcularVariacionGlobalPerception(vector<ArPoseWithTime> medidas,double robotAngle,ArPose goalPose);
        void printGlobalPerception();

    protected:
    private:
        vector<Cluster> grupos;
        vector<double> globalPerception;
        vector<double> globalPerceptionAngle;
        pthread_mutex_t mutex;
        double goalAngle;
        char logBuffer[150];

        vector<ArPoseWithTime> medidas_ant;
        vector<double> distancias_ant;

        vector<double> variaciones_dist;
        vector<double> variaciones_minimizadas;
        template <class T> vector<T> minimization(vector<T> v, int ventana);
        void agrupar();
        double calcularFactor(double distancia);

};

#endif // SEGUIMIENTO_H
