#include "seguimiento.h"


Cluster::Cluster(clusterType type, int index_inicio):
    type(type),
    index_inicio(index_inicio) {}

int Cluster::getIndex_inicio()
{
    return index_inicio;
}

int Cluster::getNumPoint()
{
    return n_point;
}

ArPose Cluster::getPoint()
{
    return centro;
}

void Cluster::incrementar_puntos()
{
    n_point++;
}





Seguimiento::Seguimiento()
{
    pthread_mutex_init(&mutex,NULL);
    //ctor
}

Seguimiento::~Seguimiento()
{
    //dtor
}

void Seguimiento::calcularVariaciones(vector<double> distancias,vector<ArPoseWithTime> medidas)
{

    if(distancias_ant.empty())
    {

        distancias_ant=distancias;
        medidas_ant=medidas;

    }
    else
    {

        if(distancias.size() == distancias_ant.size())
        {

            pthread_mutex_lock(&mutex);

            variaciones_dist.clear();

            for(unsigned int i=0; i < distancias.size(); i++)
            {

                variaciones_dist.push_back(ArMath::fabs(distancias[i]-distancias_ant[i]));

            }
            pthread_mutex_unlock(&mutex);
            variaciones_minimizadas=minimization(variaciones_dist,3);

        }
        else{
            ArLog::log(ArLog::Normal ,"Warning: vector medidas distinto tamaño, variaciiones no calculadas\n");


        }
    }

}

void Seguimiento::actualizarMedidas(vector<double> distancias,vector<ArPoseWithTime> medidas){
    distancias_ant=distancias;
    medidas_ant=medidas;
}

void Seguimiento::printVariaciones(vector<ArPoseWithTime> medidas,ArPose goalPose)
{

    pthread_mutex_lock(&mutex);


    for(unsigned int i=0; i < variaciones_dist.size(); i++ )
    {

        sprintf(logBuffer, "Angulo anterior:%.2f actual:%.2f v:%.1f v_m:%.1f d_gP:%0.1f",medidas_ant[i].getTh(),
               medidas[i].getTh(),variaciones_dist[i],variaciones_minimizadas[i],goalPose.findDistanceTo(medidas[i]));

               ArLog::log(ArLog::Normal ,logBuffer);





    }

    pthread_mutex_unlock(&mutex);

}


void Seguimiento::agrupar()
{

    // Vaciamos vector con los grupos
    grupos.clear();

    double a(100),b(1000),umbral(50);

    // Minimo de puntos para considerar cluster
    int min_n_point(3),n_point(0),index_inicio;

    for(unsigned int i=0; i < variaciones_dist.size(); i++)
    {

        if(ArMath::fabs(variaciones_dist[i])>a && ArMath::fabs(variaciones_dist[i])<b)
        {

            if(n_point==0)
            {
                // Comienzo cluster
                index_inicio=i;
                n_point++;
            }
            else
            {
                //Hay un cluster comenzado, compruebo si pertenezco a el
                if(ArMath::fabs(variaciones_dist[i]-variaciones_dist[i-1]) < umbral )
                {
                    n_point++;
                }
                else
                {
                    // Cierro el cluster actual y



                }

            }
        }







    }
}

bool Seguimiento::calcularGlobalPerception(vector<double> distancias, vector<ArPoseWithTime> medidas,double robotAngle ){


    bool percepcion=false;
    double d_max(4000),d_min(0),angle,mod,x(0),y(0);
    double angleRange(30),sensorAngle;

    globalPerception.clear();
    globalPerceptionAngle.clear();

    for(unsigned int i=0; i < distancias.size(); i++){

        angle=medidas[i].getTh();
        sensorAngle=ArMath::fabs(ArMath::subAngle(angle,robotAngle));

        if(distancias[i] <= d_min && sensorAngle <= angleRange ){

            globalPerception.push_back(1);

            globalPerceptionAngle.push_back(angle);
            x+=ArMath::cos(angle);
            y+=ArMath::sin(angle);

        }
        else if(distancias[i] < d_max && sensorAngle <= angleRange){

            mod=(d_max-distancias[i])/(d_max-d_min);
            globalPerception.push_back(mod);
            globalPerceptionAngle.push_back(angle);
            x+=ArMath::cos(angle)*mod;
            y+=ArMath::sin(angle)*mod;
        }
    }

    if(!globalPerception.empty()){

    ArPose origen(0,0,0);

    goalAngle=origen.findAngleTo(ArPose(x,y,0));
    percepcion=true;
    }

    return percepcion;

}


bool Seguimiento::calcularVariacionGlobalPerception(vector<ArPoseWithTime> medidas,double robotAngle,ArPose goalPose ){


    bool percepcion=false;
    double vd_max(200),vd_min(50),angle,mod,x(0),y(0),vd,goalDistance,factor;
    double angleRange(180),sensorAngle;

    variaciones_dist=variaciones_minimizadas;

    globalPerception.clear();
    globalPerceptionAngle.clear();

    if(variaciones_dist.size() != medidas.size()){
        ArLog::log(ArLog::Normal ,"Warning: No se pudo calcular variacion de global"
               "perception, distinto tamaño vectores\n");

               return percepcion;
    }

    for(unsigned int i=0; i < variaciones_dist.size(); i++){

        angle=medidas[i].getTh();
        goalDistance=goalPose.findDistanceTo(medidas[i]);
        factor=calcularFactor(goalDistance);
        //factor=1;
        sensorAngle=ArMath::fabs(ArMath::subAngle(angle,robotAngle));
        vd=ArMath::fabs(variaciones_dist[i]);

        if(vd >= vd_max && sensorAngle <= angleRange && factor > 0){

            globalPerception.push_back(1*factor);

            globalPerceptionAngle.push_back(angle);
            x+=ArMath::cos(angle)*factor;
            y+=ArMath::sin(angle)*factor;

        }
        else if(vd > vd_min && sensorAngle <= angleRange && factor > 0){

            mod=(vd - vd_min)/(vd_max-vd_min);
            globalPerception.push_back(mod*factor);
            globalPerceptionAngle.push_back(angle);
            x+=ArMath::cos(angle)*mod*factor;
            y+=ArMath::sin(angle)*mod*factor;
        }
    }

    if(!globalPerception.empty()){

        ArPose origen(0,0,0);

        goalAngle=origen.findAngleTo(ArPose(x,y,0));
        percepcion=true;
    }

    return percepcion;
}

double Seguimiento::calcularFactor(double distancia){

	double d_max(800),factor;

	if( distancia > d_max){
		factor=0;
	}
	else{
		factor=1 - distancia/d_max;
	}


    return factor;
}

void Seguimiento::printGlobalPerception(){

    for(unsigned int i=0; i < globalPerception.size();i++){

        sprintf(logBuffer, "Distancia: %f  Angulo: %f\n",globalPerception[i],globalPerceptionAngle[i]);
        ArLog::log(ArLog::Normal ,logBuffer);
    }
}

double Seguimiento::getGoalAngle(){
    return goalAngle;
}

template <class T> vector<T> Seguimiento::minimization(vector<T> v, int ventana)
{

    vector<T> resultado;
    int inicio,fin,media_ventana((ventana-1)/2),n(v.size());

    T minimo;


    for(int i=0; i < n; i++)
    {

        if(i < n - media_ventana)
        {
            inicio=max(0 , i - media_ventana);
            fin=inicio + ventana - 1;
        }
        else
        {
            fin=min(n-1 , i + media_ventana);
            inicio=fin - ventana + 1;
        }

        minimo=10000;

        for(int j=inicio; j <= fin; j++ )
        {
            if(ArMath::fabs(v[j]) < ArMath::fabs(minimo))
            {

                minimo=v[j];
            }
        }

        resultado.push_back(minimo);
    }

    return resultado;
}






