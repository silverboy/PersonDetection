#!/bin/bash



if [ $(hostname) == "robot" ];then
	echo "No se hace nada en host robot"
	exit
fi


if [ "$1" != "disable" ];then
	echo "Compilación remota"
	
	ruta_origen=$(pwd)		
	project_dir=$(basename $ruta_origen)
	ruta_destino='Eclipse/'$project_dir		

	rsync -arzv . jplata@robot:$ruta_destino 

	echo "Compilando"
	ssh jplata@robot 'cd '$ruta_destino'/Debug;make clean;make'

	
else
	echo "Deshabilitada compilacion remota, no se haca nada"	

fi
