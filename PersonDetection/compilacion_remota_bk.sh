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

	ssh jplata@robot "cd Eclipse; if [ ! -d $project_dir ]; then  mkdir $project_dir; fi; 
			  if [ ! -d $project_dir/Debug ]; then  mkdir $project_dir/Debug; fi;
			  if [ ! -d $project_dir/Debug/src ]; then  mkdir $project_dir/Debug/src; fi" 
	echo "Copiando archivos ini y scrip de compilacion"
	scp $ruta_origen/*.ini jplata@robot:$ruta_destino        
	scp $ruta_origen/compilacion_remota.sh jplata@robot:$ruta_destino        
	
	echo "Copiando archivos fuente y cabeceras"
	scp -r $ruta_origen/src jplata@robot:$ruta_destino        
	scp -r $ruta_origen/include jplata@robot:$ruta_destino

	echo "Copiando archivos de compilacion"
	scp $ruta_origen/Debug/makefile jplata@robot:$ruta_destino/Debug/        
	scp $ruta_origen/Debug/*.mk jplata@robot:$ruta_destino/Debug/        
	scp $ruta_origen/Debug/src/*.mk jplata@robot:$ruta_destino/Debug/src

	echo "Copiando modelo SVM"
	scp $ruta_origen/Debug/svm_model jplata@robot:$ruta_destino/Debug/  
	
	echo "Compilando"
	#ssh jplata@robot 'cd '$ruta_destino'/Debug;make clean;make'
	ssh jplata@robot 'cd '$ruta_destino'/Debug;make all'

else
	echo "Deshabilitada compilacion remota, no se haca nada"	

fi
