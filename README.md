# controlAvanzadoMotor
Control discreto de un motor dc con motoreductor

Este repositorio se crea para el desarrollo del laboratorio 2 de la asignatura control avanzado

La práctica consiste en la obtención de la función de tranferencia de un motor dc con motoreductor y encoder hall
Esto se logra de dos formas:

-Sensando la corriente que consume el motor con un sensor INA226, los datos se reciben por I2C y se exportan a txt por serial usando CoolTerm
-Sensando la velocidad del motor leyendo las interrupciones generadas por los pulsos de los dos sensores hall del encoder del motor

Con los datos obtenidos se grafican las curvas de respuesta del motor, luego se aproximan las funciones de tranferencia con la herramienta system identification de Matlab

