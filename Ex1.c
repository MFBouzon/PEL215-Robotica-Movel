/* 
Autor: Murillo Freitas Bouzon
Linguagem: C


Exercício 1 da disciplina PEL215 - Robótica Móvel


*/


//Bibliotecas webots
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>

//Bibliotecas standard
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>


#define TIME_STEP 64

/*
void delay(int time_milisec){
    double currentTime, initTime, Timeleft;
    double timeValue = (double)time_milisec/1000;
    initTime = wb_robot_get_time();
    Timeleft = 0.00;
    while(Timeleft < timeValue){
        currentTime = wb_robot_get_time();
        Timeleft = currentTime - initTime;
        wb_robot_step(TIME_STEP);
    
    }
}*/


int main()
{
  //incializa o robo
  wb_robot_init();
  
  //Criação das tags para cada uma das quatros rodas do robo
  WbDeviceTag left_motor_1 = wb_robot_get_device("front left wheel");
  WbDeviceTag right_motor_1 = wb_robot_get_device("front right wheel");
  WbDeviceTag left_motor_2 = wb_robot_get_device("back left wheel");
  WbDeviceTag right_motor_2 = wb_robot_get_device("back right wheel");
  
  
  //Criação das tags para os sensores "s0" até "s7"
  WbDeviceTag sensf[8];
  sensf[0] = wb_robot_get_device("so0");
  sensf[1] = wb_robot_get_device("so1");
  sensf[2] = wb_robot_get_device("so2");
  sensf[3] = wb_robot_get_device("so3");
  sensf[4] = wb_robot_get_device("so4");
  sensf[5] = wb_robot_get_device("so5");
  sensf[6] = wb_robot_get_device("so6");
  sensf[7] = wb_robot_get_device("so7");
  
  //Habilitação das tags dos sensores
  for(int i=0;i<8;i++)
      wb_distance_sensor_enable(sensf[i], TIME_STEP);
    
    
  //Habilita os motores, deixando-os livres para girar  
  wb_motor_set_position(left_motor_1, INFINITY);
  wb_motor_set_position(right_motor_1, INFINITY);
  wb_motor_set_position(left_motor_2, INFINITY);
  wb_motor_set_position(right_motor_2, INFINITY);
  
  
  //Define a velocidade inicial = 6.0 para os motores da esquerda e da direita
  double left_speed = 6.0;
  double right_speed = 6.0;
  
  //Variáveis para armazenar os valores dos sensores
  double sensf_value[8];
   
  //Loop principal  
  while (wb_robot_step(TIME_STEP) != -1) {
      
      //Captura os valores dos sensores
      for(int i=0;i<8;i++)
          sensf_value[i] = wb_distance_sensor_get_value(sensf[i]);
         
      /*Gira as rodas da esquerda para frente com velocidade = 6.0
        e gira as rodas da direita para trás com velocidade = -6.0,
        fazendo com que o robo vire para a direita caso os sensores 
        "s0" a "s4" retornem um valor maior que 900.0.*/
      if(sensf_value[0] > 900.0 || sensf_value[1] > 900.0 || sensf_value[2] > 900.0 || sensf_value[3] > 900.0 || sensf_value[4] > 900.0){
            left_speed = 6.0;  right_speed = -6.0;
      }
      
      /*Gira as rodas da esquerda para frente com velocidade = -6.0
        e gira as rodas da direita para trás com velocidade = 6.0,
        fazendo com que o robo vire para a esquerda caso os sensores 
        "s5" a "s7" retornem um valor maior que 900.0.*/
      else if(sensf_value[5] > 900.0 || sensf_value[6] > 900.0 || sensf_value[7] > 900.0){
            left_speed = -6.0;  right_speed = 6.0;
      }
      
      //Se nenhum dos sensores obter um valor maior do que 900.0,
      // o robo anda para frente com velocidade = 6.0.
      else{left_speed = 6.0;  right_speed = 6.0;
      }  
          
          
      //Ajusta a velocidade dos motores para o robo se movimentar    
      wb_motor_set_velocity(left_motor_1, left_speed);
      wb_motor_set_velocity(left_motor_2, left_speed);
      wb_motor_set_velocity(right_motor_1, right_speed);
      wb_motor_set_velocity(right_motor_2, right_speed);    
  }
  
  //Finaliza o robo
  wb_robot_cleanup();
  
  return 0;
}
