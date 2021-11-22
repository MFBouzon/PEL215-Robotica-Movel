/* 
Autor: Murillo Freitas Bouzon
Linguagem: C


Exercício 3 da disciplina PEL215 - Robótica Móvel


*/


//Bibliotecas webots
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include <webots/lidar.h>

//Bibliotecas standard
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>


#define TIME_STEP 64


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
}


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
  
  //Criação das tags para o gps, inertial unit e o lidar
  WbDeviceTag gps = wb_robot_get_device("gps");
  WbDeviceTag inertial_unit = wb_robot_get_device("inertial unit");
  WbDeviceTag lidar = wb_robot_get_device("lidar");
  
  //Habilitação das tags dos sensores
  for(int i=0;i<8;i++)
      wb_distance_sensor_enable(sensf[i], TIME_STEP);
    
  wb_gps_enable(gps, TIME_STEP);
  wb_inertial_unit_enable(inertial_unit, TIME_STEP);
  wb_lidar_enable(lidar, TIME_STEP);
  wb_lidar_enable_point_cloud(lidar);
    
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
  
  //Cria o grid e o inicializa
  int gridsize = 25;
  int base_map[gridsize][gridsize];
  int map[gridsize][gridsize];
  
  for(int i=0;i<gridsize;i++){
    for(int j=0;j<gridsize;j++){
        map[i][j] = 0;
        base_map[i][j] = 0;
    }
  }
  
  //Arquivo de saída
  FILE *f = fopen("slam.txt", "w");
  
  int lastx=0, lasty=0;  
  //Loop principal  
  while (wb_robot_step(TIME_STEP) != -1) {
      
      //Captura e normaliza os valores do gps
      const double *gps_values = wb_gps_get_values(gps);
      
      int x = ((gps_values[0]+5)/10)*gridsize;
      int y = ((gps_values[2]+5)/10)*gridsize;
      
      
      //Escreve os valores do momento anterior no grid atual
      for(int i=0;i<gridsize;i++){
        for(int j=0;j<gridsize;j++){
          map[i][j] = base_map[i][j];
            
        }
      }
      
      map[lasty][lastx] = base_map[lasty][lastx];
      
      lastx = x;
      lasty = y;
      
      
      printf("x: %d normalized y: %d\n", x, y);
      
      //Captura o angulo pelo o sensor de unidade de inercia
      const double *inertial_unit_values = wb_inertial_unit_get_roll_pitch_yaw(inertial_unit);
      printf("Angle: %lf\n", inertial_unit_values[2]);
      
      double angle = inertial_unit_values[2];
      
      //Captura o valor do sensor lidar
      const float *range_image = wb_lidar_get_range_image(lidar);
      printf("%lf\n", range_image[0]);
      printf("%d\n", (int)((range_image[0]/3)*gridsize));
      
      
      
      //Verifica o angulo para verificar a direção do ponto capturado pelo sensor
      int y_new=0;
      
      int x_new=0;
      
      if(angle > 1.1 && angle < 2.1)
      {
         if(range_image[0] > 0)
           y_new = y - range_image[0];
         if(y_new < 0)
           y_new = 0;
     
         x_new = x;
         if(x_new < 0)
           x_new = 0;
         if(x_new > gridsize)
           x_new = gridsize;
          
          
      }
      
      else if(angle < -1.1 && angle > - 2.1)
      {
         if(range_image[0] > 0)
           y_new = y + range_image[0];
         if(y_new < 0)
           y_new = 0;
     
         x_new = x;
          
      }
      
      else if(angle > -0.7 && angle <  0.7)
      {
         if(range_image[0] > 0)
           x_new = x + range_image[0];
         if(x_new > gridsize)
           x_new = gridsize;
     
         y_new = y;
          
      }
      
      else if(angle > 2.4 || angle <  -2.4)
      {
         if(range_image[0] > 0)
           x_new = x - range_image[0];
         if(x_new < 0)
           x_new = 0;
     
         y_new = y;
          
      }
      else{
        x_new = -1;
        y_new = -1;
      
      }
      
      
      //Realiza a localização, destacando o ponto capturado pelo sensor no grid do mapeamento
      printf("x: %d y: %d\n", x_new, y_new);
      if(x_new >= 0 && y_new >= 0){
        map[y_new][x_new] += 10;
        if(map[y_new][x_new] > 150)
           map[y_new][x_new]  = 150;
      }
      
      
      //Destaca a localização do robo no mapa
      map[y][x] = -1;
      
      //Exporta o grid
      for(int i=0;i<gridsize;i++){
        for(int j=0;j<gridsize;j++){
          if(map[i][j] != -1)
            base_map[i][j] = map[i][j];
          //printf("%d ", map[i][j]);
          fprintf(f, "%d\n", map[i][j]);
            
        }
        printf("\n");
      }
      
      
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
  fclose(f);
  //Finaliza o robo
  wb_robot_cleanup();
  
  return 0;
}
