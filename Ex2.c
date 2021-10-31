/* 
Autor: Murillo Freitas Bouzon
Linguagem: C


Exercício 2 da disciplina PEL215 - Robótica Móvel


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

//Declaração da Função delay que faz o código esperar por uma fração de tempo
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
  
  //Habilitação das tags dos sensores
  for(int i=0;i<8;i++)
      wb_distance_sensor_enable(sensf[i], TIME_STEP);
    
    
  //Habilita os motores, deixando-os livres para girar  
  wb_motor_set_position(left_motor_1, INFINITY);
  wb_motor_set_position(right_motor_1, INFINITY);
  wb_motor_set_position(left_motor_2, INFINITY);
  wb_motor_set_position(right_motor_2, INFINITY);
  
  //Declaração das variáveis utilizadas no algoritmo PID
  double position_goal =150.0, error = 0, dist = 0, 
          integral = 0, dif_error = 0, old_error = 0, old_error2 = 0, old_error3 =0, old_error4 = 0;
  
  //Define a velocidade inicial = 3.0 para os motores da esquerda e da direita
  double left_speed = 3.0;
  double right_speed = 3.0;
  
  //Declaração dos parametros Kp, Ki e Kd
  double p_gain = 0.9, i_gain = 0.000001, d_gain = 0.0002;
  
  
  
  //Variáveis para armazenar os valores dos sensores
  double sensf_value[8];
   
  //Loop principal  
  while (wb_robot_step(TIME_STEP) != -1) {
      
      //Captura os valores dos sensores
      
      for(int i=0;i<8;i++)
          sensf_value[i] = wb_distance_sensor_get_value(sensf[i]);
      
      //Captura o valor máximo entre os sensores s5, s6 e s7    
      double max_sensV = 0;
      
      for(int i=5;i<8;i++){
          if(sensf_value[i] > max_sensV)
              max_sensV = sensf_value[i];
      }
      
       
      //Algoritmo PID
      dist = 1024 - max_sensV;
      error = position_goal - dist;
      dif_error = error - old_error;
      old_error4 = old_error3;
      old_error3 = old_error2;
      old_error2 = old_error;
      old_error = error;
      //Utiliza a soma dos últimos 5 erros
      integral = error + old_error + old_error2 + old_error3 + old_error4;
      double power = p_gain*error + i_gain*integral + d_gain*dif_error;
      
      //ajusta a velocidade das rodas, somando valor de power na velocidade das rodas da direita
      left_speed = 3.0;    
      right_speed = 3.0 + power;
      
      //limita a velocidade da roda da direita entre 1.0 e 5.0
      if(right_speed < 1.0)
          right_speed = 1.0;
      if(right_speed > 5.0)
          right_speed = 5.0;
      
      
      //Se os sensores s3 ou s4 obtiverem um valor maior do que 900.0, 
      // o robô gira 90º    
      if(sensf_value[3] > 900.0 || sensf_value[4] > 900.0){
            left_speed = -6.0;  right_speed = 6.0;
            delay(700);
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
