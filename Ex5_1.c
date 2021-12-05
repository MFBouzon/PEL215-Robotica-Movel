/* 
Autor: Murillo Freitas Bouzon
Linguagem: C


Exercício 5.1 da disciplina PEL215 - Robótica Móvel


*/

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/supervisor.h>
#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define PI 3.14159265358979323846

static WbDeviceTag left_motor, right_motor;

//Função Delay
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


int main(int argc, char **argv)
{
  //inicialização do robô
  wb_robot_init();
  
  //inicialização das rodas e dos motores
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  
  //captura as transformações de translação do nó do robô
  WbNodeRef robot_node = wb_supervisor_node_get_self();
  if (robot_node == NULL) {
    fprintf(stderr, "No DEF MY_ROBOT node found in the current world file\n");
    exit(1);
  }
  WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
   
  
  double VR;  // Velocidade da roda direita (VR)
  double VL;  // Velocidade da roda esquerda (VL)
  double R;  // Raio da circunferencia que o rob� deve fazer
  double d = 0.057/2.0; // metade da dist�ncia do eixo do rob� em metros
  
  int v = 0;
  while (wb_robot_step(TIME_STEP) != -1) {
  
  /* COLOQUE SEU CODIGO AQUI */  
    
    //captura a posição atual do robô
    const double *pos = wb_supervisor_field_get_sf_vec3f(trans_field);
    //printf("Actual position: %lf %lf\n", pos[0], pos[2]);
     
    //cálculo da velocidade das rodas para o primeiro círculo
    if(v == 0){
      R = 0.5;
      double R1 = R+d;
      double R2 = R-d;
      
      
      VR = (0.5*PI*R1);
      VL = (0.5*PI*R2);
      
      printf("VR: %lf - VL: %lf\n", VR, VL);
      wb_motor_set_velocity(left_motor, VL);
      wb_motor_set_velocity(right_motor, VR);
      v = 1;
      //delay(10000);
    }
    //cálculo da velocidade das rodas para o segundo círculo
    else if(v == 1){
      if(pos[0] > -0.46009 && pos[2] > 0.0714){
        R = 0.5*2;
        double R1 = R+d;
        double R2 = R-d;
        
        
        VR = (0.5*PI*R1);
        VL = (0.5*PI*R2);
        
        printf("VR: %lf - VL: %lf\n", VR, VL);
        wb_motor_set_velocity(left_motor, VL);
        wb_motor_set_velocity(right_motor, VR);
        v = 2;
        delay(1000);
      }
     }
    //cálculo da velocidade das rodas para o terceiro círculo
    else if(v == 2){
      if(pos[0] > -0.46009 && pos[2] > 0.0714){
        R = 0.5*3;
        double R1 = R+d;
        double R2 = R-d;
        
        
        VR = (0.5*PI*R1);
        VL = (0.5*PI*R2);
        
        printf("VR: %lf - VL: %lf\n", VR, VL);
        wb_motor_set_velocity(left_motor, VL);
        wb_motor_set_velocity(right_motor, VR);
        v = 3;
        delay(100);
      }
     }
     //cálculo da velocidade das rodas para o último círculo
     else if(v == 3){
      if(pos[0] > -0.46009 && pos[2] > 0.069){
        R = 0.5*4;
        double R1 = R+d;
        double R2 = R-d;
        
        
        VR = (0.5*PI*R1);
        VL = (0.5*PI*R2);
        
        printf("VR: %lf - VL: %lf\n", VR, VL);
        wb_motor_set_velocity(left_motor, VL);
        wb_motor_set_velocity(right_motor, VR);
        v = 4;
      }
    }
    
  };
  wb_robot_cleanup();
  return 0;
}