/* 
Autor: Murillo Freitas Bouzon
Linguagem: C


Exercício 6 da disciplina PEL215 - Robótica Móvel


*/


#include <webots/keyboard.h>
#include <webots/robot.h>

#include <arm.h>
#include <base.h>
#include <gripper.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TIME_STEP 32


static void step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

//Função para delay
static void passive_wait(double sec) {
  double start_time = wb_robot_get_time();
  do {
    step();
  } while (start_time + sec > wb_robot_get_time());
}

//Função para calcular a distância euclidiana
double dist(int x1,int y1, int x2, int y2){
     return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

int main(int argc, char **argv) {
  //inicialização do robô
  wb_robot_init();
  
  //inicialização das peças do robô
  base_init();
  arm_init();
  gripper_init();
  passive_wait(1.0);
  
  double grid[24][24];
  
  //definição da posição dos obstáculos
  int objX[17], objY[17];
  
  objX[0] = 10; objY[0] = 9;
  objX[1] = 10; objY[1] = 10;
  objX[2] = 11; objY[2] = 9;
  objX[3] = 11; objY[3] = 10;
  objX[4] = 12; objY[4] = 9;
  objX[5] = 12; objY[5] = 10;
  objX[6] = 11; objY[6] = 12; 
  objX[7] = 20; objY[7] = 13;
  objX[8] = 21; objY[8] = 13;
  objX[9] = 22; objY[9] = 13;
  objX[10] = 23; objY[10] = 13; 
  objX[11] = 10; objY[11] = 14;
  objX[12] = 12; objY[12] = 14;
  objX[13] = 11; objY[13] = 15;
  objX[14] = 17; objY[14] = 21;
  objX[15] = 17; objY[15] = 22;
  objX[16] = 17; objY[16] = 23;
  
  
  //inicialização do grid de obstáculos
  int grid_obst[24][24];
  for(int i=0;i<24;i++){
    for(int j=0;j<24;j++){
      grid_obst[i][j] = 0;
    }
  }
  
  //preenchimento dos obstáculos no grid
  for(int i=0;i<17;i++){
    grid_obst[objY[i]][objX[i]] = 1;
    
  }
  
  
  
  
  //definição da posição inicial e da posição objetivo
  int iniX = 0, iniY = 0, goalX = 23, goalY = 23;
  //constantes para o cálculo dos campos potenciais
  double k_att = 5.0, k_rep = 15.0, p0 = 5.0;
  
  //cálculo dos campos potenciais para cada posição do grid
  for(int i=0;i<24;i++){
    for(int j=0;j<24;j++){
        double U_att = 0.5*k_att*dist(j, i, goalX, goalY);
        grid[i][j] = U_att;
        for(int a=0;a<17;a++){
            double pq = dist(j, i, objX[a], objY[a]);
            if(pq <= p0)
              grid[i][j] += 0.5*k_rep*(((1/pq) - (1/p0))*((1/pq) - (1/p0))); 
            else
              grid[i][j] += 0;
        }
        printf(" %.2lf", grid[i][j]);
        //double U_rep;
        //double pq = dist(j, i, goalX, goalY)
    }
    printf("\n");
  }
  
  while (wb_robot_step(TIME_STEP) != -1) {
      
      //descida de gradiente
      double min = 1000;
      int movX, movY;
      for(int i = iniY-1;i<=iniY+1;i++){
        for(int j = iniX-1; j<=iniX+1;j++){
          if(i >=0 && i <24 && j>=0 && j <24){ 
            if(grid[i][j] < min){
              min = grid[i][j];
              movX = j;
              movY = i;
          
            }
           }
        }
      }
      
      //movimentação do robô baseada na descida de gradiente
      int x = -(iniX - movX);
      int y = -(iniY - movY);
      printf("%d %d\n", x, y);
      printf("%d %d\n", movX, movY);
      if(x > 0){
         base_strafe_right_increment();
         base_strafe_right_increment();
         base_strafe_right_increment();
      }  
      else if(x < 0){
         base_strafe_left_increment();
         base_strafe_left_increment();
         base_strafe_left_increment();
      }
      if(y < 0){
         base_forwards_increment();
         base_forwards_increment();
         base_forwards_increment();
      }  
      else if(y > 0){
         base_backwards_increment();
         base_backwards_increment();
         base_backwards_increment();
      }
      passive_wait(5.0);
      base_reset();
      passive_wait(2.0);
      
      iniX = movX;
      iniY = movY;
      
      //Quebra o loop se o robô chegar na posição objetivo
      if(iniX == goalX && iniY == goalY)
        break;
  }
  
  
  wb_robot_cleanup();

  return 0;
}
