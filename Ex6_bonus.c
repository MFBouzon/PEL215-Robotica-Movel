/* 
Autor: Murillo Freitas Bouzon
Linguagem: C


Exercício 6 (Bonus) da disciplina PEL215 - Robótica Móvel


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
  int objX[17], objY[17];
  int cor[24][24];
  
  //definição da posição dos obstáculos
  
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
  
  
  
  int grid_obst[24][24];
  //inicialização do grid, da matriz de cor e da matriz de obstáculos
  for(int i=0;i<24;i++){
    for(int j=0;j<24;j++){
      grid_obst[i][j] = 0;
      grid[i][j] = -1;
      cor[i][j] = 0;
    }
  }
  
  //preenche os obstáculos na matriz de obstáculos
  for(int i=0;i<17;i++){
    grid_obst[objY[i]][objX[i]] = 1;
    
  }
  
  
  //definição da posição inicial e da posição objetivo
  int iniX = 0, iniY = 0, goalX = 23, goalY = 23;
  int atX = iniX, atY = iniY;
  //cálcula o f(x) na posição inicial 
  grid[iniY][iniX] = dist(iniX, iniY, goalX, goalY);
  //inicialização das variáveis para o algoritmo A*
  double minF = 10000, minX = 10000, minY = 10000, gAnt = 0;
  int i =0;
  
  //Algoritmo A*
  while(atX != goalX && atY != goalY){
      printf("%d %d\n", atX, atY);
      //descobre o f(x) dos vizinhos próximos
      for(int i = atY-1;i<=atY+1;i++){
        for(int j = atX-1; j<=atX+1;j++){
          if(i >=0 && i <24 && j>=0 && j <24){
            if(grid[i][j] < 0 && grid_obst[i][j] == 0){ 
                if(i == atY ^ j == atX)
                  grid[i][j] =  dist(j, i, goalX, goalY);
                else
                  grid[i][j] =  dist(j, i, goalX, goalY);
             }
           }
        }
      }
      
      //encontra o menor f(x) no grid
      minF = 10000, minX = 10000, minY = 10000;
      for(int i=0;i<24;i++){
        for(int j=0;j<24;j++){
            //printf(" %.2lf",grid[i][j]);
            if(cor[i][j] == 0 && grid[i][j] < minF && grid[i][j] > 0){
              cor[i][j] = 1;
              minF = grid[i][j];
              minX = j;
              minY = i;
            }
        }
        //printf("\n");
      }
      
        
      atX = minX;
      atY = minY;
      i++;
  }  
  
  
  
  while (wb_robot_step(TIME_STEP) != -1) {
      
      double min = 1000;
      int movX, movY;
      //realiza a descida de gradiente  
      for(int i = iniY-1;i<=iniY+1;i++){
        for(int j = iniX-1; j<=iniX+1;j++){
          if(i >=0 && i <24 && j>=0 && j <24 ){ 
            if(grid[i][j] >= 0 && grid[i][j] < min){
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
      //printf("%.2lf\n", min);
      //printf("%d %d\n", x, y);
      //printf("%d %d\n", movX, movY);
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
