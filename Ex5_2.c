/* 
Autor: Murillo Freitas Bouzon
Linguagem: C


Exercício 5.2 da disciplina PEL215 - Robótica Móvel


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

//Função para delay
static void passive_wait(double sec) {
  double start_time = wb_robot_get_time();
  do {
    step();
  } while (start_time + sec > wb_robot_get_time());
}

//Função que realiza o movimento hexagonal do robô
static void hexagonal() {
   
   base_forwards_increment();
   base_forwards_increment();
   base_forwards_increment();
   base_forwards_increment();
   passive_wait(4.0);
   base_strafe_right_increment();
   base_strafe_right_increment();
   base_strafe_right_increment();
   base_strafe_right_increment();
   passive_wait(8.0);
   base_backwards_increment();
   base_backwards_increment();
   base_backwards_increment();
   base_backwards_increment();
   passive_wait(7.0);
   base_backwards_increment();
   base_backwards_increment();
   base_backwards_increment();
   base_backwards_increment();
   passive_wait(7.0);
   base_strafe_left_increment();
   base_strafe_left_increment();
   base_strafe_left_increment();
   base_strafe_left_increment();
   passive_wait(8.0);
   base_strafe_left_increment();
   base_strafe_left_increment();
   base_strafe_left_increment();
   base_strafe_left_increment();
   passive_wait(8.0);
   base_forwards_increment();
   base_forwards_increment();
   base_forwards_increment();
   base_forwards_increment();
   passive_wait(7.0);
   base_forwards_increment();
   base_forwards_increment();
   base_forwards_increment();
   base_forwards_increment();
   passive_wait(7.0);
   base_strafe_right_increment();
   base_strafe_right_increment();
   base_strafe_right_increment();
   base_strafe_right_increment();
   passive_wait(4.0);
   base_backwards_increment();
   base_backwards_increment();
   base_backwards_increment();
   base_backwards_increment();
}

int main(int argc, char **argv) {
  //inicialização do robô
  wb_robot_init();
  
  //inicialização das peças do robô
  base_init();
  arm_init();
  gripper_init();
  passive_wait(1.0);
  
  hexagonal();

  wb_robot_cleanup();

  return 0;
}
