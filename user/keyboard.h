#ifndef _KEYBOARD_H_
#define _KEYBOARD_H_

#define ENTER_GPIO P22_3
#define UP_GPIO P33_12
#define DOWN_GPIO P33_10
#define LEFT_GPIO P33_11
#define RIGHT_GPIO P33_13

#define ENTER 0
#define UP 1
#define DOWN 2
#define LEFT 3
#define RIGHT 4

#define KEY_1 P22_2
#define KEY_2 P22_1
#define KEY_3 P22_0
#define KEY_4 P23_1

void keyboard_init();
int keyboard_scan();
void boma_init();
int boma_scan(int num);
#endif // _KEYBOARD_H_