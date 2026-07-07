#include "zf_common_headfile.h"
#include "keyboard.h"

void keyboard_init(void)
{
    gpio_init(ENTER, GPI, 0, GPI_FLOATING_IN);
    gpio_init(UP, GPI, 0, GPI_FLOATING_IN);
    gpio_init(DOWN, GPI, 0, GPI_FLOATING_IN);
    gpio_init(LEFT, GPI, 0, GPI_FLOATING_IN);
    gpio_init(RIGHT, GPI, 0, GPI_FLOATING_IN);
}

int keyboard_scan()
{

    if (gpio_get_level(ENTER_GPIO) == 0)
    {

        return ENTER;
    }

    /* if(gpio_get(ESC_GPIO) == 0){

        return ESC;
    } */

    if (gpio_get_level(UP_GPIO) == 0)
    {

        return UP;
    }

    if (gpio_get_level(DOWN_GPIO) == 0)
    {

        return DOWN;
    }

    if (gpio_get_level(LEFT_GPIO) == 0)
    {

        return LEFT;
    }

    if (gpio_get_level(RIGHT_GPIO) == 0)
    {

        return RIGHT;
    }
    return -1;
}

void boma_init(void)
{
    gpio_init(KEY_1, GPI, 0, GPI_FLOATING_IN);
    gpio_init(KEY_2, GPI, 0, GPI_FLOATING_IN);
    gpio_init(KEY_3, GPI, 0, GPI_FLOATING_IN);
    gpio_init(KEY_4, GPI, 0, GPI_FLOATING_IN);
}

int boma_scan(int num){

    bool key_1 = gpio_get_level(KEY_1);
    bool key_2 = gpio_get_level(KEY_2);
    bool key_3 = gpio_get_level(KEY_3);
    bool key_4 = gpio_get_level(KEY_4);
    switch(num){
        case 1:
            return key_1;
        case 2:
            return key_2;
        case 3:
            return key_3;
        case 4:
            return key_4;
    }
}