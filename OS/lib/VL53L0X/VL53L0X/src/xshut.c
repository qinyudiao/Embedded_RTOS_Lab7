/*!
 * @file  xshut.c
 * @brief Implementation for interfacing xshut pins for VL53L0X for multi sensor projects.
 * ----------
 * ST VL53L0X datasheet: https://www.st.com/resource/en/datasheet/vl53l0x.pdf
 * ----------
 * For future development and updates, please follow this repository: https://github.com/ZeeLivermorium/VL53L0X_TM4C123
 * ----------
 * If you find any bug or problem, please create new issue or a pull request with a fix in the repository.
 * Or you can simply email me about the problem or bug at zeelivermorium@gmail.com
 * Much Appreciated!
 * @author Zee Livermorium and Tariq Muhanna
 * @date   Sep 19, 2018
 */

#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "VL53L0X.h"

uint8_t mask = 0x04;

void xshut_Init(void) {
    /* Port B Activation */
      SYSCTL_RCGCGPIO_R |= 0x08; // 2) activate port D
    while((SYSCTL_RCGCGPIO_R & 0x08) == 0){};    // allow time for activating
    
    /* Port D Set Up */
    GPIO_PORTD_CR_R |= 0x0C;                                // allow changes to PD2-3
    GPIO_PORTD_DIR_R |= 0x0C;                               // make PD2-3 output
    GPIO_PORTD_AMSEL_R &= ~0x0C;                           // disable analog on PD2-3
    GPIO_PORTD_PCTL_R &= ((~GPIO_PCTL_PD2_M) &             // configure PD3 as GPIO
                            // (~GPIO_PCTL_PD3_M) &
                            // (~GPIO_PCTL_PD4_M) &
                          (~GPIO_PCTL_PD3_M));              
    GPIO_PORTD_AFSEL_R  &= ~0x0C;                          // disable alt functtion on PD2-3
    GPIO_PORTD_DEN_R |= 0x0C;                               // enable digital I/O on PD2-3
    
    GPIO_PORTD_DATA_R &= ~0x0C;                             // put all sensors low
    delay(50);
    GPIO_PORTD_DATA_R |= 0x0C;                             // put all sensors high
    delay(50);
    GPIO_PORTD_DATA_R = (GPIO_PORTD_DATA_R & ~0x0C) | mask;                             // the first device
}

void xshut_Switch() {
    mask = mask << 1;			                                       // must activate devices 1 by 1
    GPIO_PORTD_DATA_R |= mask;
    delay(50);
}


