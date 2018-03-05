////////////////////////////////////////////////////////////////////////////////////////
// File      : Amplifier.c
// Function  : control magnetic fields using s826
// Edited by :
////////////////////////////////////////////////////////////////////////////////////////
#include "Amplifier.h"

/////////////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////////////
static float coil_current_voltage[3] = {0,0,0};  //static float coil_current_x = 0, coil_current_y = 0, coil_current_z = 0;

/////////////////////////////////////////////////////////////////////////////////////////
// Functions
////////////////////////////////////////////////////////////////////////////////////////
int coilCurrentClear(void) {
    uint rangeCode = 2;   // 2: -5 ~ +5 V.
    coil_current_voltage[0] = 0;
    coil_current_voltage[1] = 0;
    coil_current_voltage[2] = 0;
    s826_aoPin( 0 , rangeCode, 0);
    s826_aoPin( 1 , rangeCode, 0);
    s826_aoPin( 2 , rangeCode, 0);
    s826_aoPin( 3 , rangeCode, 0);
    s826_aoPin( 4 , rangeCode, 0);
    s826_aoPin( 5 , rangeCode, 0);
}

void set_coil_current_to (int index, float d) {
//  s826_aoPin(aoPin, 2, signal pin voltage output)
//  * amplifiers running in current mode
//  aoPin     Cable #              Coil           Comp. mT/V
//    2          1              Z - Mid top         5.003
//    5          2              Z - Mid bot         4.433
//    4          3              Y - Inn right       5.143
//    1          4              Y - Inn left        5.024
//    3          5              X - Out right       4.879
//    0          6              X - Out left        5.003
//    6          7
    coil_current_voltage[index] = d;
    switch (index) {
        case 0: // X
            s826_aoPin(0, 2, d/5.003);
            s826_aoPin(3, 2, d/4.879);
            break;
        case 1: // Y
            s826_aoPin(1, 2, d/5.024);
            s826_aoPin(4, 2, d/5.143);
            break;
        case 2: // Z
            s826_aoPin(2, 2, d/5.024);
            s826_aoPin(5, 2, d/4.433);
            break;
    }
}

void set_coil_current_to_2 (float bx, float by, float bz, float dbx, float dby, float dbz) {
    s826_aoPin(3, 2, bx*0.18754719386756); //x-right
 	s826_aoPin(0, 2, bx*0.18754719386756); //x-left

 	s826_aoPin(1, 2, by*0.19230420653328); //y-left
 	s826_aoPin(4, 2, by*0.19230420653328); //y-right

 	s826_aoPin(2, 2, bz*0.191929507962791); //z-up
 	s826_aoPin(5, 2, bz*0.191929507962791); //z-down
}

void resetCoils(void) {
    set_field_xyz_2(0.0,0.0,0.0,0.0,0.0,0.0);
}

/* Get current coil current control voltage value */
float get_coil_current(int index) {
    float returnValue = 0;
    if ( (index > 2) || (index < 0) )
        printf("Error in get_coil_current.\n");
    else
        returnValue = coil_current_voltage[index];
    return returnValue;
}

// set different values to 6 amplifiers
int amplifier_set_each_coil_current (float px, float nx, float py, float ny, float pz, float nz) {
    s826_aoPin(3, 2, nx*0.18754719386756); //x-right
 	s826_aoPin(0, 2, px*0.18754719386756); //x-left

 	s826_aoPin(1, 2, ny*0.19230420653328); //y-left
 	s826_aoPin(4, 2, py*0.19230420653328); //y-right

 	s826_aoPin(2, 2, pz*0.191929507962791); //z-up
 	s826_aoPin(5, 2, nz*0.191929507962791); //z-down
    return 1;
}
