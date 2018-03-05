#ifndef COILFIELDCONTROL
#define COILFIELDCONTROL

#include "ConstantValue.h"
#include "s826_subroutine.h"
#include "vision.h"
#include "math_subroutine.h"

int coilCurrentClear(void);
void set_coil_current_to (int index, float d);
void set_coil_current_to_2 (float, float, float, float, float, float);
float get_coil_current(int index);
void set_field_xyz_2 (float bx, float by, float bz, float dbx, float dby, float dbz);
void resetCoils(void);
int amplifier_set_each_coil_current (float px, float nx, float py, float ny, float pz, float nz);       // set different values to 6 amplifiers


#endif
