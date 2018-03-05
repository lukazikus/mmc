////////////////////////////////////////////////////////////////////////////////////////
// File      : PageGeneralControl.h
// Function  : Tab: General Control
// Edited by : Tianqi
////////////////////////////////////////////////////////////////////////////////////////
#include <pthread.h>
#include <gtk/gtk.h>
#include <stdio.h>
#include <math.h>
#include "ConstantValue.h"
#include "s826_subroutine.h"

#include "Amplifier.h"

void set_field_xyz (int, float);
void set_field_xyz_2 (float, float, float, float, float, float);
void set_field_mode_xyz (float, float, float);
void set_field_mode_angle (float, float, float);

void set_current_tab_page (int d);
