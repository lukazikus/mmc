//#ifndef ROTATINGMAGNETCONTROL
//#define ROTATINGMAGNETCONTROL

#include <iostream>
#include <stdlib.h>
#include <cmath>
#include <ctime>
#include <unistd.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
//using Eigen::MatrixXf;
//using Eigen::VectorXf;
using namespace Eigen;
typedef Matrix<float,8,8> Matrix8f;
typedef Matrix<float,8,1> Vector8f;

using namespace std;

#include <time.h> //for measuring time

#include "vision.h"
#include "AccelStepper.h"
#include "s826_subroutine.h"

#define PI 3.14159265

const float M_DIST = 0.075; //distance from each magnet to the workspace [m] (same for all)
unsigned const short int M_NUM = 8; //number of magnets
const short int M_AZIM[M_NUM] =     {-25,   40,   235,     90,    198,   305,      70,   166}; //azimuth position of magnets [deg] - spherical coordinates
const short int M_INCL[M_NUM] =     {115,  105,   112,     45,     45,    55,     180,   115}; //inclination position of magnets [deg]- spherical coordinates
const short int M_AXIS[2][M_NUM] ={ {70,   225,   315,    148,    265,    25,     275,   -10},
                                    {60,   145,    20,    235,    260,   225,      90,   130},
                                    }; //rotational axis position [deg]
const float M_STRN[M_NUM] = {16.612, 16.612,16.612, 16.612,16.612, 16.612,16.612, 16.612}; //magnet moment of the magnets [Am^2]
const bool M_InDIR[M_NUM] = {0, 0, 0, 0, 1, 1, 1, 0};
const float m_mag = 1e-6; //microrobot moment


void cross(float [3], float [3], float [3]);
void unitv(float [], int );
float magnitude(float [], int);
void intermediatev(float vec1[], float vec2[], float maxangle);
void findclosestangle(float vec2[], float vec1[]);

void setup_constants();
void field_force(float B[], float F[], float M_theta[], float m_pos[], float m[]);
float f_grad__norm(float dif_grad[], float parameters[], float M_theta[], float m_pos[], float m[], float Bo[3], float Fo[3]);
void localminGrad_norm(float M_thetaf[], float parameters[], float M_thetai[], float m_pos[], float m[], float Bo[3], float Fo[3]);
void sufficientlocalminGrad_norm(float M_theta[], float M_theta0[], int iter_max, float BFaccept[6], float parameters[], float m_pos[], float m[], float Bo[3], float Fo[3]);

float f_grad_hess__norm2(float f_grad[], float f_hess[][M_NUM], float M_theta[], int ret_ind, float K, float m_pos[], float m[], float Bo[3], float Fo[3]);
void localminNewton_norm2(float theta_0[], float parameters[], float theta_start[], float m_pos[], float m[], float Bo[3], float Fo[3]);
void Newtondirection(float dir[], float f_grad[], float f_hess[][M_NUM]);

void motor_setSpeedAcceleration(float vel, float accel);
void update_motordist();
int check_motorsdonemoving();
void motors_lockFunction(int magnetsbeingused[]);
void motors_unlockFunction(void);
void motors_moveFunction(float motor_increment[]);
void motors_moveToFunction(float motor_rotation[]);

void update_linepoints(int line_index,int x1,int y1,int x2,int y2 );
int * getSavedGoalPointCoor(int goalP_index);
void horizontalworkspace_calibration(void);
int init_magnetcontrol_thread(int control_method);
int stop_magnetcontrol_thread(void);
float * get_centerP_mm(void);

void test_arraypassing(float aray[]);
void temp_gobutton_input(void);
