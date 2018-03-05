////////////////////////////////////////////////////////////////////////////////////////
// File      : PageTwistField.c
// Function  : Tab: twisted walking
// Edited by : Omid
////////////////////////////////////////////////////////////////////////////////////////
#include "PageTwistField.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// twisting field walking variables
static bool flag_twist_field = false, twisted_walking_go = false, flag_manual_field_loop = false, manual_field_go = false;
float theta = 0.0, beta = 0.0, omega = 0.0, phi = 0.0, Bmag = 0.0; // make it global because need it when drawing the field in 3d indicator
static float bx_global = 0.0, by_global = 0.0, bz_global = 0.0, dbx_global = 0.0, dby_global = 0.0, dbz_global = 0.0;
static float bx_old = 0.0, by_old = 0.0, bz_old = 0.0;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void* circ_twisting_field_walk_thread (void*threadid)
{
    printf("--twisting field thread started\n");

    struct timeval start;
    double time_current, time_last, time_elapsed, time_out, time_out_sum, vx, vy, vz; //time in second
	  double theta_local, beta_local, phi_local, omega_local;
	  double theta_localp, beta_localp, phi_localp, omega_localp;
    double vx_old, vy_old, vz_old;
    double prec = 0.000000001; //mT
    gettimeofday(&start, NULL);
    double time_initial = (double) start.tv_sec + start.tv_usec*1e-6 ; // Initial time in seconds.

    while ( flag_twist_field )
    {

        if (twisted_walking_go)
        {

            gettimeofday(&start, NULL);
            time_last = (double) start.tv_sec + start.tv_usec*1e-6 ;


			theta_local = theta;
			beta_local = beta;
			phi_local = phi;
			omega_local = omega;


			vx = Bmag * ((cosd(theta_local)*cosd(beta_local))*(cosd(90-phi_local*0.5)*cosd(360.0*omega_local*time_elapsed)) - sind(theta_local)*(cosd(90-phi_local*0.5)*sind(360.0*omega_local*time_elapsed)) + cosd(theta_local)*sind(beta_local)*cosd(phi_local*0.5));
			vy = Bmag * ((sind(theta_local)*cosd(beta_local))*(cosd(90-phi_local*0.5)*cosd(360.0*omega_local*time_elapsed)) + cosd(theta_local)*(cosd(90-phi_local*0.5)*sind(360.0*omega_local*time_elapsed)) + sind(theta_local)*sind(beta_local)*cosd(phi_local*0.5));
			vz = Bmag * (-sind(beta_local)*(cosd(90-phi_local*0.5)*cosd(360.0*omega_local*time_elapsed)) + cosd(beta_local)*cosd(phi_local*0.5));

            set_field_xyz_2 (vx, vy, vz, 0.0, 0.0, 0.0);

            field_x = vx;
            field_y = vy;
            field_z = vz;

			/*theta_localp = theta_local;
			beta_localp = beta_local;
			phi_localp = phi_local;
			omega_localp = omega_local;*/

            time_elapsed = time_last - time_initial -time_out;
            time_out_sum = time_out;
        }
        else
        {
            gettimeofday(&start, NULL);
            time_current = (double) start.tv_sec + start.tv_usec*1e-6 ;
            time_out = time_current - time_last + time_out_sum;
        }

    }
    resetCoils();
    printf("--twisting field thread ended\n");
    return NULL;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//OY circ_twisting_field_walk_thread related functions
void init_twist_field (int d)
{
    printf("@ the Beginning of circ_twisting_field_walk_thread.\n");
    flag_twist_field = true;
    pthread_t twisting_field;
    pthread_create(&twisting_field, NULL, circ_twisting_field_walk_thread, NULL);
}

void stop_twist_field (int d)
{
    flag_twist_field = false;
}

void init_twisted_walking (int d)
{
    twisted_walking_go = true;
}

void stop_twisted_walking (int d)
{
    twisted_walking_go = false;
}

void set_theta_heading (float d)
{
    theta = d;
}
void set_beta_tild (float d)
{
    beta = d;
}
void set_ang_freq (float d)
{
    omega = d;
}
void set_ang_span (float d)
{
    phi = d;
}
void set_tfield_mag (float d)
{
    Bmag = d;
}
void set_bx_mag(float d)
{
    bx_global = d;
}
void set_by_mag(float d)
{
    by_global = d;
}
void set_bz_mag(float d)
{
    bz_global = d;
}
void set_dbx_mag(float d)
{
    dbx_global = d;
}
void set_dby_mag(float d)
{
    dby_global = d;
}
void set_dbz_mag(float d)
{
    dbz_global = d;
}





void* set_manual_field_thread (void*threadid)
{
    printf("--manual field thread started\n");
    //double data[16];
    bool flag_manualisset =true, flag_resetset=false;
    //uint aiChan = (0b1000000000000000);
    //s826_aiInit(aiChan,1);
    while ( flag_manual_field_loop )
    {

        if (manual_field_go)
        {
            if(flag_manualisset){
            set_field_xyz_2 (bx_global, by_global, bz_global, dbx_global, dby_global, dbz_global);
            bx_old=bx_global;
            by_old=by_global;
            bz_old=bz_global;
            flag_manualisset = false;
            field_x = bx_global;
            field_y = by_global;
            field_z = bz_global;
            printf("--COILS SET MANUALLY\n");
            //sprintf(fab_status, "sample text....");
  			//s826_doPin(10, 1);

////

            }
            if(fabs(bx_global-bx_old)>0.01 || fabs(by_global-by_old)>0.01 || fabs(bz_global-bz_old)>0.01){
                flag_manualisset = true;
            }
            flag_resetset = true;
        //    s826_aiRead(aiChan, data);
        //    printf("Analog input %f\n",data[15]);
        }
        else if(flag_resetset)
        {
            resetCoils();
            flag_resetset = false;
            printf("--COILS RESETTED\n");
            //s826_doPin(10, 0);
        }

    }
    resetCoils();
    printf("--manual field thread ended\n");
    return NULL;
}

// set_manual_field_thread related functions
void init_manual_field (int d)
{
    printf("@ the Beginning of set_manual_field_thread.\n");
    flag_manual_field_loop = true;
    pthread_t manual_field;
    pthread_create(&manual_field, NULL, set_manual_field_thread, NULL);
}

void stop_manual_field (int d)
{
    flag_manual_field_loop = false;
}

void init_manual_field_go (int d)
{
    manual_field_go = true;
}

void stop_manual_field_go (int d)
{
    manual_field_go = false;
}
