#include "AutoFabrication.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// magnet feedback control variables
bool flag_test90 = 0, bending_go = 0;
float kp = 0.1, ki = 1.0, destination_angle = -75, control_P = 0, control_I = 0;
extern int counter;
extern float m_a;
bool reversefield = false;
float torque_angle = 90.0;
bool bending_done = false;

// rotational field variables
bool flag_rotational_field = false, rotationalfield_go = false;
float fab_amp = 0.0, fab_fre = 0.0;

// temperature control variables
bool flag_temp_control = false;
float destination_temp = 0;
bool temp_go = false;
float temp_kp = 1.0/250.0, temp_ki = 1.0/25000.0;
int temp_ind = 0;
float current_temp;
bool temp_done = false;

// motor control variables
bool stepper_on = false, runCalled=true, initSetup=false;
//const double res[] = {0.177879461046043, 0.0889397305230215, 0.0444698652615108, 0.0222349326307554, 0.0111174663153777}; // [OLD FEEDER] Linear extension resolution (mm) per step of stepper
const double res[] = {-0.0692418332755,-0.034620917127,-0.017310457585,-0.008655229771,-0.004327613907}; // [NEW FEEDER]  Linear extension resolution (mm) per step of stepper
int m = 5, stepSize=5;

// automatic feeding variables
bool flag_auto_feeding = false, feeding_go = false;
float feeding_distance = 0.0, feeding_speed = 0.0;
bool feeding_done = false;

// manual feeding variables
float feeding_increments = 0.0;
bool flag_manual_feeding = false, flag_increment = false, flag_decrement = false, flag_setSpeed = false;

// automatic fabrication variables
bool flag_auto_fab = false;
extern float microrobot_shape[51];
extern int line_number;
char fab_status[] = "Click 'Read' to read shape.";
char fab_time[] = "00:00";

// make an "S" variables
bool flag_make_s = false;
float time_s;
float radius_s;

//field control
float factor_x = 5.0964, factor_y = 4.999, factor_z = 5.1677;
float field_x, field_y, field_z, field_mag, field_angle;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Pins
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// pin assignments for motor driver
const int enablePin = 0;
const int MS1 = 1;
const int MS2 = 2;
const int MS3 = 3;
const int stepPin = 4;
const int dirPin = 5;

AccelStepper Stepper1 (1, stepPin, dirPin); //make Stepper1 object

// magnet feedback control thread
void* magnet_90test_thread (void*threadid) {
    printf("--magnet control thread started\n");
    int counterP = -1, counterC;
    float mangle, fangle_real, fangle, f1, f2;
    float des, dif;
    set_field_xyz(1, 0.0);
    FILE *magnetangle=fopen("magnetangle.txt","w");
    float mangle_history[20]; // storing historic magnet angles
    float mangle_error[20]; //storing historic errors
    //float total_error;

    while(flag_test90)
    {
        mangle = m_a;
        des = destination_angle;
        //total_error = 0.0;

        if ( counterP!=counterC )
        {
            if ( counterC%60 == 0 )
            {
                for (int i=0; i<20; i++) // update the historic magnet angles and errors
                {
                    mangle_history[i] = mangle_history[i+1];
                    mangle_error[i] = fabs(des-mangle_history[i]);
                    //total_error = total_error + mangle_error[i];
                }
                mangle_history[19] = mangle;
                mangle_error[19] = fabs(des-mangle_history[19]);
                //total_error = total_error + mangle_error[9];
                fprintf(magnetangle, "%.4f\n", mangle); //write the magnet angle into file
            }

            counterP = counterC;

            if (bending_go)
            {
                if ( (fabs(mangle_history[0] - mangle_history[19]) < 1.5) && (mangle_error[19] < 1.0) )
                {
                    bending_done = true;
                }
                else
                    bending_done = false;

                if (reversefield)  fangle = magnetAngleChange(mangle,  torque_angle);
                else               fangle = magnetAngleChange(mangle, -1*torque_angle);

                dif = mangle-des;

                if ( fabs(dif) > 180.0 )
                {
                    dif = fabs(dif)/dif*(-360.0)+mangle-des;
                }

                control_P = kp/90.0 * dif;
                control_I = ki/90.0 * dif/60.0 + control_I;

                if (control_P + control_I < 0)
                {
                    fangle = magnetAngleChange( 2*mangle-fangle, 0);
                    fangle_real = magnet2field_angle(fangle);
                    f1 = -1*(control_P+control_I)*cos(fangle_real*PI/180.0);
                    f2 = -1*(control_P+control_I)*sin(fangle_real*PI/180.0);
                }
                else
                {
                    fangle_real = magnet2field_angle(fangle);
                    f1 = (control_P+control_I) * cos(fangle_real*PI/180.0);
                    f2 = (control_P+control_I) * sin(fangle_real*PI/180.0);
                }

                if ( pow(f1, 2) + pow(f2, 2) > pow(14.0, 2) )
                {
                    f1 = f1 * 14.0/sqrt( pow(f1, 2) + pow(f2, 2) );
                    f2 = f2 * 14.0/sqrt( pow(f1, 2) + pow(f2, 2) );
                }

                set_field_xyz(0, f1);
                set_field_xyz(2, f2);
            }
        }
    }
    fclose(magnetangle);
    coilCurrentClear();
    printf("--magnet control thread ended\n");
    return NULL;
}

//rotational field thread
void* rotational_field_thread (void*threadid)
{
    printf("--rotational field thread started--\n");

    struct timeval start;
    double time_current, time_last, time_elapsed, time_out, time_out_sum, v_x, v_y, v_z; //time in second
    gettimeofday(&start, NULL);
    double time_initial = (double) start.tv_sec + start.tv_usec*1e-6 ; // Initial time in seconds.
    while( flag_rotational_field )
    {
        if ( rotationalfield_go )
        {
            gettimeofday(&start, NULL);
            time_last = (double) start.tv_sec + start.tv_usec*1e-6 ;
            time_elapsed = time_last - time_initial -time_out;
            v_x = fab_amp * sin( 2.0 * M_PI * fab_fre * time_elapsed );
            //v_y = -0.6;
            v_z = fab_amp * cos( 2.0 * M_PI * fab_fre * time_elapsed + M_PI);

            set_coil_current_to( 0, v_x );
            set_coil_current_to( 1, 0.0 );
            set_coil_current_to( 2, v_z );
            time_out_sum = time_out;
        }
        else
        {
            gettimeofday(&start, NULL);
            time_current = (double) start.tv_sec + start.tv_usec*1e-6 ;
            time_out = time_current - time_last + time_out_sum;
        }
    }
    coilCurrentClear();
    printf("--rotational field thread ended--\n");
    return NULL;
}
// temperature control thread
void* temp_control_thread (void*threadid)
{
    printf("--temperature control thread started\n");
    double V[16];
    float error_temp;
    float control_P_temp = 0.0, control_I_temp = 0.0, aoV = 0.0; //********changed here
    float start_point = 0.0;

    uint aiChan = (1<<15);
    uint airangecode = 1;
    s826_aiInit(aiChan, airangecode);
    FILE *heating=fopen("heating_temeprature.txt","w");

    while(flag_temp_control)
    {
        s826_aiRead(aiChan, V);
        current_temp = (V[15]-1.2654)/0.0051;

        if ( temp_go )
        {
            if ( current_temp < (destination_temp-8.0) )
            {
                aoV = 0.5;
				temp_done = false;
            }
            else if ( current_temp > (destination_temp+8.0) )
            {
                aoV = 0.0;
				temp_done = false;
            }
            else
            {
                temp_done = true;
                error_temp = destination_temp - current_temp;
                control_P_temp = temp_kp * error_temp;
                control_I_temp = temp_ki * error_temp + control_I_temp;
                start_point = destination_temp /800.0;
                aoV = start_point + control_P_temp + control_I_temp;
                if (aoV > 0.5) aoV = 0.5;
                if (aoV < 0.0) aoV = 0.0;
            }

            s826_aoPin(3, 0, aoV);

            // printf(" T = %.2f, ", current_temp);
            // printf("V = %.3f\n", aoV);
            // fprintf(heating, "%.4f\n", current_temp); //write the current temperature into file
        }
        else
        {
			temp_done = false;
            aoV = 0.0;
            s826_aoPin(3, 0, 0.0);
            // printf(" T = %.2f\n", current_temp);
            // fprintf(magnetangle, "%.4f\n", current_temp); //write the magnet angle into file
        }
        waitUsePeriodicTimer(1e5); //1e5:10hz  5e4:20hz  2e4:50hz 1e4:100hz
    }
    s826_aoPin(3, 0, 0.0);
    fclose(heating);
    printf("--temperature control thread ended\n");
    return NULL;
}

// set stepper stepping mode (1,2,3,4,5 for full,1/2,1/4,1/8, and 1/16 division)
void setStep(int mode) {
  switch (mode) {
    case 1:
      s826_doPin(MS1, 0);
      s826_doPin(MS2, 0);
      s826_doPin(MS3, 0);
      break;
    case 2:
      s826_doPin(MS1, 1);
      s826_doPin(MS2, 0);
      s826_doPin(MS3, 0);
      break;
    case 3:
      s826_doPin(MS1, 0);
      s826_doPin(MS2, 1);
      s826_doPin(MS3, 0);
      break;
    case 4:
      s826_doPin(MS1, 1);
      s826_doPin(MS2, 1);
      s826_doPin(MS3, 0);
      break;
    case 5:
      s826_doPin(MS1, 1);
      s826_doPin(MS2, 1);
      s826_doPin(MS3, 1);
      break;
    default:
      s826_doPin(MS1, 0);
      s826_doPin(MS2, 0);
      s826_doPin(MS3, 0);
      break;
  }
  m=mode-1;
}

void manualOverride ()
{
    Stepper1.setMaxSpeed(10/fabs(res[m])); //default

    while (flag_manual_feeding){
        if(flag_increment)
        {
            Stepper1.move(-feeding_increments/res[m]);
            flag_increment = false;
        }else if(flag_decrement)
        {
            Stepper1.move(feeding_increments/res[m]);
            flag_decrement = false;
        }
        Stepper1.run();
        if(!flag_setSpeed)
        {
            Stepper1.setMaxSpeed(feeding_speed/fabs(res[m]));
            flag_setSpeed = true;
        }
    }
}

// automatic feeding thread
void* auto_feeding_thread (void*threadid)
{
    printf("--automatic feeding thread started\n");
    Stepper1.setAcceleration(9000);
    while(flag_auto_feeding)
    {
        if (!initSetup) { //Initial Motor Setup

            setStep(stepSize); //Set mode from UI
            //printf("->Step Size Set to: %d\n",stepSize);

            if (!stepper_on) { //stepper enabling check
                s826_doPin(enablePin,0); //low to activate driver
                printf("---Stepper engaged\n");
                stepper_on=true;
            }
            Stepper1.setMaxSpeed(feeding_speed/fabs(res[m]));
            //printf("->Stepper speed set to: %.1f mm/s\n",feeding_speed);
            initSetup=true;
        }

        if (feeding_go)
        {
            if(runCalled) {
                    Stepper1.setMaxSpeed(feeding_speed/fabs(res[m]));
                    Stepper1.move(-feeding_distance/res[m]);
                    runCalled = false;
                    feeding_done = false;
                    //printf("False.\n");
            }
            Stepper1.run();
            if (Stepper1.distanceToGo() == 0 && !feeding_done)
            {
                feeding_done = true;
                //printf("True.\n");
            }

        }else{

            //printf("distance to go: %.2f\n",fabs(res[m])*Stepper1.distanceToGo());
            runCalled=true;
            if (flag_manual_feeding)
            {
                manualOverride();
            }
        }
    }
    s826_doPin(enablePin,1); //disengage stepper
    stepper_on=false; //flag
    printf("---stepper disengaged\n");
    initSetup=false; //flag
    runCalled=true;
    printf("--automatic feeding thread ended\n");
    return NULL;
}

// automatic fabrication thread
void* auto_fab_thread (void*threadid)
{
    printf("-automatic fabrication thread started\n");
	if (!flag_test90)
        init_test90();
    if (!flag_temp_control)
        init_temp_control();
    if (!flag_auto_feeding)
        init_auto_feeding();

	float this_length = 0, this_angle = 0;
    int fab_time_sec;
    int fab_time_min;

    struct timeval start;
    gettimeofday(&start, NULL);
    int start_sec = start.tv_sec; // Initial time in seconds.
    int time_sec;
    int time_min;

	for (int i=0; i<line_number; i++)
	{
		if (i%2 == 0) // this is a length
		{
			this_length = microrobot_shape[i];
			set_feeding_distance(this_length);
			set_feeding_speed(1.0);
			feeding_done = false;
			set_feeding_go(1);
			sprintf(fab_status, "Feeding -> %.2f", feeding_distance);

			while(!feeding_done)
			{
			    gettimeofday(&start, NULL);
                int time_sec = (start.tv_sec - start_sec)%60; // time in seconds.
                int time_min = (start.tv_sec - start_sec)/60; // time in minutes;
                sprintf(fab_time, "%02d:%02d", time_min, time_sec);

                if (!flag_auto_fab) break;
            }
            if (!flag_auto_fab) break;

			feeding_done = false;
			set_feeding_go(0);
		}
		else // this is an angle
		{
			// heat up
			set_destination_temp(115.0);
			temp_done = false;
			set_temp_go(1);
			sprintf(fab_status, "Heating up....");

			while(!temp_done)
            {
			    gettimeofday(&start, NULL);
                int time_sec = (start.tv_sec - start_sec)%60; // time in seconds.
                int time_min = (start.tv_sec - start_sec)/60; // time in minutes;
                sprintf(fab_time, "%02d:%02d", time_min, time_sec);

                if (!flag_auto_fab) break;
            }
            if (!flag_auto_fab) break;

			temp_done = false;

			// bend
			this_angle = this_angle + microrobot_shape[i];
			set_destination_angle(this_angle);
			bending_done = false;
			set_bending_go(1);
			sprintf(fab_status, "Bending -> %.2f", destination_angle);
			usleep(5e5);

			while(!bending_done)
            {
			    gettimeofday(&start, NULL);
                int time_sec = (start.tv_sec - start_sec)%60; // time in seconds.
                int time_min = (start.tv_sec - start_sec)/60; // time in minutes;
                sprintf(fab_time, "%02d:%02d", time_min, time_sec);

                if (!flag_auto_fab) break;
            }
            if (!flag_auto_fab) break;

			bending_done = false;
			// cool down
			set_destination_temp(80.0);
			temp_done = false;
			set_temp_go(1);
			sprintf(fab_status, "Cooling down....");

			while(!temp_done)
			{
			    gettimeofday(&start, NULL);
                int time_sec = (start.tv_sec - start_sec)%60; // time in seconds.
                int time_min = (start.tv_sec - start_sec)/60; // time in minutes;
                sprintf(fab_time, "%02d:%02d", time_min, time_sec);

                if (!flag_auto_fab) break;
            }
            if (!flag_auto_fab) break;

			temp_done = false;
			set_bending_go(0);
		}
	}

	if (!flag_auto_fab) sprintf(fab_status, "Fabrication terminated!");
	else sprintf(fab_status, "Well done!");

    stop_test90();
    stop_auto_feeding();

    printf("-automatic fabrication thread ended\n");
    return NULL;
}

// make an "S" thread
void* make_s_thread (void*threadid)
{
	printf("--make an S thread started\n");
	bool half_done = false; // if the first half of fabrication is done
	// parameters for extruder
	set_feeding_distance( 2.0 * M_PI * radius_s );
	set_feeding_speed( feeding_distance / time_s );
	// parameters for rotational field
	float fab_amp = 14.0;
	float fab_fre = 1.0/time_s;
	// set some time variables
	struct timeval start;
    double time_current, time_elapsed, f_x, f_y, f_z;
    gettimeofday(&start, NULL);
    double time_initial = (double) start.tv_sec + start.tv_usec*1e-6 ; // Initial time in seconds(double).
	// start extruding
	stop_manual_feeding();
	set_feeding_go (0);
	stop_auto_feeding();
	init_auto_feeding();
	set_feeding_go (1);
	// and start applying the rotational field
	while ( flag_make_s )
	{
	    gettimeofday(&start, NULL);
        time_current = (double) start.tv_sec + start.tv_usec*1e-6 ;
        time_elapsed = time_current - time_initial;

        int time_sec = ((int)start.tv_sec - (int)time_initial)%60; // time in seconds.
        int time_min = ((int)start.tv_sec - (int)time_initial)/60; // time in minutes;
        sprintf(fab_time, "%02d:%02d", time_min, time_sec);

		if (half_done)
		{
            f_x = fab_amp * sin( -2.0 * M_PI * fab_fre * time_elapsed );
            //v_y = -0.6;
            f_z = fab_amp * cos( 2.0 * M_PI * fab_fre * time_elapsed + M_PI);

			if ( time_elapsed > time_s )    flag_make_s = false;
		}
		else
		{
			f_x = fab_amp * sin( 2.0 * M_PI * fab_fre * time_elapsed );
            //v_y = -0.6;
            f_z = fab_amp * cos( 2.0 * M_PI * fab_fre * time_elapsed + M_PI);

			if ( time_elapsed > time_s/2.0 )    half_done = true;
		}
        set_field_xyz( 0, f_x );
        set_field_xyz( 1, 0.0 );
        set_field_xyz( 2, f_z );
	}
	set_feeding_go (0);
	stop_auto_feeding();
    printf("--make an S thread ended\n");
    return NULL;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// magnet feedback control thread related functions
int init_test90(void) // initialize magnet feedback control thread
{
    //printf("@ the Beginning of test90_thread.\n");
    flag_test90 = true;
	pthread_t test90_thread;
    pthread_create(&test90_thread, NULL, magnet_90test_thread, NULL);  // magnet feedback control thread
    return 1;
}

int stop_test90(void) // stop magnet feedback control thread
{
	flag_test90 = false;
	return 1;
}

void set_kp(float d)
{
	kp = d; //for setting kp, the gain of Proportional
}

void set_ki(float d)
{
	ki = d; //for setting ki, the gain of Integral
}

void set_destination_angle(float d)
{
	destination_angle = d; // for setting the destination angle of the magnet
}

double magnet2field_angle(double anglein) // map an angle from magnet angle coordinate to field angle angle coordinate
{
    double angleout;
    if ( anglein<=-90)
        angleout = anglein+270;
    else
        angleout = anglein-90;

    return angleout;
}

float magnetAngleChange( float m, float d) // change from magnet angle to field angle, according to torque_angle
{
    float fangle = m + d;

    if (fangle>180)   fangle = fangle-360;
    if (fangle<=-180) fangle = fangle+360;
    return fangle;
}

void set_reversefield(int d) // reverse the field angle
{
    reversefield = d;
}

void set_bending_go(int d)
{
    bending_go = d;
}

void set_torque_angle(float d) // set the angle between magnet and field
{
    torque_angle = d;
}

// rotational field thread related functions
int init_rotational_field(void) // initiate rotational field thread
{
    //printf("@ the Beginning of rotational_field_thread.\n");
    flag_rotational_field = true;
	pthread_t rotational_field;
    pthread_create(&rotational_field, NULL, rotational_field_thread, NULL);  //start swimmer thread
    return 1;
}

int stop_rotational_field(void) // stop rotational field thread
{
	flag_rotational_field = false;
	return 1;
}

void set_fab_amp(float d) // set amplitude of the rotational field
{
	fab_amp = d;
}

void set_fab_fre(float d) // set frequency of the rotational field
{
	fab_fre = d;
}

void set_rotationalfield_go(int d)
{
    rotationalfield_go = d;
}

// temperature control thread related functions

int init_temp_control(void) // initialize temperature control thread
{
    //printf("@ the Beginning of temp_control_thread.\n");
    flag_temp_control = true;
	pthread_t temp_control;
    pthread_create(&temp_control, NULL, temp_control_thread, NULL);  //start thread
    return 1;
}

int stop_temp_control(void) // stop temperature control thread
{
    flag_temp_control = false;
    return 1;
}

void set_destination_temp( float d ) // set destination temperature
{
    destination_temp = d;
}

void set_temp_go( int d ) // temperature go/stop
{
    temp_go = d;
}

// automatic feeding thread related functions

int init_auto_feeding( void ) // initiate automatic feeding thread
{
    //printf("@ the Beginning of automatic_feeding_thread.\n");
    flag_auto_feeding = true;
	pthread_t auto_feeding;
    pthread_create(&auto_feeding, NULL, auto_feeding_thread, NULL);  //start thread
    return 1;
}

int stop_auto_feeding( void ) // stop automatic feeding thread
{
    flag_auto_feeding = false;
    return 1;
}

void set_feeding_distance ( float d ) // set feeding distance
{
    feeding_distance = d;
}

void set_feeding_speed ( float d ) // set feeding speed
{
    feeding_speed = d;
    flag_setSpeed = false;
}

void set_feeding_go ( int d ) //set feeding_go flag
{
    feeding_go = d;
}

void set_feeding_increments ( float d ) //set feeding increments (manual override)
{
    feeding_increments = d;
}

int init_manual_feeding( void ) // initiates manual feeding
{
    flag_manual_feeding = true;
    return 1;
}

int stop_manual_feeding( void ) // stops manual feeding
{
    flag_manual_feeding = false;
    return 0;
}

void feederIncrement ( void ) // handles incrementing the feeder
{
    flag_increment = true;
    //printf("feederIncrement Inside!");
}

void feederDecrement ( void ) // handles decrementing the feeder
{
    flag_decrement = true;
    //printf("feederDecrement Inside!");
}

// automatic fabrication thread related functions

int init_auto_fab( void ) // initiate automatic fabrication thread
{
    //printf("@ the Beginning of automatic_fabrication_thread.\n");
    flag_auto_fab = true;
	pthread_t auto_fab;
    pthread_create(&auto_fab, NULL, auto_fab_thread, NULL);  //start thread
    return 1;
}

int stop_auto_fab( void ) // stop automatic fabrication thread
{
    flag_auto_fab = false;
    return 1;
}

// make an "S" thread related functions

int init_make_s( void ) // initiate make an "S" thread
{
    //printf("@ the Beginning of make_s_thread.\n");
    flag_make_s = true;
	pthread_t make_s;
    pthread_create(&make_s, NULL, make_s_thread, NULL);  //start thread
    return 1;
}

int stop_make_s( void ) // initiate make an "S" thread
{
    //printf("@ the Ending of make_s_thread.\n");
    flag_make_s = false;
    return 1;
}

void set_time_s ( float d ) // set total time of making an "S"
{
    time_s = d;
}

void set_radius_s ( float d ) // set the radius of the "S"
{
    radius_s = d;
}

// field control
void set_factor(int index)
{
	switch (index) // 1:3D coil system; 0:2D coil system
	{
		case 1: factor_x = 5.0964; factor_y = 4.999; factor_z = 5.1677;
				break;
		case 0: factor_x = 5.2264; factor_z = 5.8538;
				break;
	}
}

void set_field_xyz (int index, float d) // 'd' no larger than 14mT
{
        switch (index) // 0: x  1: y  2: z
    {
        case 0: field_x = d; set_coil_current_to (0, d/factor_x);
                break;
        case 1: field_y = d; set_coil_current_to (1, d/factor_y);
                break;
        case 2: field_z = d; set_coil_current_to (2, d/factor_z);
                break;
    }
	field_mag = sqrt( pow(field_x,2) + pow(field_y,2) + pow(field_z,2) ) ;
	field_angle = atan2(field_z, field_x) * 180.0/M_PI;
}

void set_field_polar (float magnitude, float angle)
{
	field_mag = magnitude; field_angle = angle;
	set_field_xyz( 0, magnitude * cos(angle * M_PI/180.0) );
	set_field_xyz( 1, 0 );
	set_field_xyz( 0, magnitude * sin(angle * M_PI/180.0) );
}
