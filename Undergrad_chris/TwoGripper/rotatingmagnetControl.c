#include "rotatingmagnetControl.h"

//initialize empty matrices - will be filled by magnetLocAxisConst
float M_LOC[M_NUM][3];
float M_ROT[M_NUM][3];
float c1[M_NUM][3];
float c2[M_NUM][3];

int motor_parm[8][3] = {  // {step, dir, enable}
			{18,19,20},
			{9, 10, 11},
			{17, 16, 15},
			{5, 4, 3},
			{14, 13, 12},
			{22, 23, 20}, //shares enable pin with 1!
			{2,1,0},
			{6, 7, 8}};
bool end_runmotorsthread = false;
float motor_steps2go[8]={0,0,0,0,0,0,0,0};
float motor_positionsrmc[8]={0,0,0,0,0,0,0,0};

//AccelStepper parameters
float max_speed = 100; //500;
float max_accel = 1300; //4500;
AccelStepper stepper1(1, motor_parm[1-1][0], motor_parm[1-1][1]);
AccelStepper stepper2(1, motor_parm[2-1][0], motor_parm[2-1][1]);
AccelStepper stepper3(1, motor_parm[3-1][0], motor_parm[3-1][1]);
AccelStepper stepper4(1, motor_parm[4-1][0], motor_parm[4-1][1]);
AccelStepper stepper5(1, motor_parm[5-1][0], motor_parm[5-1][1]);
AccelStepper stepper6(1, motor_parm[6-1][0], motor_parm[6-1][1]);
AccelStepper stepper7(1, motor_parm[7-1][0], motor_parm[7-1][1]);
AccelStepper stepper8(1, motor_parm[8-1][0], motor_parm[8-1][1]);

bool robot_inposition = false;
bool complete_fieldcorrection = true;
int num_motorsturning = 0;
bool firsttime_through = true;

static uint hor_waypoint_flag = 0;
static uint hor_force_flag = 0;
static uint ver_waypoint_flag=0;
bool userinput_click =0;
bool userinput_saved =0;

int workspace_center[3] = {320,240,0}; //in pixels
float mm_per_pixel = 0.0158; //0.0141;//0.0662;
float calibration_length = 6.0; //millimeters

bool calibrate_distances=0;


bool go_var = false;


//End points for line plotting
int l_x[12]={0,0,0,0,0,0,0,0,0,0,0,0};
int l_y[12]={0,0,0,0,0,0,0,0,0,0,0,0};
int l_R[6]={255,0,0,255,0,255};
int l_G[6]={0,255,0,0,255,255};
int l_B[6]={0,0,255,255,255,0};

void cross(float vec3[3], float vec1[3], float vec2[3])
{
    //vec3 is an empty matrix that is filled with cross product: v1 x v2

    vec3[0]=vec1[1]*vec2[2]-vec2[1]*vec1[2];
    vec3[1]=vec2[0]*vec1[2]-vec1[0]*vec2[2];
    vec3[2]=vec1[0]*vec2[1]-vec1[1]*vec2[0];
}

void unitv(float vec1[], int length)
{
    //convert vec1 to the a unit vector in the same direction
    //length is the number of elements in vec1

    float temp_vec[length];
    float norm = 0.0;

    for (int count=0; count < length; count++)
    {
        norm += vec1[count]*vec1[count];
        temp_vec[count]=vec1[count];
    }

    norm=sqrt(norm);

    for (int count=0; count < length; count++)
    {
        vec1[count]=temp_vec[count]/norm;
    }
}

float magnitude(float vec1[], int length)
{
    float mag = 0.0;
        for (int count=0; count < length; count++)
    {
        mag += vec1[count]*vec1[count];
    }
    mag=sqrt(mag);
    return mag;
}

void intermediatev(float vec1[], float vec2[], float maxangle)
{
    //vec1 desired vector
    //vec2 current vector

    float vec2o[3], vec1o[3];
    vec1o[0]=vec1[0]; vec1o[1]=vec1[1]; vec1o[2]=vec1[2];
    vec2o[0]=vec2[0]; vec2o[1]=vec2[1]; vec2o[2]=vec2[2];
    float angle;

    unitv(vec1,3);
    unitv(vec2,3);

    angle = acos ( vec1[0]*vec2[0] + vec1[1]*vec2[1] +  vec1[2]*vec2[2] );

    if (angle > maxangle)
    {
        float ratio=0.5;
        int c=0;

        while ( (angle<maxangle*0.75 || angle>maxangle) && c<200)
        {
            for(int xyz=0; xyz<3; xyz++)
            {
                vec1[xyz] = ratio * (vec1o[xyz] - vec2o[xyz]) + vec2o[xyz];
            }
            unitv(vec1,3);

            angle = acos ( vec1[0]*vec2[0] + vec1[1]*vec2[1] +  vec1[2]*vec2[2] );

            if (angle < maxangle*0.75)
                ratio=ratio+ratio*0.5;
            else
                ratio=ratio-ratio*0.5;
        }

        if (c>=200)
          printf("Intermediate vector could not be found\n");



    }
}

void findclosestangle(float vec2[], float vec1[])
{
    //vec2 is the final set of angles
    //vec1 is the initial set of angles

    float diff;

    for(int count=0; count<8; count++)
    {
        diff = abs( vec2[count] - vec1[count] );

        while (true)
        {
            if ( abs( vec2[count] + 2*PI - vec1[count] ) < diff)
            {
                vec2[count] = vec2[count] + 2*PI;
                diff = abs( vec2[count] - vec1[count] );
            }

            else if ( abs( vec2[count] - 2*PI - vec1[count] ) < diff)
            {
                vec2[count] = vec2[count] - 2*PI;
                diff = abs( vec2[count] - vec1[count] );
            }
            else
                break;
        }

    }

}

////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////// Functions for Magnet Angle Calculation ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////


void setup_constants()
{
     float norm_angle;
     float initial_direction[3];

     //Parameters for Constants
     //float M_ROTdotidir; //always equals zero
     float cross_MROT_idir[3];

    for (int count=0; count<M_NUM; count++)
     {
         //cartesian locations - M_LOC
         M_LOC[count][0] = M_DIST*sin(M_INCL[count]*PI/180)*cos(M_AZIM[count]*PI/180);
         M_LOC[count][1] = M_DIST*sin(M_INCL[count]*PI/180)*sin(M_AZIM[count]*PI/180);
         M_LOC[count][2] = M_DIST*cos(M_INCL[count]*PI/180);

         //rotational vectors - M_ROT
         M_ROT[count][0]= sin(M_AXIS[1][count]*PI/180)*cos(M_AXIS[0][count]*PI/180);
         M_ROT[count][1]= sin(M_AXIS[1][count]*PI/180)*sin(M_AXIS[0][count]*PI/180);
         M_ROT[count][2]= cos(M_AXIS[1][count]*PI/180);

	 if (M_InDIR[count] == 0)
         	norm_angle=M_AXIS[0][count]-90;
	 else
		norm_angle=M_AXIS[0][count]+90;

         initial_direction[0]= cos(norm_angle*PI/180);
         initial_direction[1]= sin(norm_angle*PI/180);
         initial_direction[2]= 0;

        //normM_LOC -> initial_direction

         // constants - c1, c2 and c3
         cross(cross_MROT_idir,M_ROT[count],initial_direction);
         //M_ROTdotidir=M_ROT[count][0]*initial_direction[0] + M_ROT[count][1]*initial_direction[1] + M_ROT[count][2]*initial_direction[2];

         for (int xyz=0; xyz<3; xyz++)
         {
             c1[count][xyz] = M_STRN[count]*( initial_direction[xyz]);
             c2[count][xyz] = M_STRN[count]*( cross_MROT_idir[xyz] );
         }

     }
}

void field_force(float B[3], float F[3], float M_theta[],float m_pos[], float m[])
{
    B[0]=0; B[1]=0; B[2]=0;
    float grad[3][3]; grad[0][0]=0; grad[0][1]=0; grad[0][2]=0; grad[1][0]=0; grad[1][1]=0; grad[1][2]=0; grad[2][0]=0; grad[2][1]=0; grad[2][2]=0;
    float M[3];
    float r[3];
    float r_mag;
    float Mdotr;

    float K;

     for (int count=0; count<M_NUM; count++)
     {
         for (int xyz=0; xyz<3; xyz++)
         {
             M[xyz] = c1[count][xyz]*cos(M_theta[count]) + c2[count][xyz]*sin(M_theta[count]);
             r[xyz] = -M_LOC[count][xyz] + m_pos[xyz];
         }

         r_mag = magnitude(r,3);
         unitv(r,3);
         Mdotr= M[0]*r[0] + M[1]*r[1] + M[2]*r[2];

         K = 3.0*(1.0e-7) / pow(r_mag, 4.0);

         for (int xyz=0; xyz<3; xyz++)
         {
             B[xyz] = B[xyz] + (1e-7) / ( pow(r_mag, 3.0) ) * ( 3.0 * (Mdotr)*r[xyz] - M[xyz] );

             for (int xyz2=0; xyz2<3; xyz2++)
             {
                 // xyz = row and xyz2 = col
                 if (xyz == xyz2)
                 {
                     grad[xyz][xyz2]= grad[xyz][xyz2] + K * ( r[xyz]*M[xyz2] + r[xyz2]*M[xyz] - ( 5.0 * r[xyz]*r[xyz2] - 1.0 )*Mdotr  );
                 }
                 else
                 {
                     grad[xyz][xyz2]= grad[xyz][xyz2] + K * ( r[xyz]*M[xyz2] + r[xyz2]*M[xyz] - ( 5.0 * r[xyz]*r[xyz2] )*Mdotr  );
                 }
             }
         }
     }

     for(int xyz=0; xyz<3; xyz++)
     {
         F[xyz] = grad[xyz][0]*m[0] + grad[xyz][1]*m[1] + grad[xyz][2]*m[2];
     }
}

float f_grad__norm(float dif_grad[], float parameters[], float M_theta[], float m_pos[], float m[], float Bo[3], float Fo[3])
{
    float dif;

    float B[3], F[3];
    field_force(B,F,M_theta,m_pos, m);

    float difB, difF;
    float BminusBo[3], FminusFo[3];

    float mult;
    mult = parameters[3];

    for (int xyz=0; xyz<3; xyz++)
    {
        BminusBo[xyz] = B[xyz] - Bo[xyz];
        FminusFo[xyz] = F[xyz] - Fo[xyz];
    }
    difB = magnitude(BminusBo,3);
    difF = magnitude(FminusFo,3);

    float multB = 1/(M_NUM*4.77e-3);
    float multF = mult*1/(M_NUM*8.82e-8);

    dif = multB*difB + multF*difF;

    float M[3], r[3], r_mag, Mdotr;
    float dif_gradB, dif_gradF;
    float grad[3][3];


    for (int count=0; count<M_NUM; count++)
    {
        dif_gradB=0; dif_gradF=0;
        grad[0][0]=0; grad[0][1]=0; grad[0][2]=0; grad[1][0]=0; grad[1][1]=0; grad[1][2]=0; grad[2][0]=0; grad[2][1]=0; grad[2][2]=0;

        for (int xyz=0; xyz<3; xyz++)
        {
             M[xyz] = -c1[count][xyz]*sin(M_theta[count]) + c2[count][xyz]*cos(M_theta[count]);
             r[xyz] = -M_LOC[count][xyz] + m_pos[xyz];
        }

        r_mag = magnitude(r,3);
        unitv(r,3);
        Mdotr= M[0]*r[0] + M[1]*r[1] + M[2]*r[2];

        for (int xyz=0; xyz<3; xyz++)
        {
            dif_gradB = dif_gradB + BminusBo[xyz]*(3*Mdotr*r[xyz]-M[xyz]);

            for (int xyz2=0; xyz2<3; xyz2++)
            {
                 // xyz = row and xyz2 = col
                 if (xyz == xyz2)
                 {
                     grad[xyz][xyz2]= grad[xyz][xyz2] + ( r[xyz]*M[xyz2] + r[xyz2]*M[xyz] - ( 5.0 * r[xyz]*r[xyz2] - 1.0 )*Mdotr  );
                 }
                 else
                 {
                     grad[xyz][xyz2]= grad[xyz][xyz2] + ( r[xyz]*M[xyz2] + r[xyz2]*M[xyz] - ( 5.0 * r[xyz]*r[xyz2] )*Mdotr  );
                 }
             }

        }
        dif_gradB = dif_gradB * multB * 1e-7 / ( difB * pow(r_mag, 3.0) );

        for(int xyz=0; xyz<3; xyz++)
        {
            dif_gradF = dif_gradF + FminusFo[xyz] * ( grad[xyz][0]*m[0] + grad[xyz][1]*m[1] + grad[xyz][2]*m[2]);
        }
        dif_gradF = dif_gradF * multF * 3 * 1e-7 / (difF * pow(r_mag, 4.0) );

        dif_grad[count] = dif_gradB + dif_gradF;

    }

    return dif;

}

void localminGrad_norm(float M_thetaf[], float parameters[], float M_thetai[], float m_pos[], float m[], float Bo[3], float Fo[3])
{
    float alpha = parameters[0];
    float min_funChange = parameters[1];

    float last_theta[M_NUM];
    for (int count=0; count<M_NUM; count++)
    {
        M_thetaf[count]=M_thetai[count];
        last_theta[count]=M_thetaf[count];
    }

    float funcval_grad[M_NUM];
    float funcval = f_grad__norm(funcval_grad, parameters,M_thetaf, m_pos, m, Bo, Fo);

    float last_funcval = funcval;


    for (int j=0; j<10000; j++)
    {
        for (int count=0; count<M_NUM; count++)
        {
            M_thetaf[count]=M_thetaf[count] - alpha*funcval_grad[count];
        }

        funcval = f_grad__norm(funcval_grad, parameters,M_thetaf, m_pos, m, Bo, Fo);

//        cout << alpha*max(funcval_grad[0],funcval_grad[M_NUM-1]) << "      "<< alpha*min(funcval_grad[0],funcval_grad[M_NUM-1]) << endl;


        while ( funcval > last_funcval && alpha*max(funcval_grad[0],funcval_grad[M_NUM-1]) > min_funChange*PI/180.0 && alpha*min(funcval_grad[0],funcval_grad[M_NUM-1]) < min_funChange*PI/-180.0)
        {
            alpha = alpha * 0.5;
            for (int count=0; count<M_NUM; count++)
            {
                M_thetaf[count]=last_theta[count] - alpha*funcval_grad[count];
            }
            funcval = f_grad__norm(funcval_grad, parameters,M_thetaf, m_pos, m, Bo, Fo);

//            cout << alpha*max(funcval_grad[0],funcval_grad[M_NUM-1])  << "  -    "<< alpha*min(funcval_grad[0],funcval_grad[M_NUM-1]) << endl;


        }

        last_funcval = funcval;
        for (int count=0; count<M_NUM; count++)
        {
            last_theta[count]=M_thetaf[count];
        }

        if (alpha*max(funcval_grad[0],funcval_grad[M_NUM-1]) < min_funChange*PI/180.0 && alpha*min(funcval_grad[0],funcval_grad[M_NUM-1]) > min_funChange*PI/-180.0)
        {
            break;
        }
    }
}

void sufficientlocalminGrad_norm(float M_theta[], float M_theta0[], int iter_max, float BFaccept[6], float parameters[],float m_pos[], float m[], float Bo[3], float Fo[3])
{
    //what should be returned if no result is found??

    float Bo_mag = magnitude(Bo,3);
    float Fo_mag = magnitude(Fo,3);
    float B_mag, F_mag;
    float B_ang, F_ang;

    float M_thetaf[M_NUM];
    float M_thetai[M_NUM];
    float t;
    int maxanglechange;
    maxanglechange = (int)parameters[2];

    int iter_num = 1;

    bool check=true; //should the set of angles found using steepest descent be compared to BFaccept
    float check_dist;
    bool mindist2prev=true;
    float c=0.0;

    //check if actM_thetai is equal to zero vector, if it is, then mindist2prev = false;
    for (int count=0; count<M_NUM; count++)
    {
        c=c+M_theta0[count];
    }
    if (c==0)
        mindist2prev=false;

    while (iter_num < iter_max)
    {
        if (iter_num==iter_max*0.9)
        {
            BFaccept[4]=BFaccept[4]*.85; //decrease accepted force magnitude by 15%
        }

        if (iter_num < iter_max*0.4)
            parameters[3] = 3.5;
        else if (iter_num < iter_max*0.7)
            parameters[3] = 2.5;
        else
            parameters[3] = 1;

        for (int count=0; count<M_NUM; count++)
        {
            if(mindist2prev) //randomize angles around actM_thetai
            {
                t = rand() % (2*maxanglechange) - maxanglechange;
                t = (float)t*PI/180.0;

                M_thetai[count] = M_theta0[count] + t;

            }
            else //completely randomize angles
            {
                t = rand() % 360;
                M_thetai[count] = t*PI/180.0; // try to ensure that new random number are being generated each time !!!!!!!!!!!!!!!!
            }

        }

        localminGrad_norm(M_thetaf, parameters, M_thetai, m_pos, m, Bo, Fo);

        if (mindist2prev)
        {
            check=true;

            for (int count=0; count<M_NUM; count++)
            {
                check_dist = abs(M_thetaf[count] - M_theta0[count]);

                if (check_dist*180.0/PI > maxanglechange*2)
                {
                    printf("Max Motor Angle: %.2f (found from opt)\n",check_dist*180.0/PI);
                    check=false;
                    //check=true;

                    count=M_NUM; //skip to end
                }

            }
            printf("\n");


        }
        else
        {
            check=true;
        }

        if (check)
        {
            float B[3], F[3];
            field_force(B,F,M_thetaf,m_pos, m);

            B_mag = magnitude(B,3);
            F_mag = magnitude(F,3);

            if (Bo_mag != 0.0 && Fo_mag != 0.0)
            {
                B_ang = abs ( acos( (Bo[0]*B[0] + Bo[1]*B[1] +  Bo[2]*B[2]) / (Bo_mag * B_mag) )*180.0/PI );
                F_ang = abs ( acos( (Fo[0]*F[0] + Fo[1]*F[1] +  Fo[2]*F[2]) / (Fo_mag * F_mag) )*180.0/PI );

                printf("Bang: %.1f  Fang: %.1f  Bmag: %.1f  Fmag: %.2f\n",B_ang,F_ang, B_mag*1e3, F_mag*1e6);

                if (B_ang < BFaccept[0] && F_ang < BFaccept[3] && B_mag > BFaccept[1] && B_mag < BFaccept[2] && F_mag > BFaccept[4] && F_mag < BFaccept[5] )
                {
                    cout << "Best angle found on iteration: "<< iter_num << endl;
                    break;
                }

            }
            else if (Bo_mag == 0.0 && Fo_mag != 0.0)
            {
                F_ang = abs ( acos( (Fo[0]*F[0] + Fo[1]*F[1] +  Fo[2]*F[2]) / (Fo_mag * F_mag) )*180/PI );

                if (B_mag < 0.1e-3 && F_ang < BFaccept[3] && F_mag > BFaccept[4] && F_mag < BFaccept[5] )
                {
                    break;
                }

            }
            else if (Bo_mag != 0.0 && Fo_mag == 0.0)
            {
                B_ang = abs ( acos( (Bo[0]*B[0] + Bo[1]*B[1] +  Bo[2]*B[2]) / (Bo_mag * B_mag) )*180/PI );

                if (B_ang < BFaccept[0] && F_mag < 0.05e-6 && B_mag > BFaccept[1] && B_mag < BFaccept[2] )
                {
                    break;
                }

            }
            else
            {
                if (B_mag < 0.1e-3 && F_mag < 0.1e-6)
                {
                    break;
                }
            }
        }


    iter_num++;
    }

    if (iter_num==iter_max)
    {
        cout << "!!!!!!!!!! Didn't find !!!!!!!!!!!!!!!!" << endl;

        for (int count=0; count<M_NUM; count++)
        {
            M_theta[count] = M_theta0[count];
        }

    }
    else
    {
        findclosestangle(M_thetaf,M_theta0);

        for (int count=0; count<M_NUM; count++)
        {
            M_theta[count] = M_thetaf[count];
        }
    }
}




float f_grad_hess__norm2(float f_grad[], float f_hess[][M_NUM], float M_theta[], int ret_ind, float K, float m_pos[], float m[], float Bo[3], float Fo[3])
{
    //Input: ret_ind
    //ret_ind == 0, only f is required
    //ret_ind == 1, f and f_grad required
    //ret_ind == 2, f, f_grad, and f_hess required

    float f; //function value

    float B[3], F[3];
    field_force(B,F,M_theta,m_pos, m);

    float difB, difF;
    float BminusBo[3], FminusFo[3];

    for (int xyz=0; xyz<3; xyz++)
    {
        BminusBo[xyz] = B[xyz] - Bo[xyz];
        FminusFo[xyz] = F[xyz] - Fo[xyz];
    }
    difB = magnitude(BminusBo,3);
    difF = magnitude(FminusFo,3);

    float multB = 1/1e-4;
    float multF = K*1/1e-12;

    f = multB*difB*difB + multF*difF*difF;

    if (ret_ind > 0) //f_grad required (maybe f_hess too)
    {
        //initialize variables required to calculate f_grad
        float Mi[3], ri[3], ri_mag, Midotri;
        float dBi[3], dFi[3];
        float gradi[3][3];

        //initialize variables required to calculate f_hess (not allowable to do this in 2nd if statement)
        float M2[3], M2dotri;
        float dBi2[3], dFi2[3];
        float grad2[3][3];

        float Mj[3], rj[3], rj_mag, Mjdotrj;
        float dBj[3], dFj[3];
        float gradj[3][3];




        for (int counti=0; counti<M_NUM; counti++)
        {
            ////////////////////// Calculate f_grad ////////////////////////////////////////////////////////////////////////

            for (int xyz=0; xyz<3; xyz++)
            {
                 Mi[xyz] = -c1[counti][xyz]*sin(M_theta[counti]) + c2[counti][xyz]*cos(M_theta[counti]);
                 ri[xyz] = -M_LOC[counti][xyz] + m_pos[xyz];
            }

            ri_mag = magnitude(ri,3);
            unitv(ri,3);
            Midotri= Mi[0]*ri[0] + Mi[1]*ri[1] + Mi[2]*ri[2];

            for (int xyz=0; xyz<3; xyz++)
            {
                for (int xyz2=0; xyz2<3; xyz2++)
                {
                     // xyz = row and xyz2 = col
                     if (xyz == xyz2)
                     {
                         gradi[xyz][xyz2]= ( ri[xyz]*Mi[xyz2] + ri[xyz2]*Mi[xyz] - ( 5.0 * ri[xyz]*ri[xyz2] - 1.0 )*Midotri  );
                     }
                     else
                     {
                         gradi[xyz][xyz2]= ( ri[xyz]*Mi[xyz2] + ri[xyz2]*Mi[xyz] - ( 5.0 * ri[xyz]*ri[xyz2] )*Midotri  );
                     }
                 }
            }

            for(int xyz=0; xyz<3; xyz++)
            {
                dBi[xyz] = 1e-7 / (pow(ri_mag, 3.0) ) * (3*Midotri*ri[xyz]-Mi[xyz]);
                dFi[xyz] = 3.0 * 1e-7 / (pow(ri_mag, 4.0) ) * ( gradi[xyz][0]*m[0] + gradi[xyz][1]*m[1] + gradi[xyz][2]*m[2]);
            }

            f_grad[counti] = 2.0 * multB * (BminusBo[0]*dBi[0] + BminusBo[1]*dBi[1] + BminusBo[2]*dBi[2])
                           + 2.0 * multF * (FminusFo[0]*dFi[0] + FminusFo[1]*dFi[1] + FminusFo[2]*dFi[2]);





            if(ret_ind > 1) ////////////////////// Calculate f_hess ////////////////////////////////////////////////////////////////////////
            {
                for (int xyz=0; xyz<3; xyz++)
                {
                     M2[xyz] = -c1[counti][xyz]*cos(M_theta[counti]) - c2[counti][xyz]*sin(M_theta[counti]);
                }

                M2dotri= M2[0]*ri[0] + M2[1]*ri[1] + M2[2]*ri[2];


                for (int xyz=0; xyz<3; xyz++)
                {
                    for (int xyz2=0; xyz2<3; xyz2++)
                    {
                         // xyz = row and xyz2 = col
                         if (xyz == xyz2)
                         {
                             grad2[xyz][xyz2]= ( ri[xyz]*M2[xyz2] + ri[xyz2]*M2[xyz] - ( 5.0 * ri[xyz]*ri[xyz2] - 1.0 )*M2dotri  );
                         }
                         else
                         {
                             grad2[xyz][xyz2]= ( ri[xyz]*M2[xyz2] + ri[xyz2]*M2[xyz] - ( 5.0 * ri[xyz]*ri[xyz2] )*M2dotri  );
                         }
                     }
                }

                for(int xyz=0; xyz<3; xyz++)
                {
                    dBi2[xyz] = 1e-7 / (pow(ri_mag, 3.0) ) * (3*M2dotri*ri[xyz]-M2[xyz]);
                    dFi2[xyz] = 3 * 1e-7 / (pow(ri_mag, 4.0) ) * ( grad2[xyz][0]*m[0] + grad2[xyz][1]*m[1] + grad2[xyz][2]*m[2]);
                }

                for (int countj=counti; countj<M_NUM; countj++)
                {
                    for (int xyz=0; xyz<3; xyz++)
                    {
                         Mj[xyz] = -c1[countj][xyz]*sin(M_theta[countj]) + c2[countj][xyz]*cos(M_theta[countj]);
                         rj[xyz] = -M_LOC[countj][xyz] + m_pos[xyz];
                    }

                    rj_mag = magnitude(rj,3);
                    unitv(rj,3);
                    Mjdotrj= Mj[0]*rj[0] + Mj[1]*rj[1] + Mj[2]*rj[2];


                    for (int xyz=0; xyz<3; xyz++)
                    {
                        for (int xyz2=0; xyz2<3; xyz2++)
                        {
                             // xyz = row and xyz2 = col
                             if (xyz == xyz2)
                             {
                                 gradj[xyz][xyz2]= ( rj[xyz]*Mj[xyz2] + rj[xyz2]*Mj[xyz] - ( 5.0 * rj[xyz]*rj[xyz2] - 1.0 )*Mjdotrj  );
                             }
                             else
                             {
                                 gradj[xyz][xyz2]= ( rj[xyz]*Mj[xyz2] + rj[xyz2]*Mj[xyz] - ( 5.0 * rj[xyz]*rj[xyz2] )*Mjdotrj  );
                             }
                         }
                    }

                    for(int xyz=0; xyz<3; xyz++)
                    {
                        dBj[xyz] = 1e-7 / (pow(ri_mag, 3.0) ) * (3*Mjdotrj*rj[xyz]-Mj[xyz]);
                        dFj[xyz] = 3 * 1e-7 / (pow(ri_mag, 4.0) ) * ( gradj[xyz][0]*m[0] + gradj[xyz][1]*m[1] + gradj[xyz][2]*m[2]);
                    }


                    f_hess[counti][countj] = 2.0 * multB * (dBj[0]*dBi[0] + dBj[1]*dBi[1] + dBj[2]*dBi[2])
                       + 2.0 * multF * (dFj[0]*dFi[0] + dFj[1]*dFi[1] + dFj[2]*dFi[2]);

                    if (counti == countj)
                    {
                        f_hess[counti][countj]  = f_hess[counti][countj]  + 2.0 * multB * (BminusBo[0]*dBi2[0] + BminusBo[1]*dBi2[1] + BminusBo[2]*dBi2[2])
                           + 2.0 * multF * (FminusFo[0]*dFi2[0] + FminusFo[1]*dFi2[1] + FminusFo[2]*dFi2[2]);
                    }
                    else
                    {
                        f_hess[countj][counti] =f_hess[counti][countj];
                    }

                }

            }

         }

    }

    return f;

}

void localminNewton_norm2(float theta_0[], float parameters[], float theta_start[], float m_pos[], float m[], float Bo[3], float Fo[3])
{

    float step_size = parameters[0];
    float step_size0 = step_size;
    float cut_off   = parameters[1];
    float K         = parameters[2];

    int cnt       = (int)parameters[9];



    float theta_1[M_NUM];
    for (int count=0; count<M_NUM; count++)
    {
        theta_0[count]=theta_start[count];
    }


    float alpha = 0.4;
    float beta = 0.7;

    float f_0, f_1;
    float grad[M_NUM];
    float hess[M_NUM][M_NUM];
    float dir[M_NUM];
    float grad_dot_dir;

    for (int j=0; j<60; j++)
    {
        step_size = step_size0;

        //f_0 = f_grad_hess__norm2(grad,hess,theta_0,2,K,m_pos,m,Bo,Fo); //Newton Method - hessian is required
        f_0 = f_grad_hess__norm2(grad,hess,theta_0,1,K,m_pos,m,Bo,Fo); //Gradient Descent - only gradient required

        if (max(grad[0],grad[M_NUM-1]) < cut_off && min(grad[0],grad[M_NUM-1]) > -cut_off)
        {
            break;
        }
        else if (j % cnt == 0)
        {
            //Compare to cutoff values
            //No, update set point if better than current
        }

        //get direction for Newton descent
        //Newtondirection(dir,grad,hess);

        //direction for Gradient descent
        for (int count=0; count<M_NUM; count++)
        {
            dir[count]=-grad[count];
        }


        grad_dot_dir = 0;
        for (int count=0; count<M_NUM; count++)
        {
            theta_1[count]=theta_0[count] + step_size*dir[count];
            grad_dot_dir = grad_dot_dir + grad[count]*dir[count];
        }
        f_1 = f_grad_hess__norm2(grad,hess,theta_1,0,K,m_pos,m,Bo,Fo);

        //Backtrack Line Search
        while(f_1 > f_0 + alpha * step_size * grad_dot_dir)
        {
            step_size = step_size * beta;
            for (int count=0; count<M_NUM; count++)
            {
                theta_1[count]=theta_0[count] + step_size*dir[count];
            }
            f_1 = f_grad_hess__norm2(grad,hess,theta_1,0,K,m_pos,m,Bo,Fo);
        }

        for (int count=0; count<M_NUM; count++)
        {
            theta_0[count]=theta_1[count];
        }
    }
}

void Newtondirection(float dir[], float f_grad[], float f_hess[][M_NUM])
{
    Matrix8f H;
    Vector8f G;
    Vector8f D;
    Vector8f E;
    float min_eigen;

    for(int i=0; i<M_NUM; i++)
    {
        G(i) = f_grad[i];
        for(int j=0; j<M_NUM;j++)
        {
            H(i,j)=f_hess[i][j];
        }
    }

    SelfAdjointEigenSolver<Matrix8f> eigensolver(H);
    E = eigensolver.eigenvalues();
    //cout << "eigen values are: " << endl << E << endl;
    //cout << "min eigen value is:" << E.minCoeff() << endl;

    min_eigen = E.minCoeff(); //E vector is (always?) ordered from smallest to largest
                              //could potentially save time by using min_eigen = E(1); -testout


    if (min_eigen < 0)
    {
        for(int i=0; i<M_NUM; i++)
        {
            H(i,i)=H(i,i)+(-min_eigen+0.01);
        }
    }

    D = H.fullPivLu().solve(-G);
    //cout << "Direction: " << endl << D << endl;

    for(int i=0; i<M_NUM; i++)
    {
        dir[i] = D(i);
    }


}




////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////// Functions to Run Motors                 ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

void motor_setSpeedAcceleration(float vel, float accel)
{
        if (vel!=0.0)
           max_speed = vel;

        if(accel !=0.0)
            max_accel=accel;

        stepper1.setMaxSpeed(max_speed);
		stepper1.setAcceleration(max_accel);

		stepper2.setMaxSpeed(max_speed);
		stepper2.setAcceleration(max_accel);

		stepper3.setMaxSpeed(max_speed);
		stepper3.setAcceleration(max_accel);

		stepper4.setMaxSpeed(max_speed);
		stepper4.setAcceleration(max_accel);

		stepper5.setMaxSpeed(max_speed);
		stepper5.setAcceleration(max_accel);

		stepper6.setMaxSpeed(max_speed);
		stepper6.setAcceleration(max_accel);

		stepper7.setMaxSpeed(max_speed);
		stepper7.setAcceleration(max_accel);

		stepper8.setMaxSpeed(max_speed);
		stepper8.setAcceleration(max_accel);
}

void* runmotorsThread(void*threadid)
{
    printf("Motors Running\n");
	while(end_runmotorsthread==false)
	{
			stepper1.run();
			stepper2.run();
			stepper3.run();
			stepper4.run();
			stepper5.run();
			stepper6.run();
			stepper7.run();
			stepper8.run();
	}
    printf("Motors Not Running Anymore\n");
}

void update_motordist()
{
    //update the motor_steps2go vector (i.e. the number of steps between the current motor positions and the goal)
    motor_steps2go[0] = stepper1.distanceToGo();
    motor_steps2go[1] = stepper2.distanceToGo();
    motor_steps2go[2] = stepper3.distanceToGo();
    motor_steps2go[3] = stepper4.distanceToGo();
    motor_steps2go[4] = stepper5.distanceToGo();
    motor_steps2go[5] = stepper6.distanceToGo();
    motor_steps2go[6] = stepper7.distanceToGo();
    motor_steps2go[7] = stepper8.distanceToGo();
}

int check_motorsdonemoving()
{
    int num_motorsmoving=0;
    update_motordist();

    for(int i=0; i<8; i++)
    {
        if (motor_steps2go[i]>0.00 || motor_steps2go[i]<0.00)
            num_motorsmoving++;
    }

    return num_motorsmoving;
}

void motors_lockFunction(int magnetsbeingused[])
{
    printf("lock\n");
    long d=0;
    end_runmotorsthread = false;

	stepper1.setCurrentPosition(d);
	stepper2.setCurrentPosition(d);
	stepper3.setCurrentPosition(d);
	stepper4.setCurrentPosition(d);
	stepper5.setCurrentPosition(d);
	stepper6.setCurrentPosition(d);
	stepper7.setCurrentPosition(d);
	stepper8.setCurrentPosition(d);

	int magnet_id;
	for(int i=0; i<8; i++)
	{
		magnet_id=magnetsbeingused[i];
		if (magnet_id != 0)
            s826_doPin(motor_parm[magnet_id-1][2],0);
	}

	pthread_t runmotorsthread; // note the thread must be positioned before this function
	pthread_create(&runmotorsthread, 	NULL, runmotorsThread, NULL);      //start test thread
}

void motors_unlockFunction(void)
{
    printf("unlock\n");
	int magnet_id;

	end_runmotorsthread = true;

	for(int magnet_id=1; magnet_id<=8; magnet_id++)
	{
		s826_doPin(motor_parm[magnet_id-1][2],1);
	}


}

void motors_moveFunction(float motor_increment[])
{
    //printf("moveFunction \n");

    stepper1.move(motor_increment[0]);
	stepper2.move(motor_increment[1]);
	stepper3.move(motor_increment[2]);
	stepper4.move(motor_increment[3]);
	stepper5.move(motor_increment[4]);
	stepper6.move(motor_increment[5]);
	stepper7.move(motor_increment[6]);
	stepper8.move(motor_increment[7]);
}

void motors_moveToFunction(float motor_rotation[])
{
    //printf("moveToFunction \n");

    stepper1.moveTo(motor_rotation[0]);
	stepper2.moveTo(motor_rotation[1]);
	stepper3.moveTo(motor_rotation[2]);
	stepper4.moveTo(motor_rotation[3]);
	stepper5.moveTo(motor_rotation[4]);
	stepper6.moveTo(motor_rotation[5]);
	stepper7.moveTo(motor_rotation[6]);
	stepper8.moveTo(motor_rotation[7]);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////// Control Function                        ////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

void update_linepoints(int line_index,int x1,int y1,int x2,int y2 )
{
    l_x[line_index]=x1;
    l_y[line_index]=y1;
    l_x[line_index+1]=x2;
    l_y[line_index+1]=y2;
}

int * getSavedGoalPointCoor(int goalP_index)
{
    //change num_goalpoints in script!!!!!!!
    /*
        change num_goalpoints in script!!!!!!!
    */

    //Square
    //const int num_goalP = 5;
    //int goalcoordX[num_goalP]={250,250,350,350,250};
    //int goalcoordY[num_goalP]={200,300,300,200,200};

    //Triangle
    const int num_goalP = 4;
    int goalcoordX[num_goalP]={150,250,350,150};
    int goalcoordY[num_goalP]={100,300,100,100};

    static int SavedGoalPointCoor[2];
    SavedGoalPointCoor[0]=goalcoordX[goalP_index];
    SavedGoalPointCoor[1]=goalcoordY[goalP_index];

    return SavedGoalPointCoor;

    //change num_goalpoints in script!!!!!!!
    /*
        change num_goalpoints in script!!!!!!!
    */

}

 void horizontalworkspace_calibration(void)
 {
     //printf("Calibrating Workspace\n");
     int *goalP_coor, *goalP_coorC, *goalP_coorR;

     goalP_coor   = getGoalPointCoor();
     goalP_coorC   = getGoalPointCoorC();
     goalP_coorR   = getGoalPointCoorR();

     //printf("%d  %d \n",goalP_coor[0],goalP_coor[1]);
     //printf("%d  %d \n",goalP_coorC[0],goalP_coorC[1]);
     //printf("%d  %d \n",goalP_coorR[0],goalP_coorR[1]);

     workspace_center[0] = goalP_coorC[0];
     workspace_center[1] = goalP_coorC[1];
     workspace_center[2] = 0;

     float x_pixeldistance, y_pixeldistance, avg_pixeldistance;

     x_pixeldistance = goalP_coorR[0]-goalP_coorC[0];
     y_pixeldistance = goalP_coor[1]-goalP_coorC[1];
     avg_pixeldistance = x_pixeldistance;//(x_pixeldistance + y_pixeldistance)/2.0;

     mm_per_pixel = calibration_length / avg_pixeldistance;

    printf("Scale: %.4f Center: %d  %d \n",mm_per_pixel,workspace_center[0],workspace_center[1]);
 }



void* horizontalwaypointfollowing_thread (void*threadid)
{
    printf("Starting Horizontal Feedback Thread\n");

    //Initialize Rotating Magnet Setup Parameters
    //magnetLocAxisConst(M_LOCrmc, M_ROTrmc, c1rmc, c2rmc, M_DIST, M_AZIM, M_INCL, M_AXIS, M_STRN, M_InDIR, M_NUM);

    int *centerP_coor, *goalP_coor;
    float centerP_mm[3]={0,0,0};

    int xyz_distance[3]; //distance from centre of microrobot to goal point in pixels
    int mag_distance; //magnitude of distance from centre to goal

    float B[3], F[3], Bdes[3], Fdes[3]; //B and F are the field and force currently being applied to the robot at its position (for the current set of magnet rotational positions)
                                        //Bdes and Fdes are the desired field and force vectors (i.e. Bo and Fo)
    float Bdes_mag=10e-3, Fdes_mag;
    float B_dir[3]={1, 1.5, 0}, F_dir[3];
    float FFdes_angle, F_mag;
    ///float B_theta0, B_theta1, deltaB_theta, maxB_theta=360*PI/180;
    float F_theta1; ///float F_theta0, deltaF_theta, maxF_theta=360*PI/180;
    bool newmotorpositions;

    float magnet_rotR[8] = {-1.7098,	0.6838, 2.7678,-0.5630,	0.0090, -1.7817, -3.0582,2.4603};
    float magnet_rotR0[8] = {0,0,0,0,0,0,0,0};

    float microrobot_position[3] = {0,0,0}; // in metres
    float microrobot_moment[3] = {0,0,0}; //microrobot magnetic moment in SI
    float BFaccept[6] = {30, 2e-3, 35e-3, 10, 0.1e-6, 0.5e-6}; //[Bangle_err Bmag_min Bmag_max Fangle_err Fmag_min Fmag_max ]
    float parameters[] = {0.2, 0.1, 15, 3.5}; //[grad_descent alpha, grad_descent cutoff, *findmagangles max_motorspin angle, *vectorDifference K_mult]
    float max_num_iterations = 40;

    int goalpoint_ind=0;
    float prev_location[2]={-100, -100};
    int stopped_counter=0;
    int on_goalP_counter=0;

    const int num_goalpoints = 5; //5 for square, 4 for triangle

    if (calibrate_distances==1)
    {
        while (true)
        {
            horizontalworkspace_calibration();

        }
    }

    //Set motors to initial position

    //motor_setSpeedAcceleration(1000,4500);
    for(int count=0; count<8; count++)
    {
        motor_positionsrmc[count] = magnet_rotR[count]*(180.0/PI)*400.0/360.0;
        magnet_rotR0[count] = magnet_rotR[count];
        //printf("%.1f ",motor_positionsrmc[count]);
    }
    motors_moveToFunction(motor_positionsrmc);

    //Get initial force and field at robot position
    centerP_coor = getCenterPointCoor();   // in vision.c, get the centre point of swimmer
    for (int xyz=0; xyz<2; xyz++) //z coordinate = 0
    {
        centerP_mm[xyz] = (float)(centerP_coor[xyz]-workspace_center[xyz])*mm_per_pixel;
        microrobot_position[xyz] = centerP_mm[xyz] /1000.0; //in metres
    }

    field_force(B,F,magnet_rotR0, microrobot_position, microrobot_moment);
    unitv(B,3);
    for (int xyz=0; xyz<3; xyz++)
    {
        microrobot_moment[xyz]=B[xyz]*m_mag;
    }
    //Calculate F based on updated microrobot_moment
    field_force(B,F,magnet_rotR0, microrobot_position, microrobot_moment);

    //horizontal control
    printf("In Horizontal Feedback Loop\n");
    switch_record_centerP(1);


    while(hor_waypoint_flag ==1)
    {
        centerP_coor = getCenterPointCoor();   // in vision.c, get the centre point of swimmer
        if (userinput_click)
            goalP_coor   = getGoalPointCoor();
        else if(userinput_saved)
            goalP_coor = getSavedGoalPointCoor(goalpoint_ind);

        mag_distance = 0;
        for (int xyz=0; xyz<2; xyz++) //z coordinate = 0
        {
            centerP_mm[xyz] = (float)(centerP_coor[xyz]-workspace_center[xyz])*mm_per_pixel;

            xyz_distance[xyz] = goalP_coor[xyz] - centerP_coor[xyz]; //in pixels
            microrobot_position[xyz] = centerP_mm[xyz] /1000.0; //in metres
            mag_distance = mag_distance + xyz_distance[xyz]*xyz_distance[xyz];
        }

        //Pseudo Proportional Control: change F_mag based on distance to target
        if (mag_distance < 55) //was 100
        {
            //F_mag=0;  //for now - dont stop on goal

            on_goalP_counter++;
            if (on_goalP_counter > 0) //used to stop on goalP momentarily
            {
                //target reached - go on to next goal
                goalpoint_ind=goalpoint_ind+1;
            }

        }
        /*
        else if(mag_distance < 500)
        {
            F_mag=0.5e-6;
            BFaccept_rmc[3] = {5};
            BFaccept_rmc[4] = {0.35e-6};
            BFaccept_rmc[5] = {0.65e-6};
            on_goalP_counter=0;
        }
        */

        else
        {
            Fdes_mag=0.55e-6;
            BFaccept[3] = {10};
            BFaccept[4] = {0.48e-6};
            BFaccept[5] = {0.62e-6};
            on_goalP_counter=0;
        }
                                    //Calculate Field and Force at Robot position
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////

        //B is field at mircorobot position, F may not be correct since microrobot_moment has not been updated
        field_force(B,F,magnet_rotR0, microrobot_position, microrobot_moment);
        unitv(B,3);
        for (int xyz=0; xyz<3; xyz++)
        {
            microrobot_moment[xyz]=B[xyz]*m_mag;
        }
        //Calculate F based on updated microrobot_moment
        field_force(B,F,magnet_rotR0, microrobot_position, microrobot_moment);

        //Show direction of current force on microrobot
        update_linepoints(0, centerP_coor[0], centerP_coor[1], centerP_coor[0]+(int)(F[0]*1e6*200), centerP_coor[1]+(int)(F[1]*1e6*200) );

        //Show direction of current field on microrobot
        //update_linepoints(4, centerP_coor[0] +(int)(B[0]*1e3*20/2), centerP_coor[1]+(int)(B[1]*1e3*20/2), centerP_coor[0]-(int)(B[0]*1e3*20/2), centerP_coor[1]-(int)(B[1]*1e3*20/2) );

                                     //Calculate Desired Field and Force
        /////////////////////////////////////////////////////////////////////////////////////////////////////

        F_theta1 = atan2(xyz_distance[1], xyz_distance[0]);

        F_dir[0] = cos(F_theta1);  //might be faster to just make xyz_distance a unit vector, perhaps try it
        F_dir[1] = sin(F_theta1);
        F_dir[2] = 0.0; //horizontal

        unitv(B_dir,3);
        //intermediatev(B_dir,B,maxB_theta);
        //intermediatev(F_dir,F,maxF_theta);

        for (int xyz=0; xyz<3; xyz++)
        {
            Bdes[xyz]=B_dir[xyz]*Bdes_mag;
            Fdes[xyz]=F_dir[xyz]*Fdes_mag; //F_mag or the F components of BF_accept will change with distance to way point
        }

        //Show direction of desired force on microrobot
        update_linepoints(2, centerP_coor[0], centerP_coor[1], centerP_coor[0]+(int)(Fdes[0]*1e6*200), centerP_coor[1]+(int)(Fdes[1]*1e6*200) );


                                    //Compare desired Force with current Force acting on robot - if similar, don't change magnet positions
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        //Check if force needs to be updated (i.e. if current force on robot is in right direction, dont change motor positions)
        F_mag = magnitude(F,3);
        FFdes_angle = abs ( acos( (F[0]*Fdes[0] + F[1]*Fdes[1] +  F[2]*Fdes[2]) / (F_mag * Fdes_mag) )*180.0/PI );

        if ( Fdes_mag>0 && (FFdes_angle < BFaccept[3] && F_mag > BFaccept[4] && F_mag < BFaccept[5]) )
        {
            newmotorpositions=false;
            //printf("Current Motor Positions are Good\n\n");
        }
        else if (Fdes_mag==0 && F_mag < 0.1e-6)
        {
            newmotorpositions=false;
           // printf("Current Motor Positions are Good\n\n");

        }
        else
        {
            newmotorpositions=true;

            //If Frmc and Fdes are really different, allow big angle change, otherwise small angle change
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            if(FFdes_angle > 40)
            {
                parameters[2] = 170;  //parameters_rmc[] = {0.04, 0.1, 15, 3.5};
            }

            else if(FFdes_angle > 16) //WAS 20
            {
                parameters[2] = 90;
            }

            else
                parameters[2] = 15;
        }

                            //If robot gets stuck on same spot, change magnet positions
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        if ( abs(prev_location[0] - centerP_coor[0]) + abs(prev_location[1] - centerP_coor[1]) < 4) //robot is stuck on same spot
            stopped_counter++;
        else
            stopped_counter=0;

        if (stopped_counter > 130)
        {
            ("RESETTING MOTORS BECAUSE ROBOT STUCK! \n");
            newmotorpositions=true;
        }
        prev_location[0] = centerP_coor[0];
        prev_location[1] = centerP_coor[1];




        //plot_line(true, l_x, l_y, l_R, l_G, l_B);  //Update arrows on GUI video

        num_motorsturning = check_motorsdonemoving(); //Update how many motors are still moving

        //motor_turning = false;
        if (num_motorsturning < 2 && newmotorpositions == true)
        {
            printf("Starting Angle Search \n");
            //find next set of magnet angles
            sufficientlocalminGrad_norm(magnet_rotR, magnet_rotR0, max_num_iterations, BFaccept, parameters, microrobot_position, microrobot_moment, Bdes, Fdes);
            printf("Angle Search Completed\n");

            for(int count=0; count<8; count++)
            {
                motor_positionsrmc[count] = magnet_rotR[count]*(180.0/PI)*400.0/360.0;
                magnet_rotR0[count] = magnet_rotR[count];
                //printf("%.1f ",motor_positionsrmc[count]);
            }
           // printf("\n\n");
            motors_moveToFunction(motor_positionsrmc);
            printf("Motor Positions Updated\n");

            field_force(B,F,magnet_rotR0, microrobot_position, microrobot_moment);
            //show force that determineMagnetAngles just found
            //update_linepoints(6, centerP_coor[0], centerP_coor[1], centerP_coor[0]+(int)(Frmc[0]*1e6*200), centerP_coor[1]+(int)(Frmc[1]*1e6*200) );


            printf("\n Robot Position(mm): %.2f   %.2f   %.2f \n",microrobot_position[0]*1000.0, microrobot_position[1]*1000.0, microrobot_position[2]*1000.0);
            printf("Current Field (mT): %.2f   %.2f   %.2f   %.2f\n",B[0]*1000, B[1]*1000, B[2]*1000,magnitude(B,3)*1e3);
            printf("Current Force (uN): %.2f   %.2f   %.2f   %.2f\n",F[0]*1e6, F[1]*1e6, F[2]*1e6,magnitude(F,3)*1e6);
            printf("Robot Orient.     : %.2f   %.2f   %.2f \n",microrobot_moment[0]*1e6, microrobot_moment[1]*1e6, microrobot_moment[2]*1e6);
            printf("Desired Field (mT): %.2f   %.2f   %.2f \n",Bdes[0]*1000, Bdes[1]*1000, Bdes[2]*1000);//F_theta0*180/PI, F_theta1*180/PI,deltaF_theta*180/PI);
            printf("Desired Force (uN): %.2f   %.2f   %.2f \n",Fdes[0]*1e6, Fdes[1]*1e6, Fdes[2]*1e6);

        }
        printf("-");

        if (userinput_saved && goalpoint_ind>=num_goalpoints) //last saved waypoint reached
        {
            hor_waypoint_flag =0;
            //turn off lines
             //plot_line(false, l_x, l_y, l_R, l_G, l_B);
        }
        usleep(0.0825e6);


    }

    printf("Feedback Thread Ended\n");
    switch_record_centerP(0);

}

void* horizontalforceheading_thread (void*threadid)
{
    printf("In force heading thread\n");

    //Initialize Rotating Magnet Setup Parameters
    //magnetLocAxisConst(M_LOCrmc, M_ROTrmc, c1rmc, c2rmc, M_DIST, M_AZIM, M_INCL, M_AXIS, M_STRN, M_InDIR, M_NUM);

    int *centerP_coor;
    int goalP_coor[2]={250,100};
    //goalP_coor[0]=workspace_center[0];
    //goalP_coor[1]=workspace_center[1];

    float centerP_mm[3]={0,0,0};

    int xyz_distance[3]; //distance from centre of microrobot to goal point in pixels
    int mag_distance; //magnitude of distance from centre to goal

    float Brmc[3], Frmc[3], Bdes[3], Fdes[3]; //Brmc and Frmc are the field and force currently being applied to the robot at its position (for the current set of magnet rotational positions)
                                              //Bdes and Fdes are the desired field and force vectors (i.e. Bo and Fo)

    float B_mag=10e-3, F_mag;
    float B_dir[3]={1, 1, 0}, F_dir[3];
    float FrmcFdes_angle, Frmc_mag;
    float B_theta0, B_theta1, deltaB_theta, maxB_theta=360*PI/180;
    float F_theta0, F_theta1, deltaF_theta, maxF_theta=360*PI/180;
    bool newmotorpositions;

    float magnet_rotRrmc[8] = {-1.7098,	0.6838, 2.7678,-0.5630,	0.0090, -1.7817, -3.0582,2.4603};

    float magnet_rotR0rmc[8] = {0,0,0,0,0,0,0,0};
    float microrobot_position[3] = {0,0,0}; // in metres
    float microrobot_moment[3] = {0,0,0}; //microrobot magnetic moment in SI
    float BFaccept_rmc[6] = {30, 2e-3, 35e-3, 10, 0.1e-6, 0.5e-6}; //[Bangle_err Bmag_min Bmag_max Fangle_err Fmag_min Fmag_max ]
    float parameters_rmc[] = {0.2, 0.1, 20, 3.5}; //[grad_descent alpha, grad_descent cutoff, findmagangles max_motorspin angle, vectorDifference K_mult]
    float max_num_iterations = 40;

    int goalpoint_ind=0;
    float prev_location[2]={-100, -100};
    int stopped_counter=0;
    int on_goalP_counter=0;
    bool reached_center=0;

    const int num_goalpoints = 5; //5 for square, 4 for triangle

    if (calibrate_distances==1)
    {
        while (true)
        {
            horizontalworkspace_calibration();

        }
    }

    //Set motors to initial position

    //motor_setSpeedAcceleration(1000,4500);
    for(int count=0; count<8; count++)
    {
        motor_positionsrmc[count] = magnet_rotRrmc[count]*(180.0/PI)*400.0/360.0;
        magnet_rotR0rmc[count] = magnet_rotRrmc [count];
        //printf("%.1f ",motor_positionsrmc[count]);
    }
    motors_moveToFunction(motor_positionsrmc);

    //horizontal control
    printf("Starting Horizontal Force heading Thread\n");

    while( hor_force_flag ==1 && reached_center==0  && go_var == false)
    {
        centerP_coor = getCenterPointCoor();   // in vision.c, get the centre point of swimmer
        //goalP is workplace center,

        mag_distance = 0;
        for (int xyz=0; xyz<2; xyz++) //z coordinate = 0
        {
            centerP_mm[xyz] = (float)(centerP_coor[xyz]-workspace_center[xyz])*mm_per_pixel;

            xyz_distance[xyz] = goalP_coor[xyz] - centerP_coor[xyz]; //in pixels
            microrobot_position[xyz] = centerP_mm[xyz] /1000.0; //in metres
            mag_distance = mag_distance + xyz_distance[xyz]*xyz_distance[xyz];
        }

        //Pseudo Proportional Control: change F_mag based on distance to target
        if (mag_distance < 80)
        {
            F_mag=0;

            on_goalP_counter++;
            if (on_goalP_counter > 100) //used to stop on goalP momentarily
            {
                    reached_center=1;

            }

        }

        else if(mag_distance < 300)
        {
            F_mag=0.2e-6;
            BFaccept_rmc[3] = {10};
            BFaccept_rmc[4] = {0.15e-6};
            BFaccept_rmc[5] = {0.25e-6};
            on_goalP_counter=0;
        }
        else if(mag_distance < 1500)
        {
            F_mag=0.4125e-6;
            BFaccept_rmc[3] = {10};
            BFaccept_rmc[4] = {0.39-6};
            BFaccept_rmc[5] = {0.43-6};
            on_goalP_counter=0;
        }

        /* //3 conditions before, at target, <1500, or >1500
        else if(mag_distance < 1500)
        {
            F_mag=0.2e-6;
            BFaccept_rmc[3] = {10};
            BFaccept_rmc[4] = {0.15e-6};
            BFaccept_rmc[5] = {0.25e-6};
            on_goalP_counter=0;
        }
            */

        else
        {
            F_mag=0.85e-6;
            BFaccept_rmc[3] = {10};
            BFaccept_rmc[4] = {0.8e-6};
            BFaccept_rmc[5] = {0.9e-6};
            on_goalP_counter=0;
        }
                                    //Calculate Field and Force at Robot position
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////

        //Brmc is field at mircorobot position, Frmc may not be correct since microrobot_moment has not been updated
        field_force(Brmc,Frmc,magnet_rotR0rmc, microrobot_position, microrobot_moment);
        unitv(Brmc,3);
        for (int xyz=0; xyz<3; xyz++)
        {
            microrobot_moment[xyz]=Brmc[xyz]*m_mag;
        }
        //Calculate Frmc based on updated microrobot_moment
        field_force(Brmc,Frmc,magnet_rotR0rmc, microrobot_position, microrobot_moment);

        //Show direction of current force on microrobot
        update_linepoints(0, centerP_coor[0], centerP_coor[1], centerP_coor[0]+(int)(Frmc[0]*1e6*200), centerP_coor[1]+(int)(Frmc[1]*1e6*200) );

        //Show direction of current field on microrobot
        //update_linepoints(4, centerP_coor[0] +(int)(Brmc[0]*1e3*20/2), centerP_coor[1]+(int)(Brmc[1]*1e3*20/2), centerP_coor[0]-(int)(Brmc[0]*1e3*20/2), centerP_coor[1]-(int)(Brmc[1]*1e3*20/2) );

                                     //Calculate Desired Field and Force
        /////////////////////////////////////////////////////////////////////////////////////////////////////

        F_theta1 = atan2(xyz_distance[1], xyz_distance[0]);

        F_dir[0] = cos(F_theta1);  //might be faster to just make xyz_distance a unit vector, perhaps try it
        F_dir[1] = sin(F_theta1);
        F_dir[2] = 0.0; //horizontal

        unitv(B_dir,3);
        //intermediatev(B_dir,Brmc,maxB_theta);
        //intermediatev(F_dir,Frmc,maxF_theta);

        for (int xyz=0; xyz<3; xyz++)
        {
            Bdes[xyz]=B_dir[xyz]*B_mag;
            Fdes[xyz]=F_dir[xyz]*F_mag; //F_mag or the F components of BF_accept will change with distance to way point
        }

        //Show direction of desired force on microrobot
        update_linepoints(2, centerP_coor[0], centerP_coor[1], centerP_coor[0]+(int)(Fdes[0]*1e6*200), centerP_coor[1]+(int)(Fdes[1]*1e6*200) );


                                    //Compare desired Force with current Force acting on robot - if similar, don't change magnet positions
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        //Check if force needs to be updated (i.e. if current force on robot is in right direction, dont change motor positions)
        Frmc_mag = magnitude(Frmc,3);
        FrmcFdes_angle = abs ( acos( (Frmc[0]*Fdes[0] + Frmc[1]*Fdes[1] +  Frmc[2]*Fdes[2]) / (Frmc_mag * F_mag) )*180.0/PI );

        if ( F_mag>0 && (FrmcFdes_angle < BFaccept_rmc[3] && Frmc_mag > BFaccept_rmc[4] && Frmc_mag < BFaccept_rmc[5]) )
        {
            newmotorpositions=false;
            //printf("Current Motor Positions are Good\n\n");
        }
        else if (F_mag==0 && Frmc_mag < 0.1e-6)
        {
            newmotorpositions=false;
           // printf("Current Motor Positions are Good\n\n");

        }
        else
        {
            newmotorpositions=true;

            //If Frmc and Fdes are really different, allow big angle change, otherwise small angle change
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            if(FrmcFdes_angle > 20 || F_mag/Frmc_mag<0.49)
            {
                parameters_rmc[2] = 180;  //parameters_rmc[] = {0.04, 0.1, 15, 3.5};
            }
            /*
            else if(FrmcFdes_angle > 45)
            {
                parameters_rmc[2] = 90;
            } */
            else
                parameters_rmc[2] = 15;
        }
        /*
        if ( abs(prev_location[0] - centerP_coor[0]) + abs(prev_location[1] - centerP_coor[1]) < 4) //robot is stuck on same spot
            stopped_counter++;
        else
            stopped_counter=0;

        if (stopped_counter > 200)
        {
            ("RESETTING MOTORS BECAUSE ROBOT STUCK! \n");
            newmotorpositions=true;
        }
        */
        prev_location[0] = centerP_coor[0];
        prev_location[1] = centerP_coor[1];



        //plot_line(true, l_x, l_y, l_R, l_G, l_B);  //Update arrows on GUI video

        num_motorsturning = check_motorsdonemoving(); //Update how many motors are still moving

        //motor_turning = false;
        if (num_motorsturning < 3 && newmotorpositions == true)
        {
            printf("Starting Angle Search \n");
            //find next set of magnet angles
            sufficientlocalminGrad_norm(magnet_rotRrmc, magnet_rotR0rmc, max_num_iterations, BFaccept_rmc, parameters_rmc, microrobot_position, microrobot_moment, Bdes, Fdes);
            printf("Angle Search Completed\n");

            for(int count=0; count<8; count++)
            {
                motor_positionsrmc[count] = magnet_rotRrmc[count]*(180.0/PI)*400.0/360.0;
                magnet_rotR0rmc[count] = magnet_rotRrmc [count];
                //printf("%.1f ",motor_positionsrmc[count]);
            }
           // printf("\n\n");
            motors_moveToFunction(motor_positionsrmc);
            printf("Motor Positions Updated\n");

            field_force(Brmc,Frmc,magnet_rotR0rmc, microrobot_position, microrobot_moment);
            //show force that determineMagnetAngles just found
            //update_linepoints(6, centerP_coor[0], centerP_coor[1], centerP_coor[0]+(int)(Frmc[0]*1e6*200), centerP_coor[1]+(int)(Frmc[1]*1e6*200) );


            printf("Robot Position(mm): %.2f   %.2f   %.2f \n",microrobot_position[0]*1000.0, microrobot_position[1]*1000.0, microrobot_position[2]*1000.0);
            printf("Current Field (mT): %.2f   %.2f   %.2f   %.2f\n",Brmc[0]*1000, Brmc[1]*1000, Brmc[2]*1000,magnitude(Brmc,3)*1e3);
            printf("Current Force (uN): %.2f   %.2f   %.2f   %.2f\n",Frmc[0]*1e6, Frmc[1]*1e6, Frmc[2]*1e6,magnitude(Frmc,3)*1e6);
            printf("Robot Orient.     : %.2f   %.2f   %.2f \n",microrobot_moment[0]*1e6, microrobot_moment[1]*1e6, microrobot_moment[2]*1e6);
            printf("Desired Field (mT): %.2f   %.2f   %.2f \n",Bdes[0]*1000, Bdes[1]*1000, Bdes[2]*1000);//F_theta0*180/PI, F_theta1*180/PI,deltaF_theta*180/PI);
            printf("Desired Force (uN): %.2f   %.2f   %.2f \n",Fdes[0]*1e6, Fdes[1]*1e6, Fdes[2]*1e6);

        }
        printf("\n");
        usleep(0.09e6);

    }
    //plot_line(false, l_x, l_y, l_R, l_G, l_B);  //Update arrows on GUI video

    //robot should be at rest at goalP (i.e. close to or at workspace center)

    parameters_rmc[2] = 30;
    F_mag = 0.8e-6;
    BFaccept_rmc[3] = 3;
    BFaccept_rmc[4] = 0.76e-6;
    BFaccept_rmc[5] = 0.84e-6;
    F_theta1 = 3.14159/2;

    F_dir[0] = cos(F_theta1);  //might be faster to just make xyz_distance a unit vector, perhaps try it
    F_dir[1] = sin(F_theta1);
    F_dir[2] = 0.0; //horizontal

    unitv(B_dir,3);
    //intermediatev(B_dir,Brmc,maxB_theta);
    //intermediatev(F_dir,Frmc,maxF_theta);

    for (int xyz=0; xyz<3; xyz++)
    {
        Bdes[xyz]=B_dir[xyz]*B_mag;
        Fdes[xyz]=F_dir[xyz]*F_mag; //F_mag or the F components of BF_accept will change with distance to way point
    }

    bool found_magpositions = false;
    int num_motoranglessame = 0;

    while (found_magpositions == false)
    {

        centerP_coor = getCenterPointCoor();   // in vision.c, get the centre point of swimmer
        for (int xyz=0; xyz<2; xyz++) //z coordinate = 0
        {
            centerP_mm[xyz] = (float)(centerP_coor[xyz]-workspace_center[xyz])*mm_per_pixel;
            microrobot_position[xyz] = centerP_mm[xyz] /1000.0; //in metres
        }


        printf("Starting Angle Search \n");
        //find next set of magnet angles
        sufficientlocalminGrad_norm(magnet_rotRrmc, magnet_rotR0rmc, max_num_iterations, BFaccept_rmc, parameters_rmc, microrobot_position, microrobot_moment, Bdes, Fdes);
        printf("Angle Search Completed\n");

        num_motoranglessame = 0;
        for(int count=0; count<8; count++)
        {
            if ( magnet_rotR0rmc[count] == magnet_rotRrmc [count])
            {
                num_motoranglessame++;
            }
        }
        found_magpositions=true;
        if (num_motoranglessame>7)
            found_magpositions=false;

        parameters_rmc[2]=parameters_rmc[2]+5;
    }

    for(int count=0; count<8; count++)
    {
        motor_positionsrmc[count] = magnet_rotRrmc[count]*(180.0/PI)*400.0/360.0;
        magnet_rotR0rmc[count] = magnet_rotRrmc [count];
        //printf("%.1f ",motor_positionsrmc[count]);
    }
    motors_moveToFunction(motor_positionsrmc);
    printf("Motor Positions For Force TestUpdated\n");

    num_motorsturning = check_motorsdonemoving();
    while(num_motorsturning>0)
    {
        num_motorsturning = check_motorsdonemoving();
    }

    printf("Recording now\n");
    switch_record_centerP(1);

    update_linepoints(2, 0,0,0,0); //only plot one line
    l_R[0]=0;
    l_G[0]=255;

    while (hor_force_flag==1)
    {
        //update_linepoints(0, centerP_coor[0], centerP_coor[1], centerP_coor[0]+(int)(Fdes[0]*1e6*200), centerP_coor[1]+(int)(Fdes[1]*1e6*200) );
        //plot_line(false, l_x, l_y, l_R, l_G, l_B);  //Update arrows on GUI video


        centerP_coor = getCenterPointCoor();
        //printf("Center %d  %d\n",centerP_coor[0],centerP_coor[1]);

       if ( abs(prev_location[0] - centerP_coor[0]) + abs(prev_location[1] - centerP_coor[1]) > 0) //robot is stuck on same spot
       {
           printf("Center %d %d\n",centerP_coor[0],centerP_coor[1]);
       }
       prev_location[0]=centerP_coor[0];
       prev_location[1]=centerP_coor[1];

    }
    printf("Feedback Thread Ended\n");
    printf("Max Motor Angle: %.2f\n", parameters_rmc[2]);

    switch_record_centerP(0);

}

void* verticalwaypointfollowing_thread (void*threadid)
{
    printf("Starting Vertical Feedback Thread\n");

    //Initialize Rotating Magnet Setup Parameters
    //magnetLocAxisConst(M_LOCrmc, M_ROTrmc, c1rmc, c2rmc, M_DIST, M_AZIM, M_INCL, M_AXIS, M_STRN, M_InDIR, M_NUM);

    int *centerP_coor, *goalP_coor;
    float centerP_mm[3]={0,-6,0};

    int xyz_distance[3]; //distance from centre of microrobot to goal point in pixels
    int mag_distance; //magnitude of distance from centre to goal

    float Brmc[3], Frmc[3], Bdes[3], Fdes[3]; //Brmc and Frmc are the field and force currently being applied to the robot at its position (for the current set of magnet rotational positions)
                                              //Bdes and Fdes are the desired field and force vectors (i.e. Bo and Fo)
    float Fdes0[3]; //currently unused

    float B_mag=10e-3, F_mag;
    float B_dir[3]={1, 0, 1}, F_dir[3];
    float FrmcFdes_angle, Frmc_mag, FdesFdes0_angle, Fdes0_mag;
    float B_theta0, B_theta1, deltaB_theta, maxB_theta=360*PI/180;
    float F_theta0, F_theta1, deltaF_theta, maxF_theta=360*PI/180;
    bool newmotorpositions;

    float magnet_rotRrmc[8] = {-0.1559,	0.6571,	1.7077,   -1.0052,	2.9494,   -1.5634,   -3.9170,	1.7324};

    float magnet_rotR0rmc[8] = {0,0,0,0,0,0,0,0};
    float microrobot_position[3] = {0,-0.006,0}; // in metres
    float microrobot_moment[3] = {0,0,0}; //microrobot magnetic moment in SI
    float BFaccept_rmc[6] = {30, 2e-3, 35e-3, 10, 0.1e-6, 0.5e-6}; //[Bangle_err Bmag_min Bmag_max Fangle_err Fmag_min Fmag_max ]
    float parameters_rmc[] = {0.2, 0.1, 15, 3.5}; //[grad_descent alpha, grad_descent cutoff, findmagangles max_motorspin angle, vectorDifference K_mult]
    float max_num_iterations = 40;

    int goalpoint_ind=0;
    float prev_location[2]={-100, -100};
    int stopped_counter=0;
    int on_goalP_counter=0;

    const int num_goalpoints = 5; //5 for square, 4 for triangle

    if (calibrate_distances==1)
    {
        while (true)
        {
            horizontalworkspace_calibration();

        }
    }

    //Set motors to initial position

    //motor_setSpeedAcceleration(1000,4500);
    for(int count=0; count<8; count++)
    {
        motor_positionsrmc[count] = magnet_rotRrmc[count]*(180.0/PI)*400.0/360.0;
        magnet_rotR0rmc[count] = magnet_rotRrmc [count];
        //printf("%.1f ",motor_positionsrmc[count]);
    }
    motors_moveToFunction(motor_positionsrmc);

    //Get initial force and field at robot position
    centerP_coor = getCenterPointCoor();   // in vision.c, get the centre point of swimmer
    for (int xyz=0; xyz<2; xyz++) //z coordinate = 0
    {
        if (xyz==0)
        {
           centerP_mm[xyz] = (float)(centerP_coor[xyz]-workspace_center[xyz])*mm_per_pixel;
           microrobot_position[xyz] = centerP_mm[xyz] /1000.0; //in metres
        }
        else
        {
            centerP_mm[xyz+1] = (float)(centerP_coor[xyz]-workspace_center[xyz])*mm_per_pixel;
            microrobot_position[xyz+1] = centerP_mm[xyz+1] /1000.0; //in metres
        }
    }

    field_force(Brmc,Frmc,magnet_rotR0rmc, microrobot_position, microrobot_moment);
    unitv(Brmc,3);
    for (int xyz=0; xyz<3; xyz++)
    {
        microrobot_moment[xyz]=Brmc[xyz]*m_mag;
    }
    //Calculate Frmc based on updated microrobot_moment
    field_force(Brmc,Frmc,magnet_rotR0rmc, microrobot_position, microrobot_moment);

    for (int xyz=0; xyz<3; xyz++)
    {
        Fdes0[xyz]=Frmc[xyz];
    }

    //horizontal control
    printf("In Vertical Feedback Loop\n");
    switch_record_centerP(1);


    while(ver_waypoint_flag ==1)
    {
        centerP_coor = getCenterPointCoor();   // in vision.c, get the centre point of swimmer
        if (userinput_click)
            goalP_coor   = getGoalPointCoor();
        else if(userinput_saved)
            goalP_coor = getSavedGoalPointCoor(goalpoint_ind);

        mag_distance = 0;
        for (int xyz=0; xyz<2; xyz++) //z coordinate = 0
        {
            if (xyz==0)
            {
               centerP_mm[xyz] = (float)(centerP_coor[xyz]-workspace_center[xyz])*mm_per_pixel;
               microrobot_position[xyz] = centerP_mm[xyz] /1000.0; //in metres
               xyz_distance[xyz] = goalP_coor[xyz] - centerP_coor[xyz]; //in pixels
               mag_distance = mag_distance + xyz_distance[xyz]*xyz_distance[xyz];
            }
            else
            {
                centerP_mm[xyz+1] = (float)(centerP_coor[xyz]-workspace_center[xyz])*mm_per_pixel;
                microrobot_position[xyz+1] = centerP_mm[xyz+1] /1000.0; //in metres
                xyz_distance[xyz+1] = goalP_coor[xyz] - centerP_coor[xyz]; //in pixels
                mag_distance = mag_distance + xyz_distance[xyz+1]*xyz_distance[xyz+1];
            }

        }

        //Pseudo Proportional Control: change F_mag based on distance to target
        if (mag_distance < 85) //was 100
        {
            //F_mag=0;  //for now - dont stop on goal

            on_goalP_counter++;
            if (on_goalP_counter > 0) //used to stop on goalP momentarily
            {
                //target reached - go on to next goal
                goalpoint_ind=goalpoint_ind+1;
            }

        }
        /*
        else if(mag_distance < 500)
        {
            F_mag=0.5e-6;
            BFaccept_rmc[3] = {5};
            BFaccept_rmc[4] = {0.35e-6};
            BFaccept_rmc[5] = {0.65e-6};
            on_goalP_counter=0;
        }
        */

        else
        {
            F_mag=0.95e-6;
            BFaccept_rmc[3] = {15};
            BFaccept_rmc[4] = {0.9e-6};
            BFaccept_rmc[5] = {1.5e-6};
            on_goalP_counter=0;
        }
                                    //Calculate Field and Force at Robot position
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////

        //Brmc is field at mircorobot position, Frmc may not be correct since microrobot_moment has not been updated
        field_force(Brmc,Frmc,magnet_rotR0rmc, microrobot_position, microrobot_moment);
        unitv(Brmc,3);
        for (int xyz=0; xyz<3; xyz++)
        {
            microrobot_moment[xyz]=Brmc[xyz]*m_mag;
        }
        //Calculate Frmc based on updated microrobot_moment
        field_force(Brmc,Frmc,magnet_rotR0rmc, microrobot_position, microrobot_moment);

        //Show direction of current force on microrobot
        update_linepoints(0, centerP_coor[0], centerP_coor[1], centerP_coor[0]+(int)(Frmc[0]*1e6*200), centerP_coor[1]+(int)(Frmc[2]*1e6*200) );

        //Show direction of current field on microrobot
        //update_linepoints(4, centerP_coor[0] +(int)(Brmc[0]*1e3*20/2), centerP_coor[1]+(int)(Brmc[1]*1e3*20/2), centerP_coor[0]-(int)(Brmc[0]*1e3*20/2), centerP_coor[1]-(int)(Brmc[1]*1e3*20/2) );

                                     //Calculate Desired Field and Force
        /////////////////////////////////////////////////////////////////////////////////////////////////////

        F_theta1 = atan2(xyz_distance[2], xyz_distance[0]);

        F_dir[0] = cos(F_theta1);  //might be faster to just make xyz_distance a unit vector, perhaps try it
        F_dir[1] = 0.0; //vertical
        F_dir[2] = sin(F_theta1)+0.2;

        unitv(F_dir,3);



        unitv(B_dir,3);
        //intermediatev(B_dir,Brmc,maxB_theta);
        //intermediatev(F_dir,Frmc,maxF_theta);

        for (int xyz=0; xyz<3; xyz++)
        {
            Bdes[xyz]=B_dir[xyz]*B_mag;
            Fdes[xyz]=F_dir[xyz]*F_mag; //F_mag or the F components of BF_accept will change with distance to way point
        }

        //Show direction of desired force on microrobot
        update_linepoints(2, centerP_coor[0], centerP_coor[1], centerP_coor[0]+(int)(Fdes[0]*1e6*200), centerP_coor[1]+(int)(Fdes[2]*1e6*200) );


                                    //Compare desired Force with current Force acting on robot - if similar, don't change magnet positions
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        //Check if force needs to be updated (i.e. if current force on robot is in right direction, dont change motor positions)
        Frmc_mag = magnitude(Frmc,3);
        FrmcFdes_angle = abs ( acos( (Frmc[0]*Fdes[0] + Frmc[1]*Fdes[1] +  Frmc[2]*Fdes[2]) / (Frmc_mag * F_mag) )*180.0/PI );

        if ( F_mag>0 && (FrmcFdes_angle < BFaccept_rmc[3] && Frmc_mag > BFaccept_rmc[4] && Frmc_mag < BFaccept_rmc[5]) )
        {
            newmotorpositions=false;
            //printf("Current Motor Positions are Good\n\n");
        }
        else if (F_mag==0 && Frmc_mag < 0.1e-6)
        {
            newmotorpositions=false;
           // printf("Current Motor Positions are Good\n\n");

        }
        else
        {
            newmotorpositions=true;

            //If Frmc and Fdes are really different, allow big angle change, otherwise small angle change
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            if(FrmcFdes_angle > 40)
            {
                parameters_rmc[2] = 170;  //parameters_rmc[] = {0.04, 0.1, 15, 3.5};
            }

            else if(FrmcFdes_angle > 16) //WAS 20
            {
                parameters_rmc[2] = 90;
            }

            else
                parameters_rmc[2] = 15;
        }

                            //If robot gets stuck on same spot, change magnet positions
        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        if ( abs(prev_location[0] - centerP_coor[0]) + abs(prev_location[1] - centerP_coor[1]) < 4) //robot is stuck on same spot
            stopped_counter++;
        else
            stopped_counter=0;

        if (stopped_counter > 130)
        {
            ("RESETTING MOTORS BECAUSE ROBOT STUCK! \n");
            newmotorpositions=true;
        }
        prev_location[0] = centerP_coor[0];
        prev_location[1] = centerP_coor[1];




        //plot_line(true, l_x, l_y, l_R, l_G, l_B);  //Update arrows on GUI video

        num_motorsturning = check_motorsdonemoving(); //Update how many motors are still moving

        //motor_turning = false;
        if (num_motorsturning < 2 && newmotorpositions == true)
        {
            printf("Starting Angle Search \n");
            //find next set of magnet angles
            sufficientlocalminGrad_norm(magnet_rotRrmc, magnet_rotR0rmc, max_num_iterations, BFaccept_rmc, parameters_rmc, microrobot_position, microrobot_moment, Bdes, Fdes);
            printf("Angle Search Completed\n");

            for(int count=0; count<8; count++)
            {
                motor_positionsrmc[count] = magnet_rotRrmc[count]*(180.0/PI)*400.0/360.0;
                magnet_rotR0rmc[count] = magnet_rotRrmc [count];
                //printf("%.1f ",motor_positionsrmc[count]);
            }
           // printf("\n\n");
            motors_moveToFunction(motor_positionsrmc);
            printf("Motor Positions Updated\n");

            field_force(Brmc,Frmc,magnet_rotR0rmc, microrobot_position, microrobot_moment);
            //show force that determineMagnetAngles just found
            //update_linepoints(6, centerP_coor[0], centerP_coor[1], centerP_coor[0]+(int)(Frmc[0]*1e6*200), centerP_coor[1]+(int)(Frmc[1]*1e6*200) );


            printf("\n Robot Position(mm): %.2f   %.2f   %.2f \n",microrobot_position[0]*1000.0, microrobot_position[1]*1000.0, microrobot_position[2]*1000.0);
            printf("Current Field (mT): %.2f   %.2f   %.2f   %.2f\n",Brmc[0]*1000, Brmc[1]*1000, Brmc[2]*1000,magnitude(Brmc,3)*1e3);
            printf("Current Force (uN): %.2f   %.2f   %.2f   %.2f\n",Frmc[0]*1e6, Frmc[1]*1e6, Frmc[2]*1e6,magnitude(Frmc,3)*1e6);
            printf("Robot Orient.     : %.2f   %.2f   %.2f \n",microrobot_moment[0]*1e6, microrobot_moment[1]*1e6, microrobot_moment[2]*1e6);
            printf("Desired Field (mT): %.2f   %.2f   %.2f \n",Bdes[0]*1000, Bdes[1]*1000, Bdes[2]*1000);//F_theta0*180/PI, F_theta1*180/PI,deltaF_theta*180/PI);
            printf("Desired Force (uN): %.2f   %.2f   %.2f \n",Fdes[0]*1e6, Fdes[1]*1e6, Fdes[2]*1e6);

        }
        printf("-");

        if (userinput_saved && goalpoint_ind>=num_goalpoints) //last saved waypoint reached
        {
            hor_waypoint_flag =0;
            //turn off lines
             //plot_line(false, l_x, l_y, l_R, l_G, l_B);
        }
        usleep(0.0825e6);


    }

    printf("Feedback Thread Ended\n");
    switch_record_centerP(0);

}




int init_magnetcontrol_thread(int control_method)
{
    switch (control_method)
    {
        case 0:     printf("@ the Beginning of Horizontal Way Point Click Thread.\n");
                    pthread_t magnet_thread0;

                    if (hor_waypoint_flag)
                    {
                        hor_waypoint_flag = 0;
                        usleep(5e5);
                    }
                    hor_waypoint_flag = 1;
                    userinput_click =1;

                    pthread_create(&magnet_thread0, NULL, horizontalwaypointfollowing_thread, NULL);  //start swimmer thread
                    break;

        case 1:     printf("@ the Beginning of Horizontal Saved Way Point Thread.\n");
                    pthread_t magnet_thread1;

                    if (hor_waypoint_flag)
                    {
                        hor_waypoint_flag = 0;
                        usleep(5e5);
                    }
                    hor_waypoint_flag = 1;
                    userinput_saved =1;
                    pthread_create(&magnet_thread1, NULL, horizontalwaypointfollowing_thread, NULL);  //start swimmer thread
                    break;

        case 2:     printf("@ the Beginning of Horizontal Force Heading Thread.\n");
                    pthread_t magnet_thread2;
                    if (hor_force_flag)
                    {
                        hor_force_flag = 0;
                        usleep(5e5);
                    }
                    hor_force_flag = 1;
                    pthread_create(&magnet_thread1, NULL, horizontalforceheading_thread, NULL);  //start swimmer thread
                    break;
        case 3:     printf("@ the Beginning of Vertical Saved Way Point Thread.\n");
                    pthread_t magnet_thread3;

                    if (ver_waypoint_flag)
                    {
                        ver_waypoint_flag = 0;
                        usleep(5e5);
                    }
                    ver_waypoint_flag = 1;
                    userinput_saved =1;
                    pthread_create(&magnet_thread3, NULL, verticalwaypointfollowing_thread, NULL);  //start swimmer thread
                    break;

    }

     return 1;
}

int stop_magnetcontrol_thread(void)
{
	//swimmer_actuationThreadControl = 0;
	hor_waypoint_flag = 0;
	hor_force_flag=0;
	return 1;
}

float * get_centerP_mm(void)
{
    int *centerP_coor;
    static float center_mm[2];
    centerP_coor = getCenterPointCoor();

    for (int xyz=0; xyz<2; xyz++)
        center_mm[xyz] = (float)(centerP_coor[xyz]-workspace_center[xyz])*mm_per_pixel;

    return center_mm;

}

void temp_gobutton_input(void)
{
    go_var = true;
}

void test_arraypassing(float aray[])
{
    for (int i=0; i<8; i++)
    {
        for (int xyz=0; xyz<3; xyz++)
        {
            printf(" %f  ",M_LOC[i][xyz]);
        }
        printf("\n");
    }

    /*
    for (int i=0; i<8; i++)
    {
        printf("%f\n",aray[i]);
    }
    */
}
