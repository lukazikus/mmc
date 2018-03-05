#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cmath>
#include <ctime>

//for measuring time
#include <time.h>

using namespace std;
//using namespace std::chrono;


//These parameters will be set by the user before the rest of the program
const float M_DIST = 0.07; //distance from each magnet to the workspace [m] (same for all)
unsigned const short int M_NUM = 8; //number of magnets
const short int M_AZIM[M_NUM] = {-20,   35,   240,    90,   210,   305,    70,   170}; //azimuth position of magnets [deg] - spherical coordinates
const short int M_INCL[M_NUM] = {100,  105,   125,    45,    50,    55,   180,   115}; //inclination position of magnets [deg]- spherical coordinates
const short int M_AXIS[2][M_NUM] ={ {90,   45,   -45,    0,    70,   -10,     0,   -60},
                                    {240, 215,   200,  130,   100,   225,    90,   125},
                                    }; //rotational axis position [deg]
const float M_STRN[M_NUM] = {16.612, 16.612,16.612, 16.612,16.612, 16.612,16.612, 16.612}; //magnet moment of the magnets [Am^2]
float M_theta[M_NUM] = {0, 0, 0, 0, 0, 0, 0, 0}; //magnet spin around rotational axis [rad]
double m_mag = 1e-6; //

#define PI 3.14159265

void magnetLocAxisConst(float M_LOC[][3],float M_ROT[][3], float c1[][3], float c2[][3], float dist, const short int az[], const short int incl[], const short int axis_rotation[][M_NUM], const float M_STRN[], const short int M_NUM);
void fieldForce(float B[], float F[], float M_LOC[][3], float M_theta[], float c1[][3], float c2[][3], float m_pos[], float m[], const short int M_NUM);
float vectorDifference(float dif_grad[], float M_LOC[][3], float M_theta[], float c1[][3], float c2[][3], float m_pos[], float m[], float Bo[3], float Fo[3], const short int M_NUM );
void searchSteepestDescent(float M_thetaf[], float parameters[], float M_LOC[][3], float M_thetai[], float c1[][3], float c2[][3], float m_pos[], float m[], float Bo[3], float Fo[3], const short int M_NUM );
void determineMagnetAngles(float M_theta[], int iter_max, float BFaccept[6], float parameters[], float M_LOC[][3], float c1[][3], float c2[][3], float m_pos[], float m[], float Bo[3], float Fo[3], const short int M_NUM );


void cross(float [3], float [3], float [3]);
void unitv(float [], int );
float magnitude(float [], int);
//float max(float [],int);

void print_vectorINT(const short int [], int);
void print_vectorFLOAT(float [], int);
void print_matrixINT(const short int vect[][3], int , int);
void print_matrixFLOAT(float vect[][3], int , int);

int main()
{
    //seed random number generator
    srand ( time(NULL) );

    //initialize empty matrices - will be filled by magnetLocAxisConst
    float M_LOC[M_NUM][3];
    float M_ROT[M_NUM][3];
    float c1[M_NUM][3];
    float c2[M_NUM][3];

    magnetLocAxisConst(M_LOC, M_ROT, c1, c2, M_DIST, M_AZIM, M_INCL, M_AXIS, M_STRN, M_NUM);

    //Print matricies
    //cout << "Magnet Cartesian Locations" << endl; print_matrixFLOAT(M_LOC,M_NUM,3);
    //cout << "Magnet Rotational Vectors" << endl; print_matrixFLOAT(M_ROT,M_NUM,3);
    //cout << "Magnet Constants" << endl; print_matrixFLOAT(c1,M_NUM,3);  print_matrixFLOAT(c2,M_NUM,3);  print_matrixFLOAT(c3,M_NUM,3);



    float Bo_mag = 2e-3, Fo_mag = 1e-6;
    //float Bo_dir[3] = {1, 0, 0}, Fo_dir[3] = {-1, 0, 1};
    float Bo_dir[] = {rand() % 50 - 25, rand() % 50 - 25, rand() % 50 - 25};
    float Fo_dir[] = {rand() % 50 - 25, rand() % 50 - 25, rand() % 50 - 25};
    unitv(Bo_dir,3); unitv(Fo_dir,3);

    float m_pos[3] = {0.0, 0.0, 0.0}; // microrobot position [m] - this will have to be provided by the control system
    //float m_dir[3] = {1, 1, 1}; //microrobot orientation - assumed to be aligned with field?
    //unitv(m_dir,3);
    float m[3]; m[0]=m_mag*Bo_dir[0]; m[1]=m_mag*Bo_dir[1]; m[2]=m_mag*Bo_dir[2];

    float Bo[3]; Bo[0]=Bo_mag*Bo_dir[0]; Bo[1]=Bo_mag*Bo_dir[1]; Bo[2]=Bo_mag*Bo_dir[2];
    float Fo[3]; Fo[0]=Fo_mag*Fo_dir[0]; Fo[1]=Fo_mag*Fo_dir[1]; Fo[2]=Fo_mag*Fo_dir[2];
    cout << "Desired Field" << endl; print_vectorFLOAT(Bo,3);
    cout << "Desired Force" << endl; print_vectorFLOAT(Fo,3);


    float M_theta[M_NUM];
    int iter_max = 1000;
    float BFaccept[6] = {10, 1e-3, 50e-3, 10, 0.75e-7, 3e-6}; //[Bangle_err Bmag_min Bmag_max Fangle_err Fmag_min Fmag_max ]
    float parameters[] = {0.2, 0.1}; // step size and cutoff
    determineMagnetAngles(M_theta, iter_max, BFaccept, parameters, M_LOC, c1, c2, m_pos, m, Bo, Fo, M_NUM );

    float B[3];
    float F[3];
    fieldForce(B,F,M_LOC,M_theta, c1, c2, m_pos, m, M_NUM);
    cout << "Field" << endl; print_vectorFLOAT(B,3);
    cout << "Force" << endl; print_vectorFLOAT(F,3);


    return 0;
}

/***************************************
*               Functions              *
***************************************/

void magnetLocAxisConst(float M_LOC[][3],float M_ROT[][3], float c1[][3], float c2[][3], float dist, const short int az[], const short int incl[], const short int axis_rotation[][M_NUM], const float M_STRN[], const short int M_NUM)
{
     float norm_angle;
     float initial_direction[3];

     //Parameters for Constants
     //float M_ROTdotidir; //always equals zero
     float cross_MROT_idir[3];

    for (int count=0; count<M_NUM; count++)
     {
         //cartesian locations - M_LOC
         M_LOC[count][0] = dist*sin(incl[count]*PI/180)*cos(az[count]*PI/180);
         M_LOC[count][1] = dist*sin(incl[count]*PI/180)*sin(az[count]*PI/180);
         M_LOC[count][2] = dist*cos(incl[count]*PI/180);

         //rotational vectors - M_ROT
         M_ROT[count][0]= sin(axis_rotation[1][count]*PI/180)*cos(axis_rotation[0][count]*PI/180);
         M_ROT[count][1]= sin(axis_rotation[1][count]*PI/180)*sin(axis_rotation[0][count]*PI/180);
         M_ROT[count][2]= cos(axis_rotation[1][count]*PI/180);

         norm_angle=axis_rotation[0][count]-90;

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

void fieldForce(float B[3], float F[3], float M_LOC[][3], float M_theta[], float c1[][3], float c2[][3], float m_pos[], float m[], const short int M_NUM )
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

float vectorDifference(float dif_grad[], float M_LOC[][3], float M_theta[], float c1[][3], float c2[][3], float m_pos[], float m[], float Bo[3], float Fo[3], const short int M_NUM )
{
    float dif;

    float B[3], F[3];
    fieldForce(B,F,M_LOC,M_theta, c1, c2, m_pos, m, M_NUM);

    float difB, difF;
    float BminusBo[3], FminusFo[3];

    for (int xyz=0; xyz<3; xyz++)
    {
        BminusBo[xyz] = B[xyz] - Bo[xyz];
        FminusFo[xyz] = F[xyz] - Fo[xyz];
    }
    difB = magnitude(BminusBo,3);
    difF = magnitude(FminusFo,3);

    float multB = 1/(M_NUM*4.77e-3);
    float multF = 1/(M_NUM*8.82e-8);

    dif = multB*difB + multF*difF;

    float M[3], r[3], r_mag, Mdotr;
    float dif_gradB=0, dif_gradF=0;
    float grad[3][3]; grad[0][0]=0; grad[0][1]=0; grad[0][2]=0; grad[1][0]=0; grad[1][1]=0; grad[1][2]=0; grad[2][0]=0; grad[2][1]=0; grad[2][2]=0;


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

void searchSteepestDescent(float M_thetaf[], float parameters[],float M_LOC[][3], float M_thetai[], float c1[][3], float c2[][3], float m_pos[], float m[], float Bo[3], float Fo[3], const short int M_NUM )
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
    float funcval = vectorDifference(funcval_grad, M_LOC,M_thetaf, c1, c2, m_pos, m, Bo, Fo, M_NUM);

    float last_funcval = funcval;


    for (int j=0; j<10000; j++)
    {
        for (int count=0; count<M_NUM; count++)
        {
            M_thetaf[count]=M_thetaf[count] - alpha*funcval_grad[count];
        }

        funcval = vectorDifference(funcval_grad, M_LOC,M_thetaf, c1, c2, m_pos, m, Bo, Fo, M_NUM);

//        cout << alpha*max(funcval_grad[0],funcval_grad[M_NUM-1]) << "      "<< alpha*min(funcval_grad[0],funcval_grad[M_NUM-1]) << endl;


        while ( funcval > last_funcval && alpha*max(funcval_grad[0],funcval_grad[M_NUM-1]) > min_funChange*PI/180.0 && alpha*min(funcval_grad[0],funcval_grad[M_NUM-1]) < min_funChange*PI/-180.0)
        {
            alpha = alpha * 0.5;
            for (int count=0; count<M_NUM; count++)
            {
                M_thetaf[count]=last_theta[count] - alpha*funcval_grad[count];
            }
            funcval = vectorDifference(funcval_grad, M_LOC,M_thetaf, c1, c2, m_pos, m, Bo, Fo, M_NUM);

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

void determineMagnetAngles(float M_theta[], int iter_max, float BFaccept[6], float parameters[], float M_LOC[][3], float c1[][3], float c2[][3], float m_pos[], float m[], float Bo[3], float Fo[3], const short int M_NUM )
{
    //what should be returned if no result is found??

    float Bo_mag = magnitude(Bo,3);
    float Fo_mag = magnitude(Fo,3);
    float B_mag, F_mag;
    float B_ang, F_ang;

    float M_thetai[M_NUM];
    float M_thetaf[M_NUM];
    float t;

    int iter_num = 1;

    while (iter_num < iter_max)
    {
        for (int count=0; count<M_NUM; count++)
        {
            t = rand() % 360;
            M_thetai[count] = t*PI/180; // try to ensure that new random number are being generated each time !!!!!!!!!!!!!!!!
        }

        searchSteepestDescent(M_thetaf, parameters, M_LOC, M_thetai, c1, c2, m_pos, m, Bo, Fo, M_NUM);

        float B[3], F[3];
        fieldForce(B,F,M_LOC,M_thetaf, c1, c2, m_pos, m, M_NUM);

        B_mag = magnitude(B,3);
        F_mag = magnitude(F,3);

        if (Bo_mag != 0 && Fo_mag != 0)
        {
            B_ang = abs ( acos( (Bo[0]*B[0] + Bo[1]*B[1] +  Bo[2]*B[2]) / (Bo_mag * B_mag) )*180/PI );
            F_ang = abs ( acos( (Fo[0]*F[0] + Fo[1]*F[1] +  Fo[2]*F[2]) / (Fo_mag * F_mag) )*180/PI );

            if (B_ang < BFaccept[0] && F_ang < BFaccept[3] && B_mag > BFaccept[1] && B_mag < BFaccept[2] && F_mag > BFaccept[4] && F_mag < BFaccept[5] )
            {
                break;
            }

        }
        else if (Bo_mag == 0 && Fo_mag != 0)
        {
            F_ang = abs ( acos( (Fo[0]*F[0] + Fo[1]*F[1] +  Fo[2]*F[2]) / (Fo_mag * F_mag) )*180/PI );

            if (B_mag < 0.1e-3 && F_ang < BFaccept[3] && F_mag > BFaccept[4] && F_mag < BFaccept[5] )
            {
                break;
            }

        }
        else if (Bo_mag != 0 && Fo_mag == 0)
        {
            B_ang = abs ( acos( (Bo[0]*B[0] + Bo[1]*B[1] +  Bo[2]*B[2]) / (Bo_mag * B_mag) )*180/PI );

            if (B_ang < BFaccept[0] && F_mag < 0.05e-6 && B_mag > BFaccept[1] && B_mag < BFaccept[2] )
            {
                break;
            }

        }
        else
        {
            if (B_mag < 0.1e-3 && F_mag < 0.05e-6)
            {
                break;
            }
        }

    iter_num++;
    }

    for (int count=0; count<M_NUM; count++)
    {
        M_theta[count] = M_thetaf[count];
    }

    if (iter_num==iter_max)
    {
        cout << "didnt find" << endl;
    }

}


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

/****************************************************
*       Functions to Print Vectors/Matricies        *
*****************************************************/

void print_vectorINT(const short int vect[], int length)
{
    for(int count=0; count<length; count++)
    {
        cout << vect[count] << "\t";
    }
    cout << endl << endl;
}

void print_vectorFLOAT(float vect[], int length)
{
    for(int count=0; count<length; count++)
    {
        cout << vect[count] << "\t";
    }
    cout << endl << endl;
}

void print_matrixINT(const short int vect[][3], int r, int c)
{
    for(int r_i=0; r_i<r; r_i++)
    {
        for(int c_i=0; c_i<c; c_i++)
        {
            cout << vect[r_i][c_i] << "\t";
        }
        cout << endl;

    }
    cout << endl << endl;
}

void print_matrixFLOAT(float vect[][3], int r, int c)
{
    for(int r_i=0; r_i<r; r_i++)
    {
        for(int c_i=0; c_i<c; c_i++)
        {
            cout << vect[r_i][c_i] << "\t";
        }
        cout << endl;

    }
    cout << endl << endl;
}
