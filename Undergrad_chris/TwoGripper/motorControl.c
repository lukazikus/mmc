/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Program created by Jiachen Zhang on 2014-08-21.
// Program modified by Jiachen Zhang on 2014-08-21~29.
// Program modified by ... on ... (Please follow this format to add any following modification info.)
// 

#include "motorControl.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SUBROUTINE

uint currentControl(int step, int torque)
{
	uint step1 = 0;

	if(step == 1)
	{
		step1 = STEPMIN + (torque * 260);
	}
	else if(step == 2)
	{
		step1 = (STEPMIN + (torque * 260)) / 2;
	}
	else if(step == 4)
	{
		step1 = (STEPMIN + (torque * 260)) / 4;
	}
	else if(step == 8)
	{
		step1 = (STEPMIN + (torque * 260)) / 8;
	}
	else if(step == 16)
	{
		step1 = (STEPMIN + (torque * 260)) / 16;
	}

	return step1;
}

int motorInit(void)
{
	s826_init();
	uint motnum = 1;
	for (;motnum<=4;motnum++)
	{
		s826_doPin((motnum-1)*6+_motor_stp   ,0);
		//s826_aoPin(0,0,0);
		s826_doPin((motnum-1)*6+_motor_enable,1);   // Dis-select all motors.
		s826_doPin((motnum-1)*6+_motor_dir   ,0); 
	}		
}

int motorClose(void)
{
	s826_close();
}

int motorStall(int motnum)
{
	s826_doPin((motnum-1)*6+_motor_enable,0);
}

int motorRelease(int motnum)
{
	s826_doPin((motnum-1)*6+_motor_enable,1);
}

int motorGo(int motnum, int step_size, int number_of_steps, int torque)
{
	// sets direction of rotation
	int dir = (number_of_steps > 0) ? 1 : 0;   // dir is 1, positive direction. dir is 0, negative direction.
	number_of_steps = abs(number_of_steps);

	//digitalWrite(_motor_dir_1, dir);
	s826_doPin((motnum-1)*6+_motor_dir,dir);   // enable:0, dir:1, ms1:2, ms2:3, ms3:4, stp:5.
 
	//sets speed
	int step1 = currentControl(step_size,torque);

	uint motorMs1=0;
	uint motorMs2=0;
	uint motorMs3=0;

	switch (step_size)
	{
		case 1 : motorMs1 = 0; motorMs2 = 0; motorMs3 = 0; break;
		case 2 : motorMs1 = 1; motorMs2 = 0; motorMs3 = 0; break;
		case 4 : motorMs1 = 0; motorMs2 = 1; motorMs3 = 0; break;
		case 8 : motorMs1 = 1; motorMs2 = 1; motorMs3 = 0; break;
		case 16: motorMs1 = 1; motorMs2 = 1; motorMs3 = 1; break;
	}

	//sets step_size
	//digitalWrite(_motor_ms_11, LOW);
	//digitalWrite(_motor_ms_12, LOW);    
	//digitalWrite(_motor_ms_13, LOW);

	s826_doPin((motnum-1)*6+_motor_ms_1,motorMs1);
	s826_doPin((motnum-1)*6+_motor_ms_2,motorMs2);
	s826_doPin((motnum-1)*6+_motor_ms_3,motorMs3);

	//digitalWrite(_motor_enable_2, LOW);    // enable motor 1
	s826_doPin((motnum-1)*6+_motor_enable,0);

	waitUsePeriodicTimer(5e5);

	int i = 0;
	
	for(;i<number_of_steps;i++)
	{
		printf("i=%i.\n",i);
		//low to high transition moves one step
		//sbi(PORTE, 3);
		s826_doPin((motnum-1)*6+_motor_stp,1);
		//s826_doPin((motnum-1)*6+_motor_stp+1,1);
		//delayMicroseconds(step1); 
		//s826_aoPin(0,0,5);

		waitUsePeriodicTimer(5e3);
		//cbi(PORTE, 3);
		s826_doPin((motnum-1)*6+_motor_stp,0);
		//s826_doPin((motnum-1)*6+_motor_stp+1,0);
		//delayMicroseconds(step1);

		//s826_aoPin(0,0,0);
		waitUsePeriodicTimer(5e3);
	}

	//cbi(PORTE, 3);
	s826_doPin((motnum-1)*6+_motor_stp,0);
	//s826_doPin((motnum-1)*6+_motor_stp+1,0);
	//s826_aoPin(0,0,0);

	// Once the motor has reached the desired angle, it will be stalled, i.e. there is still current inside the motor.
	//s826_doPin((motnum-1)*6+_motor_enable,1);

	return (16/step_size)* number_of_steps;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Drive all 4 motors simutaneously.
//
// Input:

int motorAllGo(uint *stepSize,int *nStep,int torque)
{
	//printf("Inside motorAllGo.\n MotorSelect is %i\n",motorSelect);

	// Check which motor(s) will turn.
	uint motor_enable[] = {0,0,0,0};
	int i = 0;
	for (i=0;i<4;i++)
	{
		motor_enable[i] = (nStep[i] != 0) ? 1 : 0;
	}


	int dir = 0;
	uint nAbsStep[]={0,0,0,0};
	uint motorMs1=0;
	uint motorMs2=0;
	uint motorMs3=0;
	uint delayTime = currentControl(1,torque);

	for (i=0;i<4;i++)
	{
		if (motor_enable[i])
		{
			//printf("Inside motor_enable loop\n");
			dir = (nStep[i] > 0) ? 1 : 0;
			nAbsStep[i] = abs(nStep[i]);

			s826_doPin(i*6+_motor_dir,dir);   // Set dir PIN for Motor_i.

			switch (stepSize[i])
			{
				case 1 : motorMs1 = 0; motorMs2 = 0; motorMs3 = 0; break;
				case 2 : motorMs1 = 1; motorMs2 = 0; motorMs3 = 0; break;
				case 4 : motorMs1 = 0; motorMs2 = 1; motorMs3 = 0; break;
				case 8 : motorMs1 = 1; motorMs2 = 1; motorMs3 = 0; break;
				case 16: motorMs1 = 1; motorMs2 = 1; motorMs3 = 1; break;
			}

			// Set stepSize for Motor_i.
			s826_doPin(i*6+_motor_ms_1,motorMs1);
			s826_doPin(i*6+_motor_ms_2,motorMs2);
			s826_doPin(i*6+_motor_ms_3,motorMs3);

			// Enable Motor_i (in case it is not enabled yet).
			s826_doPin(i*6+_motor_enable,0);
		}
	}

	uint flag = 0;
	uint count = 0;
	//uint outputV = 0;
	while (flag<4)
	{
		flag = 0;

		for (i=0;i<4;i++)
		{
			if (motor_enable[i] && (nAbsStep[i] > count))
			{
				//outputV = outputV | ((1)<<i);
				s826_doPin(i*6+_motor_stp,1);
				
			}
			else flag++;
		}
		
		//s826_doWhole(outputV);
		waitUsePeriodicTimer(5e3);

		// Change all DO to 0;
		//outputV = outputV & 0;   
		//s826_doWhole(outputV);
		for (i=0;i<4;i++) s826_doPin(i*6+_motor_stp,0);

		waitUsePeriodicTimer(5e3);
				
		count++;
	}

	// Once the motor has reached the desired angle, it will be stalled, i.e. there is still current inside the motor.
	//for (i=0;i<4;i++) s826_doPin(i*6+_motor_enable,1);
}



