//#include <iostream>
//using namespace std;
#include <stdio.h>
#include <string.h>
#include <math.h>

int main()
{
/*	
	int i = 0;
	for (i=0;i<10;i++)
	{
		char str_centerPointCoor[50];
		strcpy (str_centerPointCoor, "Center Point Coor. x: ");
        
        	strcat (str_centerPointCoor, "Hello you");
		printf("%s.\n",str_centerPointCoor);
	}
*/

	float i = 1.0;
	float result = 0.0;
	for (i = -10.0; i<10.0;)
	{
		result = fmodf(i,M_PI);
		printf("i is %.1f, and result is %.1f.\n",i, result);
		i = i + 0.1;
	}
	
	float angle = atan2(-1,0);
	printf("angle is %.1f.\n",angle*180/M_PI);
	return 0;
}
