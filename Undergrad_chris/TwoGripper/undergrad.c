#include "undergrad.h"

static int fThread = 0;
static Point_JZ agent(320,240);	    // agent center points
static Coil_JZ threeAxisCoil;						// coil object

// demonstration experiment: a magnet walks along rectangles
static void * Thread_demo(void * threadid) {
    printf("@ the start of Thread_demo.\n");
    int goalX[4] = {200,200,440,440};
    int goalY[4] = {150,330,330,150};
    int iGoal = 0;                          // index of current goal point
    float angle = 0.0;                      // angle of vector from present position to goal
    float dis = 0.0;                        // distance from present position to goal
    float fieldAngle = 0.0;
    float fieldAmp = 0.1;
    while (fThread) {
        return_single_agent_pos(agent);
        angle = atan2(goalY[iGoal]-agent.y, goalX[iGoal]-agent.x);
        dis = pow(goalY[iGoal]-agent.y,2) + pow(goalX[iGoal]-agent.x,2);
        printf("agent pos (%d, %d), dis square %.2f, angle %.2f.\n", agent.x, agent.y, dis, angle);
        if (dis < 300) {

            iGoal++;
            if (iGoal > 3)  iGoal = 0;
            threeAxisCoil.clear_output ();
            fieldAngle = 0;
            //break;
        } else {
            fieldAngle = fieldAngle + M_PI / 20;
            while (fieldAngle > M_PI * 2)   fieldAngle = fieldAngle - 2 * M_PI;
            threeAxisCoil.set_px_signal (fieldAmp * cos(fieldAngle) * cos(angle));
            threeAxisCoil.set_nx_signal (fieldAmp * cos(fieldAngle) * cos(angle));
            threeAxisCoil.set_py_signal (fieldAmp * cos(fieldAngle) * sin(angle));
            threeAxisCoil.set_ny_signal (fieldAmp * cos(fieldAngle) * sin(angle));
            threeAxisCoil.set_pz_signal (fieldAmp * (-1 *sin(fieldAngle)));
            threeAxisCoil.set_nz_signal (fieldAmp * (-1 *sin(fieldAngle)));
            threeAxisCoil.output_signal ();
        }
        usleep(1e5);
    }
    threeAxisCoil.clear_output ();
    printf("@ the end of Thread_demo.\n");
}

// start/stop the demonstration thread
int undergrad_start_stop_demo (int input) {
    if (input==1) {
		fThread = 0;
		usleep(1e6);
		pthread_t threadDemo;
		fThread = 1;
		pthread_create(&threadDemo, NULL, Thread_demo, NULL);
	} else
		fThread = 0;
	return 1;
}
