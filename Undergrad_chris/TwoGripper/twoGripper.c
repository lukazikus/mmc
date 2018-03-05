#include "twoGripper.h"

static int fThread = 0;              		  // only allow running one thread in a file
static int fMode = 0;                         // 0: detect cargo and go pick it up; 1: with cargo initially and deliver it
static int fDrop = 0;                         // flag of dropping cargoes set by manual input
static Point_JZ gripperOne(100,200), gripperTwo(150,400);	// gripper center points
static Point_JZ goalOne(0,0), goalTwo(0,0);			// goal for gripper
static Point_JZ gripperOneNext(0,0), gripperTwoNext(0,0);	// next desired pos. for grippers
static Pair_JZ presInfo;                                // present info. of gripper pair
static Pair_JZ nextInfo;                                // next gripper pair info.
static Vector_JZ pres2nextCom;                    // present com to goal com
static Coil_JZ threeAxisCoil;						// coil object

static struct timeval start;
static double get_currentTime(void) {
    gettimeofday(&start, NULL);
    double presTime = (double) start.tv_sec + start.tv_usec*1e-6 ;   // seconds
    return presTime;
}

// Functions
static int action_sequence (int flag) {
	multiAgent_pause_thread (true);				// pause thread for open-loop action
    usleep(1e5);
	//coilCurrentClear();
	//optim_pause_thread (true);
	if (flag == 0) {											// grasp cargo
        float globalFieldAngle = multiagent_get_global_field_angle ();
		// rotate gripper slowly to +y (rotates fast will displace cargoes)
		//threeAxisCoil.set_pz_signal (0.0);									// set positive z coil signal
		//threeAxisCoil.set_nz_signal (0.0);
        for (int i = 0; i < 101; i ++) {                                            // move gripper downwards
            threeAxisCoil.set_px_signal (0.02 * cos(globalFieldAngle) * (100-i));	// set positive x coil signal
    		threeAxisCoil.set_nx_signal (0.02 * cos(globalFieldAngle) * (100-i));
            threeAxisCoil.set_py_signal (0.02 * sin(globalFieldAngle) * (100-i));	// set positive y coil signal
    		threeAxisCoil.set_ny_signal (0.02 * sin(globalFieldAngle) * (100-i));
            threeAxisCoil.set_pz_signal (-0.02 * i);							     // set positive z coil signal
    		threeAxisCoil.set_nz_signal (-0.02 * i);
            threeAxisCoil.output_signal ();
			usleep(5e3);
        }
        threeAxisCoil.clear_output ();
        threeAxisCoil.set_pz_signal (-5.0);							     // life gripper up and move +y
        threeAxisCoil.set_nz_signal (3.5);
        threeAxisCoil.output_signal ();
        usleep(10e5);
        //threeAxisCoil.set_px_signal (0.8);
        threeAxisCoil.set_py_signal (0.8);
        threeAxisCoil.output_signal ();
        fDrop = 0;
        while (fDrop == 0) {
            usleep(1e4);
        }
        threeAxisCoil.set_px_signal (0.0);
        threeAxisCoil.set_py_signal (0.0);
        threeAxisCoil.set_pz_signal (0.0);
        threeAxisCoil.set_nz_signal (-0.05);
        threeAxisCoil.output_signal ();
        //usleep(3e6);
        /*
		float oriDiff = adjust_angle_range(M_PI_2 - globalFieldAngle);
        printf("global angle %.2f, angle diff %.2f\n", globalFieldAngle, oriDiff);
		for (float ori = globalFieldAngle; abs_f(adjust_angle_range(M_PI_2 - ori)) > M_PI_2/30.0; ) {
			if (oriDiff > 0)
				ori = ori + M_PI_2 / 30.0;
			else
				ori = ori - M_PI_2 / 30.0;
			threeAxisCoil.set_px_signal (1.0 * cos(ori));						// set positive x coil signal
			threeAxisCoil.set_nx_signal (1.0 * cos(ori));
			threeAxisCoil.set_py_signal (1.0 * sin(ori));						// set positive y coil signal
			threeAxisCoil.set_ny_signal (1.0 * sin(ori));
            printf("current ori %.2f\n", ori);
			threeAxisCoil.output_signal ();
			usleep(1e4);
		}
		threeAxisCoil.set_px_signal (0.0);						// set positive x coil signal
		threeAxisCoil.set_nx_signal (0.0);
		threeAxisCoil.set_py_signal (1.0);						// set positive y coil signal
		threeAxisCoil.set_ny_signal (1.0);
		threeAxisCoil.output_signal ();
		usleep(1e5);
		threeAxisCoil.clear_output ();							// open gripper completely
		usleep(8e5);*/
        /// pulling grippers
        /*
        float angle = 0.0;
        float sinAng = 0.0, cosAng = 0.0;
        float amp = 1.0;
        for (int i = 0; i < 4500; i ++) {
            angle = M_PI_2 / 90.0 * i;
            sinAng = sin(angle);
            cosAng = cos(angle);
            if (sinAng > 0)
                amp = 1.0;
            else
                amp = 0.2;
            if (cosAng > 0)
                amp = 1.0;
            threeAxisCoil.set_py_signal (-1 * amp * cosAng);
            threeAxisCoil.set_ny_signal (-1 * amp * cosAng);
            threeAxisCoil.set_pz_signal (amp * sinAng);
            threeAxisCoil.set_nz_signal (amp * sinAng);
            threeAxisCoil.output_signal ();
            usleep(5e3);
        }*/
        /*
        float sinAvg = 0.0, cosAvg = 0.0;
        for (int i = 0; i < 70; i ++) {
            //if (i < 150) {
                //return_multi_agent_pos  (gripperOne, gripperTwo);	    // get gripper pos (only update gripperPos here)
                //vision_return_cargo_pos (   goalOne,    goalTwo);		// if before grasping cargo, use cargo pos as goal pos
                //Vector_JZ vecOne (gripperOne, goalOne);
            	//Vector_JZ vecTwo (gripperTwo, goalTwo);
                //sinAvg = sin (0.5 * (vecOne.angle + vecTwo.angle));
                //cosAvg = cos (0.5 * (vecOne.angle + vecTwo.angle));
                //printf("vec1 angle %.2f, vec2 angle %.2f, sin %.2f, cos %.2f\n", vecOne.angle, vecTwo.angle, sinAvg, cosAvg);
            //}
            threeAxisCoil.set_py_signal (2.0);						// gradient to pull gripper
            threeAxisCoil.set_ny_signal (-0.5);
            //threeAxisCoil.set_px_signal (0.0);
            //threeAxisCoil.set_nx_signal (-0.5);

            /*
            if (cosAvg > 0) {
                threeAxisCoil.set_px_signal (0.8 * cosAvg);
                threeAxisCoil.set_nx_signal (0.0);
                printf("px %.2f, nx %.2f\n", 0.8 * cosAvg, 0.0);
            } else if (cosAvg < 0) {
                threeAxisCoil.set_nx_signal (0.8 * cosAvg);
                threeAxisCoil.set_px_signal (0.0);
                printf("px %.2f, nx %.2f\n", 0.0, 0.8 * cosAvg);
            } else {
                threeAxisCoil.set_px_signal (0.0);
                threeAxisCoil.set_nx_signal (0.0);
                printf("px %.2f, nx %.2f\n", 0.0, 0.0);
            }*/
    		//threeAxisCoil.set_ny_signal (-0.8);
    		//threeAxisCoil.set_pz_signal (-0.2);						// make gripper points a little downwards and drag it up a little

            /*threeAxisCoil.output_signal ();
            usleep(1e4);
        }
        threeAxisCoil.set_py_signal (0.8);						// stop pulling gripper
        threeAxisCoil.set_ny_signal (-0.3);
        threeAxisCoil.output_signal ();
        usleep(1e6);
        threeAxisCoil.set_py_signal (0.8);						// stop pulling gripper
        threeAxisCoil.set_ny_signal (0.0);
        threeAxisCoil.output_signal ();
        //usleep(5e5);
        for (int i = 0; i < 101; i ++) {                        // turn gripper downwards
            //threeAxisCoil.set_pz_signal (-0.005 * i);
            threeAxisCoil.set_nz_signal (-0.005 * i);
            threeAxisCoil.set_py_signal (0.005 * (100-i));
            //threeAxisCoil.set_ny_signal (0.005 * (100-i));
            threeAxisCoil.output_signal ();
            usleep(5e3);
        }
        threeAxisCoil.clear_output ();
        threeAxisCoil.set_pz_signal (-0.5);
        threeAxisCoil.set_nz_signal (-0.5);
        threeAxisCoil.output_signal ();
        usleep(5e5);*/
        //threeAxisCoil.clear_output ();                              // open gripper completely
        usleep(2e6);
        for (int i = 0; i < 101; i ++) {
            threeAxisCoil.set_pz_signal (-0.01 * i);
            threeAxisCoil.set_nz_signal (-0.03 * i);                // pull the gripper down to enhance grasping
            threeAxisCoil.output_signal ();
            usleep(3e4);
        }
        usleep(5e5);
        for (int i = 0; i < 101; i ++) {
    		threeAxisCoil.set_py_signal (0.03 * i);
    		threeAxisCoil.set_ny_signal (0.03 * i);
    		threeAxisCoil.set_pz_signal (-0.03 * (100-i));
    		threeAxisCoil.set_nz_signal (-0.03 * (100-i));
    		threeAxisCoil.output_signal ();
    		usleep(5e3);
    	}
        for (int i = 0; i < 101; i ++) {
            threeAxisCoil.set_py_signal (0.03 * (100-i));
            threeAxisCoil.set_ny_signal (0.03 * (100-i));
            threeAxisCoil.set_pz_signal (0.03 * i);
            threeAxisCoil.set_nz_signal (0.03 * i);
            threeAxisCoil.output_signal ();
            usleep(5e3);
        }
        threeAxisCoil.clear_output ();
        usleep(15e5);
        for (int i = 0; i < 100; i ++) {
            threeAxisCoil.set_pz_signal (0.03 * i);						// rotate gripper to downwards
            threeAxisCoil.set_nz_signal (0.03 * i);
            threeAxisCoil.output_signal ();
            usleep(1e4);
        }
		//threeAxisCoil.set_pz_signal (-0.5);						// rotate gripper to downwards
        //threeAxisCoil.set_nz_signal (-0.5);
	    //threeAxisCoil.output_signal ();
		//usleep(8e5);
		//threeAxisCoil.clear_output ();							// open gripper completely
		//usleep(8e5);
        /*
		for (int i = 0; i < 100; i ++) {						// make gripper grasp
			threeAxisCoil.set_pz_signal (-0.016 * i);
			threeAxisCoil.set_nz_signal (-0.016 * i);
			threeAxisCoil.output_signal ();
			usleep(2e4);
		}*/
		for (int i = 0; i < 100; i ++) {
			threeAxisCoil.set_pz_signal (0.03 * (100-i));
			threeAxisCoil.set_nz_signal (0.03 * (100-i));
			threeAxisCoil.set_px_signal (0.03 * i);
			threeAxisCoil.set_nx_signal (0.03 * i);
			threeAxisCoil.output_signal ();
			usleep(2e4);
		}
	} else {													// release cargo
        float globalFieldAngle = multiagent_get_global_field_angle ();
		// rotate gripper slowly to +y (rotates fast will displace cargoes)
		//threeAxisCoil.set_pz_signal (0.0);									// set positive z coil signal
		//threeAxisCoil.set_nz_signal (0.0);
        for (int i = 0; i < 101; i ++) {                                            // move gripper downwards
            threeAxisCoil.set_px_signal (0.016 * cos(globalFieldAngle) * (100-i));	// set positive x coil signal
    		threeAxisCoil.set_nx_signal (0.016 * cos(globalFieldAngle) * (100-i));
            threeAxisCoil.set_py_signal (0.016 * sin(globalFieldAngle) * (100-i));	// set positive y coil signal
    		threeAxisCoil.set_ny_signal (0.016 * sin(globalFieldAngle) * (100-i));
            threeAxisCoil.set_pz_signal (-0.016 * i);							     // set positive z coil signal
    		threeAxisCoil.set_nz_signal (-0.016 * i);
            threeAxisCoil.output_signal ();
			usleep(5e3);
        }
        /*
        threeAxisCoil.clear_output ();
        threeAxisCoil.set_px_signal (1.6);
        threeAxisCoil.set_nx_signal (1.6);
        threeAxisCoil.output_signal ();
		for (int i = 0; i < 101; i ++) {
            threeAxisCoil.set_px_signal (0.016 * (100-i));
            threeAxisCoil.set_nx_signal (0.016 * (100-i));
			threeAxisCoil.set_pz_signal (-0.016 * i);
			threeAxisCoil.set_nz_signal (-0.016 * i);
			threeAxisCoil.output_signal ();
			usleep(2e4);
		}*/
		threeAxisCoil.clear_output ();
        usleep(3e6);
        //threeAxisCoil.set_pz_signal (-0.5);
        //threeAxisCoil.set_nz_signal (-0.5);
        //threeAxisCoil.output_signal ();
        /*
        for (int i =0; i < 101; i ++) {
            threeAxisCoil.set_px_signal (0.005 * i);
            threeAxisCoil.set_nx_signal (0.005 * i);
            threeAxisCoil.set_pz_signal (-0.005 * (100-i));
            threeAxisCoil.set_nz_signal (-0.005 * (100-i));
            threeAxisCoil.output_signal ();
            usleep(1e4);
        }
        threeAxisCoil.set_px_signal (0.0);						// gradient to pull gripper
        threeAxisCoil.set_nx_signal (1.0);
        threeAxisCoil.output_signal ();
        usleep(2e6);
        */
        /*
        float angle = 0.0;
        for (int i = 0; i < 1000; i ++) {
            angle = M_PI_2 / 45.0 * i;
            threeAxisCoil.set_px_signal (0.5 * sin(angle));
            threeAxisCoil.set_nx_signal (0.5 * sin(angle));
            threeAxisCoil.set_pz_signal (-0.5 * cos(angle));
            threeAxisCoil.set_nz_signal (-0.5 * cos(angle));
            threeAxisCoil.output_signal ();
            usleep(5e3);
        }*/
        //threeAxisCoil.clear_output ();
	}
	multiAgent_pause_thread (false);				// pause thread for open-loop action
	return 1;
}

// decide next desired gripper positions based on present gripper and cargo positions
static int set_next_gripper_pos (void) {
	Vector_JZ vecOne (gripperOne,goalOne);
	Vector_JZ vecTwo (gripperTwo,goalTwo);

	float step1 = 50.0, step2 = 50.0;					// step size
	if (vecOne.length > vecTwo.length)					// reduce step 2 proportionally
		step2 =  step2 * vecTwo.length / vecOne.length;	// make step 2 smaller
	else if (vecOne.length < vecTwo.length)
		step1 = step1 * vecOne.length / vecTwo.length;
	if (step1 > vecOne.length)							// avoid overshoot
		step1 = vecOne.length;
	if (step2 > vecTwo.length)
		step2 = vecTwo.length;
	if (step1 < 20)
		step1 = 20;
	if (step2 < 20)
		step2 = 20;

	gripperOneNext.x = gripperOne.x + step1 * ((float) vecOne.delX) / vecOne.length;
	gripperOneNext.y = gripperOne.y + step1 * ((float) vecOne.delY) / vecOne.length;
	gripperTwoNext.x = gripperTwo.x + step2 * ((float) vecTwo.delX) / vecTwo.length;
	gripperTwoNext.y = gripperTwo.y + step2 * ((float) vecTwo.delY) / vecTwo.length;
	vision_plot_line (true, gripperOne, goalOne, gripperTwo, goalTwo);			// draw straight line path from gripper pos. to cargo.
	return 1;
}

// set next gripper point in path follow tasks
static int set_next_gripper_pos_path_follow (int iPath) {
    int startX1, startY1, startX2, startY2;
    int endX1, endX2, endY1, endY2;
    int fXY = 0;                    // 0: consider x; 1: consider y
    switch (iPath) {
        case 1:
            startX1 = 140;
            startY1 = 80;
            startX2 = 260;
            startY2 = 160;
            endX1 = 140;
            endY1 = 400;
            endX2 = 260;
            endY2 = 320;
            fXY = 1;
            break;
        case 2:
            startX1 = 140;
            startY1 = 400;
            startX2 = 260;
            startY2 = 320;
            endX1 = 500;
            endY1 = 400;
            endX2 = 380;
            endY2 = 320;
            fXY = 0;
            break;
        case 3:
            startX1 = 500;
            startY1 = 400;
            startX2 = 380;
            startY2 = 320;
            endX1 = 500;
            endY1 = 80;
            endX2 = 380;
            endY2 = 160;
            fXY = 1;
            break;
        case 4:
            startX1 = 500;
            startY1 = 80;
            startX2 = 380;
            startY2 = 160;
            endX1 = 140;
            endY1 = 80;
            endX2 = 260;
            endY2 = 160;
            fXY = 0;
            break;
    }

    Point_JZ temp1, temp2;
    float progress[2] = {0.0, 0.0};
    float dev1=0.0, dev2 = 0.0;
    if (fXY == 0) {
        temp1.x = gripperOne.x;
        temp1.y = startY1;
        temp2.x = gripperTwo.x;
        temp2.y = startY2;
        dev1 = abs_f (gripperOne.y - startY1);
        dev2 = abs_f (gripperTwo.y - startY2);
        progress[0] = ((float)(gripperOne.x - startX1)) / ((float)(endX1 - startX1));
        progress[1] = ((float)(gripperTwo.x - startX2)) / ((float)(endX2 - startX2));
    } else {
        temp1.x = startX1;
        temp1.y = gripperOne.y;
        temp2.x = startX2;
        temp2.y = gripperTwo.y;
        dev1 = abs_f (gripperOne.x - startX1);
        dev2 = abs_f (gripperTwo.x - startX2);
        progress[0] = ((float)(gripperOne.y - startY1)) / ((float)(endY1 - startY1));
        progress[1] = ((float)(gripperTwo.y - startY2)) / ((float)(endY2 - startY2));
    }
    if (progress[0] < 0)
        progress[0] = 0;
    if (progress[1] < 0)
        progress[1] = 0;

    float step1 = 50.0, step2 = 50.0;					// step size
    if (progress[0] > progress[1])
        step1 = step2 / progress[0] * progress[1];
    else if (progress[0] < progress[1])
        step2 = step1 / progress[1] * progress[0];
    if (dev1 >= 20)
        step1 = 0;
    else
        step1 = step1 - dev1 / 20.0 * step1;
    if (dev2 >= 20)
        step2 = 0;
    else
        step2 = step2 - dev2 / 20.0 * step2;
    Vector_JZ vecOne (temp1,goalOne);
	Vector_JZ vecTwo (temp2,goalTwo);

	if (step1 > vecOne.length)							// avoid overshoot
		step1 = vecOne.length;
	if (step2 > vecTwo.length)
		step2 = vecTwo.length;

    if (fXY == 0) {
        gripperOneNext.x = gripperOne.x + step1 * ((float) vecOne.delX) / vecOne.length;
        gripperOneNext.y = startY1;
        gripperTwoNext.x = gripperTwo.x + step2 * ((float) vecTwo.delX) / vecTwo.length;
        gripperTwoNext.y = startY2;
    } else {
        gripperOneNext.x = startX1;
        gripperOneNext.y = gripperOne.y + step1 * ((float) vecOne.delY) / vecOne.length;
        gripperTwoNext.x = startX2;
    	gripperTwoNext.y = gripperTwo.y + step2 * ((float) vecTwo.delY) / vecTwo.length;
    }
	vision_plot_line (true, gripperOne, goalOne, gripperTwo, goalTwo);			// draw straight line path from gripper pos. to cargo.
	return 1;
}

// convert next gripper pos to COM, sep, and ori and set it to lower layer controller
static int transfer_info_to_lower_layer (void) {
	Vector_JZ vecCOM (gripperOneNext, gripperTwoNext);
	if (vecCOM.length < 70) {								// 2 grippers cannot be too close
		//gripperTwoNext.x = gripperTwo.x;
		//gripperTwoNext.y = gripperTwo.y;
        gripperOneNext.x = gripperOne.x;
		gripperOneNext.y = gripperOne.y;
		Vector_JZ newVecCOM (gripperOneNext, gripperTwoNext);
		vecCOM = newVecCOM;
	}
	Vector_JZ sep (gripperOne, gripperTwo);										// present sep. of two grippers
	vision_set_next_com (vecCOM.centerPt);										// sim. left-mouse-click on top-view frame; vision.c
	presInfo.update_info (gripperOne, gripperTwo);                // update present gripper pair info.
    if (presInfo.sep > 200)                                   // if 2 grippers are too far away, bring them closer first
        enable_or_disable_gradient(0);
    else
        enable_or_disable_gradient(1);
    nextInfo.update_info (gripperOneNext, gripperTwoNext);                // update next (desired) gripper pair info.
	pres2nextCom.update (presInfo.comPt, nextInfo.comPt);       // vector pointing from present com to goal com
	multiAgent_set_desired_rotation_and_sep (presInfo.sep, presInfo.ori, nextInfo.sep, nextInfo.ori, pres2nextCom.length, pres2nextCom.angle);		// multiagent.c
	//printf("info. for lower-layer controller is com (%d,%d), angle %.2f, length %.2f\n",vecCOM.centerPt.x,vecCOM.centerPt.y,vecCOM.angle,vecCOM.length);
	return 1;
}

// check if both gripper has arrived at cargo/destination
static int check_if_both_gripper_arrive (void) {
	int dis[2] = {0,0};			// distance from 2 grippers to their goals
	dis[0] = gripperOne.dis_to_another_pt(goalOne);
	dis[1] = gripperTwo.dis_to_another_pt(goalTwo);
	//printf("dis from gripper to goal are %d, %d\n", dis[0], dis[1]);
	if (dis[0] < 15 && dis[1] < 15)
		return 1;
	else
		return 0;
}

static int check_grasp_result (void) {
	return 1;
}

// close gripper and rotate it to horizontal heading
static int prepare_gripper (int input) {
	threeAxisCoil.clear_output ();
    if (input == 0) {
        for (int i = 0; i < 100; i ++) {
    		threeAxisCoil.set_pz_signal (-0.02 * i);
    		threeAxisCoil.set_nz_signal (-0.02 * i);
    		threeAxisCoil.output_signal ();
    		usleep(1e4);
    	}
        usleep(1e6);                                               // long wait time for checking if amplifiers are working
    	printf("after z closing\n");
        /*
        threeAxisCoil.clear_output ();
        threeAxisCoil.set_py_signal (1.0);
        threeAxisCoil.set_pz_signal (-4.0);							     // life gripper up and move +y
        threeAxisCoil.set_nz_signal (2.0);
        threeAxisCoil.output_signal ();
        usleep(3e6);
        threeAxisCoil.clear_output ();
        */
    	for (int i = 0; i < 101; i ++) {
    		threeAxisCoil.set_px_signal (0.016 * i);
    		threeAxisCoil.set_nx_signal (0.016 * i);
    		threeAxisCoil.set_pz_signal (-0.016 * (100-i));
    		threeAxisCoil.set_nz_signal (-0.016 * (100-i));
    		threeAxisCoil.output_signal ();
    		usleep(1e4);
    	}
    } else {
        for (int i = 0; i < 101; i ++) {
            threeAxisCoil.set_pz_signal (-0.03 * i);
            threeAxisCoil.set_nz_signal (-0.03 * i);
            threeAxisCoil.output_signal ();
            usleep(2e4);
        }
        usleep(2e6);
        for (int i = 0; i < 101; i ++) {
            threeAxisCoil.set_py_signal (0.03 * i);
            threeAxisCoil.set_ny_signal (0.03 * i);
            threeAxisCoil.set_pz_signal (-0.03 * (100-i));
            threeAxisCoil.set_nz_signal (-0.03 * (100-i));
            threeAxisCoil.output_signal ();
            usleep(2e4);
        }
        for (int i = 0; i < 101; i ++) {
            threeAxisCoil.set_py_signal (0.03 * (100-i));
            threeAxisCoil.set_ny_signal (0.03 * (100-i));
            threeAxisCoil.set_pz_signal (0.03 * i);
            threeAxisCoil.set_nz_signal (0.03 * i);
            threeAxisCoil.output_signal ();
            usleep(2e4);
        }
        threeAxisCoil.clear_output ();
        usleep(1e6);
        for (int i = 0; i < 100; i ++) {
            threeAxisCoil.set_pz_signal (0.03 * i);						// rotate gripper to downwards
            threeAxisCoil.set_nz_signal (0.03 * i);
            threeAxisCoil.output_signal ();
            usleep(1e4);
        }
        //threeAxisCoil.set_pz_signal (-0.5);						// rotate gripper to downwards
        //threeAxisCoil.set_nz_signal (-0.5);
        //threeAxisCoil.output_signal ();
        //usleep(8e5);
        //threeAxisCoil.clear_output ();							// open gripper completely
        //usleep(8e5);
        /*
        for (int i = 0; i < 100; i ++) {						// make gripper grasp
            threeAxisCoil.set_pz_signal (-0.016 * i);
            threeAxisCoil.set_nz_signal (-0.016 * i);
            threeAxisCoil.output_signal ();
            usleep(2e4);
        }*/
        for (int i = 0; i < 100; i ++) {
            threeAxisCoil.set_pz_signal (0.03 * (100-i));
            threeAxisCoil.set_nz_signal (0.03 * (100-i));
            threeAxisCoil.set_px_signal (0.03 * i);
            threeAxisCoil.set_nx_signal (0.03 * i);
            threeAxisCoil.output_signal ();
            usleep(2e4);
        }
    }
    /*
    float angle = 0.0;
    for (int i = 0; i < 1000; i ++) {
        angle = M_PI_2 / 90.0 * i;
        threeAxisCoil.set_px_signal (1.6 * cos(angle));
        threeAxisCoil.set_nx_signal (1.6 * cos(angle));
        threeAxisCoil.set_pz_signal (1.6 * sin(angle));
        threeAxisCoil.set_nz_signal (1.6 * sin(angle));
        threeAxisCoil.output_signal ();
        usleep(1e4);
    }*/
    /*
    for (int i = 0; i < 101; i ++) {
        threeAxisCoil.set_px_signal (0.016 * (100-i));
        threeAxisCoil.set_nx_signal (0.016 * (100-i));
        threeAxisCoil.set_pz_signal (0.016 * i);
        threeAxisCoil.set_nz_signal (0.016 * i);
        threeAxisCoil.output_signal ();
        usleep(2e4);
    }
    threeAxisCoil.set_pz_signal (0.5);
    threeAxisCoil.set_nz_signal (0.5);
    usleep(5e5);
    for (int i = 0; i < 101; i ++) {
		threeAxisCoil.set_px_signal (0.005 * i);
		threeAxisCoil.set_nx_signal (0.005 * i);
		threeAxisCoil.set_pz_signal (0.005 * (100-i));
		threeAxisCoil.set_nz_signal (0.005 * (100-i));
		threeAxisCoil.output_signal ();
		usleep(1e4);
	}
    for (int i = 0; i < 101; i ++) {
        threeAxisCoil.set_px_signal (0.005 * (100-i));
        threeAxisCoil.set_nx_signal (0.005 * (100-i));
        threeAxisCoil.set_pz_signal (-0.005 * i);
        threeAxisCoil.set_nz_signal (-0.005 * i);
        threeAxisCoil.output_signal ();
        usleep(1e4);
    }*/
    //threeAxisCoil.clear_output ();
    //threeAxisCoil.set_nx_signal (-0.3);
    //threeAxisCoil.output_signal ();
    //usleep(10e6);
	printf("after rotating to horizontal plane.\n");
	return 1;
}

// thread: control 2 grippers
static void * Thread_two_grippers (void * threadid) {
	printf("@ the start of Thread_two_grippers.\n");

	FILE *fp = fopen("twoGripper.txt","w");

	int fArrive = 0;										// if grippers have arrived @ cargo/destination
	int fGraspResult = 0;									// grasping result, get manual input or auto detection
	double presTime = 0.0;

	return_multi_agent_pos (gripperOne, gripperTwo);		// store init. gripper pos
    if (fMode == 0) {
        vision_return_cargo_pos (goalOne, goalTwo);
    	//goalOne.offset_y (-50);
    	//goalTwo.offset_y (-50);
        goalOne.offset_x (-55);
    	goalTwo.offset_x (-55);
        vision_set_show_cargo_flag (1);
    } else {
        fGraspResult = 1;
        goalOne.x = 180;
        goalOne.y = 170;
        goalTwo.x = 180;
        goalTwo.y = 300;
        set_Bmag_MA_to(11.0);
        vision_set_show_cargo_flag (0);
    }
	prepare_gripper (fMode);
	Binary_control_law ();						// init. binary control law
	//usleep(5e5);
	while (fThread) {										// move gripper to cargo loop
		return_multi_agent_pos (gripperOne, gripperTwo);					// get gripper pos (only update gripperPos here)
		if (fGraspResult == 0) {
			vision_return_cargo_pos (goalOne, goalTwo);		// if before grasping cargo, use cargo pos as goal pos
			goalOne.offset_y (-55);
			goalTwo.offset_y (-55);
            goalOne.offset_x (+12);
        	goalTwo.offset_x (-18);
		}
		presTime = get_currentTime ();
		fprintf(fp, "%.6f %d %d %d %d %d %d %d %d %d %d %d %d\n", presTime, gripperOne.x,gripperOne.y,gripperTwo.x,gripperTwo.y,goalOne.x,goalOne.y,goalTwo.x,goalTwo.y,gripperOneNext.x,gripperOneNext.y,gripperTwoNext.x,gripperTwoNext.y);
		fArrive = check_if_both_gripper_arrive();	// break from this loop if both grippers have arrived @ cargoes
		if (fArrive==1) {
            vision_set_show_cargo_flag (0);
			action_sequence(fGraspResult);				// grasp/release cargoes
			if (fGraspResult==0) {
				fGraspResult = check_grasp_result();	// check if grasping is successful
				if (fGraspResult == 1) {				// if grippers succeed in grasping cargoes
					//goalOne.x = 180;
					//goalOne.y = 170;
					//goalTwo.x = 180;
					//goalTwo.y = 300;
                    goalOne.x = 450;
					goalOne.y = 280;
					goalTwo.x = 450;
					goalTwo.y = 170;
                    set_Bmag_MA_to(11.0);
				}
			} else										// if grippers have successfully grasped cargoes before
				break;									// break from loop if grasping is successfully
		} else {										// IF has not arrived @ goal
			set_next_gripper_pos();						// decide the next desired pos for 2 grippers
			transfer_info_to_lower_layer();				// send COM, sep, ori to lower layer controller
		}
		usleep(1e5);
	}
	fThread = 0;
	multiagent_stop_thread ();
	fclose(fp);
	printf("@ the end of Thread_two_grippers.\n");
}

// start or stop 2 grippers thread
int twoGripper_start_or_stop (int d) {
	if (d==1) {
		fThread = 0;
		usleep(1e6);
		pthread_t threadTwoGrippers;
		fThread = 1;
		pthread_create(&threadTwoGrippers, NULL, Thread_two_grippers, NULL);
	} else
		fThread = 0;
	return 1;
}

static void * Thread_define_cargo_pos (void * threadid) {
	printf("@ the start of Thread_define_cargo_pos.\n");
	int iClick = 0;					// how many points have been specified
	int xCoor[2] = {0,0};
	int yCoor[2] = {0,0};
	int * mouse;
	int initMouseCoor[2] = {0,0};
	int flag = 0;							// new click found flag
	/// Store Init Mouse Value. Has to store values from the array (mouse_init). Beacuse the array's value changes with the value in vision.c.
	mouse = getGoalPointCoor();				// initial mouse value
	for (int i = 0; i < 2; i ++)
		initMouseCoor[i] = mouse[i];
	while (iClick < 2) {
		mouse = getGoalPointCoor();
		if (iClick == 0) {									// special case for first click
			if ( ( mouse[0] != initMouseCoor[0] ) || ( mouse[1] != initMouseCoor[1] ) )
				flag = 1;
		} else {													// not the first click
			if ( ( mouse[0] != xCoor[iClick-1] ) || ( mouse[1] != yCoor[iClick-1] ) )
				flag = 1;
		}

		/// Record Detected Point
		if (flag == 1) {
			xCoor[iClick] = mouse[0];
			yCoor[iClick] = mouse[1];
			flag = 0;
			iClick ++;
		}
	}

	goalOne.x = xCoor[0];
	goalOne.y = yCoor[0];
	goalTwo.x = xCoor[1];
	goalTwo.y = yCoor[1];

	vision_draw_cargo (xCoor, yCoor);			// draw cargo on top-view; vision.c
	fThread = 0;
	printf("@ the end of Thread_define_cargo_pos.\n");
}

int twoGripper_define_cargo_pos (void) {
	if (fThread == 1) {
		fThread = 0;
		usleep(5e5);
	}
	fThread = 1;
	pthread_t threadDefineCargo;
	pthread_create(&threadDefineCargo,NULL,Thread_define_cargo_pos,NULL);
	return 1;
}

static int set_path_end_point (int iPath) {
    switch (iPath) {
        case 0:
            goalOne.x = 140;                                // set path follow starting pt
            goalOne.y = 80;
            goalTwo.x = 260;
            goalTwo.y = 160;
            break;
        case 1:
            goalOne.x = 140;                                // set path follow starting pt
            goalOne.y = 400;
            goalTwo.x = 260;
            goalTwo.y = 320;
            break;
        case 2:
            goalOne.x = 500;                                // set path follow starting pt
            goalOne.y = 400;
            goalTwo.x = 380;
            goalTwo.y = 320;
            break;
        case 3:
            goalOne.x = 500;                                // set path follow starting pt
            goalOne.y = 80;
            goalTwo.x = 380;
            goalTwo.y = 160;
            break;
        case 4:
            goalOne.x = 140;                                // set path follow starting pt
            goalOne.y = 80;
            goalTwo.x = 260;
            goalTwo.y = 160;
            break;
    }
    return 1;
}

static void * Thread_path_follow (void * threadid) {
    printf("@ the start of Thread_path_follow.\n");
    FILE *fp = fopen("twoGripper.txt","w");
    prepare_gripper (0);                            // close gripper and rotate to horizontal angle
    return_multi_agent_pos (gripperOne, gripperTwo);
    Binary_control_law ();						// init. binary control law
    int iPath = 0;                                  // index of path
    set_path_end_point (iPath);
    int fArrive = 0;										// if grippers have arrived @ cargo/destination
	double presTime = 0.0;
    while (fThread) {										// move gripper to cargo loop
		return_multi_agent_pos (gripperOne, gripperTwo);					// get gripper pos (only update gripperPos here)
		presTime = get_currentTime ();
		fprintf(fp, "%.6f %d %d %d %d %d %d %d %d\n", presTime, gripperOne.x,gripperOne.y,gripperTwo.x,gripperTwo.y,goalOne.x,goalOne.y,goalTwo.x,goalTwo.y);
        printf("record!!!!!\n");
		fArrive = check_if_both_gripper_arrive();	// break from this loop if both grippers have arrived @ cargoes
		if (fArrive==1) {
            break;
		} else {										// IF has not arrived @ goal
			set_next_gripper_pos ();						// decide the next desired pos for 2 grippers
			transfer_info_to_lower_layer();				// send COM, sep, ori to lower layer controller
		}
		usleep(1e5);
	}

    while (fThread) {
        iPath ++;
        if (iPath > 4)
            break;
        set_path_end_point (iPath);
        while (fThread) {										// move gripper to cargo loop
            return_multi_agent_pos (gripperOne, gripperTwo);					// get gripper pos (only update gripperPos here)
            presTime = get_currentTime ();
            fprintf(fp, "%.6f %d %d %d %d %d %d %d %d\n", presTime, gripperOne.x,gripperOne.y,gripperTwo.x,gripperTwo.y,goalOne.x,goalOne.y,goalTwo.x,goalTwo.y);
            printf("record!!!!!\n");
            fArrive = check_if_both_gripper_arrive();	// break from this loop if both grippers have arrived @ cargoes
            if (fArrive==1) {
                break;
            } else {										// IF has not arrived @ goal
                set_next_gripper_pos_path_follow (iPath);						// decide the next desired pos for 2 grippers
                transfer_info_to_lower_layer();				// send COM, sep, ori to lower layer controller
            }
            usleep(1e5);
        }
    }
	fThread = 0;
	multiagent_stop_thread ();
    fclose(fp);
    printf("@ the end of Thread_path_follow.\n");
}

int twoGripper_start_or_stop_path_follow (int input) {
    if (input==1) {
		fThread = 0;
		usleep(1e6);
		pthread_t threadPathFollow;
		fThread = 1;
		pthread_create(&threadPathFollow, NULL, Thread_path_follow, NULL);
	} else
		fThread = 0;
	return 1;
}

int twoGripper_drop_gripper (void) {
    fDrop = 1;
    return 1;
}
