#ifndef CALLB
#define CALLB

#include <fcntl.h>      // for open()
#include <gtk/gtk.h>
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <pthread.h>
#include <sys/time.h>
#include <gdk/gdkkeysyms.h>

//#include <s826_subroutine.h>
//#include <s826api.h>

//#include <signal.h>
//#include <GL/glut.h>
//#include <SDL/SDL.h>
//#include <opencv/cv.h>
//#include <opencv/highgui.h>
//#include <string>

//#include "magnetLowLevel.h"
//#include "LXRTclass.h"
//#include "coilDisplay.h"
//#include "magnetControl.h"
//#include "NetController.h"
//#include "callbacksTS.h"
#include "vision.h"
//#include "FWcamera.h"
#include "s826_subroutine.h"
#include "motorControl.h"
#include "quadstep.h"

#include "coilFieldControl.h"
#include "gripper.h"                                            // everything about magnetic gripper control
#include "rotatingmagnetControl.h"
#include "AutoFabrication.h"
#include "multiswimmertrial.h"
#include "twoswimmers.h"
#include "multiagent.h"
#include "optimizationagents.h"
#include "math_subroutine.h"
#include "twoGripper.h"
#include "undergrad.h"

void* controlThread(void*);
void* drawThread(void*);

extern "C" {  //use to fix C++ name mangling problem, when compiling with g++ instead of gcc. see http://cboard.cprogramming.com/cplusplus-programming/146982-gtkplus-cplusplus-compiler-signal-handlers.html

	//void on_coildraw_draw (GtkWidget *widget, cairo_t   *cr, gpointer   data); //used with gtk3
	void on_coildraw_draw (GtkWidget *widget, GdkEventExpose *event, gpointer data); //used with gtk2
    void on_field_drawingArea_expose_event (GtkWidget *widget, GdkEventExpose *event, gpointer data);

	void on_window_destroy (GtkWidget *widget, gpointer data);

	void on_b_undergradTest_toggled (GtkToggleButton *togglebutton, gpointer data);	// Undergradate demonstration

	//void on_button1_clicked (GtkWidget *widget, gpointer data);
	void on_tButton_draw_field_toggled (GtkToggleButton *togglebutton, gpointer data);
	void on_current_clicked(GtkWidget *widget, gpointer data);
	void on_enableFields_toggled (GtkToggleButton *togglebutton, gpointer data);
	void on_videoButton_toggled (GtkToggleButton *togglebutton, gpointer data);

    //void* GUI_refresh_thread(void*threadid);
	void on_twoGripper_drop_clicked (GtkWidget *widget, gpointer data);
	void on_twoGripper_pathFollow_toggled (GtkToggleButton *togglebutton, gpointer data);	// toggle two-gripper path following
	void on_twoGripper_start_toggled (GtkToggleButton *togglebutton, gpointer data);	// toggle start/stop 2 gripper control (high level & low level)
	void on_twoGripper_specifyCargo_clicked (GtkWidget *widget, gpointer data);			// define cargo positions by mouse clicking
	void on_AO_sine_toggled (GtkToggleButton *togglebutton, gpointer data);
	//void on_Direction_toggled (GtkToggleButton *togglebutton, gpointer data);
	//void on_changeDir_clicked (GtkWidget *widget, gpointer data);
	void on_button_swimmerHeadingTest_clicked (GtkWidget *widget, gpointer data);
	void on_dirAdjust_amp_changed (GtkEditable *editable, gpointer user_data);


	void on_swimmer_controlLoop_toggled (GtkToggleButton *togglebutton, gpointer data);
	//void on_videoWindow_expose_event (GtkWidget *widget, GdkEventExpose *event, gpointer data);
	//void on_videoWindow2_expose_event (GtkWidget *widget, GdkEventExpose *event, gpointer data);
	void on_b_swimmerSpeed_clicked (GtkWidget *widget, gpointer data);    // Swimmer Speed Test
    void on_tB_cilia_toggled (GtkToggleButton *togglebutton, gpointer data);   // cilia test

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Swimmer Control
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void on_swimmer_actuationFreq_changed (GtkEditable *editable, gpointer user_data);
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Feedback Control
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void on_swimmer_pointFollow_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_swimmer_pathFollow_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_tB_UT_follow_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_cB_actuation_A_control_toggled (GtkToggleButton *togglebutton, gpointer data);   // Actuation Amplitude Feedback Control
    void on_swimmer_proportional_changed (GtkEditable *editable, gpointer user_data);
    void on_swimmer_integral_changed (GtkEditable *editable, gpointer user_data);

    void on_swimmer_A_h_changed (GtkEditable *editable, gpointer user_data);                 // Actuation amplitude - horizontal
    void on_swimmer_A_v_changed (GtkEditable *editable, gpointer user_data);                 // Actuation amplitude - vertical

    void on_swimmer_dir_bias_changed (GtkEditable *editable, gpointer user_data);          // Directional bias
    void on_swimmer_vertical_bias_changed (GtkEditable *editable, gpointer user_data);     // Vertical bias
    void on_swimmer_dirAdjust_angle_changed (GtkEditable *editable, gpointer user_data);   // Compensation angle for net magnetization

    void on_swimmer_swim_dir_reverse_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_tB_swimmer_multiSwimmer_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_b_swimmer_1stQuad_clicked (GtkWidget *widget, gpointer data);    // Move to +x +y 45 degree
    void on_b_swimmer_2ndQuad_clicked (GtkWidget *widget, gpointer data);
    void on_b_swimmer_3rdQuad_clicked (GtkWidget *widget, gpointer data);
    void on_b_swimmer_4thQuad_clicked (GtkWidget *widget, gpointer data);
    void on_b_swimmer_stop_clicked    (GtkWidget *widget, gpointer data);

    void on_b_magnetizationHeading_clicked (GtkWidget *widget, gpointer data);

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Swimmer: Series Test Button Clicked
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void on_b_seriesTest_clicked (GtkWidget *widget, gpointer data);

    void on_sB_swimmer_actuationAcontrol_l_changed (GtkEditable *editable, gpointer user_data);
    void on_sB_swimmer_actuationAcontrol_h_changed (GtkEditable *editable, gpointer user_data);

    void on_b_swimmer_pX_clicked (GtkWidget *widget, gpointer data);
    void on_b_swimmer_pY_clicked (GtkWidget *widget, gpointer data);
    void on_b_swimmer_nX_clicked (GtkWidget *widget, gpointer data);
    void on_b_swimmer_nY_clicked (GtkWidget *widget, gpointer data);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Magnet Detection -- Edited by Zhe
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void on_field_angle_theta_changed (GtkEditable *editable, gpointer user_data);
    void on_field_angle_phi_changed (GtkEditable *editable, gpointer user_data);
    void on_reset_field_button_clicked (GtkWidget *widget, gpointer data);
    void on_magnet_detection_toggled (GtkToggleButton *togglebutton, gpointer data);

    void on_show_box_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_show_process_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_close_diameter_changed (GtkEditable *editable, gpointer user_data);
    //magnet feedback covoid on_button1_clicked (GtkWidget *widget, gpointer data)

    void on_test90_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_kp_changed (GtkEditable *editable, gpointer user_data);
    void on_ki_changed (GtkEditable *editable, gpointer user_data);
    void on_destination_angle_changed (GtkEditable *editable, gpointer user_data);
    void on_show_destination_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_show_field_direction_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_reverse_field_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_bending_go_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_torque_angle_changed (GtkEditable *editable, gpointer user_data);
    //temperature control
    void on_temp_control_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_destination_temp_changed (GtkEditable *editable, gpointer user_data);
    void on_temp_go_toggled (GtkToggleButton *togglebutton, gpointer data);
    //rotational field
    void on_rotational_field_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_fab_amp_changed (GtkEditable *editable, gpointer user_data);
    void on_fab_fre_changed (GtkEditable *editable, gpointer user_data);
    void on_rotationalfield_go_toggled (GtkToggleButton *togglebutton, gpointer data);
    //automatic feeding
    void on_auto_feeding_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_feeding_distance_changed (GtkEditable *editable, gpointer user_data);
    void on_feeding_speed_changed (GtkEditable *editable, gpointer user_data);
    void on_feeding_go_toggled (GtkToggleButton *togglebutton, gpointer data);
    //manual feeding
    void on_feeding_increments_changed (GtkEditable *editable, gpointer user_data);
    void on_manual_override_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_feeder_extend_button_clicked (GtkWidget *widget, gpointer data);
    void on_feeder_retract_button_clicked (GtkWidget *widget, gpointer data);
    //automatic fabrication
    void on_shape_read_clicked (GtkWidget *widget, gpointer data);
    void on_shape_fab_toggled (GtkToggleButton *togglebutton, gpointer data);
    // make an "S"
    void on_s_go_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_radius_s_changed (GtkEditable *editable, gpointer user_data);
    void on_time_s_changed (GtkEditable *editable, gpointer user_data);
    // field control callbacks
    void on_coil_selection_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_field1_fab_changed (GtkEditable *editable, gpointer user_data);
    void on_field2_fab_changed (GtkEditable *editable, gpointer user_data);
    void on_field3_fab_changed (GtkEditable *editable, gpointer user_data);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void on_AO_sineFreq_changed (GtkEditable *editable, gpointer user_data);
	//void on_AO_sineFreq_activate (GtkEntry *entry, gpointer  user_data);
	//void on_AO_sineFreq_editing_done (GtkCellEditable *editable, gpointer user_data);
	void on_current1_changed (GtkEditable *editable, gpointer user_data);
	//void on_current1_key_press_event(GtkWidget *widget, GdkEventKey *event);
	void on_current2_changed (GtkEditable *editable, gpointer user_data);
	void on_current3_changed (GtkEditable *editable, gpointer user_data);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void on_edgemap_toggled (GtkToggleButton *togglebutton, gpointer data);
	void on_binary_toggled (GtkToggleButton *togglebutton, gpointer data);
	//void on_spinbutton1_changed (GtkEditable *editable, gpointer user_data);
	void on_spinbutton2_changed (GtkEditable *editable, gpointer user_data);
	void on_cannyLow_changed (GtkEditable *editable, gpointer user_data);
	void on_cannyHigh_changed (GtkEditable *editable, gpointer user_data);
	void on_gain_changed (GtkEditable *editable, gpointer user_data);
	void on_shutter_changed (GtkEditable *editable, gpointer user_data);
	void on_dilate_changed (GtkEditable *editable, gpointer user_data);
	void on_visionParam1_changed (GtkEditable *editable, gpointer user_data);
	void on_visionParam2_changed (GtkEditable *editable, gpointer user_data);

    void on_cB_2ndRect_toggled (GtkToggleButton *togglebutton, gpointer data);   // Set 2nd Rect. Requirement
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// X-Z Camera
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void on_togglebutton1_xz_toggled (GtkToggleButton *togglebutton, gpointer data);
	void on_togglebutton2_xz_toggled (GtkToggleButton *togglebutton, gpointer data);
	void on_edgemap_xz_toggled (GtkToggleButton *togglebutton, gpointer data);
	void on_binary_xz_toggled (GtkToggleButton *togglebutton, gpointer data);
	void on_spinbutton1_xz_changed (GtkEditable *editable, gpointer user_data);
	void on_spinbutton2_xz_changed (GtkEditable *editable, gpointer user_data);
	void on_gain_xz_changed (GtkEditable *editable, gpointer user_data);
	void on_shutter_xz_changed (GtkEditable *editable, gpointer user_data);
	void on_visionParam1_xz_changed (GtkEditable *editable, gpointer user_data);
	void on_visionParam2_xz_changed (GtkEditable *editable, gpointer user_data);
	void on_sidecam_on_toggled (GtkToggleButton *togglebutton, gpointer data);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void on_stepsize_changed (GtkEditable *editable, gpointer user_data);
	void on_fileinput_clicked(GtkWidget *widget, gpointer data);
	void on_fileinput_frequency_changed (GtkEditable *editable, gpointer user_data);
	void on_clickgo_clicked(GtkWidget *widget, gpointer data);

	//calibrate the sensors
	void on_zero_output_changed (GtkEditable *editable, gpointer user_data);
	void on_gaussmeter_output1_changed (GtkEditable *editable, gpointer user_data);
	void on_oscilloscope_output1_changed (GtkEditable *editable, gpointer user_data);
        void on_calibrate_clicked (GtkWidget *widget, gpointer data);
	void on_oscilloscope_output2_changed (GtkEditable *editable, gpointer user_data);
	void on_measure_clicked (GtkWidget *widget, gpointer data);

	void on_max_speed_button_changed (GtkEditable *editable, gpointer user_data);
	void on_max_accel_button_changed (GtkEditable *editable, gpointer user_data);

	void on_magent_b1_clicked (GtkWidget *widget, gpointer data);
	void on_magnetdiagnose_clicked (GtkWidget *widget, gpointer data);
	void on_reset_clicked (GtkWidget *widget, gpointer data);

	void on_magnet_lock_clicked (GtkWidget *widget, gpointer data);
	void on_magnet_unlock_clicked (GtkWidget *widget, gpointer data);
	void on_magnet1_toggled (GtkToggleButton *togglebutton, gpointer data);
	void on_magnet2_toggled (GtkToggleButton *togglebutton, gpointer data);
	void on_magnet3_toggled (GtkToggleButton *togglebutton, gpointer data);
	void on_magnet4_toggled (GtkToggleButton *togglebutton, gpointer data);
	void on_magnet5_toggled (GtkToggleButton *togglebutton, gpointer data);
	void on_magnet6_toggled (GtkToggleButton *togglebutton, gpointer data);
	void on_magnet7_toggled (GtkToggleButton *togglebutton, gpointer data);
	void on_magnet8_toggled (GtkToggleButton *togglebutton, gpointer data);
	void on_allmagnets_toggled (GtkToggleButton *togglebutton, gpointer data);

	void on_magnet_go_clicked (GtkWidget *widget, gpointer data);
	void on_magnet_stop_clicked (GtkWidget *widget, gpointer data);

	void on_togglebutton_waveform_toggled (GtkToggleButton *togglebutton, gpointer data);
	void on_togglebutton_feedback_toggled (GtkToggleButton *togglebutton, gpointer data);

	void on_angle_mag1_changed (GtkEditable *editable, gpointer user_data);
	void on_angle_mag2_changed (GtkEditable *editable, gpointer user_data);
	void on_angle_mag3_changed (GtkEditable *editable, gpointer user_data);
	void on_angle_mag4_changed (GtkEditable *editable, gpointer user_data);
	void on_angle_mag5_changed (GtkEditable *editable, gpointer user_data);
	void on_angle_mag6_changed (GtkEditable *editable, gpointer user_data);
	void on_angle_mag7_changed (GtkEditable *editable, gpointer user_data);
	void on_angle_mag8_changed (GtkEditable *editable, gpointer user_data);
	//void on_angle_mag8_activate (GtkEditable *editable, gpointer user_data);
	void on_Bx_changed (GtkEditable *editable, gpointer user_data);
	void on_By_changed (GtkEditable *editable, gpointer user_data);
	void on_Bz_changed (GtkEditable *editable, gpointer user_data);
	void on_Fx_changed (GtkEditable *editable, gpointer user_data);
	void on_Fy_changed (GtkEditable *editable, gpointer user_data);
	void on_Fz_changed (GtkEditable *editable, gpointer user_data);

	gboolean on_videoWindow_button_press_event( GtkWidget *widget, GdkEventButton *event, gpointer data);
	gboolean on_videoWindow2_button_press_event(GtkWidget *widget, GdkEventButton *event, gpointer data);
	gboolean key_event(GtkWidget *widget, GdkEventKey *event);
	gboolean key_event_release(GtkWidget *widget, GdkEventKey *event);

    void on_b_ms_trial_clicked (GtkWidget *widget, gpointer data);
    void on_swimmer_ac_angle_changed (GtkEditable *editable, gpointer user_data);
    void on_b_ms_stop_clicked (GtkWidget *widget, gpointer data);
    void on_ad_net_magnet_dir_changed (GtkEditable *editable, gpointer user_data);
    void on_b_netMDir_clicked (GtkWidget *widget, gpointer data);
    void on_ad_net_magnet_dir2_changed (GtkEditable *editable, gpointer user_data);
    void on_b_ms_UT_follow_clicked (GtkWidget *widget, gpointer data);

    /////////////////////////////////////////////////////////
    // Magnetic Gripper
    /////////////////////////////////////////////////////////

    void on_b_rotationDemo_clicked (GtkWidget   *widget  , gpointer data     );
    void on_sb_gripping_changed    (GtkEditable *editable, gpointer user_data);
    void on_sb_gripperX_changed    (GtkEditable *editable, gpointer user_data);
    void on_sp_gripperY_changed    (GtkEditable *editable, gpointer user_data);

    /////////////////////////////////////////////////////////
    // Multi_agent control (created by Mohammad)
    /////////////////////////////////////////////////////////
    void on_imageprocessing_multiagent_clicked (GtkWidget *widget, gpointer data);
    void on_robot_fabrication_clicked (GtkWidget *widget, gpointer data);
    void on_moldfieldmag_changed (GtkEditable *editable, gpointer user_data);
    void on_cB_3rdcirc_toggled (GtkToggleButton *togglebutton, gpointer data);              // image processing of the circle orientation
    void on_visionHough_minRadius_changed (GtkEditable *editable, gpointer user_data);      // to get min radius for Houghcircle()
    void on_visionHough_maxRadius_changed (GtkEditable *editable, gpointer user_data);      // to get max radius for Houghcircle()
    void on_Houghcanny1_changed(GtkEditable *editable, gpointer user_data);                // to get canny1 for Houghcircle()
    void on_Houghcanny2_changed(GtkEditable *editable, gpointer user_data);                // to get canny2 for Houghcircle()
    void on_HoughminDis_changed(GtkEditable *editable, gpointer user_data);               // to get min distance for Houghcircle()
    void on_HoughBinary_changed (GtkEditable *editable, gpointer user_data);               // set the level of binary adjustment
    void on_Hougherosion_changed (GtkEditable *editable, gpointer user_data);               // set the level of erosion adjustment
    void on_Houghdilation_changed (GtkEditable *editable, gpointer user_data);             // set the level of dilation adjustment
    void on_show_agent_circle_toggled (GtkToggleButton *togglebutton, gpointer data);       // set agent show
    void on_preProcessMA_toggled (GtkToggleButton *togglebutton, gpointer data);              //pre-process toggle button

    void on_3coil_toggled (GtkToggleButton *togglebutton, gpointer data);                   // 3-coil setup selection
    void on_2coil_toggled (GtkToggleButton *togglebutton, gpointer data);                   // 2-coil setup selection
    void on_2coil_current1_changed (GtkEditable *editable, gpointer user_data);             // large coil field adjustment for 2-coil system
    void on_2coil_current2_changed (GtkEditable *editable, gpointer user_data);             // Small coil field adjustment for 2-coil system
    void on_2Agent_MA_toggled (GtkToggleButton *togglebutton, gpointer data);               // enables actuation field for multi-agent control::: Multi-agent
    void on_2Agents_stop_clicked (GtkWidget *widget, gpointer data);                        // stops actuation field for multi-agent control::: Multi-agent
    void on_MultiAgent_pX_clicked (GtkWidget *widget, gpointer data);                       // set +x direction actuation
    void on_MultiAgent_pY_clicked (GtkWidget *widget, gpointer data);                       // set +y direction actuation
    void on_MultiAgent_nX_clicked (GtkWidget *widget, gpointer data);                       // set -x direction actuation
    void on_MultiAgent_nY_clicked (GtkWidget *widget, gpointer data);                       // set -y direction actuation
    void on_MA_1stQuad_clicked (GtkWidget *widget, gpointer data);                          // Move to +x +y 45 degree: 1st Quadrant
    void on_MA_2ndQuad_clicked (GtkWidget *widget, gpointer data);                          // 2nd Quadrant
    void on_MA_3rdQuad_clicked (GtkWidget *widget, gpointer data);                          // 3rd Quadrant
    void on_MA_4thQuad_clicked (GtkWidget *widget, gpointer data);                          // 4th Quadrant
    void on_HoughBlur_changed (GtkEditable *editable, gpointer user_data);                 // To set Blur Marker Size in Hough transform
    void on_HoughMindistance_changed (GtkEditable *editable, gpointer user_data);          // To change Hough Binary Threshold
    void on_HoughSwapRatio_changed (GtkEditable *editable, gpointer user_data);            // To change Hough Swap Ratio between true and false states of swapping

    ////////// Control law call function for multi-aegnt project
    void on_Binarycontrollaw_clicked (GtkWidget *widget, gpointer data);                    // set up Binary controller
    void on_ConstrainControllaw_clicked (GtkWidget *widget, gpointer data);               // set up constrain law controller
    void on_Adaptivecontrollaw_clicked (GtkWidget *widget, gpointer data);                  // set up Adaptive controller for Multiagent

    void on_Pcontrol_toggled (GtkToggleButton *togglebutton, gpointer data);                //set up the P-controller
    void on_Pcontrol_straight_toggled (GtkToggleButton *togglebutton, gpointer data);      //set up the P-controller Straight-Line
    void on_ROTperiod_MA_changed (GtkEditable *editable, gpointer user_data);            // To set the period of zero-rotation in P-controller design  :: how fast to correction the rotation

    void on_P_controlMA_changed (GtkEditable *editable, gpointer user_data);                // To set K parameter in P-controller design
    void on_constrainK_changed (GtkEditable *editable, gpointer user_data);            // To set K parameter in constrained control law
    void on_constrainFactor_changed (GtkEditable *editable, gpointer user_data);            // To set Factor parameter in constrained control law
    void on_GOAL_MA_changed(GtkEditable *editable, gpointer user_data);                     // To set desired distance parameter in P-controller design
    void on_desDis_MA_changed (GtkEditable *editable, gpointer user_data);              // To set desired distance in multi-agent
    void on_desAng_MA_changed (GtkEditable *editable, gpointer user_data);              // To set desired angle in multi-agent
    void on_des_pull_ang_changed(GtkEditable *editable, gpointer user_data);            // To set desired pulling angle in multi-agent
    void on_Coil_ratio_changed(GtkEditable *editable, gpointer user_data);            // To set the ratio between gradients in Y and X pair of coils in multi-agent

    void on_Bmag_MA_changed(GtkEditable *editable, gpointer user_data);                     // To set B_ext magnitude parameter in P-controller design
    void on_ZfieldAdj_changed(GtkEditable *editable, gpointer user_data);            // To adjust z-field in constrained

    void on_AdaptiveLambda_MA_changed(GtkEditable *editable, gpointer user_data);            // To set Lambda in Adaptive-controller design
    void on_AdaptiveK_MA_changed(GtkEditable *editable, gpointer user_data);                 // To set K in Adaptive-controller design
    void on_AdaptGammaF_MA_changed(GtkEditable *editable, gpointer user_data);             // To set Fluid Gamma in Adaptive-controller design
    void on_AdaptGammaM_MA_changed(GtkEditable *editable, gpointer user_data);             // To set Mass Gamma in Adaptive-controller design
    void on_AdaptivePhi_MA_changed(GtkEditable *editable, gpointer user_data);            // To set Boundary thickness Phi in Adaptive-controller design

    void on_pix2mm_MA_changed(GtkEditable *editable, gpointer user_data);                   // To change the scale from mm to pixel

    void on_opt2agent_clicked (GtkWidget *widget, gpointer data);                         // start two-agent optimization
    void on_moment_MA_changed(GtkEditable *editable, gpointer user_data);               // To set m in optimization 2-agent
    void on_Reg1_MA_changed(GtkEditable *editable, gpointer user_data);               // To set regulator 1 in optimization 2-agent logistic function
    void on_Reg2_MA_changed(GtkEditable *editable, gpointer user_data);               // To set regulator 2 in optimization 2-agent logistic function
    void on_Reg3_MA_changed(GtkEditable *editable, gpointer user_data);            // To set regulator 3 for transverse angle in optimization 2-agent logistic function

    void on_Stepsize_multiple_changed(GtkEditable *editable, gpointer user_data);     // To decrease step-size
    void on_der_t_changed(GtkEditable *editable, gpointer user_data);                  // To  adjust step-time
    void on_stopthreshold_MA_changed(GtkEditable *editable, gpointer user_data);            // To  adjust threshold in stop criterion
    void on_pullingMA_clicked (GtkWidget *widget, gpointer data);                      // To turn on pulling control

    void on_pullingGAin_MA_changed(GtkEditable *editable, gpointer user_data);            // To  adjust the pulling gain
    void on_satAdj_changed(GtkEditable *editable, gpointer user_data);            // To set saturation field in Y and X pair of coils in multi-agent
    void on_integral_changed(GtkEditable *editable, gpointer user_data);               // To  adjust the integral gain

    void on_record_button_clicked (GtkWidget *widget, gpointer data);            //To set up region of interest

}
int sumt(int a, int b);

#endif
