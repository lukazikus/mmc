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
#include "vision.h"
#include "s826_subroutine.h"
#include "PageTwistField.h"
#include "PageGeneralControl.h"
#include "undergrad.h"

void* controlThread(void*);
void* drawThread(void*);

extern "C" {  //use to fix C++ name mangling problem, when compiling with g++ instead of gcc. see http://cboard.cprogramming.com/cplusplus-programming/146982-gtkplus-cplusplus-compiler-signal-handlers.html

    gboolean key_event (GtkWidget *widget, GdkEventKey *event);
    gboolean key_event_release(GtkWidget *widget, GdkEventKey *event);
  void on_field_drawingArea_expose_event (GtkWidget *widget, GdkEventExpose *event, gpointer data);
	void on_window_destroy (GtkWidget *widget, gpointer data);
	void on_tButton_draw_field_toggled (GtkToggleButton *togglebutton, gpointer data);
	void on_current_clicked(GtkWidget *widget, gpointer data);
	void on_videoButton_toggled (GtkToggleButton *togglebutton, gpointer data);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Magnet Detection -- Edited by Zhe
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // sinusoidal field (2d) thread functions
    void on_sinusoidal_field_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_fab_heading_s_changed (GtkEditable *editable, gpointer user_data);
    void on_fab_amp_s_changed (GtkEditable *editable, gpointer user_data);
    void on_fab_fre_s_changed (GtkEditable *editable, gpointer user_data);
    void on_sinusoidalfield_go_toggled (GtkToggleButton *togglebutton, gpointer data);
    //rotational field (vertical)
    void on_rotational_field_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_fab_heading_changed (GtkEditable *editable, gpointer user_data);
    void on_fab_amp_changed (GtkEditable *editable, gpointer user_data);
    void on_fab_fre_changed (GtkEditable *editable, gpointer user_data);
    void on_rotationalfield_go_toggled (GtkToggleButton *togglebutton, gpointer data);
    // rotational field (horizontal) thread functions
    void on_rotational_field_h_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_fab_heading_h_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_rotationalfield_go_h_toggled (GtkToggleButton *togglebutton, gpointer data);
    // sawtooth mode thread functions
    void on_sawtooth_mode_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_sawtooth_go_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_peak_angle_st_changed (GtkEditable *editable, gpointer user_data);
    // field control callbacks
    void on_cb_coil_selection_changed(GtkComboBox *combo_box, gpointer user_data);
    void on_field1_fab_changed (GtkEditable *editable, gpointer user_data);
    void on_field2_fab_changed (GtkEditable *editable, gpointer user_data);
    void on_field3_fab_changed (GtkEditable *editable, gpointer user_data);
    void on_field_mag_fab_changed (GtkEditable *editable, gpointer user_data);
    void on_xy_3d_fab_changed (GtkEditable *editable, gpointer user_data);
    void on_xz_3d_fab_changed (GtkEditable *editable, gpointer user_data);
    void on_reset_field_button_clicked (GtkWidget *widget, gpointer data);

    // twisting field callbacks
    void on_twisting_walking_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_twisted_walking_enabler_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_theta_heading_changed (GtkEditable *editable, gpointer user_data);
    void on_beta_tilt_changed (GtkEditable *editable, gpointer user_data);
    void on_ang_freq_changed (GtkEditable *editable, gpointer user_data);
    void on_ang_span_changed (GtkEditable *editable, gpointer user_data);
    void on_tfield_mag_changed (GtkEditable *editable, gpointer user_data);
    // manual field callbacks
    void on_manual_field_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_manual_field_enabler_toggled (GtkToggleButton *togglebutton, gpointer data);
    void on_bx_mag_changed (GtkEditable *editable, gpointer user_data);
    void on_by_mag_changed (GtkEditable *editable, gpointer user_data);
    void on_bz_mag_changed (GtkEditable *editable, gpointer user_data);
    void on_dbx_mag_changed (GtkEditable *editable, gpointer user_data);
    void on_dby_mag_changed (GtkEditable *editable, gpointer user_data);
    void on_dbz_mag_changed (GtkEditable *editable, gpointer user_data);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// X-Y Camera
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void on_edgemap_toggled (GtkToggleButton *togglebutton, gpointer data);
	void on_binary_toggled (GtkToggleButton *togglebutton, gpointer data);
	void on_cannyLow_changed (GtkEditable *editable, gpointer user_data);
	void on_cannyHigh_changed (GtkEditable *editable, gpointer user_data);
	void on_gain_changed (GtkEditable *editable, gpointer user_data);
	void on_shutter_changed (GtkEditable *editable, gpointer user_data);
	void on_dilate_changed (GtkEditable *editable, gpointer user_data);
	void on_binaryThreshold_changed (GtkEditable *editable, gpointer user_data);
	void on_visionParam2_changed (GtkEditable *editable, gpointer user_data);
  void on_3d_indicator_toggled (GtkToggleButton *togglebutton, gpointer data);
  void on_2d_indicator_toggled (GtkToggleButton *togglebutton, gpointer data);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// X-Z Camera
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void on_edgemap_xz_toggled (GtkToggleButton *togglebutton, gpointer data);
	void on_binary_xz_toggled (GtkToggleButton *togglebutton, gpointer data);
	void on_cannyLow_xz_changed (GtkEditable *editable, gpointer user_data);
	void on_cannyHigh_xz_changed (GtkEditable *editable, gpointer user_data);
	void on_gain_xz_changed (GtkEditable *editable, gpointer user_data);
	void on_shutter_xz_changed (GtkEditable *editable, gpointer user_data);
	void on_dilate_xz_changed (GtkEditable *editable, gpointer user_data);
	void on_binaryThreshold_xz_changed (GtkEditable *editable, gpointer user_data);
	void on_visionParam2_xz_changed (GtkEditable *editable, gpointer user_data);
  void on_3d_indicator_xz_toggled (GtkToggleButton *togglebutton, gpointer data);
  void on_2d_indicator_xz_toggled (GtkToggleButton *togglebutton, gpointer data);
	void on_topcam_on_toggled (GtkToggleButton *togglebutton, gpointer data);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Catch Mouse Click Event
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	gboolean on_videoWindow_button_press_event( GtkWidget *widget, GdkEventButton *event, gpointer data);
	gboolean on_videoWindow2_button_press_event(GtkWidget *widget, GdkEventButton *event, gpointer data);

  void on_current_tab_changed (GtkNotebook *notebook, GtkNotebookPage *notebook_page, int page, gpointer data);

  // toggle the keyboard detection thread
  void on_undergrad_keyboard_toggled (GtkToggleButton *togglebutton, gpointer data);
  // toggle walking thread
  void on_undergrad_walk_toggled (GtkToggleButton *togglebutton, gpointer data);
  void on_undergrad_xGradientSpin_changed (GtkEditable *editable, gpointer user_data);
  void on_undergrad_yGradientSpin_changed (GtkEditable *editable, gpointer user_data);


  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////       MMC project functions  ////
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void on_Pgain_changed (GtkEditable *editable, gpointer user_data);   // thic changes Proportional control gain, must use correct types
  void on_startMMC_clicked (GtkWidget *widget, gpointer data);     // this starts the pulling thread in MMC proj
  void on_stopMMC_clicked (GtkWidget *widget, gpointer data);     // this starts the pulling thread in MMC proj
  void on_HoughBlur_changed (GtkEditable *editable, gpointer user_data);           // To set Blur Marker Size in Hough transform
}

#endif
