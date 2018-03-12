////////////////////////////////////////////////////////////////////////////////////////
// File      : main.c
// Function  : Call GUI
// Edited by :
////////////////////////////////////////////////////////////////////////////////////////
#include <gtk/gtk.h>
#include <pthread.h>
#include <gdk/gdk.h>
#include "callbacks.h"    // Question: How to connect callbacks.c to the GUI?
#include "s826_subroutine.h"

int main (int argc, char *argv[])
{
    s826_init();

    GtkBuilder *builder;
    GtkWidget  *window;

    // defined in callbacks.c
    extern GtkWidget *field_drawingArea;
	  extern GtkImage  *videoWindow, *videoWindow2;
	  extern GtkLabel  *labelFPSreceive, *labelFPSreceive_xz;
    extern GtkLabel  *label_info_type, *label_info_plane, *label_info_angle;   // in callbacks.c
    extern GtkLabel  *label_current_temp; // in callbacks.c
    extern GtkLabel  *label_sampled_area, *label_current_area; // in callbacks.
    extern GtkWidget *field1_fab, *field2_fab, *field3_fab, *g_field_mag_fab, *xy_3d_fab, *xz_3d_fab;

    gtk_init (&argc, &argv);

    builder = gtk_builder_new ();
    gtk_builder_add_from_file (builder, "tutorial.glade", NULL);

	// What is this line of code for? - JZ
    window   	= GTK_WIDGET (gtk_builder_get_object (builder, "window"));

	  field_drawingArea 	= GTK_WIDGET (gtk_builder_get_object (builder, "field_drawingArea")); //need this so we can draw on the widget

    videoWindow   	= GTK_IMAGE  (gtk_builder_get_object (builder, "videoWindow"));
	  videoWindow2   	= GTK_IMAGE  (gtk_builder_get_object (builder, "videoWindow2"));

    label_info_type  = GTK_LABEL (gtk_builder_get_object (builder, "label_info_type"));
	  label_info_plane = GTK_LABEL (gtk_builder_get_object (builder, "label_info_plane"));
	  label_info_angle = GTK_LABEL (gtk_builder_get_object (builder, "label_info_angle"));

	  labelFPSreceive    = GTK_LABEL (gtk_builder_get_object (builder, "labelFPSreceive"));
	  labelFPSreceive_xz = GTK_LABEL (gtk_builder_get_object (builder, "labelFPSreceive_xz"));

    label_current_temp = GTK_LABEL (gtk_builder_get_object (builder, "label_current_temp"));

    label_sampled_area = GTK_LABEL (gtk_builder_get_object (builder, "label_sampled_area"));
    label_current_area = GTK_LABEL (gtk_builder_get_object (builder, "label_current_area"));

    field1_fab = GTK_WIDGET(gtk_builder_get_object(builder, "field1_fab"));
    field2_fab = GTK_WIDGET(gtk_builder_get_object(builder, "field2_fab"));
    field3_fab = GTK_WIDGET(gtk_builder_get_object(builder, "field3_fab"));
    g_field_mag_fab = GTK_WIDGET(gtk_builder_get_object(builder, "field_mag_fab"));
    xy_3d_fab = GTK_WIDGET(gtk_builder_get_object(builder, "xy_3d_fab"));
    xz_3d_fab = GTK_WIDGET(gtk_builder_get_object(builder, "xz_3d_fab"));

	// g_signal_connect(instance, detailed_signal, c_handler, data)
	// Connects a GCallback function to a signal for a particular object.
	// The handler will be called before the default handler of the signal.

    gtk_builder_connect_signals (builder, NULL);
    // g_signal_connect (G_OBJECT (window), "destroy",  G_CALLBACK (on_window_destroy),     NULL);
	  gtk_window_set_title(GTK_WINDOW(window), "Magnet Controller");

	  g_object_unref (G_OBJECT (builder));
	  gtk_widget_show (window);
	  gdk_threads_enter();
    gtk_main ();
	// Explain: Runs the main loop until gtk_main_quit() is called.
	// You can nest calls to gtk_main(). In that case gtk_main_quit() will make the innermost invocation of the main loop return.
	  gdk_threads_leave();

	  //pthread_t GUI_refresh;
    //pthread_create(&GUI_refresh, NULL, GUI_refresh_thread, NULL);
    return 0;
}
