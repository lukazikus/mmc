#include <gtk/gtk.h>        //declares functions, types and macros required by GTK+ applications.
#include <pthread.h>
//#include <glade/glade.h>
#include <gdk/gdk.h>
#include "callbacks.h"    // Question: How to connect callbacks.c to the GUI?
//#include "coilDisplay.h"
//#include "pololu.h"
//#include "vision.h"

#include "s826_subroutine.h"

int main (int argc, char *argv[])
{

//	printf("sum of 1 and 2 is %i.\n",sumt(1,2));
//	glutInit(&argc, argv);

    s826_init();

    GtkBuilder              *builder;
    GtkWidget               *window;
    extern GtkWidget        *coildraw, *field_drawingArea;
	extern GtkImage 	*videoWindow, *videoWindow2;
	extern GtkWidget 	*labelFPSreceive, *labelFPSreceive_xz;

	extern GtkLabel   *label_centerP_x, *label_centerP_y;
    extern GtkLabel   *label_centerP_x1, *label_centerP_y1;

    extern GtkLabel   *label_info_type, *label_info_plane, *label_info_angle;   // in callbacks.c

//  micro_fab Widgets
    //extern GtkEntry *display_temp;
    //extern GtkEntry *display_fab_status;

    gtk_init (&argc, &argv);

        builder = gtk_builder_new ();
        gtk_builder_add_from_file (builder, "tutorial.glade", NULL);

	// What is this line of code for? - JZ
        window   	= GTK_WIDGET (gtk_builder_get_object (builder, "window"));

	coildraw 	= GTK_WIDGET (gtk_builder_get_object (builder, "coildraw")); //need this so we can draw on the widget
	field_drawingArea 	= GTK_WIDGET (gtk_builder_get_object (builder, "field_drawingArea")); //need this so we can draw on the widget

    videoWindow   	= GTK_IMAGE  (gtk_builder_get_object (builder, "videoWindow"));
	videoWindow2   	= GTK_IMAGE  (gtk_builder_get_object (builder, "videoWindow2"));

	///
	label_centerP_x = GTK_LABEL (gtk_builder_get_object (builder, "label_centerP_x"));
	label_centerP_y = GTK_LABEL (gtk_builder_get_object (builder, "label_centerP_y"));
	label_centerP_x1 = GTK_LABEL (gtk_builder_get_object (builder, "label_centerP_x1"));
	label_centerP_y1 = GTK_LABEL (gtk_builder_get_object (builder, "label_centerP_y1"));

    label_info_type  = GTK_LABEL (gtk_builder_get_object (builder, "label_info_type"));
	label_info_plane = GTK_LABEL (gtk_builder_get_object (builder, "label_info_plane"));
	label_info_angle = GTK_LABEL (gtk_builder_get_object (builder, "label_info_angle"));
	///

	labelFPSreceive = GTK_WIDGET (gtk_builder_get_object (builder, "labelFPSreceive"));
	labelFPSreceive_xz=GTK_WIDGET(gtk_builder_get_object (builder, "labelFPSreceive_xz"));

    ///
	//display_temp = GTK_ENTRY (gtk_builder_get_object (builder, "temp_entry"));
	//display_fab_status = GTK_ENTRY (gtk_builder_get_object (builder, "fab_status_entry"));
	//gtk_entry_set_text(display_fab_status, "Click 'Read' and then 'Fabricate'.");

	// g_signal_connect(instance, detailed_signal, c_handler, data)
	// Connects a GCallback function to a signal for a particular object.
	// The handler will be called before the default handler of the signal.

	g_signal_connect(window, "key-press-event",   G_CALLBACK(key_event), NULL); //register keystrokes
	g_signal_connect(window, "key-release-event", G_CALLBACK(key_event_release), NULL); //register keystrokes

        gtk_builder_connect_signals (builder, NULL);
//g_signal_connect (G_OBJECT (window), "destroy",  G_CALLBACK (on_window_destroy),     NULL);
	gtk_window_set_title(GTK_WINDOW(window), "Magnet Controller");

	g_object_unref (G_OBJECT (builder));
	gtk_widget_show (window);


	gdk_threads_enter();
        gtk_main ();
	// Explain: Runs the main loop until gtk_main_quit() is called.
	//	    You can nest calls to gtk_main(). In that case gtk_main_quit() will make the innermost invocation of the main loop return.
	gdk_threads_leave();

	//pthread_t GUI_refresh;
    //pthread_create(&GUI_refresh, NULL, GUI_refresh_thread, NULL);

    return 0;
}
