#include "general_header.hpp"
#include "GUI_master.hpp"       // thread update GUI
//#include "actuation.hpp"        // actuation thread
//#include "keyboard.hpp"

static void activate ( GtkApplication *app, gpointer user_data ) {
    GtkWidget * window;
    GtkImage  * vidWindow;
    GtkLabel  * timeLabel;

    GtkBuilder * builder;

    builder = gtk_builder_new_from_file ("GUI.glade");
    window    = GTK_WIDGET ( gtk_builder_get_object (builder, "mainWindow") );
    vidWindow = GTK_IMAGE  (gtk_builder_get_object (builder, "vidWindow")); // get video window handler
    timeLabel = GTK_LABEL  ( gtk_builder_get_object (builder, "timeLabel" ) );    // get time display label handler

    /* link keyboard input */
    g_signal_connect (window,   "key-press-event", G_CALLBACK(        key_event), NULL);
    g_signal_connect (window, "key-release-event", G_CALLBACK(key_event_release), NULL);

    gtk_builder_connect_signals (builder, NULL);
    g_object_unref (G_OBJECT (builder));

    gtk_application_add_window (app, GTK_WINDOW (window));
    gtk_widget_show (window);
    GUI_master_activate ( vidWindow,timeLabel );
}

static void on_window_destroy (GtkApplication *app, gpointer user_data) {
    GUI_master_deactivate();
}

/* program starts here */
int main ( int argc, char ** argv ) {
    GtkApplication *app;
    int status;

    app = gtk_application_new ("ca.utoronto.mie.microrobotics", G_APPLICATION_FLAGS_NONE);  // create a new GTKApplication instance

    g_signal_connect (app, "activate", G_CALLBACK (activate)         , NULL);  // connect a GCallback function to a signal for a particular object
    g_signal_connect (app, "shutdown", G_CALLBACK (on_window_destroy), NULL);

    status = g_application_run (G_APPLICATION (app), argc, argv);     // runs the application
    g_object_unref (app);             // Decreases the reference count of object

    return status;
}
