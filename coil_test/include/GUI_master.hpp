#ifndef GUIMASTER
#define GUIMASTER

#include "general_header.hpp"
#include "vision.hpp"

extern "C" {
    void on_toggle_camera_stream_toggled (GtkToggleButton *togglebutton, gpointer data);
}

void GUI_master_activate (GtkImage *vidWindow, GtkLabel *timeLabel);        // start updating GUI
void GUI_master_deactivate(void);       // stop updating GUI

#endif
