// SmcExample1Cpp.cpp : main project file.

#include "stdafx.h"
#include "MainWindow.h"

using namespace Pololu::SimpleMotorController::SmcExample1Cpp;		// these 3-layer namespace is declared in MainWindow.h

[STAThreadAttribute]
int main(array<System::String ^> ^args)
{
	// Enabling Windows XP visual effects before any controls are created
	Application::EnableVisualStyles();
	Application::SetCompatibleTextRenderingDefault(false);

	// Create the main window and run it
	Application::Run(gcnew MainWindow());
	return 0;
}
