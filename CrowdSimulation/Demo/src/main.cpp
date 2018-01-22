///////////////////////////////////////////////////
// Low Level Parallel Programming 2017.
//
// 
//
// The main starting point for the crowd simulation.
//



#undef max
#include <Windows.h>
#include "ped_model.h"
#include "MainWindow.h"
#include "ParseScenario.h"

#include <QGraphicsView>
#include <QGraphicsScene>
#include <QApplication>
#include <QTimer>
#include <thread>

#include "PedSimulation.h"
#include <iostream>
#include <chrono>
#include <ctime>
#include <cstring>

#pragma comment(lib, "libpedsim.lib")

// Memory leak check with msvc++
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#ifdef _DEBUG
#define new new(_NORMAL_BLOCK, __FILE__, __LINE__)
#endif

int main(int argc, char*argv[]) {
	bool timing_mode = 0;
	int i = 1;
	QString scenefile = "scenario.xml";

	// Enable memory leak check. This is ignored when compiling in Release mode. 
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);

	// Argument handling
	while (i < argc)
	{
		if (argv[i][0] == '-' && argv[i][1] == '-')
		{
			if (strcmp(&argv[i][2], "timing-mode") == 0)
			{
				cout << "Timing mode on\n";
				timing_mode = true;
			}
			else if (strcmp(&argv[i][2], "help") == 0)
			{
				cout << "Usage: " << argv[0] << " [--help] [--timing-mode] [scenario]" << endl;
				return 0;
			}
			else
			{
				cerr << "Unrecognized command: \"" << argv[i] << "\". Ignoring ..." << endl;
			}
		}
		else // Assume it is a path to scenefile
		{
			scenefile = argv[i];
		}

		i += 1;
	}
	int retval = 0;
	{ // This scope is for the purpose of removing false memory leak positives

		// Reading the scenario file and setting up the crowd simulation model
		Ped::Model model;
		ParseScenario parser(scenefile);
		model.setup(parser.getAgents(), parser.getWaypoints());

		// GUI related set ups
		QApplication app(argc, argv);
		MainWindow mainwindow(model);

		// Default number of steps to simulate
		const int maxNumberOfStepsToSimulate = 10000;//1000000000;
		PedSimulation *simulation = new PedSimulation(model, mainwindow);

		cout << "Demo setup complete, running ..." << endl;

		// Timing of simulation


		auto start = std::chrono::steady_clock::now();

		if (timing_mode)
		{
			// Simulation mode to use when profiling (without any GUI)
			simulation->runSimulationWithoutQt(maxNumberOfStepsToSimulate);
		}
		else
		{
			// Simulation mode to use when visualizing
			mainwindow.show();
			simulation->runSimulationWithQt(maxNumberOfStepsToSimulate);
			retval = app.exec();
		}

		auto duration = std::chrono::duration_cast<std::chrono::milliseconds> (std::chrono::steady_clock::now() - start);
		float fps = ((float)simulation->getTickCount()) / ((float)duration.count())*1000.0;
		cout << "Time: " << duration.count() << " milliseconds, " << fps << " Frames Per Second." << std::endl;

		delete (simulation);
	}
	_CrtDumpMemoryLeaks();

	cout << "Done" << endl;
	cout << "Type Enter to quit.." << endl;
	getchar(); // Wait for any key. Windows convenience...
	return retval;
}
