#include <Windows.h>
#include "MainWindow.h"

#include <QGraphicsView>
#include <QtGui>
#include <QBrush>

#include <iostream>

// Memory leak check with msvc++
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#ifdef _DEBUG
#define new new(_NORMAL_BLOCK, __FILE__, __LINE__)
#endif

MainWindow::MainWindow(const Ped::Model &pedModel) : model(pedModel)
{
	// The Window 
	graphicsView = new QGraphicsView();

	setCentralWidget(graphicsView);

	// A surface for managing a large number of 2D graphical items
	scene = new QGraphicsScene(QRect(0, 0, 800, 600), this);

	// Connect
	graphicsView->setScene(scene);

	// Paint on surface
	scene->setBackgroundBrush(Qt::white);

	for (int x = 0; x <= 800; x += cellsizePixel)
	{
		scene->addLine(x, 0, x, 600, QPen(Qt::gray));
	}

	// Now add the horizontal lines, paint them green-------------v
	for (int y = 0; y <= 600; y += cellsizePixel)     //          v
	{												  //	      v
		scene->addLine(0, y, 800, y, QPen(Qt::gray));//<--- 'paint them green' I suspect a coupt has taken place
	}

	// Create viewAgents with references to the position of the model counterparts
	viewAgent = new ViewAgent(model.getAgents(), scene);

	
	const int heatmapSize = model.getHeatmapSize();
	QPixmap pixmapDummy = QPixmap(heatmapSize, heatmapSize);
	pixmap = scene->addPixmap(pixmapDummy);
	

	paint();
	graphicsView->show(); // Redundant? 
}

void MainWindow::paint() {

	// Uncomment this to paint the heatmap (Assignment 4)
	// const int heatmapSize = model.getHeatmapSize();
	// QImage image((uchar*)*model.getHeatmap(), heatmapSize, heatmapSize, heatmapSize * sizeof(int), QImage::Format_ARGB32);
	QImage image;
	 pixmap->setPixmap(QPixmap::fromImage(image));

	// Paint all agents: green, if the only agent on that position, otherwise red
	// I think this will be incorrect for the first element where two are on the same stop.
	// I guess that doesn't matter since the last one will be on top of the green one anyway..
	std::vector<QColor> colors;
	std::set<std::tuple<int, int> > positionsTaken;

	std::vector<float> agentVectorX = model.getAgents()->getX();
	auto xIterator = agentVectorX.begin();
	std::vector<float> agentVectorY = model.getAgents()->getY();
	auto yIterator = agentVectorY.begin();
	auto xEnd = agentVectorX.end();
	int i = 0;//Remove
	for (; xIterator != xEnd;)
	{
		size_t tupleSizeBeforeInsert = positionsTaken.size();
		positionsTaken.insert(std::make_pair(*xIterator, *yIterator));
		size_t tupleSizeAfterInsert = positionsTaken.size();

		if (tupleSizeBeforeInsert != tupleSizeAfterInsert) {
			colors.push_back(Qt::green);
		}
		else {
			colors.push_back(Qt::red);
			std::cout << "Red - " << "x: " << *xIterator << " y: " << *yIterator << "\n";
		}

		i++;//Remove
		xIterator++; 
		yIterator++;
	}

	viewAgent->paint(colors);
}

int MainWindow::cellToPixel(int val)
{
	return val*cellsizePixel;
}
MainWindow::~MainWindow()
{
	delete viewAgent;
}