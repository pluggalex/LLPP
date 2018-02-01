///////////////////////////////////////////////////
// Low Level Parallel Programming 2017.
//
//     ==== There is no need to change this file ====
// 

#include "ViewAgent.h"
#include "MainWindow.h"
#include <QGraphicsItemAnimation>

// Memory leak check with msvc++
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#ifdef _DEBUG
#define new new(_NORMAL_BLOCK, __FILE__, __LINE__)
#endif

ViewAgent::ViewAgent(std::shared_ptr<Ped::Tagent_collection> agent, QGraphicsScene * scene) : agent(agent)
{
	QBrush greenBrush(Qt::green);
	QPen outlinePen(Qt::black);
	outlinePen.setWidth(2);

	auto x = agent->getX();
	auto y = agent->getY();

	for (int i = 0; i < agent->size(); i++){
		rect.push_back(scene->addRect(MainWindow::cellToPixel(x[i]), MainWindow::cellToPixel(y[i]),
			MainWindow::cellsizePixel - 1, MainWindow::cellsizePixel - 1, outlinePen, greenBrush));
	}
}

void ViewAgent::paint(std::vector<QColor> colors){
	auto x = agent->getX();
	auto y = agent->getY();

	for (int i = 0; i < agent->size(); i++){
		QBrush brush(colors[i]);
		rect[i]->setBrush(brush);
		rect[i]->setRect(MainWindow::cellToPixel(x[i]), MainWindow::cellToPixel(y[i]),
			MainWindow::cellsizePixel - 1, MainWindow::cellsizePixel - 1);
	}
}

/*
const std::pair<int, int> ViewAgent::getPosition(){
	return std::make_pair(agent->getX(), agent->getY());
}
*/