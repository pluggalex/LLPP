///////////////////////////////////////////////////
// Low Level Parallel Programming 2016.
//
//     ==== Don't change this file! ====
// 
#ifndef _view_agent_h
#define _view_agent_h

#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <vector>
#include "ped_agent_collection.h"

class ViewAgent{
public:
	ViewAgent(std::shared_ptr<Ped::Tagent_collection> agent, QGraphicsScene * scene);
	void paint(std::vector<QColor> colors);
	//const std::pair<int, int> getPosition();

private:
	std::shared_ptr<Ped::Tagent_collection> agent;

	// The rectangle on the GUI representing this agent
	std::vector<QGraphicsRectItem*> rect;
};

#endif
