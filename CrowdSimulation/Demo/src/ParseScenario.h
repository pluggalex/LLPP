//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2015
//

#ifndef _parsescenario_h_
#define _parsescenario_h_

#include "ped_agent_collection.h"
#include "ped_waypoint.h"
#include <QtCore>
#include <QXmlStreamReader>
#include <vector>
#include <set>
#include <memory>

using namespace std;

class ParseScenario : public QObject
{
	Q_OBJECT

public:
	ParseScenario(QString file);

	// returns the collection of agents defined by this scenario
	std::unique_ptr<Ped::Tagent_collection> getAgents();
	std::vector<Ped::Twaypoint*> getWaypoints();
	private slots:
	void processXmlLine(QByteArray data);
	// contains all defined waypoints

private:
	QXmlStreamReader xmlReader;

	// final collection of all created agents
	std::unique_ptr<Ped::Tagent_collection> agents;
	

	// temporary collection of agents used to
	// keep track of all agents that are generated
	// within the current opened agents xml tag
	std::unique_ptr<Ped::Tagent_collection> tempAgents;

	// contains all defined waypoints
	map<QString, Ped::Twaypoint*> waypoints;

	// decides what to do on a new xml tag (tags: agent, waypoint, addwaypoint)
	void handleXmlStartElement();

	// decides what to do if an xml tag is closed
	void handleXmlEndElement();

	// creates a new waypoint on a waypoint xml tag
	void createWaypoint();

	// creates a new agents on an agent xml tag
	void createAgents();

	// add (by ID-)defined waypoint to current agents
	void addWaypointToCurrentAgents(QString &id);

	QString readString(const QString &tag);
	double readDouble(const QString &tag);
};

#endif
