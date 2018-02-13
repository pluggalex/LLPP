#include "QuadTree.h"

//Set how many levels of division and at what point they occur
const int QTree::MAXLEVEL = 5;
const int QTree::MERGEBOUNDARY = 5;
const int QTree::SPLITBOUNDARY = 50;
//std::vector<QTree*> QTree::TREENODES = std::vector<QTree*>();

// Check if current quadtree contains the point
bool QTree::inBounds(int x, int y)
{
	return (x >= topLeft.x && x <= botRight.x &&
		y >= topLeft.y && y <= botRight.y);
}

//Checks whether the tree needs to split
bool QTree::splitCheck(){
	if (data->nrAgents == QTree::SPLITBOUNDARY && level != MAXLEVEL)
		return true;

	return false;
}

void QTree::insert(Agent *agent)
{
	// Check if in bounds
	if (!inBounds(agent->pos))
		return;

	//Splitting sequence
	//Check for splitBoundary and the amount of agents
	//in the current quad
	if ((topLeft.x + botRight.x) / 2 >= agent->pos.x)
	{

		if (splitCheck()){
			// Check if agent belong in topLeftTree
			if ((topLeft.y + botRight.y) / 2 >= agent->pos.y)
			{
				//If the topLeftTree deosn't exist, create it
				//If it does exist, recurse into the next tree
				if (topLeftTree == NULL)
					topLeftTree = new QTree(Point(topLeft.x, topLeft.y),
					Point((topLeft.x + botRight.x) / 2, (topLeft.y + botRight.y) / 2));

				//Recurse
				topLeftTree->insert(agent);

			}

			else
			{
				if (botLeftTree == NULL)
					botLeftTree = new QTree(Point(topLeft.x, (topLeft.y + botRight.y) / 2),
					Point((topLeft.x + botRight.x) / 2, botRight.y));

				botLeftTree->insert(agent);
			}
		}
		else{
			data->agents.push_back(*agent);
			data->nrAgents++;
		}

	}
	else
	{
		if (splitCheck()){

			// topRightTree
			if ((topLeft.y + botRight.y) / 2 >= agent->pos.y)
			{
				if (topRightTree == NULL)
					topRightTree = new QTree(Point((topLeft.x + botRight.x) / 2,
					topLeft.y),
					Point(botRight.x, (topLeft.y + botRight.y) / 2));

				topRightTree->insert(agent);
			}

			// botRightTree
			else
			{
				if (botRightTree == NULL)
					botRightTree = new QTree(Point((topLeft.x + botRight.x) / 2,
					(topLeft.y + botRight.y) / 2),
					Point(botRight.x, botRight.y));

				botRightTree->insert(agent);
			}
		}
		else{
			data->agents.push_back(*agent);
			data->nrAgents++;
		}

	}
}