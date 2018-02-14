#include "QuadTree.h"

//Set how many levels of division and at what point they occur
const int QTree::MAXLEVEL = 5;
const int QTree::MERGEBOUNDARY = 30;
const int QTree::SPLITBOUNDARY = 50;
//std::vector<QTree*> QTree::TREENODES = std::vector<QTree*>();

// Check if current quadtree contains the point
bool QTree::inBounds(float x, float y)
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


//Should not be done when other threads can mess with the buffer
void QTree::flushBuffer(){
	auto bufferBegin = data->buffer.begin();
	int size = data->bufferIndex.load();

	for (auto agents = bufferBegin; agents < bufferBegin + size; agents++){
		data->x.push_back((*agents)[agentIndex::X]);
		data->y.push_back((*agents)[agentIndex::Y]);
		data->desiredX.push_back((*agents)[agentIndex::DESIRED_X]);
		data->desiredY.push_back((*agents)[agentIndex::DESIRED_Y]);
	}

	data->nrAgents += size;
	data->bufferIndex.store(0);
}

//Should not be done when other threads can mess with the buffer, or the tree, or anything
bool QTree::growTree(){
	if (!splitCheck())
		return false;
	
	//insert all again, now the agents should trickle down to child nodes
	std::vector<std::vector<float*>> tempBuff;
	int size = data->x.size();
	for (int i = 0; i < size; i++){
		tempBuff.emplace_back(std::initializer_list<float*>{data->x[i], data->y[i], data->desiredX[i], data->desiredY[i]});
	}
	for (auto& agent : tempBuff){
		this->insert(agent, (*agent[agentIndex::X]), (*agent[agentIndex::Y]));
	}

	//flush all buffers. 
	QTree *trees[4] = { botLeftTree, botRightTree, topLeftTree, topRightTree};
	for (int i = 0; i < 4; i++){
		if (trees[i] != nullptr)
			trees[i]->flushBuffer();
	}
	
	//Clear agent from our node in order to avoid duplicates all around the tree
	data->x.clear();
	data->y.clear();
	data->desiredX.clear();
	data->desiredY.clear();
	data->nrAgents = 0;

	return true;
}

void QTree::appendBuffer(std::vector<float*>& agent){
	int index = data->bufferIndex.load(std::memory_order_relaxed);
	int updated_index = index+1;
	while (!(data->bufferIndex).compare_exchange_weak(index, updated_index,
		std::memory_order_release,
		std::memory_order_relaxed)){
		updated_index = index + 1;
	}
	data->buffer[index] = agent;
}

void QTree::insert(std::vector<float*> agent, float sortByX, float sortByY)
{
	// Check if in bounds
	if (!inBounds(sortByX, sortByY))
		return;

	
	bool split = splitCheck();
	//data->nrAgents++;
	//Splitting sequence
	//Check for splitBoundary and the amount of agents
	//in the current quad
	if ((topLeft.x + botRight.x) / 2 >= sortByX)
	{
		if (split || topLeftTree != nullptr || botLeftTree != nullptr){
			// Check if agent belong in topLeftTree
			if ((topLeft.y + botRight.y) / 2 >= sortByY)
			{
				//If the topLeftTree deosn't exist, create it
				//If it does exist, recurse into the next tree
				if (topLeftTree == NULL)
					topLeftTree = new QTree(Point(topLeft.x, topLeft.y),
					Point((topLeft.x + botRight.x) / 2, (topLeft.y + botRight.y) / 2), level+1);

				//Recurse
				topLeftTree->insert(agent, sortByX, sortByY);
			}
			else
			{
				if (botLeftTree == NULL)
					botLeftTree = new QTree(Point(topLeft.x, (topLeft.y + botRight.y) / 2),
					Point((topLeft.x + botRight.x) / 2, botRight.y), level+1);

				botLeftTree->insert(agent, sortByX, sortByY);
			}
		}
		else
			void appendBuffer(float* x, float* y, float* desiredX, float* desiredY);
	}
	else
	{
		if (split || topRightTree != nullptr || botRightTree != nullptr){
			// topRightTree
			if ((topLeft.y + botRight.y) / 2 >= sortByY)
			{
				if (topRightTree == NULL)
					topRightTree = new QTree(Point((topLeft.x + botRight.x) / 2,
					topLeft.y),
					Point(botRight.x, (topLeft.y + botRight.y) / 2), level+1);

				topRightTree->insert(agent, sortByX, sortByY);
			}
			// botRightTree
			else
			{
				if (botRightTree == NULL)
					botRightTree = new QTree(Point((topLeft.x + botRight.x) / 2,
					(topLeft.y + botRight.y) / 2),
					Point(botRight.x, botRight.y), level+1);

				botRightTree->insert(agent, sortByX, sortByY);
			}
		}
		else
			void appendBuffer(float* x, float* y, float* desiredX, float* desiredY);
	}
}

int QTree::countChildAgents(){
	int counter = 0;
	QTree *trees[4] = { botLeftTree, botRightTree, topLeftTree, topRightTree };
	for (int i = 0; i < 4; i++){
		if (trees[i] != nullptr){
			counter += trees[i]->data->nrAgents;
		}
	}
	return counter;
}

//Prunes depending on population in regions for all child nodes.
void QTree::prune(){
	//If we are not root and have too few agents we kill all our children
	if (level > 0 && data != nullptr && countChildAgents() < MERGEBOUNDARY){
		QTree *trees[4] = { botLeftTree, botRightTree, topLeftTree, topRightTree };
		for (int i = 0; i < 4; i++){
			if (trees[i] != nullptr){
				trees[i]->prune();
				saveAgents(trees[i]);
				delete trees[i];
			}
		}
	}
	else{ //Otherwise we call all available children to do the same
		QTree *trees[4] = { botLeftTree, botRightTree, topLeftTree, topRightTree };
		for (int i = 0; i < 4; i++){
			if (trees[i] != nullptr)
				trees[i]->prune();
		}
	}
}


//Copies the childs agent vectors to ours
void QTree::saveAgents(QTree* child){

	int ourSize = this->data->nrAgents;
	int childSize = child->data->nrAgents;

	std::vector<float*>* childX = &(child->data->x);	
	data->x.insert(data->x.begin() + ourSize, (*childX).begin(), (*childX).begin() + childSize);

	std::vector<float*>* childY = &(child->data->y);
	data->y.insert(data->y.begin() + ourSize, (*childY).begin(), (*childY).begin() + childSize);

	std::vector<float*>* desiredX = &(child->data->desiredX);
	data->desiredX.insert(data->desiredX.begin() + ourSize, (*desiredX).begin(), (*desiredX).begin() + childSize);

	std::vector<float*>* desiredY = &(child->data->desiredY);
	data->desiredY.insert(data->desiredY.begin() + ourSize, (*desiredY).begin(), (*desiredY).begin() + childSize);

	this->data->nrAgents += childSize;
}

//Removes the agent from this node
void QTree::remove(int index){
	data->x.erase(data->x.begin() + index);
	data->y.erase(data->y.begin() + index);
	data->desiredX.erase(data->desiredX.begin() + index);
	data->desiredY.erase(data->desiredY.begin() + index);

	data->nrAgents--;
}