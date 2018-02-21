#include "QuadTree.h"
#include <algorithm>

#include <iostream> //ONLY USED FOR DEBUGGING

//Set how many levels of division and at what point they occur
const int QTree::MAXLEVEL = 2;
const int QTree::MERGEBOUNDARY = 0;
const int QTree::SPLITBOUNDARY = 50;
//std::vector<QTree*> QTree::TREENODES = std::vector<QTree*>();

// Check if current quadtree contains the point
bool QTree::inBounds(int x, int y)
{
	return (x >= topLeft.x && x < botRight.x &&
		y >= topLeft.y && y < botRight.y);
}

//Checks whether the tree needs to split
bool QTree::splitCheck(){
	if (data->nrAgents >= QTree::SPLITBOUNDARY && level < MAXLEVEL)
		return true;

	return false;
}

bool coordAvailable(float x, float y, std::vector<float*>& Xs, std::vector<float*>& Ys){
	for (int i = 0; i < Xs.size(); i++){
		if (*(Xs[i]) == x && *(Ys[i]) == y)
			return false;
	}
	return true;
}


bool QTree::setCoord(__agent& agent, std::vector<float*> Xs, std::vector<float*> Ys){
	for (auto prio : agent.prio){
		if (inBounds(prio.first, prio.second) && coordAvailable(prio.first, prio.second, Xs, Ys)){
			*agent.data[agentIndex::DESIRED_X] = prio.first;
			*agent.data[agentIndex::DESIRED_Y] = prio.second;
			return true;
		}
	}
	return false;
}

template <class T>
void QTree::flushInBounds(int& counter, T agents){
	if (setCoord(agents, data->x, data->y)){
		data->x.push_back(agents.data[agentIndex::X]);
		data->y.push_back(agents.data[agentIndex::Y]);
		data->desiredX.push_back(agents.data[agentIndex::DESIRED_X]);
		data->desiredY.push_back(agents.data[agentIndex::DESIRED_Y]);
		counter++;
	}
}

template <class T>
void QTree::flushIncomming(int& counter, T agents){
	if (setCoord(agents, data->x, data->y)){
		*(agents.data[agentIndex::X]) = *(agents.data[agentIndex::DESIRED_X]);
		*(agents.data[agentIndex::Y]) = *(agents.data[agentIndex::DESIRED_Y]);
		data->x.push_back(agents.data[agentIndex::X]);
		data->y.push_back(agents.data[agentIndex::Y]);
		data->desiredX.push_back(agents.data[agentIndex::DESIRED_X]);
		data->desiredY.push_back(agents.data[agentIndex::DESIRED_Y]);
		counter++;
	}
}


//Should not be done when other threads can mess with the buffer
void QTree::flushBuffer(){
	auto bufferBegin = data->buffer.begin();
	int size = data->bufferIndex.load();
	int added = 0;
	for (auto agents = bufferBegin; agents < bufferBegin + size; agents++){
		inBounds(*(*agents).data[agentIndex::X], *(*agents).data[agentIndex::Y])? 
			flushInBounds(added, *agents) : flushIncomming(added, *agents);
	}

	data->nrAgents += added;
	data->bufferIndex.store(0);
}

void QTree::flushAllBuffers(){
	QTree* trees[4] = { botLeftTree, botRightTree, topLeftTree, topRightTree };
	for (int i = 0; i < 4; i++){
		if (trees[i] != nullptr){
			trees[i]->flushAllBuffers();
		}
	}
	flushBuffer();
}

//Honestly.. We should maybee do this with function pointers or something. But it is just eaiser to 
//copy around functions changing one line, and its late.
void QTree::growAllTrees(){
	QTree *trees[4] = { botLeftTree, botRightTree, topLeftTree, topRightTree };
	for (int i = 0; i < 4; i++){
		if (trees[i] != nullptr){
			trees[i]->growAllTrees();
		}
	}
	growTree();
}

//Purge everything!
void QTree::purgeAllRegions(){
	QTree *trees[4] = { botLeftTree, botRightTree, topLeftTree, topRightTree };
	for (int i = 0; i < 4; i++){
		if (trees[i] != nullptr){
			trees[i]->purgeAllRegions();
		}
	}
	purge();
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

void QTree::appendBuffer(__agent a){
	int index = data->bufferIndex.load(std::memory_order_relaxed);
	int updated_index = index+1;
	while (!(data->bufferIndex).compare_exchange_weak(index, updated_index,
		std::memory_order_release,
		std::memory_order_relaxed)){
		updated_index = index + 1;
	}
	data->buffer[index] = a;
}

void QTree::insert(std::vector<float*> agent, float sortByX, float sortByY){
	this->insert(agent, std::vector<std::pair<float,float>>{std::pair<float, float>(sortByX, sortByY)});
}

void QTree::insert(std::vector<float*> agent, std::vector<std::pair<float, float>> prioVector)
{
	float sortByX = prioVector[0].first;
	float sortByY = prioVector[0].second;

	// Check if in bounds
	if (!inBounds(sortByX, sortByY))
		return;

	bool split = splitCheck();
	//Find where to insert the new element
	if ((topLeft.x + botRight.x) / 2 > sortByX)
	{
		if(split || topLeftTree != nullptr || botLeftTree != nullptr)
		{
			// Check if agent belong in topLeftTree
			if ((topLeft.y + botRight.y) / 2 > sortByY)
			{
				//If the topLeftTree deosn't exist, create it
				//If it does exist, recurse into the next tree
				if (topLeftTree == NULL)
					topLeftTree = new QTree(Point(topLeft.x, topLeft.y),
					Point((topLeft.x + botRight.x) / 2, (topLeft.y + botRight.y) / 2), level+1, bufferSize);

				//Recurse
				topLeftTree->insert(agent, sortByX, sortByY);
			}
			else
			{
				if (botLeftTree == NULL)
					botLeftTree = new QTree(Point(topLeft.x, (topLeft.y + botRight.y) / 2),
					Point((topLeft.x + botRight.x) / 2, botRight.y), level + 1, bufferSize);

				botLeftTree->insert(agent, sortByX, sortByY);
			}
		}
		else
		{
			__agent hurray;
			hurray.data = agent;
			hurray.prio = prioVector;

			appendBuffer(hurray);
		}
	}
	else
	{
		if (split || topRightTree != nullptr || botRightTree != nullptr){
			// topRightTree
			if ((topLeft.y + botRight.y) / 2 > sortByY)
			{
				if (topRightTree == NULL)
					topRightTree = new QTree(Point((topLeft.x + botRight.x) / 2,
					topLeft.y),
					Point(botRight.x, (topLeft.y + botRight.y) / 2), level + 1, bufferSize);

				topRightTree->insert(agent, sortByX, sortByY);
			}
			// botRightTree
			else
			{
				if (botRightTree == NULL)
					botRightTree = new QTree(Point((topLeft.x + botRight.x) / 2,
					(topLeft.y + botRight.y) / 2),
					Point(botRight.x, botRight.y), level + 1, bufferSize);

				botRightTree->insert(agent, sortByX, sortByY);
			}
		}
		else{
			__agent hurray;
			hurray.data = agent;
			hurray.prio = prioVector;

			appendBuffer(hurray);
		}
	}
}

std::vector<QTree*> QTree::getLeafNodes(){
	bool hasUsefullChildren = false;
	std::vector<QTree*> result;
	std::vector<QTree*> childRes;
	QTree* trees[4] = { botLeftTree, botRightTree, topLeftTree, topRightTree };
	for (int i = 0; i < 4; i++){
		if (trees[i] != nullptr){
			hasUsefullChildren = true;
			childRes = trees[i]->getLeafNodes();
			result.insert(result.begin()+result.size(), childRes.begin(), childRes.end());
		}
	}

	// If our children are nullptrs we must be the leaf node
	if (!hasUsefullChildren)
		result.push_back(this);

	return result;
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

//Requests data/agent to be removed from this node
void QTree::remove(float x, float y, int index){
	data->removeRequests.push_back(purgatoryData(x, y, index));
}

//Data that hasn't change, i.e. it wasn't accepted in to the new region,
//probably due to conflicts, will not be removed from this region.
//Data that has been succesfully transfered to the new region will 
//be erased from this one.
void QTree::purge(){
	//Sort to make the indexes go from high to low in order to avoid segfaults when we start erasing
	//elements from the vectors(we erase based on index). 
	std::sort(data->removeRequests.begin(), data->removeRequests.end(), 
		[](purgatoryData a, purgatoryData b){return a.index > b.index; }); 
	for (auto request : data->removeRequests){
		int index = request.index;
		if (request.x != *(data->x[index]) || request.y != *(data->y[index]))
		{
			data->x.erase(data->x.begin() + index);
			data->y.erase(data->y.begin() + index);
			data->desiredX.erase(data->desiredX.begin() + index);
			data->desiredY.erase(data->desiredY.begin() + index);

			data->nrAgents--;
		}
	}
	data->removeRequests.clear();
}

void QTree::printCorners(){
	std::cout << "(" << topLeft.x << "," << topLeft.y << ")"
		<< " (" << botRight.x << "," << botRight.y << ")\n";
}


//DONT LOOK-- Remove!
bool QTree::isCoordFree(float x, float y){
	//Find where to insert the new element
	if ((topLeft.x + botRight.x) / 2 > x)
	{
		if (topLeftTree != nullptr || botLeftTree != nullptr)
		{
			// Check if agent belong in topLeftTree
			if ((topLeft.y + botRight.y) / 2 > y)
			{
				if (topLeftTree == NULL)
					return true;
				//Recurse
				topLeftTree->isCoordFree(x,y);
			}
			else
			{
				if (botLeftTree == NULL)
					return true;
				botLeftTree->isCoordFree(x, y);
			}
		}
		else
		{
			if (coordAvailable(x, y, data->x, data->y))
				return true;
			return false;
		}
	}
	else
	{
		if (topRightTree != nullptr || botRightTree != nullptr){
			// topRightTree
			if ((topLeft.y + botRight.y) / 2 > y)
			{
				if (topRightTree == NULL)
					return true;

				topRightTree->isCoordFree(x, y);
			}
			// botRightTree
			else
			{
				if (botRightTree == NULL)
					return true;
				botRightTree->isCoordFree(x, y);
			}
		}
		else{
			if (coordAvailable(x, y, data->x, data->y))
				return true;
			return false;
		}
	}

}