#pragma once
#include <vector>
#include <atomic>

struct Point{
	Point(int _x, int _y)
	{
		x = _x;
		y = _y;
	}
	Point()
	{
		x = 0;
		y = 0;
	}

	int x;
	int y;
};

struct __agent{
	std::vector<float*> data;
	std::vector<std::pair<float, float>> prio;

};

struct purgatoryData{
	purgatoryData(float _x, float _y, int _index): 
		x(_x), y(_y), index(_index) {}
	float x;
	float y;
	int index;
};

struct Node{
	Node(int bufferSize){
		nrAgents = 0;
		bufferIndex = 0;
		/*Super size buffer. Just so that we dont have to ever worry about the
		buffer needing to grow in one thread while another thread inserts an element*/
		buffer.insert(buffer.begin(), bufferSize, __agent());
	}

	std::vector<float*> x;
	std::vector<float*> y;
	std::vector<float*> desiredX;
	std::vector<float*> desiredY;
	int nrAgents;
	std::atomic<int> bufferIndex;
	std::vector<__agent> buffer; //Really large at all times

	std::vector<purgatoryData> removeRequests;
};

typedef enum agentIndex{
	X = 0,
	Y = 1,
	DESIRED_X = 2,
	DESIRED_Y = 3
};

class QTree{

	static const int MAXLEVEL;
	static const int MERGEBOUNDARY;
	static const int SPLITBOUNDARY;

	// Hold details of the boundary of this node
	Point topLeft;
	Point botRight;

	// Node Details
	Node *data;

	//At what level of split the tree is at
	int level;
	int bufferSize;

	// Children
	QTree *botLeftTree;
	QTree *botRightTree;
	QTree *topLeftTree;
	QTree *topRightTree;

public:
	QTree(Point topL, Point botR, int _level, int _buffersize)
	{
		bufferSize = _buffersize;
		level = _level;
		data = new Node(bufferSize);
		level = _level;
		botLeftTree = nullptr;
		botRightTree = nullptr;
		topLeftTree = nullptr;
		topRightTree = nullptr;
		topLeft = topL;
		botRight = botR;
	}

	//Really only used for root
	QTree(Point topL, Point botR, 
		QTree* _blt, QTree* _brt, 
		QTree* _tlt, QTree* _trt,
		int _buffersize, int _level = 0)
	{
		bufferSize = _buffersize;
		level = _level;
		data = new Node(bufferSize);
		botLeftTree = _blt;
		botRightTree = _brt;
		topLeftTree = _tlt;
		topRightTree = _trt;
		topLeft = topL;
		botRight = botR;
	}

	~QTree(){
		if (botLeftTree != nullptr)
			delete botLeftTree;
		if (botRightTree != nullptr)
			delete botRightTree;
		if (topLeftTree != nullptr)
			delete topLeftTree;
		if (topRightTree != nullptr)
			delete topRightTree;

		delete data;
	}


	std::vector<float*> getXs(){ return data->x; }
	std::vector<float*> getYs(){ return data->y; }

	std::vector<float*> getDesiredXs(){ return data->desiredX; }
	std::vector<float*> getDesiredYs(){ return data->desiredY; }

	std::pair<int, int> getTopLeft() { return std::pair<int,int>(topLeft.x, topLeft.y); }
	std::pair<int, int> getBotRight() { return std::pair<int, int>(botRight.x, botRight.y); }

	bool inBounds(int x, int y);
	bool splitCheck();
	void insert(std::vector<float*> agent, float sortX, float sortY);
	void insert(std::vector<float*> agent, std::vector<std::pair<float, float>> proVector);
	void appendBuffer(__agent a);
	void flushBuffer();
	void flushAllBuffers();
	template <class T> void flushInBounds(int& counter, T agent);
	template <class T> void flushIncomming(int& counter, T agent);
	bool growTree();
	void growAllTrees();
	void remove(float x, float y, int index);
	void purgeAllRegions();
	void purge();
	void prune();
	void saveAgents(QTree* child);
	int countChildAgents();
	void printCorners();
	bool isCoordFree(float x, float y);
	bool QTree::setCoord(__agent& agent, std::vector<float*> Xs, std::vector<float*> Ys);
	std::vector<QTree*> getLeafNodes();
};