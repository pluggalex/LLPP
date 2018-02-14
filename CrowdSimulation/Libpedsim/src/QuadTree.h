#pragma once
#include <vector>
#include <atomic>

struct Point{
	Point(float x, float y)
	{
		x = x;
		y = y;
	}
	Point()
	{
		x = 0;
		y = 0;
	}

	float x;
	float y;
};


struct Node{
	Node(){
		nrAgents = 0;
		bufferIndex = 0;
	}

	std::vector<float*> x;
	std::vector<float*> y;
	std::vector<float*> desiredX;
	std::vector<float*> desiredY;
	int nrAgents;
	std::atomic<int> bufferIndex;
	std::vector<std::vector<float*>> buffer; //Really large at all times
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
	//static std::vector<QTree*> TREENODES;

	// Hold details of the boundary of this node
	Point topLeft;
	Point botRight;

	// Node Details
	Node *data;

	//At what level of split the tree is at
	int level;

	// Children
	QTree *botLeftTree;
	QTree *botRightTree;
	QTree *topLeftTree;
	QTree *topRightTree;

public:
	QTree(Point topL, Point botR, int _level = 0)
	{
		data = nullptr;
		level = _level;
		botLeftTree = nullptr;
		botRightTree = nullptr;
		topLeftTree = nullptr;
		topRightTree = nullptr;
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
	}


	
	bool inBounds(float x, float y);
	bool splitCheck();
	void insert(std::vector<float*> agent, float sortX, float sortY);
	void appendBuffer(std::vector<float*>& agent);
	void flushBuffer();
	bool growTree();
	void remove(int index);
	void prune();
	void saveAgents(QTree* child);
	int countChildAgents();
};