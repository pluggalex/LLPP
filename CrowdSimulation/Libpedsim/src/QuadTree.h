#pragma once
#include <vector>

struct Point{
	Point(int x, int y)
	{
		x = x;
		y = y;
	}
	Point()
	{
		x = 0;
		y = 0;
	}

	int x;
	int y;
};


struct Node{
	Node(){
		nrAgents = 0;
	}

	std::vector<float*> x;
	std::vector<float*> y;
	std::vector<float*> desiredX;
	std::vector<float*> desiredY;
	int nrAgents;

	std::vector<std::vector<float*>> buffer;
	int bufferIndex;
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
	QTree(Point topL, Point botR)
	{
		data = nullptr;
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


	
	bool inBounds(int x, int y);
	bool splitCheck();
	void insert(/*Agent*/);
	void flushBuffer();
	void remove(/*agent*/);
};