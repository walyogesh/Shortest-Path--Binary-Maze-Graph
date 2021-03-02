#include <iostream>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>    // for std::find
#define HASHVALUE 11 //used in Compare(Hashing) function 

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// This Class holds Path Node data.
// 
//   1. Passed function parameters(Vertex class objects) by const reference to avoid extra copy
//   2. Constructor's initialization is done in the Initializer list. All Getters are const functions
//   3. Member variables are declared and initialized as per their size i.e smallest first 
//   4. Written All necessary constructors as per rule of 5
//   5. overloaded operator == to cpmare nodes directly
//   6. Added null checks before we use pointer
// Improvements Can be Done:
//  We could create separate PathNode.cpp file to separate declaration from definition of the class
//  we could add guard to mLinks,unique queue push function to use class in multi threaded environment with the help of lock_guard & mutex 
/////////////////////////////////////////////////////////////////////////////////////////////////////////


struct Vertex
{
    Vertex( int xpos, int ypos)
        : X(xpos)
        , Y(ypos)
    {}

    inline bool operator<(const Vertex& rhs) const
    {
        return ((X < rhs.X) && (Y < rhs.Y));   
    }

    inline bool operator==(const Vertex& rhs) const
    {
        return ((X == rhs.X) && (Y == rhs.Y));
    }
   
     int X;
     int Y;
};

class PathNode
{
public:
    PathNode(const char value, const Vertex& position)
        : mvalue(value)
        , mPosition(position)
    {}

    bool operator==(const PathNode& rhs) const
    {
        return (mPosition == rhs.mPosition);
    }

    //copy constructor
    PathNode(const PathNode& other)
        : mvalue(other.mvalue)
        , mPosition(other.mPosition)
        , mLinks(other.mLinks)
    {}

    //assignment 
    PathNode& operator=(const PathNode& other)
    {
        if (this == &other)//self check
        {
            return *this;
        }
        mvalue = other.mvalue;
        mPosition = other.mPosition;
        mLinks = other.mLinks;
        return *this;
    }

    //move constructor
    PathNode(PathNode&& other) noexcept
        : mvalue(other.mvalue)
        , mPosition(other.mPosition)
        , mLinks(std::move(other.mLinks))
    {}

    //destructor
    ~PathNode()
    {}

    void AddLinksFromMap(const unsigned char* pMap, const int nMapWidth, const int nMapHeight, const int nStartX, const int nStartY)
    {
        if ((nStartX == mPosition.X + 1) && (nStartY == mPosition.Y)
            || (nStartX == mPosition.X - 1) && (nStartY == mPosition.Y)
            || (nStartX == mPosition.X) && (nStartY == mPosition.Y - 1)
            || (nStartX == mPosition.X) && (nStartY == mPosition.Y + 1))
        {
            mLinks.emplace_back(new PathNode(1, Vertex(nStartX, nStartY)));
        }
        else
        {
            if (mPosition.Y > 0)//Up side of the Node(x, y-1)
            {
                if (pMap[mPosition.X + ((mPosition.Y - 1) * nMapWidth)])
                {
                    mLinks.emplace_back(new PathNode(1, Vertex(mPosition.X, mPosition.Y - 1)));
                }
            }
            if (mPosition.X > 0) //Left side of the Node(x-1,y)
            {
                if (pMap[(mPosition.X - 1) + (mPosition.Y * nMapWidth)])
                {
                    mLinks.emplace_back(new PathNode(1, Vertex(mPosition.X - 1, mPosition.Y)));
                }
            }
            if (mPosition.Y < nMapHeight - 1) //Down side of the Node(x, y+1)
            {
                if (pMap[(mPosition.X) + ((mPosition.Y + 1) * nMapWidth)])
                {
                    mLinks.emplace_back(new PathNode(1, Vertex(mPosition.X, mPosition.Y + 1)));
                }
            }
            if (mPosition.X < nMapWidth - 1) //Right side of the Node(x+1, y )
            {
                if (pMap[(mPosition.X + 1) + (mPosition.Y * nMapWidth)])
                {
                    mLinks.emplace_back(new PathNode(1, Vertex(mPosition.X + 1, mPosition.Y)));
                }
            }
        }

    }

    void RemoveLink(PathNode* pathNode)
    {
        auto i = std::find(mLinks.begin(), mLinks.end(), pathNode);
        mLinks.erase(i);
    }

    inline const Vertex& GetPosition() const { return mPosition; }

    inline int GetMapPosition(int width) const { return mPosition.X + mPosition.Y * width; }

    inline bool IsVaild(const int nMapWidth, const int nMapHeight, const unsigned char* pMap) const
    {
        return  mPosition.X < nMapWidth
             && mPosition.Y < nMapHeight
             && pMap[mPosition.X + 1, mPosition.Y]
             && pMap[mPosition.X - 1, mPosition.Y]
             && pMap[mPosition.X, mPosition.Y + 1]
             && pMap[mPosition.X, mPosition.Y - 1];
    }

    const std::vector<PathNode*>& GetLinks() const { return mLinks; }

private:
    char mvalue;
    Vertex mPosition;
    std::vector<PathNode*> mLinks;
};

struct Comapre{

    size_t operator()(const PathNode& rhs) const
    {
        return (rhs.GetPosition().X * HASHVALUE) + rhs.GetPosition().Y;
    };
};

class UniqueQueue
{
public:
    bool push(PathNode* t)
    {
        if (mSet.insert(*t).second) //achieved uniqueness 
        {
            mQueue.push(t);
            return true;
        }
        return false;
    }
    inline bool empty() const { return mQueue.empty(); }
    inline PathNode* GetFront() const { return mQueue.front(); }
    inline void pop() { mQueue.pop(); }
private:
    std::queue<PathNode*> mQueue;
    std::unordered_set<PathNode, Comapre> mSet;
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// - To find shortest path for the given start and target node I have created a data structure which is Unique, Queue
// - It holds PathNode which has information about nodes value, its linked pathNodes and its position in the map
// - This function also has a local map = visted[][] which holds nodes parent/previous node value
//   **** Working of the Funtion: ****
//   -Add target node into the queue
//   -Iterate till queue is empty
//         i) Initialize the current node with top of the queue which is always valid node i.e node with 1 pathNode value and within the boundaries of map
//        ii) Check if Current Node's is matching with start Node ; calculate the Path and Return PathNode count if it mathes
//        iii) if its not matching then Push its(Current Node's) linked node into the queue
//             - Also add current node as a linked node's parent node into the local map 
//               because linked nodes are successor of current node
//        iv)  pop the queue, it will pop current node. , goto step 1) until queue is empty
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int FindPath(const int nStartX, const int nStartY,
             const int nTargetX, const int nTargetY,
             const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
             int* pOutBuffer, const int nOutBufferSize )
{
    char targetValue = pMap[nTargetX  + (nTargetY * nMapWidth)];

    PathNode* targetNode = new PathNode(targetValue, Vertex(nTargetX, nTargetY));
    UniqueQueue PathNodeQueue;
    PathNodeQueue.push(targetNode);

    //map of current node and its parent/previous node at the time of finding path
    std::unordered_map<int, int > visted;

    //added start node into visted map 
    visted[nStartX  + (nStartY * nMapWidth)];

    int targetMapPosition = targetNode->GetMapPosition(nMapWidth);
    while (!PathNodeQueue.empty())
    {
        PathNode* currentNode = PathNodeQueue.GetFront();
        int currentMapPosition = currentNode->GetMapPosition(nMapWidth);
        if (currentNode->GetPosition().X == nStartX && currentNode->GetPosition().Y==nStartY)
        {
            if (*currentNode == *targetNode)
            {
                pOutBuffer[0] = targetMapPosition;
                return 0;// start node is target node
            }

            int numberOfPathNodes = 0;
            int ptrNode = visted.at(currentMapPosition);
            while (ptrNode != targetMapPosition)
            {
                pOutBuffer[numberOfPathNodes++] = ptrNode;
                ptrNode = visted[ptrNode];
            }
            pOutBuffer[numberOfPathNodes] = targetMapPosition;
            return numberOfPathNodes+1;
        }
        else
        {
            currentNode->AddLinksFromMap(pMap, nMapWidth, nMapHeight, nStartX,nStartY);
            for(auto link : currentNode->GetLinks())
            {
                if (PathNodeQueue.push(link))
                {
                    visted[link->GetMapPosition(nMapWidth)] = currentMapPosition;
                }
            }
            PathNodeQueue.pop();
        }   
    }
    return -1;
}

int main()
{
    // input maze
    //unsigned char mat[100] =
    //{
    //      1, 1, 1, 1, 1, 0, 0, 1, 1, 1, //0-9
    //      1, 1, 1, 1, 1, 1, 0, 1, 0, 1, //10-19
    //      1, 0, 1, 0, 1, 1, 1, 0, 0, 1,
    //      1, 0, 1, 1, 1, 0, 1, 1, 0, 1,
    //      1, 0, 0, 1, 0, 0, 0, 1, 0, 1,
    //      1, 1, 1, 1, 1, 0, 0, 1, 1, 0,
    //      0, 1, 0, 0, 0, 0, 0, 0, 0, 1,
    //      0, 1, 1, 1, 1, 1, 1, 1, 0, 0,
    //      1, 1, 1, 1, 1, 0, 0, 1, 1, 1,
    //      0, 0, 1, 0, 0, 1, 1, 0, 0, 1 
    //};

    //const int nOutBuffserSize = 21;
    //int* pOutBuffer = new int[nOutBuffserSize];
    // Find shortest path from source (0, 0) to destination (7, 5)
    //int size = FindPath(0, 0, 2, 1, mat,10,10, pOutBuffer, nOutBuffserSize);

    unsigned char pMap[] = { 1, 1, 1, 1, 
                             0, 1, 0, 1, 
                             0, 1, 1, 1 };
    int pOutBuffer[12];
   
    int size = FindPath(0, 0, 3, 2, pMap, 4, 3, pOutBuffer, 12);

    //unsigned char pMap[] = { 0, 0, 1, 
    //                         0, 1, 1, 
    //                         1, 0, 1 };
    //int pOutBuffer[7];
    //int size = FindPath(1, 0, 2, 2, pMap, 3, 3, pOutBuffer, 7);

    for (int i = 0; i< size;i++)
    {
        std::cout << pOutBuffer[i] << " ";
    }

    return 0;
}