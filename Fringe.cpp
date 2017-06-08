#include <cassert>
#include <limits>
#include <memory>
#include <algorithm>

namespace
{
    constexpr auto Infinity = std::numeric_limits<int>::max();

    class FringeMap
    {
    public:
        struct Node
        {
            Node* Next = nullptr;
            Node* Previous = nullptr;
            int Value;
            int GScore;
            int Parent;
        };

        FringeMap(int size)
        {
            Start = nullptr;
            End = nullptr;
            Pointers = new Node[size];
            for (auto i = 0; i < size; ++i)
            {
                Pointers[i].Value = -1;
                Pointers[i].GScore = -1;
                Pointers[i].Parent = -1;
            }
        }
        ~FringeMap()
        {
            delete[] Pointers;
        }

        void Add(int data)
        {
            Node* node = &Pointers[data];
            node->Value = data;
            if (End != nullptr)
            {
                End->Next = node;
                node->Previous = End;
            }
            if (Start == nullptr)
                Start = node;
            End = node;
        }
        void SetGScore(int node, int value)
        {
            Pointers[node].GScore = value;
        }
        void SetParent(int node, int value)
        {
            Pointers[node].Parent = value;
        }
        void Remove(int data)
        {
            auto node = &Pointers[data];
            if (node->Previous != nullptr)
                node->Previous->Next = node->Next;
            if (node->Next != nullptr)
                node->Next->Previous = node->Previous;
            if (node == End)
                End = node->Previous;
            if (node == Start)
                Start = node->Next;
            node->Value = -1;
        }

        Node* StartNode() const
        {
            return Start;
        }
        Node* EndNode() const
        {
            return End;
        }

        int GetGScore(int node) const
        {
            return Pointers[node].GScore;
        }
        int GetParent(int node) const
        {
            return Pointers[node].Parent;
        }
        bool Contains(int data) const
        {
            return Pointers[data].Value >= 0;
        }
        bool IsEmpty() const
        {
            return Start == nullptr;
        }

    private:
        Node* Start;
        Node* End;
        Node* Pointers;
    };

    int HeuristicCostEstimate(int startX, int startY, int targetX, int targetY)
    {
        int dx = std::abs(startX - targetX);
        int dy = std::abs(startY - targetY);
        return dx + dy;
        //return (int)std::floor(std::sqrt(dx * dx + dy * dy));
    }
    int ReversePath(const FringeMap& fringe, int currentNode, int* outBuffer, int outBufferSize)
    {
        auto pathSize = 0;
        auto searchNode = currentNode;
        while ((searchNode = fringe.GetParent(searchNode)) >= 0)
            pathSize++;

        if (pathSize > outBufferSize)
            return pathSize;

        searchNode = currentNode;
        for (int i = std::min(outBufferSize, pathSize - 1); i >= 0; --i)
        {
            outBuffer[i] = searchNode;
            searchNode = fringe.GetParent(searchNode);
        }
        return pathSize;
    }

    constexpr std::pair<int, int> Direction[]
    {
        { -1, 0 },
        { 1, 0 },
        { 0, -1 },
        { 0, 1 }
    };
}

int FindPath_Fringe(const int nStartX, const int nStartY,
                   const int nTargetX, const int nTargetY,
                   const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
                   int* pOutBuffer, const int nOutBufferSize)
{
    auto fringe = FringeMap{nMapWidth * nMapHeight};
    fringe.Add(nStartX + nStartY * nMapWidth);
    fringe.SetGScore(nStartX + nStartY * nMapWidth, 0);
    auto flimit = HeuristicCostEstimate(nStartX, nStartY, nTargetX, nTargetY);
    int goal = nTargetX + nTargetY * nMapWidth;

    while (!fringe.IsEmpty())
    {
        auto fmin = Infinity;
        auto currentNode = fringe.StartNode();
        auto endNode = fringe.EndNode();
        while (currentNode != nullptr)
        {
            auto node = currentNode->Value;
            auto g = fringe.GetGScore(node);
            auto nodeX = node % nMapWidth;
            auto nodeY = node / nMapWidth;
            auto f = g + HeuristicCostEstimate(nodeX, nodeY, nTargetX, nTargetY);
            if (f > flimit)
            {
                fmin = std::min(f, fmin);
                continue;
            }
            if (node == goal)
                return ReversePath(fringe, goal, pOutBuffer, nOutBufferSize);
            for (auto i = 3; i >= 0; --i)
            {
                auto neighbourX = nodeX + Direction[i].first;
                auto neighbourY = nodeY + Direction[i].second;
                auto neighbourNode = neighbourX + neighbourY * nMapWidth;
                if (neighbourX < 0 || neighbourX >= nMapWidth || neighbourY < 0 || neighbourY >= nMapHeight || pMap[neighbourNode] == 0)
                    continue;

                auto gChild = g + 1;
                auto gCached = fringe.GetGScore(neighbourNode);
                if (gCached >= 0 && gChild >= gCached)
                    continue;
                if (fringe.Contains(neighbourNode))
                    fringe.Remove(neighbourNode);
                fringe.Add(neighbourNode);
                fringe.SetGScore(neighbourNode, gChild);
                fringe.SetParent(neighbourNode, node);
            }

            if (currentNode == endNode)
            {
                fringe.Remove(currentNode->Value);
                break;
            }
            else
            {
                auto temp = currentNode;
                currentNode = currentNode->Next;
                fringe.Remove(temp->Value);
            }
        }
        flimit = fmin;
    }

    return -1;
}