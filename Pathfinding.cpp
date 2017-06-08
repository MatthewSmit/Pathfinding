#define NDEBUG

#include <cassert>
#include <limits>
#include <memory>
#include <algorithm>

namespace
{
    constexpr auto Infinity = std::numeric_limits<int>::max();

    class PriorityQueue
    {
    public:
        explicit PriorityQueue(int maxSize) :
            items(new PriorityNode[maxSize]),
            itemIndices(new int[maxSize]),
            currentSize(),
            maxSize(maxSize)
        {
            for (auto i = 0; i < maxSize; ++i)
                itemIndices[i] = -1;
        }
        ~PriorityQueue()
        {
            delete[] items;
            delete[] itemIndices;
        }

        void Add(int node, int priority)
        {
            assert(currentSize < maxSize);

            auto index = currentSize;
            itemIndices[node] = index;
            items[index].Value = node;
            items[index].Priority = priority;
            currentSize++;

            SiftUp(index);
        }
        int Pop()
        {
            if (currentSize == 0)
                return -1;

            auto node = items[0].Value;
            currentSize--;
            if (currentSize > 0)
            {
                SwapItems(0, currentSize);
                SiftDown(0);
            }
            itemIndices[node] = -1;
            return node;
        }
        int Find(int node) const
        {
            auto index = itemIndices[node];
            if (index >= 0)
                return items[index].Priority;
            return -1;
        }
        void Remove(int node)
        {
            if (currentSize == 0)
                return;
            if (currentSize == 1)
            {
                currentSize--;
                return;
            }

            auto index = itemIndices[node];
            SwapItems(index, currentSize - 1);
            currentSize--;
            SiftDown(index);
        }

        bool IsEmpty() const
        {
            return currentSize == 0;
        }

    private:
        struct PriorityNode
        {
            int Value;
            int Priority;
        };

        PriorityNode* items;
        int* itemIndices;
        int currentSize;
        int maxSize;

        void SiftDown(int index)
        {
            while ((index + 1) * 2 <= currentSize)
            {
                auto left = 2 * index + 1;
                auto right = left + 1;
                auto smallest = left;
                if (right < currentSize && items[left].Priority > items[right].Priority)
                    smallest = right;
                if (items[index].Priority > items[smallest].Priority)
                    SwapItems(index, smallest);
                else break;
                index = smallest;
            }
        }
        void SiftUp(int index)
        {
            auto parent = (index - 1) / 2;
            while (index >= 0 && items[index].Priority < items[parent].Priority)
            {
                SwapItems(index, parent);
                index = parent;
                parent = (index - 1) / 2;
            }
        }
        void SwapItems(int index, int swapIndex)
        {
            std::swap(items[index], items[swapIndex]);
            itemIndices[items[index].Value] = index;
            itemIndices[items[swapIndex].Value] = swapIndex;
        }
    };

    // Ideally an instance of this class could get reused per thread to avoid constantly allocating and freeing memory
    class PathMap
    {
    public:
        PathMap(int width, int height) :
            size(width * height),
            nodes(new Node[size]),
            openSet(PriorityQueue(size)),
            width(width),
            height(height)
        {
            // Start the map with a default value of infinity
            for (auto i = 0; i < size; ++i)
            {
                nodes[i].score = Infinity;
                nodes[i].cameFrom = -1;
                nodes[i].closed = false;
            }
        }
        ~PathMap()
        {
            delete[] nodes;
        }

        void AddToOpen(int x, int y, int priority)
        {
            auto node = y * width + x;
            assert(node >= 0 && node < size);
            openSet.Add(node, priority);
        }
        void AddToOpen(int node, int priority)
        {
            assert(node >= 0 && node < size);
            openSet.Add(node, priority);
        }
        int FindOpen(int node) const
        {
            assert(node >= 0 && node < size);
            return openSet.Find(node);
        }
        void RemoveOpen(int node)
        {
            assert(node >= 0 && node < size);
            openSet.Remove(node);
        }
        int PopOpen()
        {
            return openSet.Pop();
        }

        void SetValues(int x, int y, int score, int cameFrom)
        {
            auto node = y * width + x;
            assert(node >= 0 && node < size);
            nodes[node].score = score;
            nodes[node].cameFrom = cameFrom;
        }
        void SetClosed(int node)
        {
            assert(node >= 0 && node < size);
            nodes[node].closed = true;
        }

        int GetCameFrom(int node) const
        {
            assert(node >= 0 && node < size);
            return nodes[node].cameFrom;
        }
        int GetGScore(int node) const
        {
            assert(node >= 0 && node < size);
            return nodes[node].score;
        }
        int GetGScore(int x, int y) const
        {
            auto node = y * width + x;
            assert(node >= 0 && node < size);
            return nodes[node].score;
        }
        bool IsClosed(int node) const
        {
            assert(offset >= 0 && offset < size);
            return nodes[node].closed;
        }
        bool OpenIsEmpty() const
        {
            return openSet.IsEmpty();
        }

    private:
        struct Node
        {
            int score;
            int cameFrom;
            bool closed;
        };

        int size;
        Node* nodes;
        PriorityQueue openSet;
        int width;
        int height;
    };

    int HeuristicCostEstimate(int startX, int startY, int targetX, int targetY)
    {
        int dx = std::abs(startX - targetX);
        int dy = std::abs(startY - targetY);
        return (int)std::floor(std::sqrt(dx * dx + dy * dy));
    }
    int ReconstructPath(const PathMap& map, int currentNode, int* outBuffer, int outBufferSize)
    {
        auto pathSize = 0;
        auto searchNode = currentNode;
        while ((searchNode = map.GetCameFrom(searchNode)) >= 0)
            pathSize++;

        if (pathSize > outBufferSize)
            return pathSize;

        searchNode = currentNode;
        for (int i = std::min(outBufferSize, pathSize - 1); i >= 0; --i)
        {
            outBuffer[i] = searchNode;
            searchNode = map.GetCameFrom(searchNode);
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

int FindPath_AStar(const int nStartX, const int nStartY,
                 const int nTargetX, const int nTargetY,
                 const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
                 int* pOutBuffer, const int nOutBufferSize)
{
    auto target = nTargetX + nTargetY * nMapWidth;
    auto map = PathMap(nMapWidth, nMapHeight);
    map.SetValues(nStartX, nStartY, 0, -1);
    map.AddToOpen(nStartX, nStartY, HeuristicCostEstimate(nStartX, nStartY, nTargetX, nTargetY));

    while (!map.OpenIsEmpty())
    {
        auto currentNode = map.PopOpen();
        if (currentNode == target)
            return ReconstructPath(map, target, pOutBuffer, nOutBufferSize);

        map.SetClosed(currentNode);
        auto nodeX = currentNode % nMapWidth;
        auto nodeY = currentNode / nMapWidth;

        for (auto i = 0; i < 4; ++i)
        {
            auto neighbourX = nodeX + Direction[i].first;
            auto neighbourY = nodeY + Direction[i].second;
            auto neighbourNode = neighbourX + neighbourY * nMapWidth;
            if (neighbourX < 0 || neighbourX >= nMapWidth || neighbourY < 0 || neighbourY >= nMapHeight || map.IsClosed(neighbourNode) || pMap[neighbourNode] == 0)
                continue;
            auto neighbourGScore = map.GetGScore(currentNode) + 1;

            if (neighbourGScore < map.GetGScore(neighbourX, neighbourY))
            {
                map.SetValues(neighbourX, neighbourY, neighbourGScore, currentNode);
                auto fScore = neighbourGScore + HeuristicCostEstimate(neighbourX, neighbourY, nTargetX, nTargetY);
                auto currentFScore = map.FindOpen(neighbourNode);
                if (currentFScore > fScore)
                {
                    map.RemoveOpen(neighbourNode);
                    map.AddToOpen(neighbourNode, fScore);
                }
                else if (currentFScore < 0)
                    map.AddToOpen(neighbourNode, fScore);
            }
        }
    }
    return -1;
}

int FindPath(const int nStartX, const int nStartY,
             const int nTargetX, const int nTargetY,
             const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
             int* pOutBuffer, const int nOutBufferSize)
{
    return FindPath_AStar(nStartX, nStartY, nTargetX, nTargetY, pMap, nMapWidth, nMapHeight, pOutBuffer, nOutBufferSize);
}