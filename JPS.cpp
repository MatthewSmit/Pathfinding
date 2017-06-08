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

    int HeuristicCostEstimate(int startX, int startY, int targetX, int targetY)
    {
        int dx = std::abs(startX - targetX);
        int dy = std::abs(startY - targetY);
        return (int)std::floor(std::sqrt(dx * dx + dy * dy));
    }
    int LineHeuristic(int startX, int startY, int targetX, int targetY)
    {
        int dx = std::abs(startX - targetX);
        int dy = std::abs(startY - targetY);
        assert((dx == 0 || dy == 0) && dx != dy);
        return std::max(dx, dy);
    }
    template <typename T>
    int signum(T value)
    {
        return (T(0) < value) - (value < T(0));
    }

    constexpr std::pair<int, int> Direction[]
    {
        { -1, 0 },
        { 1, 0 },
        { 0, -1 },
        { 0, 1 }
    };

    // Ideally an instance of this class could get reused per thread to avoid constantly allocating and freeing memory
    class PathMap
    {
    public:
        PathMap(const unsigned char* data, const int width, const int height) :
            data(data),
            width(width),
            height(height),
            size(width * height),
            nodes(new Node[size]),
            openSet(PriorityQueue(size))
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

        int FindPath(const int startX, const int startY, const int targetX, const int targetY, int* outBuffer, const int outBufferSize)
        {
            auto target = targetX + targetY * width;
            auto start = startX + startY * width;
            nodes[start].score = 0;
            AddToOpen(startX, startY, HeuristicCostEstimate(startX, startY, targetX, targetY));

            while (!openSet.IsEmpty())
            {
                auto currentNode = openSet.Pop();
                if (currentNode == target)
                    return ReconstructPath(*this, target, outBuffer, outBufferSize);

                nodes[currentNode].closed = true;
                auto nodeX = currentNode % width;
                auto nodeY = currentNode / width;

                int neighbours[]
                    {
                        -1, -1, -1, -1
                    };
                auto parent = nodes[currentNode].cameFrom;
                if (parent >= 0)
                {
                    auto parentX = parent % width;
                    auto parentY = parent / width;
                    auto dx = signum(nodeX - parentX);
                    auto dy = signum(nodeY - parentY);
                    auto i = 0;

                    if (dx == 0)
                    {
                        // Moving vertically
                        auto neighbourX = nodeX - 1;
                        auto neighbourY = nodeY;
                        auto neighbourNode = neighbourX + neighbourY * width;
                        if (neighbourX >= 0 && data[neighbourNode] == 1)
                            neighbours[i++] = neighbourNode;

                        neighbourX = nodeX + 1;
                        neighbourY = nodeY;
                        neighbourNode = neighbourX + neighbourY * width;
                        if (neighbourX < width && data[neighbourNode] == 1)
                            neighbours[i++] = neighbourNode;

                        neighbourX = nodeX;
                        neighbourY = nodeY + dy;
                        neighbourNode = neighbourX + neighbourY * width;
                        if (neighbourY >= 0 && neighbourY < height && data[neighbourNode] == 1)
                            neighbours[i] = neighbourNode;
                    }
                    else
                    {
                        // Moving horizontally
                        auto neighbourX = nodeX;
                        auto neighbourY = nodeY - 1;
                        auto neighbourNode = neighbourX + neighbourY * width;
                        if (neighbourY >= 0 && data[neighbourNode] == 1)
                            neighbours[i++] = neighbourNode;

                        neighbourX = nodeX;
                        neighbourY = nodeY + 1;
                        neighbourNode = neighbourX + neighbourY * width;
                        if (neighbourY < height && data[neighbourNode] == 1)
                            neighbours[i++] = neighbourNode;

                        neighbourX = nodeX + dx;
                        neighbourY = nodeY;
                        neighbourNode = neighbourX + neighbourY * width;
                        if (neighbourX >= 0 && neighbourX < width && data[neighbourNode] == 1)
                            neighbours[i] = neighbourNode;
                    }
                }
                else
                {
                    for (auto i = 0, j = 0; i < 4; ++i)
                    {
                        auto neighbourX = nodeX + Direction[i].first;
                        auto neighbourY = nodeY + Direction[i].second;
                        auto neighbourNode = neighbourX + neighbourY * width;
                        if (IsValid(neighbourX, neighbourY, neighbourNode))
                            neighbours[j++] = neighbourNode;
                    }
                }

                for (auto i = 0; i < 4; ++i)
                {
                    if (neighbours[i] == -1)
                        break;

                    auto neighbourNode = neighbours[i];
                    auto neighbourX = neighbourNode % width;
                    auto neighbourY = neighbourNode / width;
                    auto jumpPoint = Jump(neighbourX, neighbourY, nodeX, nodeY, target);

                    if (jumpPoint != -1 && !nodes[jumpPoint].closed)
                    {
                        auto jumpX = jumpPoint % width;
                        auto jumpY = jumpPoint / width;
                        auto jumpGScore = nodes[currentNode].score + LineHeuristic(nodeX, nodeY, jumpX, jumpY);

                        if (jumpGScore < nodes[jumpPoint].score)
                        {
                            nodes[jumpPoint].score = jumpGScore;
                            nodes[jumpPoint].cameFrom = currentNode;
                            auto fScore = jumpGScore + HeuristicCostEstimate(jumpX, jumpY, targetX, targetY);
                            auto currentFScore = openSet.Find(jumpPoint);
                            if (currentFScore > fScore)
                            {
                                openSet.Remove(jumpPoint);
                                AddToOpen(jumpPoint, fScore);
                            }
                            else if (currentFScore < 0)
                                AddToOpen(jumpPoint, fScore);
                        }
                    }
                }
            }
            return -1;
        }
        int ReconstructPath(const PathMap& map, int currentNode, int* outBuffer, int outBufferSize)
        {
            std::vector<int> path{};
            auto searchNode = currentNode;
            do
            {
                path.push_back(searchNode);
                searchNode = nodes[searchNode].cameFrom;
            }
            while (searchNode >= 0);
            std::reverse(path.begin(), path.end());

            auto outPtr = 0;
            for (auto i = 0; i < path.size() - 1; ++i)
            {
                auto coord0 = path[i];
                auto coord1 = path[i + 1];
                auto coord0X = coord0 % width;
                auto coord0Y = coord0 / width;
                auto coord1X = coord1 % width;
                auto coord1Y = coord1 / width;
                auto dx = signum(coord1X - coord0X);
                auto dy = signum(coord1Y - coord0Y);
                assert((dx == 0 || dy == 0) && dx != dy);

                if (dx != 0)
                {
                    for (auto x = coord0X + dx; x != coord1X; x += dx)
                    {
                        if (outPtr < outBufferSize)
                            outBuffer[outPtr++] = x + coord0Y * width;
                        else ++outPtr;
                    }
                }
                else
                {
                    for (auto y = coord0Y + dy; y != coord1Y; y += dy)
                    {
                        if (outPtr < outBufferSize)
                            outBuffer[outPtr++] = coord0X + y * width;
                        else ++outPtr;
                    }
                }

                if (outPtr < outBufferSize)
                    outBuffer[outPtr++] = coord1;
                else ++outPtr;
            }
            return outPtr;
        }
        int Jump(int nodeX, int nodeY, int parentX, int parentY, int target)
        {
            auto node = nodeX + nodeY * width;
            auto dx = nodeX - parentX;
            auto dy = nodeY - parentY;

            if (!IsValid(nodeX, nodeY, node))
                return -1;

            if (node == target)
                return node;

            if (dx != 0)
            {
                if ((IsValid(nodeX, nodeY - 1) && !IsValid(nodeX - dx, nodeY - 1)) ||
                    (IsValid(nodeX, nodeY + 1) && !IsValid(nodeX - dx, nodeY + 1)))
                    return node;
            }
            else
            {
                if ((IsValid(nodeX - 1, nodeY) && !IsValid(nodeX - 1, nodeY - dy)) ||
                    (IsValid(nodeX + 1, nodeY) && !IsValid(nodeX + 1, nodeY - dy)))
                    return node;
                if (Jump(nodeX + 1, nodeY, nodeX, nodeY, target) || Jump(nodeX - 1, nodeY, nodeX, nodeY, target))
                    return node;
            }
            return Jump(nodeX + dx, nodeY + dy, nodeX, nodeY, target);
        }

        inline bool IsValid(int x, int y) const
        {
            return IsValid(x, y, x + y * width);
        }
        inline bool IsValid(int x, int y, int node) const
        {
            return !(x < 0 || x >= width || y < 0 || y >= height || data[node] == 0);
        }

    private:
        struct Node
        {
            int score;
            int cameFrom;
            bool closed;
        };

        const unsigned char* data;
        const int width;
        const int height;
        const int size;
        Node* nodes;
        PriorityQueue openSet;
    };
}

int FindPath_JPS(const int nStartX, const int nStartY,
                   const int nTargetX, const int nTargetY,
                   const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
                   int* pOutBuffer, const int nOutBufferSize)
{
    auto map = PathMap(pMap, nMapWidth, nMapHeight);
    return map.FindPath(nStartX, nStartY, nTargetX, nTargetY, pOutBuffer, nOutBufferSize);
}

int FindPath(const int nStartX, const int nStartY,
             const int nTargetX, const int nTargetY,
             const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
             int* pOutBuffer, const int nOutBufferSize)
{
    return FindPath_JPS(nStartX, nStartY, nTargetX, nTargetY, pMap, nMapWidth, nMapHeight, pOutBuffer, nOutBufferSize);
}