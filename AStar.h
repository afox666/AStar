//
// Created by afox on 22-10-9.
//

#ifndef ASTAR_ASTAR_H
#define ASTAR_ASTAR_H

#include <iostream>
#include <iomanip>
#include <map>
#include <set>
#include <array>
#include <vector>
#include <utility>
#include <queue>
#include <tuple>
#include <algorithm>
#include <cstdlib>

struct GraphLocation {
    int x;
    int y;

    bool operator==(GraphLocation a) const {
        return a.x == this->x && a.y == this->y;
    }

    bool operator!=(GraphLocation a) {
        return !(a == *this);
    }

    bool operator<(GraphLocation a) {
        return std::tie(this->x, this->y) < std::tie(a.x, a.y);
    }

    std::ostream &operator<<(std::ostream &out) {
        out << '(' << this->x << ',' << this->y << ')';
        return out;
    }
};

class SquareGrid {
public:
    SquareGrid(int width, int height)
            : width_(width), height_(height) {}

    static std::array<GraphLocation, 4> DIRS;

    int width_, height_;
    std::set<GraphLocation> walls_;

    bool InBounds(GraphLocation id) const;

    bool Passable(GraphLocation id) const;

    std::vector<GraphLocation> Neighbors(GraphLocation id) const;
};

template<typename T, typename priority_t>
class PriorityQueue {
public:
    typedef std::pair<priority_t, T> PQElement;
    std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> elements;

    inline bool Empty() const {
        return elements.empty();
    }

    inline void Put(T item, priority_t priority) {
        elements.emplace(priority, item);
    }

    T Get() {
        T best_item = elements.top().second;
        elements.pop();
        return best_item;
    }
};

// a -> b cost
inline double Heuristic(GraphLocation &a, GraphLocation &b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

template<typename Location, typename Graph>
void AStarSearch(Graph graph, Location start, Location goal,
                 std::map<Location, Location> &came_from,
                 std::map<Location, double> &cost_so_far) {
    PriorityQueue<Location, double> frontier;
    frontier.Put(start, 0);

    came_from[start] = start;
    cost_so_far[start] = 0;

    while(!frontier.Empty()) {
        Location current = frontier.Get();

        if(current == goal) {
            break;
        }

        for(auto next : graph.Neighobors(current)) {
            double new_cost = cost_so_far[current] + graph.cost(current, next); // G(n)
            if(cost_so_far.find(next) == cost_so_far.end() || new_cost < cost_so_far[next]) {
                cost_so_far[next] = new_cost;
                double priority = new_cost + Heuristic(current, next); // G(n) + H(n)
                frontier.Put(next, priority);
                came_from[next] = current;
            }
        }
    }
}

#endif //ASTAR_ASTAR_H
