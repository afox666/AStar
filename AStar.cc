//
// Created by afox on 22-10-9.
//
#include "AStar.h"

std::array<GraphLocation, 4> SquareGrid::DIRS =
        {GraphLocation{1, 0}, GraphLocation{0, -1}, GraphLocation{-1, 0}, GraphLocation{0, 1}};

bool SquareGrid::InBounds(GraphLocation id) const {
    return 0 <= id.x && id.x < width_ && 0 <= id.y && id.y < height_;
}

bool SquareGrid::Passable(GraphLocation id) const {
    return walls_.find(id) == walls_.end();
}

std::vector<GraphLocation> SquareGrid::Neighbors(GraphLocation id) const {
    std::vector<GraphLocation> results;

    for (GraphLocation dir: DIRS) {
        GraphLocation next{id.x + dir.x, id.y + dir.y};
        if (InBounds(next) && Passable(next)) {
            results.push_back(next);
        }
    }

    if ((id.x + id.y) % 2 == 0) {
        // aesthetic improvement on square grids
        std::reverse(results.begin(), results.end());
    }

    return results;
}

int main() {

}
