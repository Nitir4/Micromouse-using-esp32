#include "AsciiRenderer.h"

#include <sstream>

#include "Direction.h"

std::string AsciiRenderer::render(const Maze& maze, const Position& mousePosition, Direction mouseDirection) {
    std::ostringstream out;

    for (int y = 0; y < maze.height(); ++y) {
        for (int x = 0; x < maze.width(); ++x) {
            out << '+';
            out << (maze.hasWall({x, y}, Direction::North) ? "---" : "   ");
        }
        out << "+\n";

        for (int x = 0; x < maze.width(); ++x) {
            out << (maze.hasWall({x, y}, Direction::West) ? '|' : ' ');

            char cellSymbol = ' ';
            Position current{x, y};
            if (maze.start() == current) {
                cellSymbol = 'S';
            }
            if (maze.isGoal(current)) {
                cellSymbol = 'G';
            }
            if (mousePosition == current) {
                cellSymbol = directionSymbol(mouseDirection);
            }

            out << ' ' << cellSymbol << ' ';
        }

        out << (maze.hasWall({maze.width() - 1, y}, Direction::East) ? "|\n" : " \n");
    }

    for (int x = 0; x < maze.width(); ++x) {
        out << '+';
        out << (maze.hasWall({x, maze.height() - 1}, Direction::South) ? "---" : "   ");
    }
    out << "+\n";

    return out.str();
}
