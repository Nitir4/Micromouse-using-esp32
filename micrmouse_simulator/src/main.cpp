#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "MazeLoader.h"
#include "FloodFillAlgorithm.h"
#include "RandomWalkAlgorithm.h"
#include "RightHandRuleAlgorithm.h"
#include "Simulator.h"

namespace {

Direction parseDirection(const std::string& text) {
    if (text == "north") {
        return Direction::North;
    }
    if (text == "east") {
        return Direction::East;
    }
    if (text == "south") {
        return Direction::South;
    }
    if (text == "west") {
        return Direction::West;
    }

    throw std::runtime_error("Unsupported direction: " + text);
}

RenderMode parseRenderMode(const std::string& text) {
    if (text == "all") {
        return RenderMode::AllSteps;
    }
    if (text == "final") {
        return RenderMode::FinalOnly;
    }
    if (text == "none") {
        return RenderMode::None;
    }

    throw std::runtime_error("Unsupported render mode: " + text);
}

struct Options {
    std::string mazeFile;
    int width = 8;
    int height = 8;
    Position start{0, 0};
    std::vector<Position> goals{{7, 7}};
    Direction startDirection = Direction::East;
    std::string algorithm = "right-hand";
    std::size_t maxSteps = 100;
    bool interactive = false;
    RenderMode renderMode = RenderMode::AllSteps;
    bool hasSeed = false;
    std::uint32_t seed = 0;
};

void printUsage(const char* executable) {
    std::cout
        << "Usage: " << executable << " [options]\n"
        << "Options:\n"
        << "  --maze <path>             Load maze from file\n"
        << "  --width <n>               Generated maze width (default: 8)\n"
        << "  --height <n>              Generated maze height (default: 8)\n"
        << "  --start <x> <y>           Generated maze start cell\n"
        << "  --direction <dir>         Initial heading: north, east, south, west\n"
        << "  --goal <x> <y>            Add a generated maze goal cell (repeatable)\n"
        << "  --algorithm <name>        right-hand, random, or flood-fill\n"
        << "  --seed <n>                Seed for the random algorithm\n"
        << "  --max-steps <n>           Step limit before failure\n"
        << "  --render <mode>           all, final, or none\n"
        << "  --interactive             Wait for Enter after each step\n"
        << "  --help                    Show this help message\n";
}

Options parseOptions(int argc, char** argv) {
    Options options;
    bool customGoals = false;

    for (int index = 1; index < argc; ++index) {
        const std::string arg = argv[index];

        if (arg == "--maze") {
            if (index + 1 >= argc) {
                throw std::runtime_error("--maze requires a file path");
            }
            options.mazeFile = argv[++index];
        } else if (arg == "--width") {
            if (index + 1 >= argc) {
                throw std::runtime_error("--width requires a value");
            }
            options.width = std::stoi(argv[++index]);
        } else if (arg == "--height") {
            if (index + 1 >= argc) {
                throw std::runtime_error("--height requires a value");
            }
            options.height = std::stoi(argv[++index]);
        } else if (arg == "--start") {
            if (index + 2 >= argc) {
                throw std::runtime_error("--start requires x and y values");
            }
            options.start = {std::stoi(argv[++index]), std::stoi(argv[++index])};
        } else if (arg == "--direction") {
            if (index + 1 >= argc) {
                throw std::runtime_error("--direction requires a value");
            }
            options.startDirection = parseDirection(argv[++index]);
        } else if (arg == "--goal") {
            if (index + 2 >= argc) {
                throw std::runtime_error("--goal requires x and y values");
            }
            if (!customGoals) {
                options.goals.clear();
                customGoals = true;
            }
            options.goals.push_back({std::stoi(argv[++index]), std::stoi(argv[++index])});
        } else if (arg == "--algorithm") {
            if (index + 1 >= argc) {
                throw std::runtime_error("--algorithm requires a value");
            }
            options.algorithm = argv[++index];
        } else if (arg == "--seed") {
            if (index + 1 >= argc) {
                throw std::runtime_error("--seed requires a value");
            }
            options.seed = static_cast<std::uint32_t>(std::stoul(argv[++index]));
            options.hasSeed = true;
        } else if (arg == "--max-steps") {
            if (index + 1 >= argc) {
                throw std::runtime_error("--max-steps requires a value");
            }
            options.maxSteps = static_cast<std::size_t>(std::stoul(argv[++index]));
        } else if (arg == "--render") {
            if (index + 1 >= argc) {
                throw std::runtime_error("--render requires a value");
            }
            options.renderMode = parseRenderMode(argv[++index]);
        } else if (arg == "--interactive") {
            options.interactive = true;
        } else if (arg == "--help") {
            printUsage(argv[0]);
            std::exit(0);
        } else {
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }

    if (options.goals.empty()) {
        options.goals.push_back({options.width - 1, options.height - 1});
    }

    return options;
}

std::unique_ptr<IMazeSource> buildMazeSource(const Options& options) {
    if (!options.mazeFile.empty()) {
        return std::make_unique<FileMazeSource>(options.mazeFile);
    }
    return std::make_unique<GeneratedMazeSource>(options.width, options.height, options.start, options.goals);
}

std::unique_ptr<IMouseAlgorithm> buildAlgorithm(const Options& options) {
    const std::string& name = options.algorithm;
    if (name == "right-hand") {
        return std::make_unique<RightHandRuleAlgorithm>();
    }
    if (name == "random") {
        if (options.hasSeed) {
            return std::make_unique<RandomWalkAlgorithm>(options.seed);
        }
        return std::make_unique<RandomWalkAlgorithm>();
    }
    if (name == "flood-fill") {
        return std::make_unique<FloodFillAlgorithm>(
            options.width, options.height, options.start, options.goals, options.startDirection);
    }

    throw std::runtime_error("Unsupported algorithm: " + name);
}

}  // namespace

int main(int argc, char** argv) {
    try {
        Options options = parseOptions(argc, argv);
        std::unique_ptr<IMazeSource> mazeSource = buildMazeSource(options);
        Maze maze = mazeSource->load();
        if (!options.mazeFile.empty()) {
            options.width = maze.width();
            options.height = maze.height();
            options.start = maze.start();
            options.goals = maze.goals();
        }
        std::unique_ptr<IMouseAlgorithm> algorithm = buildAlgorithm(options);

        Simulator simulator(
            std::move(maze), options.startDirection, options.maxSteps, options.interactive, options.renderMode);
        const SimulationResult result = simulator.run(*algorithm);

        std::cout << (result.success ? "SUCCESS" : "FAILED") << "\n";
        std::cout << "Steps taken: " << result.stepsTaken << "\n";
        std::cout << "Final position: (" << result.finalPosition.x << ", " << result.finalPosition.y << ")\n";
        std::cout << "Final direction: " << directionSymbol(result.finalDirection) << "\n";
        std::cout << "Status: " << result.statusMessage << "\n";

        return result.success ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << "\n";
        return 1;
    }
}
