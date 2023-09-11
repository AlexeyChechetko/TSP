#include <cmath>
#include <cstdint>
#include <sstream>
#include <vector>
#include <iostream>
#include <fstream>

#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

namespace operations_research {
    struct DataModel {
        std::vector<std::vector<int>> locations;
        const int num_vehicles = 1;
        const RoutingIndexManager::NodeIndex depot{0};
    };

    // @brief Generate distance matrix.
    std::vector<std::vector<int64_t>> ComputeEuclideanDistanceMatrix(
        const std::vector<std::vector<int>>& locations) {
        std::vector<std::vector<int64_t>> distances =
            std::vector<std::vector<int64_t>>(
                locations.size(), std::vector<int64_t>(locations.size(), int64_t{ 0 }));
        for (int from_node = 0; from_node < locations.size(); from_node++) {
            for (int to_node = 0; to_node < locations.size(); to_node++) {
                if (from_node != to_node)
                    distances[from_node][to_node] = static_cast<int64_t>(
                        std::hypot((locations[to_node][0] - locations[from_node][0]),
                            (locations[to_node][1] - locations[from_node][1])));
            }
        }
        return distances;
    }

    //! @brief Print the solution
    //! @param[in] manager Index manager used.
    //! @param[in] routing Routing solver used.
    //! @param[in] solution Solution found by the solver.
    void PrintSolution(const operations_research::RoutingIndexManager& manager,
        const operations_research::RoutingModel& routing, const operations_research::Assignment& solution) {
        // Inspect solution.
        std::cout << "Objective: " << solution.ObjectiveValue() << " miles" << std::endl;
        int64_t index = routing.Start(0);
        std::cout << "Route:";
        int64_t distance{ 0 };
        std::stringstream route;
        while (routing.IsEnd(index) == false) {
            route << manager.IndexToNode(index).value() << " -> ";
            int64_t previous_index = index;
            index = solution.Value(routing.NextVar(index));
            distance += routing.GetArcCostForVehicle(previous_index, index, int64_t{ 0 });
        }
        std::cout << route.str() << manager.IndexToNode(index).value();
        std::cout << std::endl;
        std::cout << "Route distance: " << distance << "miles" << std::endl;
        std::cout << "Problem solved in " << routing.solver()->wall_time() << "ms";
    }

    void Tsp() {
        // Instantiate the data problem.
        DataModel data;

        std::ifstream myfile;
        myfile.open("C:/Users/Alex/source/Visual Studio c++/Projects VS/TSP/data/tsp_33810_1", std::ios::in);
        if (myfile.is_open())
        {
            int n;
            double x, y;
            std::vector<int> coordinate;

            myfile >> n;
            while (myfile >> x >> y)
            {
                coordinate.push_back(x);
                coordinate.push_back(y);
                data.locations.push_back(coordinate);
                coordinate.clear();
            }
            myfile.close();
        }
        else
        {
            std::cout << "Smth wrong with the path to the folder or smth else." << std::endl;
        }

        // Create Routing Index Manager
        RoutingIndexManager manager(data.locations.size(), data.num_vehicles,
            data.depot);

        // Create Routing Model.
        RoutingModel routing(manager);

        const auto distance_matrix = ComputeEuclideanDistanceMatrix(data.locations);
        const int transit_callback_index = routing.RegisterTransitCallback(
            [&distance_matrix, &manager](const int64_t from_index,
                const int64_t to_index) -> int64_t {
                    // Convert from routing variable Index to distance matrix NodeIndex.
                    const int from_node = manager.IndexToNode(from_index).value();
                    const int to_node = manager.IndexToNode(to_index).value();
                    return distance_matrix[from_node][to_node];
            });

        // Define cost of each arc.
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index);

        // Setting first solution heuristic.
        RoutingSearchParameters searchParameters = DefaultRoutingSearchParameters();
        searchParameters.set_first_solution_strategy(
            FirstSolutionStrategy::PATH_CHEAPEST_ARC);
        searchParameters.mutable_time_limit()->set_seconds(180);
        // Solve the problem.
        const Assignment* solution = routing.SolveWithParameters(searchParameters);

        // Print solution on console.
        PrintSolution(manager, routing, *solution);
    }
}  // namespace operations_research

int main(int /*argc*/, char* /*argv*/[]) 
{
    operations_research::Tsp();
    return EXIT_SUCCESS;
}