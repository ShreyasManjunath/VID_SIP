#include "planner_node/TSP.h"
#include "ros/ros.h"

VID::TSP::TSP(std::vector<Eigen::Vector4f>& VP, int depotIn, int num_vehicles_in)
{
    this->viewPoints = VP;
    this->num_vehicles = 1;
    this->depot = RoutingIndexManager::NodeIndex{0};
    // this->manager = std::make_unique<RoutingIndexManager>(
    //             new RoutingIndexManager{(int)viewPoints.size(), num_vehicles, depot}
    //             );
    // this->routing = std::make_unique<RoutingModel>(new RoutingModel{*manager});

    this->manager = new RoutingIndexManager{(int)viewPoints.size(), num_vehicles, depot};
    this->routing = new RoutingModel{*manager};
}

VID::TSP::~TSP()
{
    delete manager; manager = nullptr;
    delete routing; routing = nullptr;
    delete solution; solution = nullptr;
}

void VID::TSP::setDepot(int depotIn)
{
    this->depot = depotIn;
}

std::vector<std::vector<float>> VID::TSP::ComputeEuclideanDistanceMatrix(const std::vector<Eigen::Vector4f>& VPs)
{
    std::vector<std::vector<float>> mDistances = 
      std::vector<std::vector<float>>(VPs.size(), std::vector<float>(VPs.size(), float{0.0}));

    for(int fromNode = 0; fromNode < VPs.size(); fromNode++)
    {
        for(int toNode = 0; toNode < VPs.size(); toNode++)
        {
            if(fromNode != toNode)
            {
                mDistances[fromNode][toNode] = static_cast<float>(
                    std::hypot(
                        VPs[toNode][0] - VPs[fromNode][0],
                        VPs[toNode][1] - VPs[fromNode][1],
                        VPs[toNode][2] - VPs[fromNode][2]
                    )
                );
            }
        }
    }
    this->distances = mDistances;
    return mDistances;
}

void VID::TSP::solve()
{
    this->ComputeEuclideanDistanceMatrix(this->viewPoints);
    const int transit_callback_index = (*routing).RegisterTransitCallback(
      [this](int from_index, int to_index) -> float {
        // Convert from routing variable Index to distance matrix NodeIndex.
        auto from_node = (*manager).IndexToNode(from_index).value();
        auto to_node = (*manager).IndexToNode(to_index).value();
        return distances[from_node][to_node];
      });


    // Define cost of each arc
    (*routing).SetArcCostEvaluatorOfAllVehicles(transit_callback_index);

    // Setting first solution heuristic.
    RoutingSearchParameters searchParams = DefaultRoutingSearchParameters();
    searchParams.set_first_solution_strategy(FirstSolutionStrategy::PATH_CHEAPEST_ARC);
    searchParams.set_local_search_metaheuristic(
    LocalSearchMetaheuristic::GUIDED_LOCAL_SEARCH);
    searchParams.mutable_time_limit()->set_seconds(1);
    searchParams.set_log_search(false);

    // Solve the TSP problem
    const Assignment* mSolution = (*routing).SolveWithParameters(searchParams);
    solution = new Assignment{mSolution};
    mSolution = nullptr;
    this->ArrangeFinalRoute();
}

void VID::TSP::ArrangeFinalRoute()
{
    int index = (*routing).Start(0);
    float total_distance{0.0};
    std::stringstream route;
    while((*routing).IsEnd(index) == false)
    {
        auto node_index = (*manager).IndexToNode(index).value();
        route << node_index << " -> ";
        int previous_index = index;
        index = (*solution).Value((*routing).NextVar(index));

        finalRoute.push_back(viewPoints[node_index]);

        total_distance += (*routing).GetArcCostForVehicle(previous_index, index, int{0});
    }
    finalRoute.push_back(viewPoints[(*manager).IndexToNode(index).value()]);
    LOG(INFO) << route.str() << (*manager).IndexToNode(index).value();
    LOG(INFO) << "Route distance: " << total_distance << " meters";
    LOG(INFO) << "Problem solved in " << (*routing).solver()->wall_time() << "ms";

}

std::vector<Eigen::Vector4f> VID::TSP::getFinalRoute() const
{   
    return this->finalRoute;
}