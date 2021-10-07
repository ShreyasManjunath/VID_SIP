#ifndef __TSP_H__
#define __TSP_H__

#include <stdlib.h>
#include <cmath>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <memory>

#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

using namespace operations_research;

namespace VID
{
    
    class TSP
    {
        private:
            std::vector<Eigen::Vector4f> viewPoints;
            std::vector<Eigen::Vector4f> finalRoute;
            std::vector<std::vector<float>> distances;
            // RoutingIndexManager manager{0, 1, RoutingIndexManager::NodeIndex{0}};
            // RoutingModel routing{manager};
            RoutingIndexManager* manager;
            RoutingModel* routing;
            const Assignment* solution;
            // std::unique_ptr<RoutingIndexManager> manager;
            // std::unique_ptr<RoutingModel> routing;

            int num_vehicles;
            RoutingIndexManager::NodeIndex depot{0};

            // Private Methods
            void ArrangeFinalRoute();

        public:
            TSP() = delete;
            TSP(std::vector<Eigen::Vector4f>& VP, int depotIn, int num_vehicles_in);
            ~TSP();
            //  Methods to use.
            void init(std::vector<Eigen::Vector4f>& VP, int depotIn, int num_vehicles_in);
            void setDepot(int depotIn);
            std::vector<std::vector<float>> ComputeEuclideanDistanceMatrix(const std::vector<Eigen::Vector4f>& VPs);
            void solve();
            std::vector<Eigen::Vector4f> getFinalRoute() const;
    };
}
#endif