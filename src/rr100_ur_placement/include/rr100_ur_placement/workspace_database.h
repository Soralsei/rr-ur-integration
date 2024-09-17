#pragma once
#include <map>
#include <vector>
#include <memory>
#include "Eigen/Dense"

namespace rhoban
{
    using Layer = std::vector<Eigen::Vector2d>;
    using LayerPtr = std::shared_ptr<Layer>;
    class WorkspaceDatabase
    {
    private:
        std::vector<double> heights;
        std::vector<LayerPtr> layers;
    public:
        WorkspaceDatabase(const std::string json_path);
        ~WorkspaceDatabase();

        Layer& getLayer(double z);
        std::vector<LayerPtr>& getAllLayers();
        std::vector<double>& getAllHeights();
    private:
        void initialize(const std::string json_path);
        std::size_t getClosestZPosition(double z);
    };
    
} // namespace rhoban
