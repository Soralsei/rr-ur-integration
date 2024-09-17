#include "rr100_ur_placement/workspace_database.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

// Taken from : https://stackoverflow.com/a/40628179
template<typename InputIterator, typename ValueType>
InputIterator closest(InputIterator first, InputIterator last, ValueType value)
{
    return std::min_element(first, last, [&](ValueType x, ValueType y)
    {   
        return std::abs(x - value) < std::abs(y - value);
    });
}

namespace rhoban
{
    using json = nlohmann::json;

    WorkspaceDatabase::WorkspaceDatabase(const std::string json_path)
    {
        initialize(json_path);        
    }

    WorkspaceDatabase::~WorkspaceDatabase(){}

    void WorkspaceDatabase::initialize(const std::string json_path) 
    {
        std::ifstream config(json_path);
        const json data = json::parse(config);
        for (auto &element : data)
        {
            try
            {
                LayerPtr layer = std::make_shared<Layer>();
                for (auto &point : element.at("points"))
                {
                    Eigen::Vector2d p(point.at("x"), point.at("y"));
                    layer->push_back(p);
                }
                heights.push_back(element.at("z"));
                layers.push_back(std::move(layer));
            }
            catch(const json::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
        }
    }

    Layer& WorkspaceDatabase::getLayer(double z)
    {
        std::size_t closest_z = getClosestZPosition(z);
        return *layers[closest_z];

    }

    std::vector<LayerPtr>& WorkspaceDatabase::getAllLayers()
    {
        return layers;
    }

    std::vector<double>& WorkspaceDatabase::getAllHeights()
    {
        return heights;
    }

    std::size_t WorkspaceDatabase::getClosestZPosition(double z)
    {
        auto pos = closest(heights.begin(), heights.end(), z);
        return pos - heights.begin();
    }
} // namespace rhoban
