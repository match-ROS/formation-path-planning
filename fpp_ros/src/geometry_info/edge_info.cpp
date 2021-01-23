#include <fpp_ros/geometry_info/edge_info.h>

namespace geometry_info
{
    EdgeInfo::EdgeInfo(Eigen::Vector2f start_point, Eigen::Vector2f end_point)
        : start_point_(start_point), end_point_(end_point)
    {
        this->calculateEdgeVector();
    }

    Eigen::Vector2f EdgeInfo::getStartPoint()
    {
        return this->start_point_;
    }

    Eigen::Vector2f EdgeInfo::getEndPoint()
    {
        return this->end_point_;
    }

    Eigen::Vector2f EdgeInfo::getEdgeVector()
    {
        return this->edge_vector_;
    }

    void EdgeInfo::calculateEdgeVector()
    {
        this->edge_vector_ = this->end_point_ - this->start_point_;
    }
}