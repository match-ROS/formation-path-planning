#include <fpp_ros/geometry_info/edge_info.h>

namespace geometry_info
{
    EdgeInfo::EdgeInfo(Eigen::Vector2f start_point, Eigen::Vector2f end_point)
        : start_point_(nullptr), end_point_(nullptr), edge_vector_(nullptr)
    {
        this->start_point_ = std::make_shared<Eigen::Vector2f>(start_point);
        this->end_point_ = std::make_shared<Eigen::Vector2f>(end_point);

        this->calculateEdgeVector();
    }   

    void EdgeInfo::setStartPoint(Eigen::Vector2f start_point)
    {
        this->start_point_ = std::make_shared<Eigen::Vector2f>(start_point);
        this->calculateEdgeVector();
    }

    Eigen::Vector2f EdgeInfo::getStartPoint()
    {
        return *this->start_point_.get();
    }

    void EdgeInfo::setEndPoint(Eigen::Vector2f end_point)
    {
        this->end_point_ = std::make_shared<Eigen::Vector2f>(end_point);
        this->calculateEdgeVector();
    }

    Eigen::Vector2f EdgeInfo::getEndPoint()
    {
        return *this->end_point_.get();
    }

    Eigen::Vector2f EdgeInfo::getEdgeVector()
    {
        return *this->edge_vector_.get();
    }

    void EdgeInfo::calculateEdgeVector()
    {
        if(this->start_point_ && !this->end_point_)
        {
            this->edge_vector_ = std::make_shared<Eigen::Vector2f>(*this->end_point_.get() - *this->start_point_.get());
        }
    }
}