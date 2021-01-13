#include <memory> // Usage of smart pointers
#include <vector>
#include <list>
#include <Eigen/Dense>
#include <shared_ptr.h>

namespace geometry_info
{
    class EdgeInfo
    {
        public:
            EdgeInfo(Eigen::Vector2d start_point, Eigen::Vector2d end_point);

            void setStartPoint(Eigen::Vector2d start_point);
            Eigen::Vector2d getStartPoint();
            void setEndPoint(Eigen::Vector2d end_point);
            Eigen::Vector2d getEndPoint();
            Eigen::Vector2d getEdgeVector();


        private:
            void calculateEdgeVector();

            std::shared_ptr<Eigen::Vector2d> start_point_;
            std::shared_ptr<Eigen::Vector2d> end_point_;

            std::shared_ptr<Eigen::Vector2d> edge_vector_;
    };
}