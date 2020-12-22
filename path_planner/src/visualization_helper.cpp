#include <debug_helper/visualization_helper.h>

namespace visualization_helper
{
    VisualizationHelper::VisualizationHelper()
    {
        this->max_marker_index_ = 0;
    }
    
    VisualizationHelper::VisualizationHelper(std::string topic_prefix) : VisualizationHelper()
    {
        ros::NodeHandle private_nh("~/" + topic_prefix);
        this->marker_array_publisher_ = private_nh.advertise<visualization_msgs::MarkerArray>("marker_array", 1);
        this->marker_publisher_ = private_nh.advertise<visualization_msgs::Marker>("marker", 1);
    }

    bool VisualizationHelper::visualizeMarkerArrays()
    {
        if(this->marker_array_list_.size() > 0)
        {
            for(std::pair<std::string, visualization_msgs::MarkerArray> const& marker_array: this->marker_array_list_)
            {
                this->setTimeStamps(marker_array.first);
                this->setMarkerIds(marker_array.first);
                this->marker_array_publisher_.publish(this->marker_array_list_[marker_array.first]);
            }
            return true;
        }
        else
        {
            ROS_ERROR("VisualizationHelper: No marker arrays to be visualized. Please add marker arrays and markers before visualizing.");
            return false;
        }
        
    }

    bool VisualizationHelper::visualizeMarkerArray(std::string marker_array_identificator)
    {
        if(this->isMarkerArrayExisting(marker_array_identificator))
        {
            this->setTimeStamps(marker_array_identificator);
            this->setMarkerIds(marker_array_identificator);
            this->marker_array_publisher_.publish(this->marker_array_list_[marker_array_identificator]);
            ros::Duration(0.1); // Wait for markers to be shown, maybe this helps to visualize them every time
            return true;
        }
        else
        {
            ROS_ERROR("VisualizationHelper: Identificator of marker_array was not found. Please check the given identificators that were handed to this method.");
            return false;
        }
    }

    std::string VisualizationHelper::addNewMarkerArray()
    {
        std::string unnamed_marker_array = "UnnamedMarkerArray" + std::to_string(this->marker_array_list_.size() - 1);
        if(this->addNewMarkerArray(unnamed_marker_array))
        {
            return unnamed_marker_array;
        }
        else
        {
            ROS_ERROR("VisualizationHelper: Something went wrong when adding an unnamed marker array with the index %i",
                      this->marker_array_list_.size() - 1);
            return "";
        }
    }

    bool VisualizationHelper::addNewMarkerArray(std::string marker_array_identificator)
    {
        if(!this->isMarkerArrayExisting(marker_array_identificator))
        {
            visualization_msgs::MarkerArray marker_array;
            this->marker_array_list_[marker_array_identificator] = marker_array;
            return true;
        }
        else
        {
            ROS_ERROR("VisualizationHelper: Marker array identificator was already existing. Error while inserting new marker array with indentificator: %s",
                      marker_array_identificator);
            return false;
        }
    }

    std::string VisualizationHelper::addMarkerTemplate(visualization_msgs::Marker marker_template)
    {
        std::string unnamed_marker_template = "UnnamedMarkerTemplate" + std::to_string(this->marker_array_list_.size() - 1);
        if(this->addMarkerTemplate(unnamed_marker_template, marker_template))
        {
            return unnamed_marker_template;
        }
        else
        {
            ROS_ERROR("VisualizationHelper: Something went wrong when adding an unnamed marker templare with the index %i",
                      this->marker_template_list_.size() - 1);
            return "";
        }
    }

    bool VisualizationHelper::addMarkerTemplate(std::string marker_template_identifier, visualization_msgs::Marker marker_template)
    {
        if(!this->isMarkerTemplateExisting(marker_template_identifier))
        {
            this->marker_template_list_[marker_template_identifier] = marker_template;
            return true;
        }
        else
        {
            ROS_ERROR("VisualizationHelper: Marker array identificator was already existing. Error while inserting new marker array with indentificator: %s",
                      marker_template_identifier);
            return false;
        }
    }

    bool VisualizationHelper::isMarkerArrayIdentExisting(std::string marker_array_identificator)
    {
        if(this->marker_array_list_.find(marker_array_identificator) != this->marker_array_list_.end())
        {
            return true;
        }
        else
        {
            false;
        }
    }

    bool VisualizationHelper::addMarkerToExistingMarkerArray(std::string marker_array_identificator,
                                                             visualization_msgs::Marker marker_to_add)
    {
        // Check if marker array exists
        if (!this->isMarkerArrayExisting(marker_array_identificator))
        {
            return false;
        }

        // No further checks for the content of the marker
        this->marker_array_list_[marker_array_identificator].markers.push_back(marker_to_add);
    }

    bool VisualizationHelper::addMarkersToExistingMarkerArray(std::string marker_array_identificator,
                                                              std::vector<visualization_msgs::Marker> marker_list_to_add)
    {
        // Check if marker array exists
        if (!this->isMarkerArrayExisting(marker_array_identificator))
        {
            return false;
        }

        for (visualization_msgs::Marker marker: marker_list_to_add)
        {
            // No further checks for the content of the marker
            this->marker_array_list_[marker_array_identificator].markers.push_back(marker);
        }

        return true;
    }

    bool VisualizationHelper::addMarkerToExistingMarkerArray(std::string marker_array_identificator,
                                                             geometry_msgs::Pose pose,
                                                             std::string marker_template_identificator)
    {
        // Check if marker array exists
        if (!this->isMarkerArrayExisting(marker_array_identificator) ||
            !this->isMarkerTemplateExisting(marker_template_identificator))
        {
            return false;
        }

        visualization_msgs::Marker marker_to_add = this->marker_template_list_[marker_template_identificator];
        marker_to_add.pose = pose;

        this->marker_array_list_[marker_array_identificator].markers.push_back(marker_to_add);
        return true;
    }
    
    bool VisualizationHelper::clearMarkerArray(std::string marker_array_identificator)
    {
        // Check if marker array exists
        if (!this->isMarkerArrayExisting(marker_array_identificator))
        {
            return false;
        }

        this->marker_array_list_[marker_array_identificator].markers.clear();
    }

    void VisualizationHelper::publishMarker(visualization_msgs::Marker marker_to_publish)
    {
        this->marker_publisher_.publish(marker_to_publish);
    }

    geometry_msgs::Pose VisualizationHelper::createGeometryPose(float x_coord, float y_coord)
    {
        geometry_msgs::Pose pose_to_return;
        geometry_msgs::Quaternion default_quaternion;
        tf::Quaternion default_tf_quaternion;
        default_tf_quaternion.setRPY(0.0, 0.0, 0.0);
        tf::quaternionTFToMsg(default_tf_quaternion, default_quaternion);
        pose_to_return.orientation = default_quaternion;
        pose_to_return.position.x = x_coord;
        pose_to_return.position.y = y_coord;
        pose_to_return.position.z = 0.0;

        return pose_to_return;
    }

    //PRIVATE METHODS
    bool VisualizationHelper::isMarkerArrayExisting(std::string marker_array_identificator)
    {
        return this->marker_array_list_.find(marker_array_identificator) != this->marker_array_list_.end();
    }

    bool VisualizationHelper::isMarkerTemplateExisting(std::string marker_template_identificator)
    {
        return this->marker_template_list_.find(marker_template_identificator) != this->marker_template_list_.end();
    }

    void VisualizationHelper::setTimeStamps(std::string marker_array_identificator)
    {
        for(int marker_counter = 0; marker_counter < (this->marker_array_list_[marker_array_identificator].markers.size() - 1); marker_counter++)
        {
            this->marker_array_list_[marker_array_identificator].markers[marker_counter].header.stamp = ros::Time::now();
        }
    }

    void VisualizationHelper::setMarkerIds(std::string marker_array_identificator)
    {
        double max_id = this->max_marker_index_;
        for (int marker_counter = 0;
             marker_counter < (this->marker_array_list_[marker_array_identificator].markers.size() - 1);
             marker_counter++)
        {
            max_id = max_id + marker_counter;
            this->marker_array_list_[marker_array_identificator].markers[marker_counter].id = max_id;
        }
        this->max_marker_index_ = max_id;
    }
}