#include <debug_helper/visualization_helper.h>

namespace visualization_helper
{
    VisualizationHelper::VisualizationHelper()
    {
        this->max_marker_index_ = 0;
    }
    
    VisualizationHelper::VisualizationHelper(std::string name) : VisualizationHelper()
    {
        ros::NodeHandle private_nh("~/" + name);
        this->marker_array_publisher_ = private_nh.advertise<visualization_msgs::MarkerArray>("marker_array", 1);
        this->marker_publisher_ = private_nh.advertise<visualization_msgs::Marker>("marker", 1);
    }

    bool VisualizationHelper::visualizeMarkerArrays()
    {
        if(this->marker_array_list_.size() > 0)
        {
            for (int marker_array_counter = 0;
                 marker_array_counter < this->marker_array_list_.size() - 1;
                 marker_array_counter++)
            {
                this->setTimeStamps(marker_array_counter);
                this->setMarkerIds(marker_array_counter);
                this->marker_array_publisher_.publish(this->marker_array_list_[marker_array_counter]);
            }
            return true;
        }
        else
        {
            ROS_ERROR("VisualizationHelper: No marker arrays to be visualized. Please add marker arrays and markers before visualizing.");
            return false;
        }
        
    }

    bool VisualizationHelper::visualizeMarkerArray(int marker_array_index)
    {
        if(this->isMarkerArrayExisting(marker_array_index))
        {
            this->setTimeStamps(marker_array_index);
            this->setMarkerIds(marker_array_index);
            this->marker_array_publisher_.publish(this->marker_array_list_[marker_array_index]);
            return true;
        }
        else
        {
            ROS_ERROR("VisualizationHelper: Index of marker_array was not found. Please check the given indexes that were handed to this method.");
            return false;
        }
    }

    int VisualizationHelper::addNewMarkerArray()
    {
        visualization_msgs::MarkerArray marker_array;
        this->marker_array_list_.push_back(marker_array);
        // this->max_marker_index_list_.push_back(0);
        return (this->marker_array_list_.size() - 1); // -1 so the index points to the right element (vector starts at 0)
    }

    bool VisualizationHelper::addMarkerToExistingMarkerArray(int marker_array_index, visualization_msgs::Marker marker_to_add)
    {
        // Check if marker array exists
        if (!this->isMarkerArrayExisting(marker_array_index))
        {
            return false;
        }

        // No further checks for the content of the marker
        this->marker_array_list_[marker_array_index].markers.push_back(marker_to_add);
    }

    bool VisualizationHelper::addMarkersToExistingMarkerArray(int marker_array_index, std::vector<visualization_msgs::Marker> marker_list_to_add)
    {
        // Check if marker array exists
        if (!this->isMarkerArrayExisting(marker_array_index))
        {
            return false;
        }

        for (visualization_msgs::Marker marker: marker_list_to_add)
        {
            // No further checks for the content of the marker
            this->marker_array_list_[marker_array_index].markers.push_back(marker);
        }

        return true;
    }

    bool VisualizationHelper::addMarkerToExistingMarkerArray(int marker_array_index, geometry_msgs::Pose pose, int marker_template_index)
    {
        // Check if marker array exists
        if (!this->isMarkerArrayExisting(marker_array_index) ||
            !this->isMarkerTemplateExisting(marker_template_index))
        {
            return false;
        }

        visualization_msgs::Marker marker_to_add = this->marker_template_list_[marker_template_index];
        marker_to_add.pose = pose;

        this->marker_array_list_[marker_array_index].markers.push_back(marker_to_add);

        return true;
    }
    
    bool VisualizationHelper::clearMarkerArray(int marker_array_index)
    {
        // Check if marker array exists
        if (!this->isMarkerArrayExisting(marker_array_index))
        {
            return false;
        }

        this->marker_array_list_[marker_array_index].markers.clear();
    }

    void VisualizationHelper::publishMarker(visualization_msgs::Marker marker_to_publish)
    {
        this->marker_publisher_.publish(marker_to_publish);
    }

    int VisualizationHelper::addMarkerTemplate(visualization_msgs::Marker marker_template)
    {
        this->marker_template_list_.push_back(marker_template);
        return (this->marker_template_list_.size() - 1);
    }

    //PRIVATE METHODS
    bool VisualizationHelper::isMarkerArrayExisting(int marker_array_index)
    {
        return this->marker_array_list_.size() > marker_array_index;
    }

    bool VisualizationHelper::isMarkerExisting(int marker_array_index, int marker_index)
    {
        if (this->isMarkerArrayExisting(marker_array_index))
        {
            return this->marker_array_list_[marker_array_index].markers.size() > marker_index;
        }
        else
        {
            return false;
        }
    }

    bool VisualizationHelper::isMarkerTemplateExisting(int marker_template_index)
    {
        return this->marker_template_list_.size() > marker_template_index;
    }

    void VisualizationHelper::setTimeStamps(int marker_array_index)
    {
        for(int marker_counter = 0; marker_counter < (this->marker_array_list_[marker_array_index].markers.size() - 1); marker_counter++)
        {
            this->marker_array_list_[marker_array_index].markers[marker_counter].header.stamp = ros::Time::now();
        }
    }

    void VisualizationHelper::setMarkerIds(int marker_array_index)
    {
        double max_id = this->max_marker_index_;
        for (int marker_counter = 0;
             marker_counter < (this->marker_array_list_[marker_array_index].markers.size() - 1);
             marker_counter++)
        {
            max_id = max_id + marker_counter;
            this->marker_array_list_[marker_array_index].markers[marker_counter].id = max_id;
        }
        this->max_marker_index_ = max_id;
    }
}