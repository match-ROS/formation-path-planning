#pragma once

#include "ros/ros.h"

#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>

namespace visualization_helper
{
    class VisualizationHelper
    {
        public:
            VisualizationHelper();
            VisualizationHelper(std::string name);

            /**
             * @brief This method publishes all the data that was stored until now since the last cycle
             * 
             * @return true All marker arrays were visualized successfully
             * @return false No marker arrays that can be visualized
             */
            bool visualizeMarkerArrays();

            /**
             * @brief Publish the marker array with the specified index
             * 
             * @param marker_array_index Index of the marker array that should be published
             * @return true Marker array exists and was published
             * @return false Marker array with the specified index was not existing
             */
            bool visualizeMarkerArray(int marker_array_index);

            /**
             * @brief Add new marker array to the internal list
             * 
             * @return int Index of the marker array that was added so the external code can add marker directly to the marker array
             */
            int addNewMarkerArray();

            /**
             * @brief Add an additional marker from to the marker array that index was also handed through a parameter
             * 
             * @param marker_array_index Index of the marker array where the marker should be added
             * @param marker_to_add Marker that should be added to the marker array
             * @return true Marker array existed and marker was successfluyy added
             * @return false Marker array does not exist
             */
            bool addMarkerToExistingMarkerArray(int marker_array_index, visualization_msgs::Marker marker_to_add);

            /**
             * @brief Add additional markers from a list to the marker array that index was also handed through a parameter
             * 
             * @param marker_list_to_add Index of the marker array where the marker should be added
             * @param marker_to_add List of markers that should be added to the marker array
             * @return true Marker array existed and marker was successfluyy added
             * @return false Marker array does not exist
             */
            bool addMarkersToExistingMarkerArray(int marker_array_index, std::vector<visualization_msgs::Marker> marker_list_to_add);

            bool addMarkerToExistingMarkerArray(int marker_array_index, geometry_msgs::Pose pose, int marker_template_index);

            /**
             * @brief Clear the marker array with the specified index to not publish some points again
             * 
             * @param marker_array_index Index of the marker array
             * @return true Marker array exists and was cleared
             * @return false Marker array with the specified index was not existing
             */
            bool clearMarkerArray(int marker_array_index);

            /**
             * @brief Publishes the marker from the parameter on the ~/marker topic
             * 
             * @param marker_to_publish Marker that should be published directly
             */
            void publishMarker(visualization_msgs::Marker marker_to_publish);

            visualization_msgs::Marker createMarker();

            int addMarkerTemplate(visualization_msgs::Marker marker_template);

            geometry_msgs::Pose createGeometryPose(float x_coord, float y_coord);

        private:
            /**
             * @brief Checks if the marker array with the specified index is existing
             * 
             * @param marker_array_index Index of the marker array
             * @return true Marker array is existing
             * @return false Marker array is existing
             */
            bool isMarkerArrayExisting(int marker_array_index);

            /**
             * @brief Checks if the marker in the specified marker array is existing
             * 
             * @param marker_array_index Index of the marker array
             * @param marker_index Index of the marker in the marker array
             * @return true Marker is existing
             * @return false Marker is not existing
             */
            bool isMarkerExisting(int marker_array_index, int marker_index);

            bool isMarkerTemplateExisting(int marker_template_index);

            /**
             * @brief Sets the time stamps of all markers in the specified marker array
             * 
             * @param marker_array_index 
             */
            void setTimeStamps(int marker_array_index);

            void setMarkerIds(int marker_array_index);

            ros::Publisher marker_array_publisher_;
            ros::Publisher marker_publisher_;

            /**
             * @brief This list of marker arrays gives the user the chance to display multiple data sources in different colors.
             * The first marker array in the list will be published first and might be overwritten by other markers in higher lists.
             * 
             */
            std::vector<visualization_msgs::MarkerArray> marker_array_list_;

            /**
             * @brief This list contains a marker that is used as a template for other markers
             * This makes adding new markers really easy as only the position must be set and a template selected (that was initialized at the beginning)
             * 
             */
            std::vector<visualization_msgs::Marker> marker_template_list_;

            /**
             * @brief This vector collects the maximum marker ids of the markers in the marker arrays
             * 
             */
            // std::vector<double> max_marker_index_list_;
            double max_marker_index_;
    };
}