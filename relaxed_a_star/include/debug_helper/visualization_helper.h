#pragma once

#include "ros/ros.h"

#include <visualization_msgs/MarkerArray.h>

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
             * @param marker_array_index Index of the marker array where the marker should be added
             * @param marker_to_add List of markers that should be added to the marker array
             * @return true Marker array existed and marker was successfluyy added
             * @return false Marker array does not exist
             */
            bool addMarkerToExistingMarkerArray(int marker_array_index, std::vector<visualization_msgs::Marker> marker_to_add);

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

            ros::Publisher marker_array_publisher_;
            ros::Publisher marker_publisher_;

            /**
             * @brief This list of marker arrays gives the user the chance to display multiple data sources in different colors.
             * The first marker array in the list will be published first and might be overwritten by other markers in higher lists.
             * 
             */
            std::vector<visualization_msgs::MarkerArray> marker_array_list_;
    };
}