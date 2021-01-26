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
            VisualizationHelper(std::string topic_prefix);

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
             * @param marker_array_identifier identifier of the marker array that should be published
             * @return true Marker array exists and was published
             * @return false Marker array with the specified index was not existing
             */
            bool visualizeMarkerArray(std::string marker_array_identifier);

            /**
             * @brief Add new marker array to the internal list
             * 
             * @return string Index of the marker array that was added so the external code can add marker directly to the marker array
             */
            std::string addNewMarkerArray();

            bool addNewMarkerArray(std::string marker_array_identifier);

            std::string addMarkerTemplate(visualization_msgs::Marker marker_template);
            bool addMarkerTemplate(std::string marker_template_identifier, visualization_msgs::Marker marker_template);

            /**
             * @brief Checks if the marker array with the specified index is existing
             * 
             * @param marker_array_identifier identifier of the marker array
             * @return true Marker array is existing
             * @return false Marker array is existing
             */
            bool isMarkerArrayExisting(std::string marker_array_identifier);

            bool isMarkerTemplateExisting(std::string marker_template_identifier);

            /**
             * @brief Add an additional marker from to the marker array that index was also handed through a parameter
             * 
             * @param marker_array_identifier identifier of the marker array where the marker should be added
             * @param marker_to_add Marker that should be added to the marker array
             * @return true Marker array existed and marker was successfluyy added
             * @return false Marker array does not exist
             */
            bool addMarkerToExistingMarkerArray(std::string marker_array_identifier,
                                                visualization_msgs::Marker marker_to_add);

            /**
             * @brief Add additional markers from a list to the marker array that index was also handed through a parameter
             * 
             * @param marker_array_identifier identifier of the marker array where the marker should be added
             * @param marker_list_to_add List of markers that should be added to the marker array
             * @return true Marker array existed and marker was successfluyy added
             * @return false Marker array does not exist
             */
            bool addMarkersToExistingMarkerArray(std::string marker_array_identifier,
                                                 std::vector<visualization_msgs::Marker> marker_list_to_add);

            bool addMarkerToExistingMarkerArray(std::string marker_array_identifier,
                                                geometry_msgs::Pose pose,
                                                std::string marker_template_identifier);

            bool addMarkerLineToMarkerArray(std::string marker_array_identifier,
                                            std::vector<geometry_msgs::Point> points,
                                            std::string marker_template_identifier);

            /**
             * @brief Clear the marker array with the specified index to not publish some points again
             * 
             * @param marker_array_identifier identifier of the marker array
             * @return true Marker array exists and was cleared
             * @return false Marker array with the specified index was not existing
             */
            bool clearMarkerArray(std::string marker_array_identifier);

            /**
             * @brief Publishes the marker from the parameter on the ~/marker topic
             * 
             * @param marker_to_publish Marker that should be published directly
             */
            void publishMarker(visualization_msgs::Marker marker_to_publish);

            visualization_msgs::Marker createMarker();

            geometry_msgs::Pose createGeometryPose(float x_coord, float y_coord);
            geometry_msgs::Point createGeometryPoint(float x_coord, float y_coord);

        private:
            /**
             * @brief Sets the time stamps of all markers in the specified marker array
             * 
             * @param marker_array_identifier identifier of the marker array
             */
            void setTimeStamps(std::string marker_array_identifier);

            void setMarkerIds(std::string marker_array_identifier);

            ros::Publisher marker_array_publisher_;
            ros::Publisher marker_publisher_;

            /**
             * @brief This list of marker arrays gives the user the chance to display multiple data sources in different colors.
             * The first marker array in the list will be published first and might be overwritten by other markers in higher lists.
             * 
             */
            std::map<std::string, visualization_msgs::MarkerArray> marker_array_list_;

            /**
             * @brief This list contains a marker that is used as a template for other markers
             * This makes adding new markers really easy as only the position must be set and a template selected (that was initialized at the beginning)
             * 
             */
            std::map<std::string, visualization_msgs::Marker> marker_template_list_;

            /**
             * @brief This vector collects the maximum marker ids of the markers in the marker arrays
             * 
             */
            // std::vector<double> max_marker_index_list_;
            double max_marker_index_;
    };
}