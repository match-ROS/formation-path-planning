// void initializeOccupancyMap()
// {
//     this->occupancy_map_ = std::shared_ptr<bool[]>(new bool[this->array_size_]);
//     for(int i=0; i<this->array_size_;i++)
//     {
//         this->occupancy_map_[i]=false;
//     }
    // FILE *fp = fopen("/home/hlurz/Downloads/First.pgm", "w+");
    // if(!fp)
    // {
    //     ROS_INFO("NO FUCKING CLKUE");
    // }
    // bool b = this->costmap_->saveMap("/home/hlurz/Downloads/First.pgm");
    // ROS_INFO("%i", b);
    // for(int cell_counter_x = 0; cell_counter_x < this->costmap_->getSizeInCellsX(); cell_counter_x++)
    // {
    //     for(int cell_counter_y = 0; cell_counter_y < this->costmap_->getSizeInCellsY(); cell_counter_y++)
    //     {
    //         unsigned int cell_cost = static_cast<unsigned int>(this->costmap_->getCost(cell_counter_x, cell_counter_y));
    //         int map_cell[2] = {cell_counter_x, cell_counter_y};
    //         // if(cell_counter_x >=400 && cell_counter_x <= 600 && cell_counter_y >= 400 && cell_counter_y<=600)
    //         // {
    //         //     for(tempStruct x: already_written)
    //         //     {
    //         //         if(this->getArrayIndexByCostmapCell(map_cell)==x.array_index)
    //         //         {
    //         //             ROS_ERROR("SAME CELL TWICE! array: %i, current x:%i, current y:%i, past x: %i, past y: %i",x.array_index, cell_counter_x, cell_counter_y, x.x, x.y);
    //         //         }        
    //         //     }
    //         //     tempStruct t;
    //         //     t.array_index = this->getArrayIndexByCostmapCell(map_cell);
    //         //     t.x = cell_counter_x;
    //         //     t.y = cell_counter_y;
    //         //     already_written.push_back(t);
    //         //     ROS_INFO("%i %i %i",cell_counter_x, cell_counter_y, already_written.size());
    //         // }

    //         if(cell_cost == 0) // Cell is free
    //         {
    //             this->occupancy_map_[this->getArrayIndexByCostmapCell(map_cell)] = false; // False because cell is not occupied
    //             // this->occupancy_map_[this->getArrayIndexByCostmapCell(map_cell)] = false; // False because cell is not occupied
    //             // ROS_INFO("FREE - x: %i, y: %i, casted cost: %i, occupancy_map: %i", map_cell[0], map_cell[1], cell_cost, this->occupancy_map_[this->getArrayIndexByCostmapCell(map_cell)]);
    //         }
    //         else
    //         {
    //             this->occupancy_map_[this->getArrayIndexByCostmapCell(map_cell)] = true;  // True because cell is occupied
    //             // ROS_INFO("BLOCKED - x: %i, y: %i, casted cost: %i, occupancy_map: %i", map_cell[0], map_cell[1], cell_cost, this->occupancy_map_[this->getArrayIndexByCostmapCell(map_cell)]);
    //         }
    //         // ROS_INFO("array: %i, x: %i, y: %i, casted cost: %i, occupancy_map: %i", this->getArrayIndexByCostmapCell(map_cell), map_cell[0], map_cell[1], cell_cost, this->occupancy_map_[this->getArrayIndexByCostmapCell(map_cell)]);
    //     }
    // }
    // ROS_INFO("DONE");
    // int counter=0;
    // for(int i=0; i < 1024*1024; i++)
    // {
    //     int map_cell[2];
    //     this->getCostmapPointByArrayIndex(i, map_cell);
    //     int cost = this->costmap_->getCost(map_cell[0], map_cell[1]);
    //     if(cost > 0 && this->occupancy_map_[i] == false || cost == 0 && this->occupancy_map_[i] == true)
    //     {
    //         counter=counter+1;
    //         //ROS_INFO("array: %i, x: %i, y: %i, cost: %i, blocked: %i", i, map_cell[0], map_cell[1], cost, this->occupancy_map_[i]);
    //     }
    //     // if(i == this->getArrayIndexByCostmapCell(740,437))
    //     // {
    //     //     ROS_INFO("x: %i, y: %i, cost: %i, blocked: %i", map_cell[0], map_cell[1], cost, this->occupancy_map_[i]);
    //     // }
    // }
    // ROS_INFO("DONEDONE %i", counter);
    // this->costmap_->saveMap("/home/hlurz/Downloads/Second.pgm");
    // publishOccupancyGrid();
// }

// ROS_ERROR("SERVICE CALLBACK");
// // Compare costmap then to now
// std::shared_ptr<bool[]> occumap2 = std::shared_ptr<bool[]>(new bool[this->array_size_]);
// for(int cell_counter_x = 0; cell_counter_x < this->costmap_->getSizeInCellsX(); cell_counter_x++)
// {
//     for(int cell_counter_y = 0; cell_counter_y < this->costmap_->getSizeInCellsY(); cell_counter_y++)
//     {
//         unsigned int cell_cost = static_cast<unsigned int>(this->costmap_->getCost(cell_counter_x, cell_counter_y));
//         int map_cell[2] = {cell_counter_x, cell_counter_y};
//         if(cell_cost == 0) // Cell is free
//         {
//             occumap2[this->getArrayIndexByCostmapCell(map_cell)] = false; // False because cell is not occupied
//             // ROS_INFO("FREE - x: %i, y: %i, casted cost: %i, occupancy_map: %i", map_cell[0], map_cell[1], cell_cost, this->occupancy_map_[this->getArrayIndexByCostmapCell(map_cell)]);
//         }
//         else
//         {
//             occumap2[this->getArrayIndexByCostmapCell(map_cell)] = true;  // True because cell is occupied
//             // ROS_INFO("BLOCKED - x: %i, y: %i, casted cost: %i, occupancy_map: %i", map_cell[0], map_cell[1], cell_cost, this->occupancy_map_[this->getArrayIndexByCostmapCell(map_cell)]);
//         }
//     }
// }

// ROS_INFO("VISUALIZATION MARKERS BEGIN");
// visualization_msgs::MarkerArray marker_array;
// for(int i=0; i<this->array_size_;i++)
// {
//     if(this->occupancy_map_[i] != occumap2[i])
//     {
//         visualization_msgs::Marker marker;
//         marker.type=visualization_msgs::Marker::SPHERE;
//         marker.action=visualization_msgs::Marker::ADD;
//         marker.color.a=1.0; //Otherwise will be invincible
//         marker.color.r= 1.0;
//         marker.header.frame_id="map";
//         marker.header.stamp=ros::Time::now();
//         geometry_msgs::Vector3 v;
//         v.x=0.1;
//         v.y=0.1;
//         v.z=0.1;
//         marker.scale=v;
//         marker.ns="my_namespace";
//         marker.id=i;
//         int point[2];
//         this->getCostmapPointByArrayIndex(i, point);
//         this->costmap_->mapToWorld(point[0], point[1], marker.pose.position.x, marker.pose.position.y);
//         marker.pose.position.z = 0;
//         tf::quaternionTFToMsg(tf::Quaternion(0,0,0), marker.pose.orientation);
//         marker_array.markers.push_back(marker);
//     }
// }
// this->marker_array_publisher_.publish(marker_array);
// ROS_INFO("VISUALIZATION MARKERS END");