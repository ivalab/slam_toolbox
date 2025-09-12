/**
 * @file occ_map.cpp
 * @author Yanwei Du (yanwei.du@gatech.edu)
 * @brief None
 * @version 0.1
 * @date 09-11-2025
 * @copyright Copyright (c) 2025
 */




 bool SlamToolbox::updateMap()
/*****************************************************************************/
{
  if (sst_.getNumSubscribers() == 0)
  {
    return true;
  }
  boost::mutex::scoped_lock lock(smapper_mutex_);
  karto::OccupancyGrid* occ_grid = smapper_->getOccupancyGrid(resolution_);
  if(!occ_grid)
  {
    return false;
  }

  vis_utils::toNavMap(occ_grid, map_.map);

  // publish map as current
  map_.map.header.stamp = ros::Time::now();
  sst_.publish(map_.map);
  sstm_.publish(map_.map.info);
  
  delete occ_grid;
  occ_grid = nullptr;
  return true;
}