

#ifndef _LIDAR_H_
#define _LIDAR_H_

/**
 * @brief initialize LiDAR
 * 
 * @return 0 for success, -1 for failure
 */
int lidar_Init(void);

/**
 * @brief get distance measurement from LiDAR
 * 
 * @return distance in millimeter; return 8191 when out of reange
 */
int lidar_GetData(int index);

#endif