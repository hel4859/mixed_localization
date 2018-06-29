
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/console.h>
#include <vector>

#include <stdio.h>
#include "mrpt_bridge/point_cloud2.h"

#include <mrpt/version.h>
#if MRPT_VERSION>=0x130
#	include <mrpt/maps/CSimplePointsMap.h>
#	include <mrpt/maps/CColouredPointsMap.h>
	using namespace mrpt::maps;
#else
#	include <mrpt/slam/CSimplePointsMap.h>
#	include <mrpt/slam/CColouredPointsMap.h>
	using namespace mrpt::slam;
#endif

#define z_Max -0.5 //4.5
#define z_Min -2.5  //-2
#define z_Gap 2 //1.5//7

#define x_Min -3
#define x_Max 30
#define y_Min -30
#define y_Max 30

#define distance_Max 30
#define distance_Min 3

#define lidar_max_range 50

using namespace std;

#include"pointcloud_receive.h"
namespace mrpt_bridge
{

  inline bool check_field(const sensor_msgs::PointField& input_field, std::string check_name,
                          const sensor_msgs::PointField** output)
  {
    bool coherence_error = false;
    if (input_field.name == check_name)
    {
      if (input_field.datatype != sensor_msgs::PointField::FLOAT32
          && input_field.datatype != sensor_msgs::PointField::FLOAT64
          && input_field.datatype != sensor_msgs::PointField::UINT16 )
      {
        *output = NULL;
        coherence_error = true;
      }
      else
      {
        *output = &input_field;
      }
    }
    return coherence_error;
  }

  inline void get_float_from_field(const sensor_msgs::PointField* field, const unsigned char* data, float& output)
  {
    if (field != NULL)
    {
      if (field->datatype == sensor_msgs::PointField::FLOAT32)
        output = *(reinterpret_cast<const float*>(&data[field->offset]));
      else
        output = (float)(*(reinterpret_cast<const double*>(&data[field->offset])));

    }
    else
      output = 0.0;
  }

  //get int type data from the point cloud field
  inline void get_int_from_field(const sensor_msgs::PointField* field, const unsigned char* data, int& output)
  {
    if (field != NULL)
    {
      if (field->datatype == sensor_msgs::PointField::UINT16)
        output = *(reinterpret_cast<const int*>(&data[field->offset]));
      else
        output = (int)(*(reinterpret_cast<const int*>(&data[field->offset])));
    }
    else
      output = 0;
  }

  /** Convert sensor_msgs/PointCloud2 -> mrpt::slam::CSimplePointsMap
   *
   * \return true on sucessful conversion, false on any error.
   */
  bool copy(const sensor_msgs::PointCloud2 &msg, CSimplePointsMap &obj)
  {
    // Copy point data
    unsigned int num_points = msg.width * msg.height;
    obj.clear();
    obj.reserve(num_points);

    bool incompatible_clouds = false;
    const sensor_msgs::PointField *x_field = NULL, *y_field = NULL, *z_field = NULL, *ring_field = NULL, *intensity_field = NULL;

    for (unsigned int i = 0; i < msg.fields.size() && !incompatible_clouds; i++)
    {
      incompatible_clouds |= check_field(msg.fields[i], "x", &x_field);
      incompatible_clouds |= check_field(msg.fields[i], "y", &y_field);
      incompatible_clouds |= check_field(msg.fields[i], "z", &z_field);
    }

    if (incompatible_clouds || (x_field == NULL && y_field == NULL && z_field == NULL))
      return false;

    // If not, memcpy each group of contiguous fields separately
    for (unsigned int row = 0; row < msg.height; ++row)
    {
      const unsigned char* row_data = &msg.data[row * msg.row_step];
      for (uint32_t col = 0; col < msg.width; ++col)
      {
        const unsigned char* msg_data = row_data + col * msg.point_step;

        float x, y, z;
        get_float_from_field(x_field, msg_data, x);
        get_float_from_field(y_field, msg_data, y);
        get_float_from_field(z_field, msg_data, z);
        //if(z >pointsMap_heightMin && z<pointsMap_heightMax) //改过
        if(x>x_Min && z<z_Max && y<y_Max && y>y_Min)
        {
            obj.insertPoint(x, y, z);
        }
        else continue;
      }
    }
    return true;
  }

  /** copy colored point cloud?
    */


  bool copy2colouredPointsMapSingle(const sensor_msgs::PointCloud2& msg, CColouredPointsMap &obj)
  {
    // Copy point data
    unsigned int num_points = msg.width * msg.height;//  msg.width * msg.height;
    //printf("width:%i\theight:%i\n",msg.width,msg.height);
    obj.clear();
    obj.reserve(num_points);

    bool incompatible_clouds = false;
    const sensor_msgs::PointField *x_field = NULL, *y_field = NULL, *z_field = NULL, *ring_field = NULL, *intensity_field = NULL;

    for (unsigned int i = 0; i < msg.fields.size() && !incompatible_clouds; i++)
    {
      incompatible_clouds |= check_field(msg.fields[i], "x", &x_field);
      incompatible_clouds |= check_field(msg.fields[i], "y", &y_field);
      incompatible_clouds |= check_field(msg.fields[i], "z", &z_field);
      incompatible_clouds |= check_field(msg.fields[i], "ring", &ring_field);
      incompatible_clouds |= check_field(msg.fields[i], "intensity", &intensity_field);
    }
      //printf("%s, %s",ring_field, intensity_field);
    if (incompatible_clouds || (x_field == NULL && y_field == NULL && z_field == NULL && intensity_field==NULL && ring_field==NULL))
    {
        //printf("losing data fields %s, %s",ring_field, intensity_field);
        ROS_INFO("losing data fields <point_cloud2.cpp:copy2colouredPointsMap>");
        return false;
    }

    // If not, memcpy each group of contiguous fields separately
    for (unsigned int row = 0; row < msg.height; ++row)
    {
      const unsigned char* row_data = &msg.data[row * msg.row_step    ];
      for (uint32_t col = 0; col < msg.width; ++col)
      {
        const unsigned char* msg_data = row_data + col * msg.point_step;

        float x, y, z, r, g, b, intensity;
        g = 0.2;
        int ring;
        get_float_from_field(x_field, msg_data, x);
        get_float_from_field(y_field, msg_data, y);
        get_float_from_field(z_field, msg_data, z);
        get_float_from_field(intensity_field, msg_data, intensity);
        get_int_from_field(ring_field, msg_data, ring);

        {
          r = 1 - 0.8*(float)ring/16;
          b = 0.8*(float)ring/16;
        }

        //if(x>x_Min && x<x_Max && z>z_Min &&z<z_Max && y<y_Max && y>y_Min){
        float laser_point_range = sqrt(x*x+y*y+z*z);
        if(laser_point_range > distance_Min && laser_point_range < distance_Max){
            obj.insertPoint(x,y,z,r,g,b);
            //printf("%i, %f\n",ring, intensity);
            //obj.insertPoint(x,y,z,intensity/100,intensity/100,intensity/100);
        }
        //else continue;
      }
    }
    return true;
  }

  bool copy(const CSimplePointsMap &obj, const std_msgs::Header &msg_header, sensor_msgs::PointCloud2 &msg)
  {
    //MRPT_TODO("Implement pointcloud2 mrpt2ros");
    throw ros::Exception("not implemented yet.");
    return true;
  }

} //namespace image_proc
