/************************************************************ 
 Copyright (C), 2019-, skywell-mobility Tech. Co., Ltd. 
 Author:hl
 Version:v1.0
 Date: 2021/1/12
 Description:点云栅格类，该类记录一个栅格点云的数据，
             并提取栅格点云的特征。
*******************************************************/
#ifndef _GRID_H_
#define _GRID_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
namespace skywell {

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> CloudT;


class Feature
{
	public:
		float min_x_ = std::numeric_limits<float>::max();
		float min_y_ = std::numeric_limits<float>::max();
		float min_z_ = std::numeric_limits<float>::max();

		float max_x_ = - std::numeric_limits<float>::max();
		float max_y_ = - std::numeric_limits<float>::max();
		float max_z_ = - std::numeric_limits<float>::max();

		float deviation_z_ = 0.0f;
		float abs_height_ = 0.0f;
		float avg_z_ = 0.0f;
		int point_size_ = 0;
};

class Grid
{
	public:
		typename pcl::PointCloud<PointT> grid_cloud_;    // 栅格内的原始点云
		pcl::PointIndices grid_inliers_;    // 栅格内点云索引
		pcl::PointIndices grid_ground_inliers_;    // 栅格内地面点云索引
	public: 
		Feature current_feature_;
		Feature ground_feature_;
	public:// 以下属性标记栅格是否有效及栅格索引
		bool is_vaild_ = false;
		int index_ = 0; //索引号
		int col_ = 0; // 行索引号
		int row_ = 0;  // 列索引号
	public:
		void addPoint(PointT point,int index)
		{

			if (point.x < current_feature_.min_x_)
			{
				current_feature_.min_x_ = point.x;
			}
			if (point.y < current_feature_.min_y_)
			{
				current_feature_.min_y_ = point.y;
			}
			if (point.z < current_feature_.min_z_)
			{
				current_feature_.min_z_ = point.z;
			}

			if (point.x > current_feature_.max_x_)
			{
				current_feature_.max_x_ = point.x;
			}
			if (point.y > current_feature_.max_y_)
			{
				current_feature_.max_y_ = point.y;
			}
			if (point.z > current_feature_.max_z_)
			{
				current_feature_.max_z_ = point.z;
			}

			if (current_feature_.max_z_ < 0)
			{
				current_feature_.deviation_z_ = current_feature_.max_z_;
			}
			if (current_feature_.min_z_ > 0)
			{
				current_feature_.deviation_z_ = current_feature_.min_z_;
			}

			current_feature_.abs_height_ = current_feature_.max_z_ - current_feature_.min_z_;
			current_feature_.avg_z_ = (current_feature_.avg_z_ * current_feature_.point_size_ + point.z) / (current_feature_.point_size_ + 1);
			
			current_feature_.point_size_ = current_feature_.point_size_ + 1;

			grid_cloud_.push_back(point);
			grid_inliers_.indices.push_back(index);
		}

		void addGroundPoint(PointT point)
		{
			if (point.z < ground_feature_.min_z_)
			{
				ground_feature_.min_z_ = point.z;
			}

			if (point.z > ground_feature_.max_z_)
			{
				ground_feature_.max_z_ = point.z;
			}

			if (ground_feature_.max_z_ < 0)
			{
				ground_feature_.deviation_z_ = ground_feature_.max_z_;
			}
			if (ground_feature_.min_z_ > 0)
			{
				ground_feature_.deviation_z_ = ground_feature_.min_z_;
			}

			ground_feature_.abs_height_ = ground_feature_.max_z_ - ground_feature_.min_z_;
			ground_feature_.avg_z_ = (ground_feature_.avg_z_ * ground_feature_.point_size_ + point.z) / (ground_feature_.point_size_ + 1);
			
			ground_feature_.point_size_ = ground_feature_.point_size_ + 1;
		}
	public:
		void UpdateGroundFeature(Feature &feature)
		{
			is_vaild_ = true;
			ground_flag_ = 0;
			check_flag_ = 0;
			if ((feature.point_size_ > (ground_feature_.point_size_ * 0.6))
				&& (feature.avg_z_ > ground_feature_.avg_z_))
			{
				ground_feature_.min_z_ = feature.min_z_;
				ground_feature_.max_z_ = feature.max_z_;
				ground_feature_.deviation_z_ = feature.deviation_z_;
				ground_feature_.abs_height_ = feature.abs_height_;
				ground_feature_.avg_z_ = feature.avg_z_;
				ground_feature_.point_size_ = feature.point_size_;
			}
		};
	public:// 以下属性作为栅格处理时状态标记
		int ground_flag_ = -1;// -1,未判断出结果，0，全部为地面点，1，绝对包含障碍物点
		int check_flag_ = -1;// -1：未校验过，0：检验完毕，1： 校验中
};


}
#endif
