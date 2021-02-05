/************************************************************ 
 Copyright (C), 2019-, skywell-mobility Tech. Co., Ltd. 
 Author:hl
 Version:v1.0
 Date: 2021/1/12
 Description:栅格点云分割，基于Grid类，进行点云地面分割。
*******************************************************/
#ifndef _GRID_SEGMENT_H_
#define _GRID_SEGMENT_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/make_shared.hpp>

#include "grid.h"

namespace skywell {

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> CloudT;


class GridIndex
{
	public:
		int index_;
		int radius_;
		int col_;
		int row_;
};



class GridSegment
{
	public:
		void setInputCloud(pcl::PointCloud<PointT>::Ptr cloud_ptr);
		void setGridRange(float minx,float miny,float maxx,float maxy);
		void setGridSize(float size);
		void setHeightThreshold(float height_threshold);
		void initGrid(void);
		void processGrid(void);
		void firstCheckGrid(int index,boost::shared_ptr<std::vector<Grid>> &ground_grid_list);
		void secondCheckGrid(int index,boost::shared_ptr<std::vector<Grid>> &ground_grid_list);
		void segment(boost::shared_ptr<std::vector<Grid>> &ground_grid_list,pcl::PointIndices &out_indices);
	private:
		int getIndex(int col,int row);
		void getRangeIndex(int col,int row ,std::vector<int>  &index_vec);
		int getOriginIndex(void);
		void searchNearestGroundGrid(int index,std::vector<int>  &index_vec);
		float calProbability( Feature &first,Feature &second);
		float calProbability(float first,float second);
		void CopyFeature(Feature &source,Feature &target);
	private:
		typename pcl::PointCloud<PointT>::Ptr cloud_ptr_ {new pcl::PointCloud<PointT>()};
		float minx_,miny_,maxx_,maxy_,size_;
		float height_threshold_ = 0.2;
		std::vector<GridIndex> grid_index_list_;

	public:
		boost::shared_ptr<std::vector<Grid>> grid_list_ = boost::make_shared<std::vector<Grid>>();
		int col_ = 0;
		int row_ = 0;
};




class SubGrid
{
	public:
		void checkGrid(int index,float height_threshold,
						Feature &ground,Grid &history_grid)
		{
			if ((*grid_list_)[index].check_flag_ == -1)
			{
				(*grid_list_)[index].check_flag_ = 1;
				if (
					((*grid_list_)[index].current_feature_.abs_height_ < height_threshold)
					&& ((*grid_list_)[index].current_feature_.avg_z_ < height_threshold)
					)
				{
					// 满足条件判断为地面点
					(*grid_list_)[index].check_flag_ = 0;
					(*grid_list_)[index].ground_flag_ = 0;
				}else
				{
					float _probability = calProbability((*grid_list_)[index].current_feature_.avg_z_,ground.avg_z_);
					if (_probability > 0.8)
					{
						(*grid_list_)[index].check_flag_ = 0;
						(*grid_list_)[index].ground_flag_ = 0;
					}else
					{
						if ((*grid_list_)[index].current_feature_.abs_height_ > height_threshold)
						{
							for (int i = 0 ; i < (*grid_list_)[index].current_feature_.point_size_ ; i++)
							{
								if (((*grid_list_)[index].grid_cloud_[i].z < ground.max_z_ + 0.05f)
									||((*grid_list_)[index].grid_cloud_[i].z < 0.05f))
								{
									(*grid_list_)[index].grid_ground_inliers_.indices.push_back((*grid_list_)[index].grid_inliers_.indices[i]);
								}
							}
							(*grid_list_)[index].check_flag_ = 0;
							(*grid_list_)[index].ground_flag_ = 1;
						}else
						{
							if ((history_grid.is_vaild_)
								&& ((*grid_list_)[index].current_feature_.avg_z_ < history_grid.ground_feature_.avg_z_ + height_threshold ))
							{
								(*grid_list_)[index].check_flag_ = 0;
								(*grid_list_)[index].ground_flag_ = 0;
							}else
							{
								if ((*grid_list_)[index].current_feature_.avg_z_ < ground.avg_z_ + height_threshold)
								{
									(*grid_list_)[index].check_flag_ = 0;
									(*grid_list_)[index].ground_flag_ = 0;
								}
							}
						}
					}
				}
			}
		};


	public:
		void segment(Grid &grid,float size,float height_threshold,
					Feature &ground,Grid &history_grid,
					pcl::PointIndices &out_indices)
		{
			if (grid.grid_cloud_.points.size() <= 0)
			{
				return ;
			}

			minx_ = grid.current_feature_.min_x_;
			maxx_ = grid.current_feature_.max_x_;

			miny_ = grid.current_feature_.min_y_;
			maxy_ = grid.current_feature_.max_y_;

			col_ = (maxx_ - minx_) / size + 1;
			row_ = (maxy_ - miny_) / size + 1;
			int _grid_size = col_ * row_;
			grid_list_->resize(_grid_size);
			for (int i = 0; i < _grid_size; ++i)
			{
				(*grid_list_)[i].is_vaild_ = false;
			}

			//栅格
			
			size_t _point_size = grid.grid_cloud_.points.size();
			for (size_t i = 0; i < _point_size ; ++ i)
			{
				int _col = (maxx_ - grid.grid_cloud_[i].x) / size;
				int _row = (maxy_ - grid.grid_cloud_[i].y) / size;
				int _index = _col * row_ + _row;
				if ((*grid_list_)[_index].is_vaild_ == false)
				{
					(*grid_list_)[_index].is_vaild_ = true;
				}
				(*grid_list_)[_index].addPoint(grid.grid_cloud_[i],grid.grid_inliers_.indices[i]);
			}


			//分割
			
			for (int i = 0; i < _grid_size; ++i)
			{
				if ((*grid_list_)[i].is_vaild_)
				{
					checkGrid(i,height_threshold,ground,history_grid);
				}
			}
			for (int i = 0; i < _grid_size ; ++i)
			{
				
				if ((*grid_list_)[i].ground_flag_ == 0)
				{
					out_indices.indices.insert(out_indices.indices.end(),
						(*grid_list_)[i].grid_inliers_.indices.begin(),
						(*grid_list_)[i].grid_inliers_.indices.end());
				}
				if ((*grid_list_)[i].ground_flag_ == 1)
				{
					//printf("i = %d\n",i);
						out_indices.indices.insert(out_indices.indices.end(),
						(*grid_list_)[i].grid_ground_inliers_.indices.begin(),
						(*grid_list_)[i].grid_ground_inliers_.indices.end());
				}
			}
		};

		float calProbability(float first,float second)
		{
			float _probability = 1.0
			 / (1.0 + fabs(first - second)
			 / (fabs(first) + fabs(second)));
			return _probability;
		};


	public:
		float minx_,miny_,maxx_,maxy_;
		std::vector<GridIndex> grid_index_list_;
	public:
		boost::shared_ptr<std::vector<Grid>> grid_list_ = boost::make_shared<std::vector<Grid>>();
		int col_ = 0;
		int row_ = 0;	
};


}
#endif

