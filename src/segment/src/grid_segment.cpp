/************************************************************ 
 Copyright (C), 2019-, skywell-mobility Tech. Co., Ltd. 
 Author:hl
 Version:v1.0
 Date: 2021/1/12
 Description:栅格点云分割，基于Grid类，进行点云地面分割。
*******************************************************/
#include "grid_segment.h"
namespace skywell {

void GridSegment::setInputCloud(pcl::PointCloud<PointT>::Ptr cloud_ptr)
{
	cloud_ptr_ = cloud_ptr;
};

void GridSegment::setGridRange(float minx,float miny,float maxx,float maxy)
{
	minx_ = minx;
	miny_ = miny;
	maxx_ = maxx;
	maxy_ = maxy;
}

void GridSegment::setGridSize(float size)
{
	size_ = size;
};

void GridSegment::setHeightThreshold(float height_threshold)
{
	height_threshold_ = height_threshold;
};

void GridSegment::initGrid(void)
{
	col_ = (maxx_ - minx_) / size_;
	row_ = (maxy_ - miny_) / size_;
	int _grid_size = col_ * row_;
	grid_list_->resize(_grid_size);
	for (int i = 0; i < _grid_size; ++i)
	{
		(*grid_list_)[i].is_vaild_ = false;
	}
	
	// 根据到原点距离进行排序，作为栅格判断的顺序，即从原点向四周扩散

	int _origin_col = (maxx_ - 0.0) / size_;
	int _origin_row = (maxy_ - 0.0) / size_;

	for (int i = 0; i < col_ ; i++)
	{
		int _tf_origin_col = i - _origin_col;
		for (int j = 0;j < row_ ;j++ )
		{
			int _tf_origin_row = j - _origin_row;

			int _origin_radius = sqrt(_tf_origin_col * _tf_origin_col + _tf_origin_row * _tf_origin_row);
			int _index = i * row_ + j;
			
			GridIndex _grid_index;
			_grid_index.radius_ = _origin_radius;
			_grid_index.index_ = _index;
			_grid_index.col_ = i;
			_grid_index.row_ = j;

			(*grid_list_)[_index].col_ = i;
			(*grid_list_)[_index].row_ = j;
			(*grid_list_)[_index].index_ = grid_index_list_.size();

			grid_index_list_.push_back(_grid_index);
		}
	}

	std::sort(grid_index_list_.begin(), grid_index_list_.end(),
	[](const GridIndex &a, const GridIndex &b) { return a.radius_ < b.radius_; });
};


void GridSegment::processGrid(void)
{
	size_t _point_size = cloud_ptr_->points.size();
	for (size_t i = 0; i < _point_size ; ++i)
	{
		if ((cloud_ptr_->points[i].x > minx_) 
			&& (cloud_ptr_->points[i].x < maxx_)
			&& (cloud_ptr_->points[i].y > miny_) 
			&& (cloud_ptr_->points[i].y < maxy_))
		{
			int _col = (maxx_ - cloud_ptr_->points[i].x) / size_;
			int _row = (maxy_ - cloud_ptr_->points[i].y) / size_;
			int _index = _col * row_ + _row;

			if ((*grid_list_)[_index].is_vaild_ == false)
			{
				(*grid_list_)[_index].is_vaild_ = true;
			}
			(*grid_list_)[_index].addPoint(cloud_ptr_->points[i],i);
		}
	}

};


void GridSegment::firstCheckGrid(int index,boost::shared_ptr<std::vector<Grid>> &ground_grid_list)
{
	if ((*grid_list_)[index].check_flag_ == -1)
	{
		(*grid_list_)[index].check_flag_ = 1;

		if (((*grid_list_)[index].current_feature_.abs_height_ < height_threshold_)
			&&((*grid_list_)[index].current_feature_.avg_z_ < height_threshold_))
		{
			// 满足条件判断为地面点
			(*grid_list_)[index].check_flag_ = 0;
			(*grid_list_)[index].ground_flag_ = 0;

			CopyFeature((*grid_list_)[index].current_feature_,(*grid_list_)[index].ground_feature_);
			(*ground_grid_list)[index].UpdateGroundFeature((*grid_list_)[index].ground_feature_);
		}
		else
		{
			std::vector<int> _range_indexs;
			getRangeIndex((*grid_list_)[index].col_,(*grid_list_)[index].row_,_range_indexs);

			int _ground_points_size = 0;

			float _min_z = std::numeric_limits<float>::max();
			float _max_z = -std::numeric_limits<float>::max();
		
			float _optimal_probability = 0.0;
			int _optimal_index_ = 0;

			for (auto num : _range_indexs)
			{
				if ((*grid_list_)[num].is_vaild_ == false)
				{
					continue;
				}else
				{
					if ((*grid_list_)[num].check_flag_ == -1)
					{
						firstCheckGrid(num,ground_grid_list);
					}
					
					if (((*grid_list_)[num].ground_flag_ == 0)&&((*grid_list_)[num].current_feature_.point_size_ > 2))
					{
						float _probability = calProbability((*grid_list_)[index].current_feature_,(*grid_list_)[num].ground_feature_);
						if ((*grid_list_)[num].current_feature_.point_size_ > _ground_points_size)
						{
							_optimal_index_ = num;
							_optimal_probability = _probability;
							_ground_points_size = (*grid_list_)[num].current_feature_.point_size_;
						}
					}
				}
			}
			if (_optimal_index_ > 0)
			{
				if (_optimal_probability > 0.80)
				{
					(*grid_list_)[index].check_flag_ = 0;
					(*grid_list_)[index].ground_flag_ = 0;
					CopyFeature((*grid_list_)[index].current_feature_,(*grid_list_)[index].ground_feature_);
					(*ground_grid_list)[index].UpdateGroundFeature((*grid_list_)[index].ground_feature_);
				}
			}else
			{
				(*grid_list_)[index].check_flag_ = 0;
				(*grid_list_)[index].ground_flag_ = -1;
			}
		}
	}
};

void GridSegment::secondCheckGrid(int index,boost::shared_ptr<std::vector<Grid>> &ground_grid_list)
{
	if ((*grid_list_)[index].ground_flag_ == -1)
	{
		std::vector<int> _range_indexs;
		getRangeIndex((*grid_list_)[index].col_,(*grid_list_)[index].row_,_range_indexs);
		int _ground_points_size = 0;

		float _min_z = std::numeric_limits<float>::max();
		float _max_z = -std::numeric_limits<float>::max();
	
		float _optimal_probability = 0.0;
		int _optimal_index_ = 0;

		for (auto num : _range_indexs)
		{
			if ((*grid_list_)[num].is_vaild_ == false)
			{
				continue;
			}else
			{
				if (((*grid_list_)[num].ground_flag_ == 0)&&((*grid_list_)[num].current_feature_.point_size_ > 2))
				{
					if ((*grid_list_)[num].current_feature_.point_size_ > _ground_points_size)
					{
						_optimal_index_ = num;
						_ground_points_size = (*grid_list_)[num].current_feature_.point_size_;
					}
				}
			}
		}
		if (_optimal_index_ > 0)
		{

			SubGrid _sub_grid;
			_sub_grid.segment((*grid_list_)[index],0.2,height_threshold_,
				(*grid_list_)[_optimal_index_].current_feature_,
				(*ground_grid_list)[index],
				(*grid_list_)[index].grid_ground_inliers_);
				(*grid_list_)[index].check_flag_ = 0;
				(*grid_list_)[index].ground_flag_ = 1;
		}
		else
		{	
			/*
			if ((*grid_list_)[index].current_feature_.abs_height_ < 0.2)
			{
				printf("-----------------------------\n");
				printf("current.abs_height_ = %f\n",(*grid_list_)[index].current_feature_.abs_height_);
				printf("current.min_z_ = %f\n",(*grid_list_)[index].current_feature_.min_z_);
				printf("current.max_z_ = %f\n",(*grid_list_)[index].current_feature_.max_z_);
				printf("current.avg_z_ = %f\n",(*grid_list_)[index].current_feature_.avg_z_);
				printf("current.point_size_ = %d\n",(*grid_list_)[index].current_feature_.point_size_);
			}else
			{
				printf("++++++++++++++++++++++++++++++\n");
				printf("current.abs_height_ = %f\n",(*grid_list_)[index].current_feature_.abs_height_);
				printf("current.min_z_ = %f\n",(*grid_list_)[index].current_feature_.min_z_);
				printf("current.max_z_ = %f\n",(*grid_list_)[index].current_feature_.max_z_);
				printf("current.avg_z_ = %f\n",(*grid_list_)[index].current_feature_.avg_z_);
				printf("current.point_size_ = %d\n",(*grid_list_)[index].current_feature_.point_size_);
			}*/
			/*
			for (int i = 0 ; i < (*grid_list_)[index].current_feature_.point_size_ ; i++)
			{
				//if ((*grid_list_)[index].grid_cloud_[i].z < (*grid_list_)[_optimal_index_].ground_feature_.max_z_)
				{
					(*grid_list_)[index].grid_ground_inliers_.indices.push_back((*grid_list_)[index].grid_inliers_.indices[i]);
				}
			}
			(*grid_list_)[index].check_flag_ = 0;
			(*grid_list_)[index].ground_flag_ = 1;
			*/



			//printf("-----------------\n");
			// 没有在栅格周围的八个栅格中找到参考，继续向四周扩散，找到最近的一个作为参考
			// 查找方向为向原点靠近
			//
			int _ground_points_size = 0;
			int _optimal_index_ = 0;
			std::vector<int> _range_indexs;
			searchNearestGroundGrid(index,_range_indexs);
			for (auto num : _range_indexs)
			{
				if ((*grid_list_)[num].current_feature_.point_size_ > 2)
				{
					if ((*grid_list_)[num].current_feature_.point_size_ > _ground_points_size)
					{
						_optimal_index_ = num;
						_ground_points_size = (*grid_list_)[num].current_feature_.point_size_;
					}
				}
			}
			if (_optimal_index_ > 0)
			{
				for (int i = 0 ; i < (*grid_list_)[index].current_feature_.point_size_ ; i++)
				{
					if (((*grid_list_)[index].grid_cloud_[i].z < (*grid_list_)[_optimal_index_].current_feature_.avg_z_ + 0.2f )
						|| ((*grid_list_)[index].grid_cloud_[i].z < 0.05f))
					{
						(*grid_list_)[index].grid_ground_inliers_.indices.push_back((*grid_list_)[index].grid_inliers_.indices[i]);
					}
				}
				(*grid_list_)[index].check_flag_ = 0;
				(*grid_list_)[index].ground_flag_ = 1;
			}
		}
	}
};
/*
void GridSegment::secondCheckGrid(int index,boost::shared_ptr<std::vector<Grid>> &ground_grid_list)
{
	if ((*grid_list_)[index].check_flag_ == -1)
	{
		(*grid_list_)[index].check_flag_ = 1;

		if (((*grid_list_)[index].current_feature_.abs_height_ < height_threshold_)
			&&((*grid_list_)[index].current_feature_.avg_z_ < height_threshold_))
		{
			// 满足条件判断为地面点
			(*grid_list_)[index].check_flag_ = 0;
			(*grid_list_)[index].ground_flag_ = 0;

			CopyFeature((*grid_list_)[index].current_feature_,(*grid_list_)[index].ground_feature_);
			(*ground_grid_list)[index].UpdateGroundFeature((*grid_list_)[index].ground_feature_);
		}
		else
		{
			std::vector<int> _range_indexs;
			getRangeIndex((*grid_list_)[index].col_,(*grid_list_)[index].row_,_range_indexs);

			int _ground_points_size = 0;

			float _min_z = std::numeric_limits<float>::max();
			float _max_z = -std::numeric_limits<float>::max();
		
			float _optimal_probability = 0.0;
			int _optimal_index_ = 0;

			for (auto num : _range_indexs)
			{
				if ((*grid_list_)[num].is_vaild_ == false)
				{
					continue;
				}else
				{
					if ((*grid_list_)[num].check_flag_ == -1)
					{
						secondCheckGrid(num,ground_grid_list);
					}
					
					if (((*grid_list_)[num].ground_flag_ == 0)&&((*grid_list_)[num].current_feature_.point_size_ > 2))
					{
						float _probability = calProbability((*grid_list_)[index].current_feature_,(*grid_list_)[num].ground_feature_);
						if ((*grid_list_)[num].current_feature_.point_size_ > _ground_points_size)
						{
							_optimal_index_ = num;
							_optimal_probability = _probability;
							_ground_points_size = (*grid_list_)[num].current_feature_.point_size_;
						}
					}
				}
			}
			if (_optimal_index_ > 0)
			{
				if (_optimal_probability > 0.80)
				{
					(*grid_list_)[index].check_flag_ = 0;
					(*grid_list_)[index].ground_flag_ = 0;
					CopyFeature((*grid_list_)[index].current_feature_,(*grid_list_)[index].ground_feature_);
					(*ground_grid_list)[index].UpdateGroundFeature((*grid_list_)[index].ground_feature_);
				}
				else // 里面包含障碍点云
				{
					
					if ((*ground_grid_list)[index].is_vaild_)
					{
						if (((*grid_list_)[index].current_feature_.abs_height_ < height_threshold_)
						&&((*grid_list_)[index].current_feature_.avg_z_ < ((*ground_grid_list)[index].ground_feature_.avg_z_ + height_threshold_)))
						{
							for (int i = 0 ; i < (*grid_list_)[index].current_feature_.point_size_ ; i++)
							{
								(*grid_list_)[index].grid_ground_inliers_.indices.push_back((*grid_list_)[index].grid_inliers_.indices[i]);
							}
							(*grid_list_)[index].check_flag_ = 0;
							(*grid_list_)[index].ground_flag_ = 1;
						}else
						{
							SubGrid _sub_grid;
							_sub_grid.segment((*grid_list_)[index],0.2,height_threshold_,
								(*grid_list_)[_optimal_index_].current_feature_,
								(*ground_grid_list)[index],
								(*grid_list_)[index].grid_ground_inliers_);
								(*grid_list_)[index].check_flag_ = 0;
								(*grid_list_)[index].ground_flag_ = 1;
						}
					}else
					{
						SubGrid _sub_grid;
						_sub_grid.segment((*grid_list_)[index],0.2,height_threshold_,
							(*grid_list_)[_optimal_index_].current_feature_,
							(*ground_grid_list)[index],
							(*grid_list_)[index].grid_ground_inliers_);

							(*grid_list_)[index].check_flag_ = 0;
							(*grid_list_)[index].ground_flag_ = 1;
					}
				}
			}
			else
			{
				//printf("-----------------\n");
				// 没有在栅格周围的八个栅格中找到参考，继续向四周扩散，找到最近的一个作为参考
				// 查找方向为向原点靠近
				//
				
				int _ground_points_size = 0;
				int _optimal_index_ = 0;
				std::vector<int> _range_indexs;
				searchNearestGroundGrid(index,_range_indexs);
				for (auto num : _range_indexs)
				{
					if ((*grid_list_)[num].current_feature_.point_size_ > 2)
					{
						if ((*grid_list_)[num].current_feature_.point_size_ > _ground_points_size)
						{
							_optimal_index_ = num;
							_ground_points_size = (*grid_list_)[num].current_feature_.point_size_;
						}
					}
				}
				if (_optimal_index_ > 0)
				{
					for (int i = 0 ; i < (*grid_list_)[index].current_feature_.point_size_ ; i++)
					{
						if ((*grid_list_)[index].grid_cloud_[i].z < 0.2)
						//if (((*grid_list_)[index].grid_cloud_[i].z > ((*grid_list_)[_optimal_index_].current_feature_.avg_z_ - height_threshold_))
						//	&& ((*grid_list_)[index].grid_cloud_[i].z < ((*grid_list_)[_optimal_index_].current_feature_.avg_z_ + height_threshold_)))
						{
							(*grid_list_)[index].grid_ground_inliers_.indices.push_back((*grid_list_)[index].grid_inliers_.indices[i]);
						}
					}
					(*grid_list_)[index].check_flag_ = 0;
					(*grid_list_)[index].ground_flag_ = 1;
				}
			}
		}
	}
};
*/

void GridSegment::segment(boost::shared_ptr<std::vector<Grid>> &ground_grid_list,pcl::PointIndices &out_indices)
{
	if (ground_grid_list->size() == 0)
	{
		int _grid_size = col_ * row_;
		ground_grid_list->resize(_grid_size);
		for (int i = 0; i < _grid_size; ++i)
		{
			(*ground_grid_list)[i].is_vaild_ = false;
		}
	}
	
	int _grid_size = grid_index_list_.size();
	
	for (int i = 0; i < _grid_size ; ++i)
	{
		int _index = grid_index_list_[i].index_;

		if ((*grid_list_)[_index].is_vaild_)
		{
			firstCheckGrid(_index,ground_grid_list);
		}
	}
	
	for (int i = 0; i < _grid_size ; ++i)
	{
		int _index = grid_index_list_[i].index_;
		if ((*grid_list_)[_index].is_vaild_)
		{
			secondCheckGrid(_index,ground_grid_list);
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

	printf("out_indices.indices.size() = %ld\n",out_indices.indices.size());
};

int GridSegment::getIndex(int col,int row)
{
	if ((col >= 0)&&(col <= col_) && (row >= 0) && (row <= row_))
	{
		return (col* row_ + row);
	}
	return -1;
};


void GridSegment::getRangeIndex(int col,int row ,std::vector<int> &index_vec)
{
	//查找周围八个栅格
	int _index = -1;
	_index = getIndex(col,row - 1);
	if (_index != -1)
	{
		index_vec.push_back(_index);
	}
	_index = getIndex(col - 1,row - 1);
	if (_index != -1)
	{
		index_vec.push_back(_index);
	}
	_index = getIndex(col - 1,row);
	if (_index != -1)
	{
		index_vec.push_back(_index);
	}
	_index = getIndex(col - 1,row + 1);
	if (_index != -1)
	{
		index_vec.push_back(_index);
	}
	_index = getIndex(col ,row + 1);
	if (_index != -1)
	{
		index_vec.push_back(_index);
	}
	_index = getIndex(col + 1,row + 1);
	if (_index != -1)
	{
		index_vec.push_back(_index);
	}
	_index = getIndex(col + 1,row);
	if (_index != -1)
	{
		index_vec.push_back(_index);
	}
	_index = getIndex(col + 1,row - 1);
	if (_index != -1)
	{
		index_vec.push_back(_index);
	}
};

int GridSegment::getOriginIndex(void)
{
	 int _col = (maxx_ - 0.0) / size_;
	 int _row = (maxy_ - 0.0) / size_;
	 return (_col* row_ + _row);
};

void GridSegment::searchNearestGroundGrid(int index,std::vector<int> &index_vec)
{
	int i = (*grid_list_)[index].index_;
	int _col = grid_index_list_[i].col_;
	int _row = grid_index_list_[i].row_;

	int _radius = grid_index_list_[i].radius_;
	while (i > 0)
	{
		i--;
		if ((*grid_list_)[grid_index_list_[i].index_].ground_flag_ == 0)
		{
			int _nearest_col = grid_index_list_[i].col_;
			int _nearest_row = grid_index_list_[i].row_;
			int _distance = sqrt((_col - _nearest_col) *(_col - _nearest_col ) 
					+ (_row - _nearest_row) * (_row - _nearest_row));
			if (_distance < 3.0)
			{
				index_vec.push_back(grid_index_list_[i].index_);
			}
			if (_radius - grid_index_list_[i].radius_ > 3.0)
			{
				break;
			}
		}
	}
};

float GridSegment::calProbability( Feature &first,Feature &second)
{
	float _min_z_probability = 1.0
		 / (1.0 + fabs(first.min_z_ - second.min_z_)
		 / (fabs(first.min_z_) + fabs(second.min_z_)));

	float _max_z_probability =  1.0 
		 / (1.0 + fabs(first.max_z_ - second.max_z_)
		 / (fabs(first.max_z_) + fabs(second.max_z_)));

	float _deviation_z_probability = 1.0 
		 / (1.0 + fabs(first.deviation_z_ - second.deviation_z_)
		 /(fabs(first.deviation_z_) + fabs(second.deviation_z_)));

	float _abs_height_probability = 1.0
		 / (1.0 + fabs(first.abs_height_ - second.abs_height_)
		 / (first.abs_height_ + second.abs_height_));

	float _avg_z_probability =  1.0
		 / (1.0 + fabs(first.avg_z_ - second.avg_z_)
		 / (fabs(first.avg_z_) + fabs(second.avg_z_)));

	float _probability = ( _deviation_z_probability
						 + _min_z_probability
						 + _max_z_probability
						 + _abs_height_probability
						 + _avg_z_probability)/5.0;

	return _probability;

};

float GridSegment::calProbability(float first,float second)
{
	float _probability = 1.0
	 / (1.0 + fabs(first - second)
	 / (fabs(first) + fabs(second)));
	return _probability;
};

void GridSegment::CopyFeature(Feature &source,Feature &target)
{
	target.min_z_ = source.min_z_;
	target.max_z_ = source.max_z_;
	target.deviation_z_ = source.deviation_z_;
	target.abs_height_ = source.abs_height_;
	target.avg_z_ = source.avg_z_;
	target.point_size_ = source.point_size_;
};

}