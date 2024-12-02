/*
Copyright (c) 2010-2023, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <rtabmap/core/LocalCloudMaker.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d_mapping.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UTimer.h>

#ifdef RTABMAP_OCTOMAP
#include <rtabmap/core/global_map/OctoMap.h>
#endif

#include <pcl/io/pcd_io.h>

namespace rtabmap {

LocalCloudMaker::LocalCloudMaker(const ParametersMap & parameters) :
	parameters_(parameters),
	cloudDecimation_(Parameters::defaultGridDepthDecimation()),
	rangeMax_(Parameters::defaultGridRangeMax()),
	rangeMin_(Parameters::defaultGridRangeMin()),
	//roiRatios_(Parameters::defaultGridDepthRoiRatios()), // initialized in parseParameters()
	footprintLength_(Parameters::defaultGridFootprintLength()),
	footprintWidth_(Parameters::defaultGridFootprintWidth()),
	footprintHeight_(Parameters::defaultGridFootprintHeight()),
	scanDecimation_(Parameters::defaultGridScanDecimation()),
	cellSize_(Parameters::defaultGridCellSize()),
	preVoxelFiltering_(Parameters::defaultGridPreVoxelFiltering()),
	occupancySensor_(Parameters::defaultGridSensor()),
	projMapFrame_(Parameters::defaultGridMapFrameProjection()),
	maxObstacleHeight_(Parameters::defaultGridMaxObstacleHeight()),
	normalKSearch_(Parameters::defaultGridNormalK()),
	groundNormalsUp_(Parameters::defaultIcpPointToPlaneGroundNormalsUp()),
	maxGroundAngle_(Parameters::defaultGridMaxGroundAngle()*M_PI/180.0f),
	clusterRadius_(Parameters::defaultGridClusterRadius()),
	minClusterSize_(Parameters::defaultGridMinClusterSize()),
	flatObstaclesDetected_(Parameters::defaultGridFlatObstacleDetected()),
	minGroundHeight_(Parameters::defaultGridMinGroundHeight()),
	maxGroundHeight_(Parameters::defaultGridMaxGroundHeight()),
	normalsSegmentation_(Parameters::defaultGridNormalsSegmentation()),
	// grid3D_(Parameters::defaultGrid3D()),
	groundIsObstacle_(Parameters::defaultGridGroundIsObstacle()),
	noiseFilteringRadius_(Parameters::defaultGridNoiseFilteringRadius()),
	noiseFilteringMinNeighbors_(Parameters::defaultGridNoiseFilteringMinNeighbors()),
	scan2dUnknownSpaceFilled_(Parameters::defaultGridScan2dUnknownSpaceFilled()),
	rayTracing_(Parameters::defaultGridRayTracing())
{
	this->parseParameters(parameters);
}

LocalCloudMaker::~LocalCloudMaker()
{
}

void LocalCloudMaker::parseParameters(const ParametersMap & parameters)
{
	uInsert(parameters_, parameters);

	Parameters::parse(parameters, Parameters::kGridSensor(), occupancySensor_);
	Parameters::parse(parameters, Parameters::kGridDepthDecimation(), cloudDecimation_);
	if(cloudDecimation_ == 0)
	{
		cloudDecimation_ = 1;
	}
	Parameters::parse(parameters, Parameters::kGridRangeMin(), rangeMin_);
	Parameters::parse(parameters, Parameters::kGridRangeMax(), rangeMax_);
	Parameters::parse(parameters, Parameters::kGridFootprintLength(), footprintLength_);
	Parameters::parse(parameters, Parameters::kGridFootprintWidth(), footprintWidth_);
	Parameters::parse(parameters, Parameters::kGridFootprintHeight(), footprintHeight_);
	Parameters::parse(parameters, Parameters::kGridScanDecimation(), scanDecimation_);
	Parameters::parse(parameters, Parameters::kGridCellSize(), cellSize_);
	UASSERT(cellSize_>0.0f);

	Parameters::parse(parameters, Parameters::kGridPreVoxelFiltering(), preVoxelFiltering_);
	Parameters::parse(parameters, Parameters::kGridMapFrameProjection(), projMapFrame_);
	Parameters::parse(parameters, Parameters::kGridMaxObstacleHeight(), maxObstacleHeight_);
	Parameters::parse(parameters, Parameters::kGridMinGroundHeight(), minGroundHeight_);
	Parameters::parse(parameters, Parameters::kGridMaxGroundHeight(), maxGroundHeight_);
	Parameters::parse(parameters, Parameters::kGridNormalK(), normalKSearch_);
	Parameters::parse(parameters, Parameters::kIcpPointToPlaneGroundNormalsUp(), groundNormalsUp_);
	if(Parameters::parse(parameters, Parameters::kGridMaxGroundAngle(), maxGroundAngle_))
	{
		maxGroundAngle_ *= M_PI/180.0f;
	}
	Parameters::parse(parameters, Parameters::kGridClusterRadius(), clusterRadius_);
	UASSERT_MSG(clusterRadius_ > 0.0f, uFormat("Param name is \"%s\"", Parameters::kGridClusterRadius().c_str()).c_str());
	Parameters::parse(parameters, Parameters::kGridMinClusterSize(), minClusterSize_);
	Parameters::parse(parameters, Parameters::kGridFlatObstacleDetected(), flatObstaclesDetected_);
	Parameters::parse(parameters, Parameters::kGridNormalsSegmentation(), normalsSegmentation_);
	// Parameters::parse(parameters, Parameters::kGrid3D(), grid3D_);
	Parameters::parse(parameters, Parameters::kGridGroundIsObstacle(), groundIsObstacle_);
	Parameters::parse(parameters, Parameters::kGridNoiseFilteringRadius(), noiseFilteringRadius_);
	Parameters::parse(parameters, Parameters::kGridNoiseFilteringMinNeighbors(), noiseFilteringMinNeighbors_);
	Parameters::parse(parameters, Parameters::kGridScan2dUnknownSpaceFilled(), scan2dUnknownSpaceFilled_);
	Parameters::parse(parameters, Parameters::kGridRayTracing(), rayTracing_);

	// convert ROI from string to vector
	ParametersMap::const_iterator iter;
	if((iter=parameters.find(Parameters::kGridDepthRoiRatios())) != parameters.end())
	{
		std::list<std::string> strValues = uSplit(iter->second, ' ');
		if(strValues.size() != 4)
		{
			ULOGGER_ERROR("The number of values must be 4 (%s=\"%s\")", iter->first.c_str(), iter->second.c_str());
		}
		else
		{
			std::vector<float> tmpValues(4);
			unsigned int i=0;
			for(std::list<std::string>::iterator jter = strValues.begin(); jter!=strValues.end(); ++jter)
			{
				tmpValues[i] = uStr2Float(*jter);
				++i;
			}

			if(tmpValues[0] >= 0 && tmpValues[0] < 1 && tmpValues[0] < 1.0f-tmpValues[1] &&
				tmpValues[1] >= 0 && tmpValues[1] < 1 && tmpValues[1] < 1.0f-tmpValues[0] &&
				tmpValues[2] >= 0 && tmpValues[2] < 1 && tmpValues[2] < 1.0f-tmpValues[3] &&
				tmpValues[3] >= 0 && tmpValues[3] < 1 && tmpValues[3] < 1.0f-tmpValues[2])
			{
				roiRatios_ = tmpValues;
			}
			else
			{
				ULOGGER_ERROR("The roi ratios are not valid (%s=\"%s\")", iter->first.c_str(), iter->second.c_str());
			}
		}
	}

	if(maxGroundHeight_ == 0.0f && !normalsSegmentation_)
	{
		UWARN("\"%s\" should be not equal to 0 if not using normals "
				"segmentation approach. Setting it to cell size (%f).",
				Parameters::kGridMaxGroundHeight().c_str(), cellSize_);
		maxGroundHeight_ = cellSize_;
	}
	if(maxGroundHeight_ != 0.0f &&
		maxObstacleHeight_ != 0.0f &&
		maxObstacleHeight_ < maxGroundHeight_)
	{
		UWARN("\"%s\" should be lower than \"%s\", setting \"%s\" to 0 (disabled).",
				Parameters::kGridMaxGroundHeight().c_str(),
				Parameters::kGridMaxObstacleHeight().c_str(),
				Parameters::kGridMaxObstacleHeight().c_str());
		maxObstacleHeight_ = 0;
	}
	if(maxGroundHeight_ != 0.0f &&
		minGroundHeight_ != 0.0f &&
		maxGroundHeight_ < minGroundHeight_)
	{
		UWARN("\"%s\" should be lower than \"%s\", setting \"%s\" to 0 (disabled).",
				Parameters::kGridMinGroundHeight().c_str(),
				Parameters::kGridMaxGroundHeight().c_str(),
				Parameters::kGridMinGroundHeight().c_str());
		minGroundHeight_ = 0;
	}
}

template<typename PointT>
pcl::PCLPointCloud2::Ptr LocalCloudMaker::segmentCloud(
		const typename pcl::PointCloud<PointT>::Ptr & cloudIn,
		const pcl::IndicesPtr & indicesIn,
		const Transform & pose,
		const pcl::PointXYZ & viewPoint,
		pcl::IndicesPtr & groundIndices,
		pcl::IndicesPtr & obstaclesIndices,
		pcl::IndicesPtr * flatObstacles) const
{
	groundIndices.reset(new std::vector<int>);
	obstaclesIndices.reset(new std::vector<int>);
	if(flatObstacles)
	{
		flatObstacles->reset(new std::vector<int>);
	}

	typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::IndicesPtr indices(new std::vector<int>);

	if(preVoxelFiltering_)
	{
		// voxelize to grid cell size
		cloud = util3d::voxelize(cloudIn, indicesIn, cellSize_);

		indices->resize(cloud->size());
		for(unsigned int i=0; i<indices->size(); ++i)
		{
			indices->at(i) = i;
		}
	}
	else
	{
		cloud = cloudIn;
		if(indicesIn->empty() && cloud->is_dense)
		{
			indices->resize(cloud->size());
			for(unsigned int i=0; i<indices->size(); ++i)
			{
				indices->at(i) = i;
			}
		}
		else
		{
			indices = indicesIn;
		}
	}

	// add pose rotation without yaw
	float roll, pitch, yaw;
	pose.getEulerAngles(roll, pitch, yaw);
	UDEBUG("node.getPose()=%s projMapFrame_=%d", pose.prettyPrint().c_str(), projMapFrame_?1:0);
	cloud = util3d::transformPointCloud(cloud, Transform(0,0, projMapFrame_?pose.z():0, roll, pitch, 0));

	// filter footprint
	if(footprintLength_ > 0.0f || footprintWidth_ > 0.0f || footprintHeight_ > 0.0f)
	{
		indices = util3d::cropBox(
				cloud,
				indices,
				Eigen::Vector4f(
						footprintLength_>0.0f?-footprintLength_/2.0f:std::numeric_limits<int>::min(),
						footprintWidth_>0.0f&&footprintLength_>0.0f?-footprintWidth_/2.0f:std::numeric_limits<int>::min(),
						0,
						1),
				Eigen::Vector4f(
						footprintLength_>0.0f?footprintLength_/2.0f:std::numeric_limits<int>::max(),
						footprintWidth_>0.0f&&footprintLength_>0.0f?footprintWidth_/2.0f:std::numeric_limits<int>::max(),
						footprintHeight_>0.0f&&footprintLength_>0.0f&&footprintWidth_>0.0f?footprintHeight_:std::numeric_limits<int>::max(),
						1),
				Transform::getIdentity(),
				true);
	}

	// filter ground/obstacles zone
	if(minGroundHeight_ != 0.0f || maxObstacleHeight_ != 0.0f)
	{
		indices = util3d::passThrough(cloud, indices, "z",
				minGroundHeight_!=0.0f?minGroundHeight_:std::numeric_limits<int>::min(),
				maxObstacleHeight_>0.0f?maxObstacleHeight_:std::numeric_limits<int>::max());
		UDEBUG("indices after max obstacles height filtering = %d", (int)indices->size());
	}

	if(indices->size())
	{
		if(normalsSegmentation_ && !groundIsObstacle_)
		{
			UDEBUG("normalKSearch=%d", normalKSearch_);
			UDEBUG("maxGroundAngle=%f", maxGroundAngle_);
			UDEBUG("Cluster radius=%f", clusterRadius_);
			UDEBUG("flatObstaclesDetected=%d", flatObstaclesDetected_?1:0);
			UDEBUG("maxGroundHeight=%f", maxGroundHeight_);
			UDEBUG("groundNormalsUp=%f", groundNormalsUp_);
			util3d::segmentObstaclesFromGround<PointT>(
					cloud,
					indices,
					groundIndices,
					obstaclesIndices,
					normalKSearch_,
					maxGroundAngle_,
					clusterRadius_,
					minClusterSize_,
					flatObstaclesDetected_,
					maxGroundHeight_,
					flatObstacles,
					Eigen::Vector4f(viewPoint.x, viewPoint.y, viewPoint.z+(projMapFrame_?pose.z():0), 1),
					groundNormalsUp_);
			UDEBUG("viewPoint=%f,%f,%f", viewPoint.x, viewPoint.y, viewPoint.z+(projMapFrame_?pose.z():0));
			//UWARN("Saving ground.pcd and obstacles.pcd");
			//pcl::io::savePCDFile("ground.pcd", *cloud, *groundIndices);
			//pcl::io::savePCDFile("obstacles.pcd", *cloud, *obstaclesIndices);
		}
		else
		{
			UDEBUG("");
			// passthrough filter
			groundIndices = rtabmap::util3d::passThrough(cloud, indices, "z",
					minGroundHeight_!=0.0f?minGroundHeight_:std::numeric_limits<int>::min(),
					maxGroundHeight_!=0.0f?maxGroundHeight_:std::numeric_limits<int>::max());

			pcl::IndicesPtr notObstacles = groundIndices;
			if(indices->size())
			{
				notObstacles = util3d::extractIndices(cloud, indices, true);
				notObstacles = util3d::concatenate(notObstacles, groundIndices);
			}
			obstaclesIndices = rtabmap::util3d::extractIndices(cloud, notObstacles, true);
		}

		UDEBUG("groundIndices=%d obstaclesIndices=%d", (int)groundIndices->size(), (int)obstaclesIndices->size());

		// Do radius filtering after voxel filtering ( a lot faster)
		if(noiseFilteringRadius_ > 0.0 && noiseFilteringMinNeighbors_ > 0)
		{
			UDEBUG("Radius filtering (%ld ground %ld obstacles, radius=%f k=%d)",
					groundIndices->size(),
					obstaclesIndices->size()+(flatObstacles?(*flatObstacles)->size():0),
					noiseFilteringRadius_,
					noiseFilteringMinNeighbors_);
			if(groundIndices->size())
			{
				groundIndices = rtabmap::util3d::radiusFiltering(cloud, groundIndices, noiseFilteringRadius_, noiseFilteringMinNeighbors_);
			}
			if(obstaclesIndices->size())
			{
				obstaclesIndices = rtabmap::util3d::radiusFiltering(cloud, obstaclesIndices, noiseFilteringRadius_, noiseFilteringMinNeighbors_);
			}
			if(flatObstacles && (*flatObstacles)->size())
			{
				*flatObstacles = rtabmap::util3d::radiusFiltering(cloud, *flatObstacles, noiseFilteringRadius_, noiseFilteringMinNeighbors_);
			}
			UDEBUG("Radius filtering end (%ld ground %ld obstacles)",
					groundIndices->size(),
					obstaclesIndices->size()+(flatObstacles?(*flatObstacles)->size():0));

			if(groundIndices->empty() && obstaclesIndices->empty())
			{
				UWARN("Cloud (with %d points) is empty after noise "
						"filtering. Occupancy grid cannot be "
						"created.",
						(int)cloud->size());

			}
		}
	}

	pcl::PCLPointCloud2::Ptr output(new pcl::PCLPointCloud2);
	pcl::toPCLPointCloud2(*cloud, *output);
	return output;
}

pcl::PCLPointCloud2::Ptr LocalCloudMaker::segmentCloud(
		const pcl::PCLPointCloud2::Ptr & cloudIn,
		const pcl::IndicesPtr & indicesIn,
		const Transform & pose,
		const pcl::PointXYZ & viewPoint,
		pcl::IndicesPtr & groundIndices,
		pcl::IndicesPtr & obstaclesIndices,
		pcl::IndicesPtr * flatObstacles) const
{
	groundIndices.reset(new std::vector<int>);
	obstaclesIndices.reset(new std::vector<int>);
	if(flatObstacles)
	{
		flatObstacles->reset(new std::vector<int>);
	}

	pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
	pcl::IndicesPtr indices(new std::vector<int>);

	if(preVoxelFiltering_)
	{
		// voxelize to grid cell size
		cloud = util3d::voxelize(cloudIn, indicesIn, cellSize_);

		indices->resize(cloud->height * cloud->width);
		for(unsigned int i=0; i<indices->size(); ++i)
		{
			indices->at(i) = i;
		}
	}
	else
	{
		cloud = cloudIn;
		if(indicesIn->empty() && cloud->is_dense)
		{
			indices->resize(cloud->height * cloud->width);
			for(unsigned int i=0; i<indices->size(); ++i)
			{
				indices->at(i) = i;
			}
		}
		else
		{
			indices = indicesIn;
		}
	}

	// add pose rotation without yaw
	float roll, pitch, yaw;
	pose.getEulerAngles(roll, pitch, yaw);
	UDEBUG("node.getPose()=%s projMapFrame_=%d", pose.prettyPrint().c_str(), projMapFrame_?1:0);
	cloud = util3d::transformPointCloud(cloud, Transform(0,0, projMapFrame_?pose.z():0, roll, pitch, 0));

	// filter footprint
	if(footprintLength_ > 0.0f || footprintWidth_ > 0.0f || footprintHeight_ > 0.0f)
	{
		indices = util3d::cropBox(
				cloud,
				indices,
				Eigen::Vector4f(
						footprintLength_>0.0f?-footprintLength_/2.0f:std::numeric_limits<int>::min(),
						footprintWidth_>0.0f&&footprintLength_>0.0f?-footprintWidth_/2.0f:std::numeric_limits<int>::min(),
						0,
						1),
				Eigen::Vector4f(
						footprintLength_>0.0f?footprintLength_/2.0f:std::numeric_limits<int>::max(),
						footprintWidth_>0.0f&&footprintLength_>0.0f?footprintWidth_/2.0f:std::numeric_limits<int>::max(),
						footprintHeight_>0.0f&&footprintLength_>0.0f&&footprintWidth_>0.0f?footprintHeight_:std::numeric_limits<int>::max(),
						1),
				Transform::getIdentity(),
				true);
	}

	// filter ground/obstacles zone
	if(minGroundHeight_ != 0.0f || maxObstacleHeight_ != 0.0f)
	{
		indices = util3d::passThrough(cloud, indices, "z",
				minGroundHeight_!=0.0f?minGroundHeight_:std::numeric_limits<int>::min(),
				maxObstacleHeight_>0.0f?maxObstacleHeight_:std::numeric_limits<int>::max());
		UDEBUG("indices after max obstacles height filtering = %d", (int)indices->size());
	}

	if(indices->size())
	{
		if(normalsSegmentation_ && !groundIsObstacle_)
		{
			UDEBUG("normalKSearch=%d", normalKSearch_);
			UDEBUG("maxGroundAngle=%f", maxGroundAngle_);
			UDEBUG("Cluster radius=%f", clusterRadius_);
			UDEBUG("flatObstaclesDetected=%d", flatObstaclesDetected_?1:0);
			UDEBUG("maxGroundHeight=%f", maxGroundHeight_);
			UDEBUG("groundNormalsUp=%f", groundNormalsUp_);
			util3d::segmentObstaclesFromGround(
					cloud,
					indices,
					groundIndices,
					obstaclesIndices,
					normalKSearch_,
					maxGroundAngle_,
					clusterRadius_,
					minClusterSize_,
					flatObstaclesDetected_,
					maxGroundHeight_,
					flatObstacles,
					Eigen::Vector4f(viewPoint.x, viewPoint.y, viewPoint.z+(projMapFrame_?pose.z():0), 1),
					groundNormalsUp_);
			UDEBUG("viewPoint=%f,%f,%f", viewPoint.x, viewPoint.y, viewPoint.z+(projMapFrame_?pose.z():0));
			//UWARN("Saving ground.pcd and obstacles.pcd");
			//pcl::io::savePCDFile("ground.pcd", *cloud, *groundIndices);
			//pcl::io::savePCDFile("obstacles.pcd", *cloud, *obstaclesIndices);
		}
		else
		{
			UDEBUG("");
			// passthrough filter
			groundIndices = rtabmap::util3d::passThrough(cloud, indices, "z",
					minGroundHeight_!=0.0f?minGroundHeight_:std::numeric_limits<int>::min(),
					maxGroundHeight_!=0.0f?maxGroundHeight_:std::numeric_limits<int>::max());

			pcl::IndicesPtr notObstacles = groundIndices;
			if(indices->size())
			{
				notObstacles = util3d::extractIndices(cloud, indices, true);
				notObstacles = util3d::concatenate(notObstacles, groundIndices);
			}
			obstaclesIndices = rtabmap::util3d::extractIndices(cloud, notObstacles, true);
		}

		UDEBUG("groundIndices=%d obstaclesIndices=%d", (int)groundIndices->size(), (int)obstaclesIndices->size());

		// Do radius filtering after voxel filtering ( a lot faster)
		if(noiseFilteringRadius_ > 0.0 && noiseFilteringMinNeighbors_ > 0)
		{
			UDEBUG("Radius filtering (%ld ground %ld obstacles, radius=%f k=%d)",
					groundIndices->size(),
					obstaclesIndices->size()+(flatObstacles?(*flatObstacles)->size():0),
					noiseFilteringRadius_,
					noiseFilteringMinNeighbors_);
			if(groundIndices->size())
			{
				groundIndices = rtabmap::util3d::radiusFiltering(cloud, groundIndices, noiseFilteringRadius_, noiseFilteringMinNeighbors_);
			}
			if(obstaclesIndices->size())
			{
				obstaclesIndices = rtabmap::util3d::radiusFiltering(cloud, obstaclesIndices, noiseFilteringRadius_, noiseFilteringMinNeighbors_);
			}
			if(flatObstacles && (*flatObstacles)->size())
			{
				*flatObstacles = rtabmap::util3d::radiusFiltering(cloud, *flatObstacles, noiseFilteringRadius_, noiseFilteringMinNeighbors_);
			}
			UDEBUG("Radius filtering end (%ld ground %ld obstacles)",
					groundIndices->size(),
					obstaclesIndices->size()+(flatObstacles?(*flatObstacles)->size():0));

			if(groundIndices->empty() && obstaclesIndices->empty())
			{
				UWARN("Cloud (with %d points) is empty after noise "
						"filtering. Occupancy grid cannot be "
						"created.",
						(int)(cloud->height * cloud->width));

			}
		}
	}
	return cloud;
}

void LocalCloudMaker::createLocalMap(
		const Signature & node,
		pcl::PCLPointCloud2Ptr & cloud_out,		
		pcl::IndicesPtr & groundIndices,
		pcl::IndicesPtr & obstacleIndices,
		pcl::IndicesPtr & emptyIndices,
		pcl::PointXYZ & viewPoint)
{
	cloud_out.reset(new pcl::PCLPointCloud2);
	groundIndices.reset(new pcl::Indices);
	obstacleIndices.reset(new pcl::Indices);
	emptyIndices.reset(new pcl::Indices);
	
	if(!node.sensorData().pointCloud2Raw().isEmpty())
	{
		// PointCloud2
		UDEBUG("PointCloud2");

		const Transform & t = node.sensorData().pointCloud2Raw().localTransform();
		PointCloud2 cloud = util3d::downsample(node.sensorData().pointCloud2Raw(), scanDecimation_);

		// If ray tracing enabled, clipping will be done in occupancy2DFromLaserScan()
		if(rangeMin_ > 0.0f || rangeMax_ > 0.0f)
		{
			cloud = util3d::rangeFiltering(cloud, rangeMin_, rangeMax_);
		}

		// update viewpoint
		viewPoint = pcl::PointXYZ(t.x(), t.y(), t.z());

		createLocalMap(cloud, node.getPose(), cloud_out, groundIndices, obstacleIndices, emptyIndices, viewPoint);
	}
	else if(!node.sensorData().laserScanRaw().isEmpty() && !node.sensorData().laserScanRaw().is2d())
	{
		// 3D LaserScan
		UDEBUG("3D laser scan");

		if(occupancySensor_ == 0 || occupancySensor_ == 2)
		{
			const Transform & t = node.sensorData().laserScanRaw().localTransform();
			LaserScan scan = util3d::downsample(node.sensorData().laserScanRaw(), scanDecimation_);
#ifdef RTABMAP_OCTOMAP
			// If ray tracing enabled, clipping will be done in OctoMap or in occupancy2DFromLaserScan()
			float maxRange = rayTracing_?0.0f:rangeMax_;
#else
			// If ray tracing enabled, clipping will be done in occupancy2DFromLaserScan()
			float maxRange = !grid3D_ && rayTracing_?0.0f:rangeMax_;
#endif
			if(rangeMin_ > 0.0f || maxRange > 0.0f)
			{
				scan = util3d::rangeFiltering(scan, rangeMin_, maxRange);
			}

			// update viewpoint
			viewPoint = pcl::PointXYZ(t.x(), t.y(), t.z());

			UDEBUG("scan format=%d", scan.format());

			createLocalMap(scan, node.getPose(), cloud_out, groundIndices, obstacleIndices, emptyIndices, viewPoint);
		}

		if(occupancySensor_ >= 1)
		{
			pcl::IndicesPtr indices(new std::vector<int>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
			UDEBUG("Depth image : decimation=%d max=%f min=%f",
					cloudDecimation_,
					rangeMax_,
					rangeMin_);
			cloud = util3d::cloudRGBFromSensorData(
					node.sensorData(),
					cloudDecimation_,
#ifdef RTABMAP_OCTOMAP
					// If ray tracing enabled, clipping will be done in OctoMap or in occupancy2DFromLaserScan()
					rayTracing_?0.0f:rangeMax_,
#else
					// If ray tracing enabled, clipping will be done in occupancy2DFromLaserScan()
					!grid3D_&&rayTracing_?0.0f:rangeMax_,
#endif
					rangeMin_,
					indices.get(),
					parameters_,
					roiRatios_);

			// update viewpoint
			viewPoint = pcl::PointXYZ(0,0,0);
			if(node.sensorData().cameraModels().size())
			{
				// average of all local transforms
				float sum = 0;
				for(unsigned int i=0; i<node.sensorData().cameraModels().size(); ++i)
				{
					const Transform & t = node.sensorData().cameraModels()[i].localTransform();
					if(!t.isNull())
					{
						viewPoint.x += t.x();
						viewPoint.y += t.y();
						viewPoint.z += t.z();
						sum += 1.0f;
					}
				}
				if(sum > 0.0f)
				{
					viewPoint.x /= sum;
					viewPoint.y /= sum;
					viewPoint.z /= sum;
				}
			}
			else
			{
				// average of all local transforms
				float sum = 0;
				for(unsigned int i=0; i<node.sensorData().stereoCameraModels().size(); ++i)
				{
					const Transform & t = node.sensorData().stereoCameraModels()[i].localTransform();
					if(!t.isNull())
					{
						viewPoint.x += t.x();
						viewPoint.y += t.y();
						viewPoint.z += t.z();
						sum += 1.0f;
					}
				}
				if(sum > 0.0f)
				{
					viewPoint.x /= sum;
					viewPoint.y /= sum;
					viewPoint.z /= sum;
				}
			}

			createLocalMap(LaserScan(util3d::laserScanFromPointCloud(*cloud, indices), 0, 0.0f), node.getPose(), cloud_out, groundIndices, obstacleIndices, emptyIndices, viewPoint);
		}
	}
	else
	{
		UWARN("Cannot create local cloud: (node=%d, %s=%d).", node.id(), Parameters::kGridSensor().c_str(), occupancySensor_);
	}
}

void LocalCloudMaker::createLocalMap(
		const LaserScan & scan,
		const Transform & pose,
		pcl::PCLPointCloud2Ptr & cloud_out,
		pcl::IndicesPtr & groundIndices,
		pcl::IndicesPtr & obstacleIndices,
		pcl::IndicesPtr & emptyIndices,
		pcl::PointXYZ & viewPointInOut) const
{
	cloud_out.reset(new pcl::PCLPointCloud2);
	groundIndices.reset(new pcl::Indices);
	obstacleIndices.reset(new pcl::Indices);
	emptyIndices.reset(new pcl::Indices);
	
	if(projMapFrame_)
	{
		//we should rotate viewPoint in /map frame
		float roll, pitch, yaw;
		pose.getEulerAngles(roll, pitch, yaw);
		Transform viewpointRotated = Transform(0,0,0,roll,pitch,0) * Transform(viewPointInOut.x, viewPointInOut.y, viewPointInOut.z, 0,0,0);
		viewPointInOut.x = viewpointRotated.x();
		viewPointInOut.y = viewpointRotated.y();
		viewPointInOut.z = viewpointRotated.z();
	}

	if(scan.size())
	{
		pcl::PCLPointCloud2::Ptr cloudSegmented(new pcl::PCLPointCloud2);
		if(scan.hasRGB() && scan.hasNormals())
		{
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_pcl = util3d::laserScanToPointCloudRGBNormal(scan, scan.localTransform());
			cloudSegmented = segmentCloud<pcl::PointXYZRGBNormal>(cloud_pcl, pcl::IndicesPtr(new std::vector<int>), pose, viewPointInOut, groundIndices, obstacleIndices);
			UDEBUG("groundIndices=%d, obstaclesIndices=%d", (int)groundIndices->size(), (int)obstacleIndices->size());
		}
		else if(scan.hasRGB())
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pcl = util3d::laserScanToPointCloudRGB(scan, scan.localTransform());
			cloudSegmented = segmentCloud<pcl::PointXYZRGB>(cloud_pcl, pcl::IndicesPtr(new std::vector<int>), pose, viewPointInOut, groundIndices, obstacleIndices);
			UDEBUG("groundIndices=%d, obstaclesIndices=%d", (int)groundIndices->size(), (int)obstacleIndices->size());
		}
		else if(scan.hasNormals())
		{
			pcl::PointCloud<pcl::PointNormal>::Ptr cloud_pcl = util3d::laserScanToPointCloudNormal(scan, scan.localTransform());
			cloudSegmented = segmentCloud<pcl::PointNormal>(cloud_pcl, pcl::IndicesPtr(new std::vector<int>), pose, viewPointInOut, groundIndices, obstacleIndices);
			UDEBUG("groundIndices=%d, obstaclesIndices=%d", (int)groundIndices->size(), (int)obstacleIndices->size());
		}
		else
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl = util3d::laserScanToPointCloud(scan, scan.localTransform());
			cloudSegmented = segmentCloud<pcl::PointXYZ>(cloud_pcl, pcl::IndicesPtr(new std::vector<int>), pose, viewPointInOut, groundIndices, obstacleIndices);
			UDEBUG("groundIndices=%d, obstaclesIndices=%d", (int)groundIndices->size(), (int)obstacleIndices->size());
		}

		if((obstacleIndices->size() || groundIndices->size()))
		{
			if(groundIsObstacle_ && groundIndices->size())
			{
				obstacleIndices->insert(obstacleIndices->end(), groundIndices->begin(), groundIndices->end());
			}
		}

		// transform back in base frame
		float roll, pitch, yaw;
		pose.getEulerAngles(roll, pitch, yaw);
		Transform tinv = Transform(0,0, projMapFrame_?pose.z():0, roll, pitch, 0).inverse();

		cloud_out = util3d::transformPointCloud(cloudSegmented, tinv);
	}
	
	UDEBUG("ground=%d obstacles=%d empty=%d", groundIndices->size(), obstacleIndices->size(), emptyIndices->size());
}

void LocalCloudMaker::createLocalMap(
		const PointCloud2 & cloud2,
		const Transform & pose,
		pcl::PCLPointCloud2Ptr & cloud_out,		
		pcl::IndicesPtr & groundIndices,
		pcl::IndicesPtr & obstacleIndices,
		pcl::IndicesPtr & emptyIndices,
		pcl::PointXYZ & viewPointInOut) const
{
	cloud_out.reset(new pcl::PCLPointCloud2);
	groundIndices.reset(new pcl::Indices);
	obstacleIndices.reset(new pcl::Indices);
	emptyIndices.reset(new pcl::Indices);
	
	if(projMapFrame_)
	{
		//we should rotate viewPoint in /map frame
		float roll, pitch, yaw;
		pose.getEulerAngles(roll, pitch, yaw);
		Transform viewpointRotated = Transform(0,0,0,roll,pitch,0) * Transform(viewPointInOut.x, viewPointInOut.y, viewPointInOut.z, 0,0,0);
		viewPointInOut.x = viewpointRotated.x();
		viewPointInOut.y = viewpointRotated.y();
		viewPointInOut.z = viewpointRotated.z();
	}


	if(cloud2.size())
	{
		pcl::PCLPointCloud2::Ptr cloud_ptr = pcl::make_shared<pcl::PCLPointCloud2>(cloud2.cloud());
		pcl::PCLPointCloud2::Ptr cloudSegmented = segmentCloud(cloud_ptr, pcl::IndicesPtr(new std::vector<int>), pose, viewPointInOut, groundIndices, obstacleIndices);

		if((obstacleIndices->size() || groundIndices->size()))
		{
			if(groundIsObstacle_ && groundIndices->size())
			{
				obstacleIndices->insert(obstacleIndices->end(), groundIndices->begin(), groundIndices->end());
			}
		}

		// transform back in base frame
		float roll, pitch, yaw;
		pose.getEulerAngles(roll, pitch, yaw);
		Transform tinv = Transform(0,0, projMapFrame_?pose.z():0, roll, pitch, 0).inverse();

		cloud_out = util3d::transformPointCloud(cloudSegmented, tinv);
	}

	UDEBUG("ground=%d obstacles=%d empty=%d", groundIndices->size(), obstacleIndices->size(), emptyIndices->size());
}

} // namespace rtabmap
