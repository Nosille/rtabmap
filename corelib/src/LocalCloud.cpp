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

#include <rtabmap/core/LocalCloud.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UStl.h>

namespace rtabmap {

LocalCloud::LocalCloud( 
			 const pcl::PCLPointCloud2 & cloud2In,
			 const pcl::Indices & groundIn,
			 const pcl::Indices & obstaclesIn,
			 const pcl::Indices & emptyIn,
			 float cellSizeIn,
			 const pcl::PointXYZ & viewPointIn) :
		cloud2(cloud2In),
		groundIndices(groundIn),
		obstacleIndices(obstaclesIn),
		emptyIndices(emptyIn),
		cellSize(cellSizeIn),
		viewPoint(viewPointIn)
{
	UASSERT(cellSize > 0.0f);
}

bool LocalCloud::is3D() const
{
	return true;
}

void LocalCloudCache::add(int nodeId,
		const pcl::PCLPointCloud2 & cloud2,
		const pcl::Indices & ground,
		const pcl::Indices & obstacles,
		const pcl::Indices & empty,
		float cellSize,
		const pcl::PointXYZ & viewPoint)
{
	add(nodeId, LocalCloud(cloud2, ground, obstacles, empty, cellSize, viewPoint));
}

void LocalCloudCache::add(int nodeId, const LocalCloud & LocalCloud)
{
	UDEBUG("nodeId=%d, totalPoints=%d, (ground=%d/%d obstacles=%d/%d empty=%d/%d)",
			nodeId, LocalCloud.cloud2.height * LocalCloud.cloud2.width, LocalCloud.groundIndices.size(),  LocalCloud.cloud2.point_step,  LocalCloud.obstacleIndices.size(),  LocalCloud.cloud2.point_step, LocalCloud.emptyIndices.size(),  LocalCloud.cloud2.point_step);
	if(nodeId < 0)
	{
		UWARN("Cannot add nodes with negative id (nodeId=%d)", nodeId);
		return;
	}
	uInsert(LocalClouds_, std::make_pair(nodeId==0?-1:nodeId, LocalCloud));
}

bool LocalCloudCache::shareTo(int nodeId, LocalCloudCache & anotherCache) const
{
	if(uContains(LocalClouds_, nodeId) && !uContains(anotherCache.localClouds(), nodeId))
	{
		const LocalCloud & LocalCloud = LocalClouds_.at(nodeId);
		anotherCache.add(nodeId, LocalCloud.cloud2, LocalCloud.groundIndices, LocalCloud.obstacleIndices, LocalCloud.emptyIndices, LocalCloud.cellSize, LocalCloud.viewPoint);
		return true;
	}
	return false;
}

unsigned long LocalCloudCache::getMemoryUsed() const
{
	unsigned long memoryUsage = 0;
	memoryUsage += LocalClouds_.size()*(sizeof(int) + sizeof(LocalCloud) + sizeof(std::map<int, LocalCloud>::iterator)) + sizeof(std::map<int, LocalCloud>);
	for(std::map<int, LocalCloud>::const_iterator iter=LocalClouds_.begin(); iter!=LocalClouds_.end(); ++iter)
	{
		memoryUsage += sizeof(iter->second.cloud2);
		memoryUsage += sizeof(iter->second.groundIndices);
		memoryUsage += sizeof(iter->second.obstacleIndices);
		memoryUsage += sizeof(iter->second.emptyIndices);
		memoryUsage += sizeof(int);
		memoryUsage += sizeof(pcl::PointXYZ);
	}
	return memoryUsage;
}

void LocalCloudCache::clear(bool temporaryOnly)
{
	if(temporaryOnly)
	{
		//clear only negative ids
		for(std::map<int, LocalCloud>::iterator iter=LocalClouds_.begin(); iter!=LocalClouds_.end();)
		{
			if(iter->first < 0)
			{
				LocalClouds_.erase(iter++);
			}
			else
			{
				break;
			}
		}
	}
	else
	{
		LocalClouds_.clear();
	}
}

} // namespace rtabmap
