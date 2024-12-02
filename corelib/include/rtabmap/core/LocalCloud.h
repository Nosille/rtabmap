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

#ifndef SRC_LOCALCLOUD_H_
#define SRC_LOCALCLOUD_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <map>
#include <opencv2/core.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/impl/point_types.hpp>

namespace rtabmap {

class RTABMAP_CORE_EXPORT LocalCloud
{
public:
	LocalCloud(
		 const pcl::PCLPointCloud2 & cloud2,
		 const pcl::Indices & ground,
		 const pcl::Indices & obstacles,
		 const pcl::Indices & empty,
		 float cellSize,
		 const pcl::PointXYZ & viewPoint = pcl::PointXYZ(0,0,0));
	virtual ~LocalCloud() {}
	bool is3D() const;
public:
	pcl::PCLPointCloud2 cloud2;
	pcl::Indices groundIndices;
	pcl::Indices obstacleIndices;
	pcl::Indices emptyIndices;
	float cellSize;
	pcl::PointXYZ viewPoint;
};

class RTABMAP_CORE_EXPORT LocalCloudCache
{
public:
	LocalCloudCache() {}
	virtual ~LocalCloudCache() {}

	void add(int nodeId,
			const pcl::PCLPointCloud2 & cloud2,
			const pcl::Indices & ground,
			const pcl::Indices & obstacles,
			const pcl::Indices & empty,
			float cellSize,
			const pcl::PointXYZ & viewPoint = pcl::PointXYZ(0,0,0));

	void add(int nodeId, const LocalCloud & localCloud);

	bool shareTo(int nodeId, LocalCloudCache & anotherCache) const;

	unsigned long getMemoryUsed() const;
	void clear(bool temporaryOnly = false);

	size_t size() const {return LocalClouds_.size();}
	bool empty() const {return LocalClouds_.empty();}
	const std::map<int, LocalCloud> & localClouds() const {return LocalClouds_;}

	std::map<int, LocalCloud>::const_iterator find(int nodeId) const {return LocalClouds_.find(nodeId);}
	std::map<int, LocalCloud>::const_iterator begin() const {return LocalClouds_.begin();}
	std::map<int, LocalCloud>::const_iterator end() const {return LocalClouds_.end();}

private:
	std::map<int, LocalCloud> LocalClouds_;
};

} /* namespace rtabmap */

#endif /* SRC_LOCALCLOUD_H_ */
