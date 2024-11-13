/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

#ifndef CORELIB_INCLUDE_RTABMAP_CORE_POINTCLOUD2_H_
#define CORELIB_INCLUDE_RTABMAP_CORE_POINTCLOUD2_H_

#include "rtabmap/core/rtabmap_core_export.h" // DLL export/import defines

#include <rtabmap/core/Transform.h>
#include <pcl/PCLPointCloud2.h>

namespace rtabmap {

class RTABMAP_CORE_EXPORT PointCloud2
{
public:
	
	bool isCloudHasNormals();
	bool isCloudHasRGB();
	bool isCloudHasIntensity();
	
public:
	PointCloud2();
	PointCloud2(const rtabmap::PointCloud2 & pointcloud,
			const Transform & localTransform = Transform::getIdentity());
	PointCloud2(const pcl::PCLPointCloud2 & cloud,
			const Transform & localTransform = Transform::getIdentity());

	const pcl::PCLPointCloud2 & cloud() const {return cloud_;}
	Transform localTransform() const {return localTransform_;}

	bool empty() const {return cloud_.data.empty();}
	bool isEmpty() const {return cloud_.data.empty();}			
	bool hasNormals() const {return std::any_of(cloud_.fields.begin(), cloud_.fields.end(), [](pcl::PCLPointField field){return field.name=="normal_x";});}
	bool hasRGB() const {return std::any_of(cloud_.fields.begin(), cloud_.fields.end(), [](pcl::PCLPointField field){return field.name=="rgb" || field.name=="rgba";});}
	bool hasIntensity() const {return std::any_of(cloud_.fields.begin(), cloud_.fields.end(), [](pcl::PCLPointField field){return field.name=="intensity";});}
	// bool isCompressed() const {return !cloud_.data.empty() && cloud_.width>=1 && cloud_.height==1;}
	bool isCompressed() const {return false;}
	PointCloud2 clone() const;

	void clear() {cloud_ = pcl::PCLPointCloud2();}

	/**
	 * Concatenate clouds, localTransform is ignored.
	 */
	PointCloud2 & operator+=(const PointCloud2 &);
	/**
	 * Concatenate clouds, localTransform is ignored.
	 */
	PointCloud2 operator+(const PointCloud2 &);

private:
	void init(const pcl::PCLPointCloud2 & cloud,
			const Transform & localTransform = Transform::getIdentity());

private:
	pcl::PCLPointCloud2 cloud_;
	Transform localTransform_;
};

}

#endif /* CORELIB_INCLUDE_RTABMAP_CORE_POINTCLOUD2_H_ */
