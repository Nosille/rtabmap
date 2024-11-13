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

#include <rtabmap/core/PointCloud2.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UConversion.h>

namespace rtabmap {

bool PointCloud2::isCloudHasNormals()
{
	return hasNormals();
}
bool PointCloud2::isCloudHasRGB()
{
	return hasRGB();
}
bool PointCloud2::isCloudHasIntensity()
{
	return hasIntensity();
}

PointCloud2::PointCloud2() :
		localTransform_(Transform::getIdentity())
{
}

PointCloud2::PointCloud2(
		const PointCloud2 & pointcloud,
		const Transform & localTransform)
{
	init(pointcloud.cloud(), localTransform);
}

PointCloud2::PointCloud2(
		const pcl::PCLPointCloud2 & cloud,
		const Transform & localTransform)
{
	init(cloud, localTransform);
}

void PointCloud2::init(
		const pcl::PCLPointCloud2 & cloud,
		const Transform & localTransform)
{
	UASSERT(!localTransform.isNull());

	cloud_ = cloud;
	localTransform_ = localTransform;
}

PointCloud2 PointCloud2::clone() const
{
	return PointCloud2(pcl::PCLPointCloud2(cloud_), localTransform_.clone());
}

PointCloud2 & PointCloud2::operator+=(const PointCloud2 & cloud)
{
	*this = *this + cloud;
	return *this;
}

PointCloud2 PointCloud2::operator+(const PointCloud2 & cloud)
{
	PointCloud2 dest;
	if(!cloud.empty())
	{
		if(this->empty())
		{
			dest = cloud.clone();
		}
		else
		{
			pcl::PCLPointCloud2 destCloud = this->cloud_ + cloud.cloud_;
			dest = PointCloud2(destCloud);
		}
	}
	else
	{
		dest = this->clone();
	}
	return dest;
}

}
