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

#include "rtabmap/utilite/UtiLite.h"

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
			const Transform & localTransform);
	PointCloud2(const pcl::PCLPointCloud2 & cloud,
			const Transform & localTransform = Transform::getIdentity());

	pcl::PCLPointCloud2 cloud() const {return cloud_;}
	Transform localTransform() const {return localTransform_;}

	bool empty() const {return cloud_.data.empty();}
	bool isEmpty() const {return cloud_.data.empty();}			
	int size() const {return cloud_.height * cloud_.width;}			
	bool hasNormals() const {return std::any_of(cloud_.fields.begin(), cloud_.fields.end(), [](pcl::PCLPointField field){return field.name=="normal_x";});}
	bool hasRGB() const {return std::any_of(cloud_.fields.begin(), cloud_.fields.end(), [](pcl::PCLPointField field){return field.name=="rgb" || field.name=="rgba";});}
	bool hasIntensity() const {return std::any_of(cloud_.fields.begin(), cloud_.fields.end(), [](pcl::PCLPointField field){return field.name=="intensity";});}
	// bool isCompressed() const {return !cloud_.data.empty() && cloud_.width>=1 && cloud_.height==1;}
	bool isCompressed() const {return false;}
	bool isOrganized() const {return !cloud_.is_dense;}	
	PointCloud2 clone() const;

    struct hasField
    {
		std::string key;
		hasField(const pcl::PCLPointField& item): key(item.name) {}
	
		bool operator()(const pcl::PCLPointField& field) {
			return (field.name == key);
		}
    };

	static inline int sizeOfPointField(int datatype)
	{
		if ((datatype == pcl::PCLPointField::PointFieldTypes::INT8) || (datatype == pcl::PCLPointField::PointFieldTypes::UINT8))
			return 1;
		else if ((datatype == pcl::PCLPointField::PointFieldTypes::INT16) || (datatype == pcl::PCLPointField::PointFieldTypes::UINT16))
			return 2;
		else if ((datatype == pcl::PCLPointField::PointFieldTypes::INT32) || (datatype == pcl::PCLPointField::PointFieldTypes::UINT32) ||
			(datatype == pcl::PCLPointField::PointFieldTypes::FLOAT32))
			return 4;
		else if ((datatype == pcl::PCLPointField::PointFieldTypes::INT64) || (datatype == pcl::PCLPointField::PointFieldTypes::UINT64) ||
			(datatype == pcl::PCLPointField::PointFieldTypes::FLOAT64))
			return 8;
		else
			{
				std::stringstream err;
				err << "PointField of type " << datatype << " does not exist";
				throw std::runtime_error(err.str());
			}
		
		return -1;
	};

    /**
     * @brief Function adding new fields to a PointCloud2 and adjusting the internal data storage of the PointCloud2
     *        
     * @param cloud_in the cloud to use
     * @param cloud_out the cloud to return
     * @param newFields the fields to add
     * @param do_copy   flag to copy data from input to output clouds (false allows user to wait after follow up operations before copying)
     * 
     */
	static void appendPointCloud2Fields(const pcl::PCLPointCloud2::Ptr &cloud_in, pcl::PCLPointCloud2::Ptr &cloud_out, const std::vector<pcl::PCLPointField> &newFields, bool do_copy)
    {
		int in_count = cloud_in->fields.size();
		int out_count = in_count + newFields.size();

		//Setup new cloud
			pcl::PCLPointCloud2::Ptr cloud_new(new pcl::PCLPointCloud2);	  
		cloud_new->fields.clear();
		cloud_new->header = cloud_in->header;
		cloud_new->height = cloud_in->height;
		cloud_new->width  = cloud_in->width;
		cloud_new->fields.reserve(out_count);
		cloud_new->is_bigendian = cloud_in->is_bigendian;
		cloud_new->is_dense = cloud_in->is_dense;
		int offset = 0;

		//Copy existing fields
		for (int i = 0; i < in_count; ++i) 
		{
			std::string field_name = cloud_in->fields[i].name;
			int datatype = cloud_in->fields[i].datatype;
			int count = cloud_in->fields[i].count;
			if (field_name == "rgb" || field_name == "rgba")
			{
				if((datatype == pcl::PCLPointField::PointFieldTypes::FLOAT32) ||
					(datatype == pcl::PCLPointField::PointFieldTypes::UINT32)  ||
					(datatype == pcl::PCLPointField::PointFieldTypes::UINT8 && count == 4))
				{
					offset = appendPointField(cloud_new, "rgba", 1, pcl::PCLPointField::PointFieldTypes::UINT32, cloud_in->fields[i].offset);
				}
			}
			else
			{
				offset = appendPointField(cloud_new, field_name, 1, datatype, cloud_in->fields[i].offset);  // use input cloud offsets to maintain existing padding
			}
		}
		offset = cloud_in->point_step; // use input cloud offsets to maintain existing padding

		//Add new fields
		for (size_t i = 0; i < newFields.size(); ++i) 
		{
			// Create the corresponding PointField
			if (newFields[i].name == "xyz") 
			{
				// Do x, y and z
				offset = appendPointField(cloud_new, "x", 1, pcl::PCLPointField::PointFieldTypes::FLOAT32, offset);
				offset = appendPointField(cloud_new, "y", 1, pcl::PCLPointField::PointFieldTypes::FLOAT32, offset);
				offset = appendPointField(cloud_new, "z", 1, pcl::PCLPointField::PointFieldTypes::FLOAT32, offset);
				offset += sizeOfPointField(pcl::PCLPointField::PointFieldTypes::FLOAT32);
			}
			else if((newFields[i].name == "rgb") || (newFields[i].name == "rgba")) 
			{
				offset = appendPointField(cloud_new, newFields[i].name, 1, pcl::PCLPointField::PointFieldTypes::UINT32, offset);
			}
			else
			{
				offset = appendPointField(cloud_new, newFields[i].name, newFields[i].count, newFields[i].datatype, offset);
			}
		}

		// Resize the point cloud accordingly
		cloud_new->point_step = offset;
		cloud_new->row_step = cloud_new->width * cloud_new->point_step;
		cloud_new->data.resize(cloud_new->height * cloud_new->row_step);

		// Initialize new data
		for(uint32_t i=0; i < cloud_in->data.size(); i++)
		{
			cloud_new->data[i] = 0;
		}

		if(do_copy)
		{
		//Copy existing data and initialize new data
			for(uint32_t i=0; i < cloud_in->height; i++)
			{
				for(uint32_t j=0; j < cloud_in->width; j++)
				{
					uint32_t input_step = cloud_in->point_step;
					uint32_t output_step = cloud_new->point_step;            
					uint32_t count = std::min(input_step, output_step);
					uint32_t input_begin = i * cloud_in->row_step + j * cloud_in->point_step;
					uint32_t output_begin = i * cloud_new->row_step + j * cloud_new->point_step;
					//copy
					for(uint32_t k=0; k < count; k++)
					{
					cloud_new->data[output_begin+k] = cloud_in->data[input_begin+k];
					}
					//Initialize
					for(uint32_t k=count; k < output_step; k++)
					{
					cloud_new->data[output_begin+k] = 0;
					}
				}
			}
		}

		cloud_out = cloud_new;
    };

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

    /** Private Function that adds a PointField to the end of "fields" member of a PointCloud2.  Does not adjust the data size!
     * @param cloud the PointCloud2 to add a field to
     * @param name the name of the new field
     * @param offset the offset of the new field
     * @param count the number of elements in the new field
     * @param datatype the datatype of the new elements
     * @return the minimum offset to the next PointField that can be added to the PointCLoud2
     */
	static int appendPointField(pcl::PCLPointCloud2::Ptr &cloud, const std::string &name, const int &count, const int &datatype, const int &offset)
    {
		pcl::PCLPointField point_field;
		point_field.name = name;
		point_field.count = count;
		point_field.datatype = datatype;
		point_field.offset = offset;

		bool compare = std::any_of(cloud->fields.begin(), cloud->fields.end(), hasField(point_field));
		if (compare) 
		{
			for(size_t i = 0; i < cloud->fields.size(); i++)
			{
			if(cloud->fields[i].name == point_field.name)
				UASSERT(cloud->fields[i].datatype == point_field.datatype);
			}
			return offset;
		}
		else 
		{
			cloud->fields.push_back(point_field);

			// Update the offset
			return offset + point_field.count * sizeOfPointField(datatype);
		}
    };

private:
	pcl::PCLPointCloud2 cloud_;
	Transform localTransform_;
};

}

#endif /* CORELIB_INCLUDE_RTABMAP_CORE_POINTCLOUD2_H_ */
