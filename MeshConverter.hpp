/*
 * MeshConverter.hpp
 *
 *  Created on: Oct 29, 2012
 *      Author: russc
 */

#ifndef MESHCONVERTER_HPP_
#define MESHCONVERTER_HPP_

#include <stdlib.h>
#include <vector>
#include <map>
#include <set>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

typedef pcl::PointXYZ Point;

class MeshConverter
{
public:
    MeshConverter();
    ~MeshConverter();

    void testPCL();

protected:

private:
    
};

#endif /* MESHCONVERTER_HPP_ */
