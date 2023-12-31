#ifndef _SIMPLE_DISTANCE_INCLUDE
#define _SIMPLE_DISTANCE_INCLUDE


#include <glm/glm.hpp>
#include "ImplicitFunction.h"
#include "PointCloud.h"
#include "NearestNeighbors.h"


class SimpleDistance : public ImplicitFunction
{

public:
	void init(const PointCloud *pointCloud, float samplingRadius);

	bool operator()(const glm::vec3 &P, float &value) const;
	
private:
	const PointCloud *cloud;
	float rad;
	NearestNeighbors nn;
};


#endif // _SIMPLE_DISTANCE_INCLUDE



