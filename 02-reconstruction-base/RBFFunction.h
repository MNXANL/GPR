#ifndef _RBF_FUNCTION_INCLUDE
#define _RBF_FUNCTION_INCLUDE


#include "NearestNeighbors.h"
#include "ImplicitFunction.h"
#include "PointCloud.h"


class RBFFunction : public ImplicitFunction
{

public:
	void init(const PointCloud *pointCloud, float standardDeviation, float supportRadius);

	bool operator()(const glm::vec3 &P, float &value) const;
	
private:
	
	const PointCloud *cloud;
	float stDev;
	NearestNeighbors nn;

};


#endif // _RBF_FUNCTION_INCLUDE


