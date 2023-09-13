#include "SimpleDistance.h"
#include <iostream>

// STEP ONE

/* Initialize everything to be able to compute the implicit distance of [Hoppe92] 
   at arbitrary points that are close enough to the point cloud.
 */

void SimpleDistance::init(const PointCloud *pointCloud, float samplingRadius)
{
    cloud = pointCloud;
	nn.setPoints( &cloud->getPoints() );

    rad = samplingRadius;
}


/* This operator returns a boolean that, 
   if true,
     signals that the value parameter has been modified 
     to contain the value of the implicit function of [Hoppe92] at point P.
 */

bool SimpleDistance::operator()(const glm::vec3 &P, float &value) const
{
	std::vector< std::pair< size_t, float > > neighInRadi(0);
	nn.getNeighborsInRadius( P, rad, neighInRadi );
	
	if( 0 < neighInRadi.size() )
	{
		value = 0.0f; // undefined case
		int i = neighInRadi[ 0 ].first; // get closest point AKA i-th point

		glm::vec3 Pi = cloud->getPoints()[ i ];
		glm::vec3 Ni = cloud->getNormals()[ i ];
		Ni = glm::normalize( Ni );

		float PN = glm::dot( P - Pi, Ni );
		glm::vec3 Z = P - (PN*Ni);
 
		if ( PN < rad )
			value = PN;
		return true;
	}
	return false;
}
/*

no known conversion for argument 3 from 
‘std::vector<std::pair<long unsigned int, float> >()’ to 
‘std::vector<std::pair<long unsigned int, float> >&’

*/


