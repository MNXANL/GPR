#include <iostream>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>
#include "RBFFunction.h"

// STEP TWO

/* Initialize everything to be able to compute the implicit distance to the reconstructed
   point cloud at arbitrary points that are close enough to the point cloud. As should be
   obvious by the name of the class, the distance has to be computed using RBFs.
 */

void RBFFunction::init(const PointCloud *pointCloud, 
	float standardDeviation, float supportRadius)
{

    cloud = pointCloud;
	nn.setPoints( &cloud->getPoints() );

    stDev = standardDeviation;

	int N = pointCloud->size();
}


/* This operator returns a boolean that 
   if true 
     signals that the value parameter has been modified 
     to contain the value of the RBF implicit distance at point P.
 */

bool RBFFunction::operator()(const glm::vec3 &P, float &value) const
{
	std::vector< std::pair< size_t, float > > neighInRadi(0);
	nn.getNeighborsInRadius( P, stDev, neighInRadi );
	
	float d = 0.1f; //distance
	if( 0 < neighInRadi.size() )
	{ 

	    /*


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



	    Eigen::MatrixXd matA( double, N, N );
	    for ( int i = 0; i < N; ++i ){
	    	for ( int j = 0; j < N; ++j )
	    	{
	    		glm::vec3 pi = pointCloud->getPoints()[ i ];
	    		glm::vec3 pj = pointCloud->getPoints()[ j ];
	    		// fill A matrix
	    		double R = glm::length( pi - pj );
	    		double fi = 0.0;
	    		if ( R < 3*stDev )
	    		{
	    			fi = exp( ( R*R ) / (2*(stDev*stDev)));
	    		}
	    		matA << fi;
	    	}
	    }
	    */


		return true;
	} 
	return false;
}






