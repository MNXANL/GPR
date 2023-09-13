#include "NormalEstimator.h"
#include "NearestNeighbors.h"
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>


// This method has to compute a normal per point in the 'points' vector and put it in the 
// 'normals' vector. The 'normals' vector already has the same size as the 'points' vector. 
// There is no need to push_back new elements, only to overwrite them ('normals[i] = ...;')
// In order to compute the normals you should use PCA. The provided 'NearestNeighbors' class
// wraps the nanoflann library that computes K-nearest neighbors effciently. 


void NormalEstimator::computePointCloudNormals(const vector<glm::vec3> &points, vector<glm::vec3> &normals)
{
	// Preliminar: setup nearest neigbours class for later
	NearestNeighbors nn;
	nn.setPoints( &points );
	vector< size_t > myNeighbours;
	vector< float > dists_sq;
	
	int N = points.size();
	unsigned int M = 100; // Number of neighbours

	for (int i = 0; i < N; ++i) { 
		nn.getKNearestNeighbors( points[i], M, myNeighbours, dists_sq ); 
		// Step 1: get centroid of the neighbours
		glm::vec3 centroidPoint = glm::vec3(0,0,0); // Centroid point
		  
		for (int j = 0; j < M; ++j) {
			centroidPoint += points[ myNeighbours[ j ] ];
		} 
	    centroidPoint = centroidPoint / float(M); 
		
		// Steps 2 + 3 (CoVar calc optimized via loop fusion)  
		Matrix3f CoVar = Matrix3f::Zero(); // Covariance mtx
		for (int j = 0; j < M; ++j) {
			glm::vec3 translatedPoint = points[ myNeighbours[ j ] ] - centroidPoint; 
			// Convert to EigenTuxFamily's Vector before applying op.
			Vector3f myPoint = Vector3f( translatedPoint.x, translatedPoint.y, translatedPoint.z ); 

			CoVar += myPoint * myPoint.transpose();
		}
 
		// Steps 4+5: get normal from biggest eigen (already sorted increasingly, col=0 for smallest)
		SelfAdjointEigenSolver< Matrix3f > eigensolver( CoVar );
 
		glm::vec3 normBig = glm::normalize( glm::vec3(
			eigensolver.eigenvectors().col( 0 )[ 0 ],
			eigensolver.eigenvectors().col( 0 )[ 1 ],
			eigensolver.eigenvectors().col( 0 )[ 2 ]
		) ) ;


		//Step 5.5: invert the normal direction if it's flipped
		if (normBig.z < 0)
			normBig *= -1.0f;  

		// Step 6: set the normal
		normals[ i ] = normBig;
  
	}
} 