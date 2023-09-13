#include <iostream>
#include <iomanip>
#include <algorithm>
#include "IterativeClosestPoint.h"
#include <Eigen/Dense>
#include <Eigen/LU>
#include <glm/gtc/matrix_transform.hpp>


void IterativeClosestPoint::setClouds(PointCloud *pointCloud1, PointCloud *pointCloud2)
{ 
	R = Matrix3f::Zero();
	t = Vector3f::Zero(); 

	pointIdx = new vector<int>(0);
	cloud1 = pointCloud1; 
	cloud2 = pointCloud2;

	// PCA
	// Preliminar: setup nearest neigbours class for later
	NearestNeighbors nn1, nn2;
	nn1.setPoints( &cloud1->getPoints() );
	nn2.setPoints( &cloud2->getPoints() );

	vector< size_t > neighPC1, neighPC2;
	vector< float > dists_sq;
}

// This method should mark the border points in cloud 1. It also changes their color (for example to red).
// You will need to add an attribute to this class that stores this property for all points in cloud 1. 

void IterativeClosestPoint::markBorderPoints()
{
	// TODO
	// PCA : setup nearest neigbours class for later
	NearestNeighbors nn1;
	nn1.setPoints( &cloud1->getPoints() );

	vector< size_t > neighPC1;
	vector< float > dists_sq;
	
	int N = cloud1->getPoints().size();
	int M = 100; // Number of neighbours


	cloud1_BP = vector< bool >( N, false );

	for (int i = 0; i < N; ++i) { 
		
		// Step 1: get kNN
		glm::vec3 Pi = cloud1->getPoints()[i];
		nn1.getKNearestNeighbors( Pi, M, neighPC1, dists_sq ); 

		glm::vec3 centroidP = glm::vec3(0,0,0); 
		for (int j = 0; j < M; ++j) {
			centroidP += cloud1->getPoints()[ neighPC1[ j ] ];
		} 
	    centroidP = centroidP / float(M);  
		
		// Step 2a, 2b: compute PCA
		Matrix3f S = Matrix3f::Zero(); // Covariance mtx
		for (int j = 0; j < M; ++j) {
			glm::vec3 Pi = cloud1->getPoints()[ neighPC1[ j ] ] - centroidP; 
			Vector3f Ppt = Vector3f( Pi.x, Pi.y, Pi.z );
			S += Ppt * Ppt.transpose();	 
		}
		
		
		// Steps 4+5: get normal from biggest eigen (already sorted, but INCREASINGLY!)
		SelfAdjointEigenSolver< Matrix3f > eigensolver( S ); 
		glm::vec3 v1 = glm::normalize( glm::vec3(		// largest eigenvalue
			eigensolver.eigenvectors().col( 2 )[ 0 ],
			eigensolver.eigenvectors().col( 2 )[ 1 ],
			eigensolver.eigenvectors().col( 2 )[ 2 ]
		) ) ; 
		glm::vec3 v2 = glm::normalize( glm::vec3(
			eigensolver.eigenvectors().col( 1 )[ 0 ],
			eigensolver.eigenvectors().col( 1 )[ 1 ],
			eigensolver.eigenvectors().col( 1 )[ 2 ]
		) ) ; 
		glm::vec3 v3 = glm::normalize( glm::vec3(		// smallest eigenvalue
			eigensolver.eigenvectors().col( 0 )[ 0 ],
			eigensolver.eigenvectors().col( 0 )[ 1 ],
			eigensolver.eigenvectors().col( 0 )[ 2 ]
		) ) ;

		// Steps 4+5 
		float maxDA = 0.0f;
		float prevAngle = 0.0f; 
		for (int j = 0; j < M; ++j) {
			glm::vec3 Pj = cloud1->getPoints()[ neighPC1[ j ] ];

			glm::vec3 XYproj = glm::vec3(
				glm::dot( ( Pj - Pi ), v1 ),
				glm::dot( ( Pj - Pi ), v2 ),
				glm::dot( ( Pj - Pi ), v3 )
			); 
			float currAngle = glm::atan( XYproj[1], XYproj[0]  );

			if ( 0 < j && maxDA < glm::abs( currAngle - prevAngle ) ) {
			 	maxDA = glm::abs( currAngle - prevAngle );
			} 
			prevAngle = currAngle;
		} 
 
		cloud1_BP[ i ] = ( 4.5f > maxDA );
	} 
}

void IterativeClosestPoint::drawBorderPointsRed( ShaderProgram &program )
{  
	std::cout << "Painting borders...\n" ;
	if ( cloud1 != NULL )
		cloud1->setBorderColours( cloud1_BP, program );
}




void IterativeClosestPoint::markBorderPoints2()
{
	// TODO
	// PCA : setup nearest neigbours class for later
	NearestNeighbors nn1;
	nn1.setPoints( &cloud2->getPoints() );

	vector< size_t > neighPC1;
	vector< float > dists_sq;
	
	int N = cloud2->getPoints().size();
	int M = 100; // Number of neighbours


	cloud2_BP = vector< bool >( N, false );

	for (int i = 0; i < N; ++i) { 
		
		// Step 1: get kNN
		glm::vec3 Pi = cloud2->getPoints()[i];
		nn1.getKNearestNeighbors( Pi, M, neighPC1, dists_sq ); 

		glm::vec3 centroidP = glm::vec3(0,0,0); 
		for (int j = 0; j < M; ++j) {
			centroidP += cloud2->getPoints()[ neighPC1[ j ] ];
		} 
	    centroidP = centroidP / float(M);  
		
		// Step 2a, 2b: compute PCA
		Matrix3f S = Matrix3f::Zero(); // Covariance mtx
		for (int j = 0; j < M; ++j) {
			glm::vec3 Pi = cloud2->getPoints()[ neighPC1[ j ] ] - centroidP; 
			Vector3f Ppt = Vector3f( Pi.x, Pi.y, Pi.z );
			S += Ppt * Ppt.transpose();	 
		}
		
		
		// Steps 4+5: get normal from biggest eigen (already sorted, but INCREASINGLY!)
		SelfAdjointEigenSolver< Matrix3f > eigensolver( S ); 
		glm::vec3 v1 = glm::normalize( glm::vec3(		// largest eigenvalue
			eigensolver.eigenvectors().col( 2 )[ 0 ],
			eigensolver.eigenvectors().col( 2 )[ 1 ],
			eigensolver.eigenvectors().col( 2 )[ 2 ]
		) ) ; 
		glm::vec3 v2 = glm::normalize( glm::vec3(
			eigensolver.eigenvectors().col( 1 )[ 0 ],
			eigensolver.eigenvectors().col( 1 )[ 1 ],
			eigensolver.eigenvectors().col( 1 )[ 2 ]
		) ) ; 
		glm::vec3 v3 = glm::normalize( glm::vec3(		// smallest eigenvalue
			eigensolver.eigenvectors().col( 0 )[ 0 ],
			eigensolver.eigenvectors().col( 0 )[ 1 ],
			eigensolver.eigenvectors().col( 0 )[ 2 ]
		) ) ;

		// Steps 4+5 
		float maxDA = 0.0f;
		float prevAngle = 0.0f; 
		for (int j = 0; j < M; ++j) {
			glm::vec3 Pj = cloud2->getPoints()[ neighPC1[ j ] ];

			glm::vec3 XYproj = glm::vec3(
				glm::dot( ( Pj - Pi ), v1 ),
				glm::dot( ( Pj - Pi ), v2 ),
				glm::dot( ( Pj - Pi ), v3 )
			); 
			float currAngle = glm::atan( XYproj[1], XYproj[0]  );

			if ( 0 < j && maxDA < glm::abs( currAngle - prevAngle ) ) {
			 	maxDA = glm::abs( currAngle - prevAngle );
			} 
			prevAngle = currAngle;
		} 
 
		cloud2_BP[ i ] = ( 4.5f > maxDA );
	} 
}



// This method should compute the closest point in cloud 1 for all non border points in cloud 2. 
// This correspondence will be useful to compute the ICP step matrix that will get cloud 2 closer to cloud 1.
// Store the correspondence in this class as the following method is going to need it.
// As it is evident in its signature this method also returns the correspondence. 
// The application draws this if available.


vector<int> *IterativeClosestPoint::computeCorrespondence()
{
	markBorderPoints2(); // borders of cloud 2
	pointIdx->clear();
	NearestNeighbors nn1;
	nn1.setPoints( &cloud1->getPoints() );
	vector< size_t > neigh;
	vector< float > dists_sq;
	for (int i = 0; i < cloud2->getPoints().size(); ++i)
	{
		// no border point
		if ( cloud2_BP[i] ) 
		{ 
			pointIdx->push_back( -1 );
		}
		else
		{
			glm::vec3 Qi = cloud2->getPoints()[ i ];   
			nn1.getKNearestNeighbors( Qi, 1, neigh, dists_sq );  
			pointIdx->push_back( neigh[ 0 ] );
		}

	}   
	return pointIdx;
}


// This method should compute the rotation and translation of an ICP step from the correspondence
// information between clouds 1 and 2. Both should be encoded in the returned 4x4 matrix.
// To do this use the SVD algorithm in Eigen.

glm::mat4 IterativeClosestPoint::computeICPStep()
{
	computeCorrespondence();
	// PCA variant
	int M = pointIdx->size(); 
 
	// Step 1: get centroid of the neighbours 
	glm::vec3 centroidP = glm::vec3(0,0,0); 
	glm::vec3 centroidQ = glm::vec3(0,0,0);
	  
	for (int j = 0; j < M; ++j) {
		int i = (*pointIdx)[ j ];
		centroidP += cloud1->getPoints()[ i ];
		centroidQ += cloud2->getPoints()[ j ];
	} 
    centroidP = centroidP / float(M); 
    centroidQ = centroidQ / float(M); 

    Vector3f cP = Vector3f( centroidP.x, centroidP.y, centroidP.z );
    Vector3f cQ = Vector3f( centroidQ.x, centroidQ.y, centroidQ.z );


	// Steps 2 + 3 (CoVar calc optimized via loop fusion)  
	Matrix3f S = Matrix3f::Zero(); // Covariance mtx
	for (int j = 0; j < M; ++j) {
		int i = (*pointIdx)[ j ];
		glm::vec3 Pi = cloud1->getPoints()[ i ] - centroidP; 
		glm::vec3 Qi = cloud2->getPoints()[ j ] - centroidQ; 
		
		Vector3f Ppt = Vector3f( Pi.x, Pi.y, Pi.z );
		Vector3f Qpt = Vector3f( Qi.x, Qi.y, Qi.z );

		S += Ppt * Qpt.transpose();	 // notation-wise, it should be Q(P^T), but this just works
	} 
 
	jacobiSolver = Eigen::JacobiSVD< Matrix3f >( S, ComputeFullU | ComputeFullV);
	Matrix3f U = jacobiSolver.matrixU();
	Matrix3f V = jacobiSolver.matrixV();

	R = V * U.transpose(); 
	if ( S.determinant() == -1 )
	{
		Vector3f dPos;
		dPos << 1, 1, -1;
		Matrix3f diag = dPos.matrix().asDiagonal();
		std::cout << diag << std::endl;
		R *= diag;
	}

	t = cP - (R * cQ); 

	glm::mat4 newR = glm::mat4 ( 
		R( 0, 0 ), R( 0, 1 ), R( 0, 2 ),  t[0],
		R( 1, 0 ), R( 1, 1 ), R( 1, 2 ),  t[1],
		R( 2, 0 ), R( 2, 1 ), R( 2, 2 ),  t[2],
		0.0f,          0.0f,          0.0f,           0.0f
	);

	return newR;
}





float calcFrobenius( glm::mat4 &mat )
{
	float val = 0.0f;
	for (int i = 0; i < 3; ++i){
		for (int j = 0; j < 3; ++j){
			float absVal = std::abs( mat[i][j] );
			val += ( absVal * absVal );
			//std::cout << mat[i][j] << " ";
		}
		//std::cout << std::endl;
	}
	//std::cout << "##################################" << std::endl;
	//std::cout << "##################################" << std::endl;
	
	return std::sqrt( val );
}


// This method should perform the whole ICP algorithm with as many steps as needed.
// It should stop when maxSteps are performed, when the Frobenius norm of the transformation matrix of
// a step is smaller than a small threshold, or when the correspondence does not change from the 
// previous step.

vector<int> *IterativeClosestPoint::computeFullICP(unsigned int maxSteps)
{
	glm::mat4 myICP;
	// TODO

	vector< int > tmpIdx(0);
	for (int iter = 0 ; iter < maxSteps ; ++iter)
	{
		myICP = computeICPStep();
		myICP = myICP - glm::mat4( 1,0,0,0,  0,1,0,0,  0,0,1,0,  0,0,0,1 );
		float frobNorm = calcFrobenius( myICP );

		std::cout << "  [1]--> Iteration [" << iter << "/" << maxSteps;
		std::cout << "] \twith frob =" << std::fixed << std::setprecision(9) << frobNorm;
		std::cout << "\t norm =" << std::fixed << std::setprecision(9) << t.norm() << std::endl;

// VERIFICATION STEPS
		if (iter == 10) break; //REMOVE ME
// STEP 1: Frobenius norm check
		if ( frobNorm > 2.00f && t.norm() < 0.1f ) break;   

// STEP 2: Correspondence similarity check
		bool eqCorresp = false;
		for (int j = 0; j < tmpIdx.size(); ++j )
	 	{
	 		if (tmpIdx[j] != (*pointIdx)[j] )  
	 			break; 
	 		if ( j == tmpIdx.size()-1 ) eqCorresp = true;
	 	} 
	 	if ( eqCorresp ) break;

	 	// set current correspondences as previous ones (as ICP will re-calculate them!)
 		tmpIdx.clear();
	 	for (int j = 0; j < pointIdx->size(); ++j )
	 		tmpIdx.push_back( (*pointIdx)[j] ); 


		// set the transformation matrix myICP (= R*t) unto Q
		int M = cloud2->getPoints().size();
		for (int i = 0; i < M; ++i)
		{
			glm::vec3 Qp = cloud2->getPoints()[ i ];
			glm::vec4 V = myICP * glm::vec4( Qp.x, Qp.y, Qp.z, 0.0f );

			cloud2->getPoints()[ i ] = glm::vec3( V.x, V.y, V.z );
		} 
	}  
	// return final correspondence
	return pointIdx;
}
/*
import math

lst1 = [0.989743, -0.0452041, -0.135522,0.0518585, 0.997596, 0.0459773,0.133118, -0.0525336, 0.989707 ]
math.sqrt( sum( [abs( l )**2  for l in lst1] ) )

lst10 = [0.999866, 0.00465295, -0.0156754, -0.00469354, 0.999985, -0.0025595,0.0156633, 0.00263287, 0.999874 ]
math.sqrt( sum( [abs( l )**2  for l in lst10] ) )



*/