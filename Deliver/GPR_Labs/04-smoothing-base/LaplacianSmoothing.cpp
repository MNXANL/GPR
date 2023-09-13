#include <iostream>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>
#include "LaplacianSmoothing.h"



void LaplacianSmoothing::setMesh(TriangleMesh *newMesh)
{
	mesh = newMesh;
}


/* This method should apply nIterations iterations of the laplacian vector multiplied by lambda 
   to each of the vertices. */

glm::vec3 LaplacianSmoothing::Delta( vector< glm::vec3 > &V, const glm::vec3 &Pi, const vector< unsigned int > &neigh )
{
	float K = float( 1.0 / (float)neigh.size() );
	glm::vec3 sumVtxDiff = glm::vec3(0.0f, 0.0f, 0.0f);
	for ( int i = 0; i < neigh.size(); ++i )
	{
		glm::vec3 Pj = V[ neigh[ i ] ];
		sumVtxDiff += Pj - Pi;
	}

	return glm::vec3
	( 
		sumVtxDiff.x * K, 
		sumVtxDiff.y * K,  
		sumVtxDiff.z * K 
	);
}

void LaplacianSmoothing::iterativeLaplacian(int nIterations, float lambda)
{
	vector< glm::vec3 > V = mesh->getVertices();
	vector< glm::vec3 > W = V;
	for ( int iter = 0; iter < nIterations; ++iter )
	{
		for ( int i = 0; i < V.size(); ++i )
		{
			vector<unsigned int> neighbors;
			mesh->getNeighbors( i, neighbors );

			W[i] = V[i] + lambda*Delta( V, V[i], neighbors );
		}
		V = W;
	}
	// replacing the smoothed positions into the mesh
	for ( int i = 0; i < W.size(); ++i )
		mesh->getVertices()[i] = W[i];
	
}

/* This method should apply nIterations iterations of the bilaplacian operator using lambda 
   as a scaling factor. */

void LaplacianSmoothing::iterativeBilaplacian(int nIterations, float lambda)
{
	vector< glm::vec3 > V = mesh->getVertices();
	vector< glm::vec3 > W = V;
	for ( int iter = 0; iter < nIterations; ++iter )
	{
		for ( int i = 0; i < V.size(); ++i )
		{
			vector<unsigned int> neighbors;
			mesh->getNeighbors( i, neighbors );
			glm::vec3 V1;

			V1   = V[i] + lambda*Delta( V, V[i], neighbors );  // p'
			W[i] = V1   - lambda*Delta( V, V1,   neighbors );  // p''  
		}
		V = W;
	}
	// replacing the smoothed positions into the mesh
	for ( int i = 0; i < W.size(); ++i )
		mesh->getVertices()[i] = W[i];
}

/* This method should apply nIterations iterations of Taubin's operator using lambda 
   as a scaling factor, and computing the corresponding nu value. */
void LaplacianSmoothing::iterativeLambdaNu(int nIterations, float lambda)
{
	vector< glm::vec3 > V = mesh->getVertices();
	vector< glm::vec3 > W = V;
	for ( int iter = 0; iter < nIterations; ++iter )
	{
		for ( int i = 0; i < V.size(); ++i )
		{
			vector<unsigned int> neighbors;
			mesh->getNeighbors( i, neighbors );
			glm::vec3 V1;
			float mu = lambda / ( (0.1 * lambda) - 1 );
			std::cout << " --> l = " << lambda << std::endl;
			std::cout << " --> u = " << mu << std::endl;

			V1   = V[i] + lambda*Delta( V, V[i], neighbors );  // p'
			W[i] = V1   +     mu*Delta( V, V1,   neighbors );  // p'' 
		} 
		V = W;
	}
	// replacing the smoothed positions into the mesh
	for ( int i = 0; i < W.size(); ++i )
		mesh->getVertices()[i] = W[i];
}






















/* This method should compute new vertices positions by making the laplacian zero, while
   maintaing the vertices marked as constraints fixed. */

void LaplacianSmoothing::globalLaplacian(const vector<bool> &constraints)
{ 
	
	vector< glm::vec3 > V = mesh->getVertices();
	//vector< glm::vec3 > W = V;
    //Matrix< double, V.size(), V.size() > M = Matrix< double, V.size() >::Identity();
	//Matrix< double, V.size(), V.size() > C;
	for ( int i = 0; i < V.size(); ++i )
	{
		// creating M (diagonal) matrix
		vector<unsigned int> neighbors;
		mesh->getNeighbors( i, neighbors );
		//M[i] = neighbors.size();

		// creating C matrix
		for ( int j = 0; j < V.size(); ++j )
		{
/*
			if ( i == j )
				//C[i][j] = 1.0;
			else 
			{
				vector< unsigned int >::iterator it = neighbors.find( neighbors.begin(), neighbors.end(), j );
				// Weight is the negative sum of weights of all neighbors. If all neighbors have w=1, 
				// the size of the array is just enough for the calculation :)
				if ( i != j && it != neighbors.end() )
				{	
					C[i][j] = float( -1.0*neighbors.size() );
				}
				else
				{
					C[i][j] = 0.0;
				} 
			}
*/
		}
	}
	
	// Matrix merger
}

/* This method has to optimize the vertices' positions in the least squares sense, 
   so that the laplacian is close to zero and the vertices remain close to their 
   original locations. The constraintWeight parameter is used to control how close 
   the vertices have to be to their original positions. */

void LaplacianSmoothing::globalBilaplacian(const vector<bool> &constraints, float constraintWeight)
{
	//...
}