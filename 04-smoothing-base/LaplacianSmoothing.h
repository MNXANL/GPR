#ifndef _LAPLACIAN_SMOOTHING_INCLUDE
#define _LAPLACIAN_SMOOTHING_INCLUDE


#include "TriangleMesh.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>


using namespace std;
using namespace Eigen;


class LaplacianSmoothing
{
public:
	void setMesh(TriangleMesh *newMesh);
	void iterativeLaplacian(int nIterations, float lambda);
	void iterativeBilaplacian(int nIterations, float lambda);
	void iterativeLambdaNu(int nIterations, float lambda);
	
	void globalLaplacian(const vector<bool> &constraints);
	void globalBilaplacian(const vector<bool> &constraints, float constraintWeight);
	
private:
	glm::vec3 Delta( vector< glm::vec3 > &V, const glm::vec3 &Pi, const vector< unsigned int > &neigh );
	TriangleMesh *mesh;
	
};


#endif // _LAPLACIAN_SMOOTHING_INCLUDE


