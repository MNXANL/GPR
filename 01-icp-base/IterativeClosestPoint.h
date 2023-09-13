#ifndef _ITERATIVE_CLOSEST_POINT_INCLUDE
#define _ITERATIVE_CLOSEST_POINT_INCLUDE


#include "PointCloud.h"
#include "NearestNeighbors.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>


using namespace std;
using namespace Eigen;

typedef Eigen::Matrix< float, Dynamic, Dynamic> MatrixXf;


class IterativeClosestPoint
{

public:
	void setClouds(PointCloud *pointCloud1, PointCloud *pointCloud2);
	
	void markBorderPoints();
	void markBorderPoints2();
	vector<int> *computeCorrespondence();
	glm::mat4 computeICPStep();
	
	vector<int> *computeFullICP(unsigned int maxSteps = 100);
	void drawBorderPointsRed( ShaderProgram &program );
private:
	vector< int > *pointIdx; 
	vector< bool > cloud1_BP; // Border Points cloud 1
	vector< bool > cloud2_BP; // Border Points cloud 2
	PointCloud *cloud1, *cloud2;
	
	Eigen::JacobiSVD< Matrix3f > jacobiSolver;
	Matrix3f U,V; // Rotation matrix
	Matrix3f R; // Rotation matrix
	Vector3f t; // Translation vector
};


#endif // _ITERATIVE_CLOSEST_POINT_INCLUDE


