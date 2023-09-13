#ifndef _MONGE_PATCH_INCLUDE
#define _MONGE_PATCH_INCLUDE


#include <vector>
#include "glm/glm.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>


using namespace std;
using namespace Eigen;

typedef Eigen::Matrix< double, 6, 6 > 	Matrix6d;
typedef Eigen::Matrix< double, 6, 1 > 	Vector6d;

class MongePatch
{

public:
	void init(const glm::vec3 &P, const glm::vec3 &normal, const vector<glm::vec3> &closest);
	
	void principalCurvatures(float &kmin, float &kmax) const;

private:
	// Coordinate sytem
	glm::vec3 Ori;		// origin (= point)
	glm::vec3 u, v, w;  // axis of the coord system 
	vector< glm::vec3 > closestCoords;

	Vector6d vecS;
};


#endif // _MONGE_PATCH_INCLUDE


