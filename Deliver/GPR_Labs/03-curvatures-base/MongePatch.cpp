#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>
#include "MongePatch.h"


// Given a point P, its normal, and its closest neighbors (including itself) 
// compute a quadratic Monge patch that approximates the neighborhood of P.
// The resulting patch will be used to compute the principal curvatures of the 
// surface at point P.

void MongePatch::init(const glm::vec3 &P, const glm::vec3 &normal, const vector< glm::vec3 > &closest)
{	
	// Create orientation system {u, v, w} | P is the coordinate Origin
	w = glm::normalize( -normal );
	glm::vec3 ux = glm::normalize( glm::cross( (P*glm::vec3(1, 0, 0)), w ) ) ;
	glm::vec3 vx = glm::normalize( glm::cross( w, ux ) );

	glm::vec3 uy = glm::normalize( glm::cross( (P*glm::vec3(0, 1, 0)), w ) ) ;
	glm::vec3 vy = glm::normalize( glm::cross( w, uy ) );

	glm::vec3 uz = glm::normalize( glm::cross( (P*glm::vec3(0, 0, 1)), w ) ) ;
	glm::vec3 vz = glm::normalize( glm::cross( w, uz ) );


	// get the best norm for each origin-axis combination
	float bestNorm = max( max( glm::length(vx), glm::length(vy) ), glm::length(vz) );
	if ( abs( bestNorm - glm::length(vx) ) < 0.0001 ) {
		u = ux; v = vx;
	}
	else if ( abs( bestNorm - glm::length(vy) ) < 0.0001 ) {
		u = uy; v = vy;
	}
	else {
		u = uz; v = vz;
	}

	Matrix6d mtxA = Matrix6d::Zero();
	Vector6d vecB = Vector6d::Zero();

	// Transform neighbours + least square calc to get A,B from [As = B]
	for ( int i = 0; i < closest.size(); ++i )
	{
		double Ui = glm::dot( u, closest[i] - P );
		double Vi = glm::dot( v, closest[i] - P );
		double Wi = glm::dot( w, closest[i] - P );
		Vector6d qi;  qi << Ui*Ui, Ui*Vi, Vi*Vi,   Ui, Vi, 1; 

		mtxA += qi * qi.transpose();
		vecB += Wi * qi;
	}
	// solver to get S
	vecS = mtxA.fullPivHouseholderQr().solve( vecB ); 
}

// Return the values of the two principal curvatures for this patch

void MongePatch::principalCurvatures(float &kmin, float &kmax) const
{
	// Setting up hessian matrix and 	
	Matrix2d Hw;
	Hw << 2*vecS[0],   vecS[1],
	        vecS[1], 2*vecS[2];

	EigenSolver< Matrix2d > solver;
	solver.compute( Hw, false ); //false -> no eigenvecs (don't need'em)

	// Last step: Set Kmin, Kmax
	kmin = (float) min( solver.eigenvalues()[ 0 ].real(), solver.eigenvalues()[ 1 ].real() );
	kmax = (float) max( solver.eigenvalues()[ 0 ].real(), solver.eigenvalues()[ 1 ].real() );
	 
}
