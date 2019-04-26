#pragma once

#include "Eigen/Dense"

#include <iostream>

using namespace Eigen;

/*  C++ implementaion of Matlab function Cor_ini = dhy_MDS(M, space)

	MDS initialization.
	Input:
	- M: The adjacent matrix of network, of shape (N, N).
	- dim: 2 for 2D, 3 for 3D.
	Output:
	- Coordinates of nodes in the network, of shape (N, 2) if 2D
	  or (N, 3) if 3D.       */
MatrixXd MDS(MatrixXd M, int dim);

/*  SVD ICP algorithm.
	Inputs:
	- Cor_r: Relative coordinates, of shape (N, D).
	- Cor_a: Absolute coordinates of anchor nodes, of shape (C, D), where
			 C must larger or equal to 3.
	- Ind: Index of anchor nodes, of shape (C,)
	Outputs:
	- The absolute coordinates, of shape (N, D).                  */
MatrixXd ICP(MatrixXd coor_r, MatrixXd coor_ap, VectorXd ap_index);


/*  Compute node location by MDS initialization and Adam gradient descent
	in 2D space.

	Inputs:
	- M: Adjacent matrix of nodes in the network, of shape (N, N), where
		 M(i, j) is the raw distance between node i and j, measured by
		 sensors.
	- dim: 2 for 2D, 3 for 3D.
	- converge: Converge parameter.
	- iter: Maximun umber of iterations.

  Input example:
		M =	[0.0  r01  r02  r03 
				 r10  0.0  r12  r13
			   r20  r21  0.0  r23
				 r30  r31  r32  0.0]
		r01 is the range between 0 and 1, and here we assume that 0 is at the origin,
		and 1 lies along the positive x-axis, and 2 the positive y direction.

	Outputs:
	- Coordinates of nodes in the network, of shape (N, 2) if 2D
	  or (N, 3) if 3D. 

	Output example:
		coor = [x0, y0, z0
						x1, y1, z1
						x2, y2, z2
						x3, y3, z3]                                          */
MatrixXd MDS_Adam(MatrixXd M, int dim, double converge, int iter=500);

/*  Compute anchor 3D location by MDS initialization and Adam gradient descent
	with pre-calibrated heights of a part of anchors.

	Inputs:
	- M: Adjacent matrix of nodes in the network, of shape (N, N), where
		 M(i, j) is the raw distance between node i and j, measured by
		 sensors.
	- height: pre-calibrated heights, of shape (C,1).
	- hIndex: index of the corresponding anchors with pre-calibrated heights,
		 of shape (C,1) and the index starts from 0.
		 The first index correspond to the anchor at the origin, the second one
		 lies along the positive x-axis and the third one is perpendicular to
		 the x-z plane and points to the positive y direction.
		 In this way, at least 3 anchors should have pre-calibrated heights.
		 And the established coordinate depends on the third anchor.
		 If it lies in the left hand of x-axis, then we establish a left-hand
		 coordinate, otherwise a right-hand coordinate.

	Outputs:
	-  Node locations, of shape (N, 3), where ith row is node i's
		   coordinate (x_i, y_i, z_i).                                
			 
			 */
MatrixXd AncSelfLocalize(MatrixXd M, VectorXd height, VectorXd index);