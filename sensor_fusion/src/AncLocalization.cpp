#include "AncLocalization.h"
#include <assert.h>

MatrixXd MDS(MatrixXd M, int dim) 
{
	assert(dim == 2 || dim == 3 && "Wrong dimension");
	MatrixXd P = M.cwiseProduct(M);

	P = -0.5 * (((P.rowwise() - P.colwise().mean()).colwise() - P.rowwise().mean()).array() + P.mean());

	JacobiSVD<MatrixXd> svd(P, ComputeFullV | ComputeFullU);
	MatrixXd u = svd.matrixU();
	MatrixXd s = svd.singularValues();
	return u.leftCols(dim) * s.topRows(dim).cwiseSqrt().asDiagonal();
}


MatrixXd MDS_Adam(MatrixXd M, int dim, double converge, int nIterations)
{
	assert(dim == 2 || dim == 3 && "Wrong dimension");
	int n = M.rows();
	double lr = 0.05;
	double beta1 = 0.9;
	double beta2 = 0.999;
	MatrixXd m = MatrixXd::Zero(n, dim);
	MatrixXd v = MatrixXd::Zero(n, dim);

	MatrixXd coor = MDS(M, dim);
	RowVectorXd firstrow = coor.row(0);
	coor.rowwise() -= firstrow;

	if (dim == 2)
	{
		double mo = coor.row(1).norm();
		double c = coor(1, 0) / mo;
		double s = coor(1, 1) / mo;
		Matrix2d cs;
		cs << c, -s, s, c;

		coor *= cs;
	}
	else if (dim == 3)
	{
		double mo = coor.block<1, 2>(1, 1).norm();
		double c = coor(1, 1) / mo;
		double s = coor(1, 2) / mo;
		Matrix2d cs;
		cs << c, -s, s, c;
		coor.rightCols(2) *= cs;

		mo = coor.block<1, 2>(1, 0).norm();
		c = coor(1, 0) / mo;
		s = coor(1, 1) / mo;
		cs << c, -s, s, c;
		coor.leftCols(2) *= cs;

		mo = coor.block<1, 2>(2, 0).norm();
		c = coor(2, 1) / mo;
		s = coor(2, 2) / mo;
		cs << c, -s, s, c;
		coor.rightCols(2) *= cs;
	}

	int iter = 0;
	while (1)
	{
		iter++;
		if (iter > nIterations)
		{
			std::cout << "Iterations exceed " << nIterations << std::endl;
			break;
		}

		VectorXd coor_square = coor.array().square().matrix().rowwise().sum();
		MatrixXd M_t = -2 * (coor * coor.transpose());
		M_t.colwise() += coor_square;
		M_t.rowwise() += coor_square.transpose();

		M_t = (M_t.array() < 0).select(0, M_t);
		M_t = M_t.array().sqrt();

		M_t += MatrixXd::Identity(n, n);
		MatrixXd dM = M_t - M;
		MatrixXd g = MatrixXd::Zero(n, dim);

		MatrixXd dx = coor.col(0).replicate(1, n) - coor.col(0).transpose().replicate(n, 1);
		g.col(0) = (dM.array() * dx.array() / M_t.array()).rowwise().sum();
		MatrixXd dy = coor.col(1).replicate(1, n) - coor.col(1).transpose().replicate(n, 1);
		g.col(1) = (dM.array() * dy.array() / M_t.array()).rowwise().sum();
		if (dim == 3)
		{
			MatrixXd dz = coor.col(2).replicate(1, n) - coor.col(2).transpose().replicate(n, 1);
			g.col(2) = (dM.array() * dz.array() / M_t.array()).rowwise().sum();
		}

		m = beta1 * m + (1 - beta1)*g;
		v = beta2 * v + (1 - beta2)*(g.cwiseProduct(g));

		MatrixXd m_unbias = m / (1 - pow(beta1, iter));
		MatrixXd v_unbias = v / (1 - pow(beta2, iter));
		MatrixXd update = lr * m_unbias.array() / (v_unbias.array().sqrt() + 1e-8);

		coor -= update;

		if (dim == 2)
		{
			coor.row(0).setZero();
			coor(1, 1) = 0;
		}
		else if (dim == 3)
		{
			coor.row(0).setZero();
			coor.block<1,2>(1, 1).setZero();
			coor(2, 2) = 0;
		}

		if (update.array().abs().maxCoeff() < converge)
			break;
	}
	return coor;
}

MatrixXd ICP(MatrixXd coor_r, MatrixXd coor_ap, VectorXd ap_index)
{
	int cols = coor_r.cols();
	int rows = ap_index.size();
	MatrixXd coor_t(rows, cols);

	for (int i = 0; i < rows; i++)
		coor_t.row(i) << coor_r.row(ap_index(i));

	RowVectorXd mean_ap = coor_ap.colwise().mean();
	RowVectorXd mean_t = coor_t.colwise().mean();

	coor_ap.rowwise() -= mean_ap;
	coor_t.rowwise() -= mean_t;

	MatrixXd W = coor_ap.transpose() * coor_t;

	JacobiSVD<MatrixXd> svd(W, ComputeFullV | ComputeFullU);
	MatrixXd u = svd.matrixU();
	MatrixXd s = svd.singularValues();
	MatrixXd v = svd.matrixV();

	MatrixXd R = v * u.transpose();
	RowVectorXd t = mean_ap - mean_t * R;

	MatrixXd coor_a = coor_r * R;
	coor_a.rowwise() += t;

	return coor_a;
}


MatrixXd AncSelfLocalize(MatrixXd M, VectorXd height, VectorXd hIndex)
{
	int nhei = height.size();
	int nAnc = M.rows();
	assert(nAnc >= nhei && nAnc == M.cols());
	assert(nhei == hIndex.size() && nhei >= 3);

	MatrixXd M_p = MatrixXd::Zero(nhei, nhei);
	for (int i = 0; i < nhei; i++)
	{
		for (int j = i+1; j < nhei; j++)
		{
			M_p(i, j) = M(hIndex(i), hIndex(j)) *
				cos(asin((height(i) - height(j)) / M(hIndex(i), hIndex(j))));
			M_p(j, i) = M_p(i, j);
		}
	}

	// Start from 2D 
	VectorXd ap_index(2);
	ap_index << 0,1;
	MatrixXd coor_ap(2, 2);
	coor_ap << 0, 0, M_p(0, 1), 0;
	MatrixXd ancWithHeightPos2d = MDS_Adam(M_p, 2, 1e-5);
	ancWithHeightPos2d = ICP(ancWithHeightPos2d, coor_ap, ap_index);
	if (ancWithHeightPos2d(2, 1) < 0) // To ensure positive y
		ancWithHeightPos2d.col(1) *= -1;

	// Extend from 2D to 3D
	MatrixXd ancWithHeightPos3d(nhei, 3);
	ancWithHeightPos3d << ancWithHeightPos2d, height;

	if (nhei == nAnc)
		return ancWithHeightPos3d;

	MatrixXd ancPos3d = MDS_Adam(M, 3, 1e-3);
	ancPos3d = ICP(ancPos3d, ancWithHeightPos3d, hIndex);
	if (ancPos3d(hIndex(2), 1) < 0)
		ancPos3d.col(1) *= -1;

	for (int i = 0; i < nhei; i++)
		ancPos3d(hIndex(i), 2) = height(i);

	return ancPos3d;
}