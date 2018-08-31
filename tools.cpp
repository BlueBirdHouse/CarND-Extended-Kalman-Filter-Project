#include <iostream>
#include "tools.h"
#include <cmath>

const double PI = atan(1.)*4.;

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

float Tools::fixAngle(float angle_in)
{
	//将任何角度修正到0-2Pi范围内
	float angle_out = angle_in;
	while (angle_out >= 2 * PI)
	{
		angle_out = angle_out - 2 * PI;
	}
	while (angle_out <= -2 * PI)
	{
		angle_out = angle_out + 2 * PI;
	}
	if (angle_out > 0)
	{
		return angle_out;
	}
	else
	{
		return angle_out + 2 * PI;
	}
}

float Tools::angleAddition(float angleA, float angleB)
{
	MatrixXd T_A = MatrixXd(2, 2);
	T_A << cos(angleA), -sin(angleA), 
		sin(angleA), cos(angleA);

	MatrixXd T_B = MatrixXd(2, 2);
	T_B << cos(angleB), -sin(angleB),
		sin(angleB), cos(angleB);

	MatrixXd T_Add = T_B * T_A;
	float angle = atan2(T_Add(1, 0), T_Add(0, 0));
	//angle = Tools::fixAngle(angle);
	return angle;
}


VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) 
{
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	if ((estimations.size() != ground_truth.size()) || (ground_truth.size() == 0))
	{
		cout << "RMSE求解输入的维度不对！";
		assert(false);
		return rmse;
	}

	VectorXd total_list(4);
	total_list << 0, 0, 0, 0;
	for (size_t i = 0; i < ground_truth.size(); i++)
	{
		VectorXd err_list(4);
		err_list = (estimations[i] - ground_truth[i]);
		err_list = err_list.cwiseProduct(err_list);
		total_list = total_list + err_list;
	}
	//	cout << total_list << endl;
	double length_vector = ground_truth.size();
	total_list /= length_vector;
	rmse = total_list.cwiseSqrt();

	return rmse;

}

