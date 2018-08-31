#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools 
{
	public:
		Tools();
		virtual ~Tools();
		
		//将任何角度修正到0-2Pi范围内
		static float fixAngle(float angle_in);
		//角度加法
		static float angleAddition(float angleA, float angleB);

		//RMSE计算函数
		VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

};

#endif /* TOOLS_H_ */
