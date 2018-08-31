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
		
		//���κνǶ�������0-2Pi��Χ��
		static float fixAngle(float angle_in);
		//�Ƕȼӷ�
		static float angleAddition(float angleA, float angleB);

		//RMSE���㺯��
		VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

};

#endif /* TOOLS_H_ */
