#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include <Eigen/Dense>

class MeasurementPackage {
public:
  long long timestamp_;

  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  Eigen::VectorXd raw_measurements_;

  //要获取观测，只能使用这个函数，因为其中的角度是经过修正的
  Eigen::VectorXd Get_raw_measurements();
  //具有打印测量的功能
  void Printer();
};

#endif /* MEASUREMENT_PACKAGE_H_ */
