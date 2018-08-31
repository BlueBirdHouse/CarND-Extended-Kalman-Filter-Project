
#include <uWS/uWS.h>
#include <iostream>
//#include <cmath>

//һ�ֿ��Խ�����ת��Ϊ���������紫��Ľṹ
#include "json.hpp"

//�Լ��ĳ����ļ�
#include "FusionEKF.h"
#include "tools.h"
#include "measurement_package.h"


using namespace std; 
using namespace uWS;

// for convenience
using json = nlohmann::json;

string hasData(string s) 
{
	/*
	�ж�һ���ַ���������û�б�־[����](JSON data)������оͽ�������ȡ���������û�оͷ���һ�����ַ�����
	*/
	//find�������Ҳ���ָ��ֵ������»᷵��npos,���ֵ��size_t�����ֵ
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("]");
	if (found_null != string::npos) 
	{
		return "";
	}
	else if (b1 != string::npos && b2 != string::npos) 
	{
		return s.substr(b1, b2 - b1 + 1);
	}
	return "";
}

int main()
{
  Hub h;

  // Create a Kalman Filter instance
  FusionEKF fusionEKF;

  // used to compute the RMSE later
  Tools tools;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  h.onMessage([&fusionEKF, &tools, &estimations, &ground_truth](WebSocket<SERVER> *ws, char *data, size_t length, OpCode opCode)
  {
	//[&fusionEKF, &tools, &estimations, &ground_truth]�������¼���ʹ�õ��ĳ������
	//ws ������Ϣ�Ķ˿�
	//data ���յ�����Ϣ
	//length ���յ�����Ϣ����
	//opCode
	// "42" at the start of the message means there's a websocket message event.
    // 4˵������Ϣ��
    // 2˵�����¼���

    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(std::string(data));
      if (s != "") 
	  {
      	
        auto j = json::parse(s);

        std::string event = j[0].get<std::string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          string sensor_measurment = j[1]["sensor_measurement"];
          
			//׼�����۲���Ϣ
          MeasurementPackage meas_package;
          istringstream iss(sensor_measurment);
    	  long long timestamp;

    	  // reads first element from the current line
    	  string sensor_type;
    	  iss >> sensor_type;

    	  if (sensor_type.compare("L") == 0) 
		  {
			  ////�����һ���Ǽ����״���Ϣ�Ͷ�����

			  //���ô���������
      	  		meas_package.sensor_type_ = MeasurementPackage::LASER;

				//���ò���ֵ
          		meas_package.raw_measurements_ = VectorXd(2);
          		float px;
      	  		float py;
          		iss >> px;
          		iss >> py;
          		meas_package.raw_measurements_ << px, py;

				//����ʱ���
          		iss >> timestamp;
          		meas_package.timestamp_ = timestamp;
          } else if (sensor_type.compare("R") == 0) 
		  {

      	  		meas_package.sensor_type_ = MeasurementPackage::RADAR;
          		meas_package.raw_measurements_ = VectorXd(3);
          		float ro;
      	  		float theta;
      	  		float ro_dot;
          		iss >> ro;
          		iss >> theta;
          		iss >> ro_dot;
          		meas_package.raw_measurements_ << ro,theta, ro_dot;
          		iss >> timestamp;
          		meas_package.timestamp_ = timestamp;
          }
		  //��ӡ�յ�����Ϣ
		  //meas_package.Printer();

		  //��ֵ��Ϣ
          float x_gt;
    	  float y_gt;
    	  float vx_gt;
    	  float vy_gt;
    	  iss >> x_gt;
    	  iss >> y_gt;
    	  iss >> vx_gt;
    	  iss >> vy_gt;
    	  VectorXd gt_values(4);
    	  gt_values(0) = x_gt;
    	  gt_values(1) = y_gt; 
    	  gt_values(2) = vx_gt;
    	  gt_values(3) = vy_gt;
    	  ground_truth.push_back(gt_values);
          
          //ִ�� ProcessMeasurment(meas_package) ������һ�ι۲�
    	  fusionEKF.ProcessMeasurement(meas_package);    	  

    	  //ȡ����һ�εĹ۲���Ϣ

    	  VectorXd estimate(4);

    	  double p_x = fusionEKF.ekf_.x_(0);
    	  double p_y = fusionEKF.ekf_.x_(1);
    	  double v1  = fusionEKF.ekf_.x_(2);
    	  double v2 = fusionEKF.ekf_.x_(3);

    	  estimate(0) = p_x;
    	  estimate(1) = p_y;
    	  estimate(2) = v1;
    	  estimate(3) = v2;
    	  
    	  estimations.push_back(estimate);

		  //������һ�ε�����
    	  VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);

		  //׼���ظ���Ϣ
          json msgJson;
          msgJson["estimate_x"] = p_x;
          msgJson["estimate_y"] = p_y;
          msgJson["rmse_x"] =  RMSE(0);
          msgJson["rmse_y"] =  RMSE(1);
          msgJson["rmse_vx"] = RMSE(2);
          msgJson["rmse_vy"] = RMSE(3);

          auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
	  
        }
      } else 
	  {
        
        std::string msg = "42[\"manual\",{}]";
        ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }

  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) 
  {
	  //����������Ӿ���ʾ�Ѿ�����
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](WebSocket<uWS::SERVER> *ws, int code, char *message, size_t length) 
  {
	//����Ͽ��ͶϿ�
    ws->close();
    std::cout << "Disconnected" << std::endl;
  });

  //��ض˿ڣ�IP��ַһ��Ҫ��
  int port = 4567;
  if (h.listen("127.0.0.1", port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  //���з���
  h.run();
}
