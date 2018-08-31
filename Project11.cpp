
#include <uWS/uWS.h>
#include <iostream>
//#include <cmath>

//一种可以将数据转换为有利于网络传输的结构
#include "json.hpp"

//自己的程序文件
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
	判断一个字符串里面有没有标志[或者](JSON data)，如果有就将内容提取出来，如果没有就返回一个空字符串。
	*/
	//find函数在找不到指定值得情况下会返回npos,这个值是size_t的最大值
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
	//[&fusionEKF, &tools, &estimations, &ground_truth]将会在事件中使用到的程序变量
	//ws 发送信息的端口
	//data 接收到的信息
	//length 接收到的信息长度
	//opCode
	// "42" at the start of the message means there's a websocket message event.
    // 4说明来信息了
    // 2说明来事件了

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
          
			//准备填充观测信息
          MeasurementPackage meas_package;
          istringstream iss(sensor_measurment);
    	  long long timestamp;

    	  // reads first element from the current line
    	  string sensor_type;
    	  iss >> sensor_type;

    	  if (sensor_type.compare("L") == 0) 
		  {
			  ////如果这一次是激光雷达信息就读进来

			  //设置传感器类型
      	  		meas_package.sensor_type_ = MeasurementPackage::LASER;

				//设置测量值
          		meas_package.raw_measurements_ = VectorXd(2);
          		float px;
      	  		float py;
          		iss >> px;
          		iss >> py;
          		meas_package.raw_measurements_ << px, py;

				//设置时间戳
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
		  //打印收到的信息
		  //meas_package.Printer();

		  //真值信息
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
          
          //执行 ProcessMeasurment(meas_package) 处理这一次观测
    	  fusionEKF.ProcessMeasurement(meas_package);    	  

    	  //取出这一次的观测信息

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

		  //计算这一次的性能
    	  VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);

		  //准备回复信息
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
	  //如果发现链接就显示已经链接
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](WebSocket<uWS::SERVER> *ws, int code, char *message, size_t length) 
  {
	//如果断开就断开
    ws->close();
    std::cout << "Disconnected" << std::endl;
  });

  //监控端口，IP地址一定要对
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

  //运行服务
  h.run();
}
