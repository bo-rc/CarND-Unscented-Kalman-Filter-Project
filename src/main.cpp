#include <uWS/uWS.h>
#include <iostream>
#include <vector>
#include "json.hpp"
#include <math.h>
#include "UKF_fusion.h"
#include "Lidar.h"
#include "Radar.h"
#include "tools.h"
#include <memory>
#include <fstream>
#include <string>

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("]");
    if (found_null != std::string::npos) {
	return "";
    }
    else if (b1 != std::string::npos && b2 != std::string::npos) {
	return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

int main()
{
    uWS::Hub h;

    // Create a Kalman Filter instance
    UKF_fusion ukf;
    // Create two sensors, a lidar and a radar
    vector<std::unique_ptr<Sensor>> sensors;
    sensors.push_back(std::unique_ptr<Sensor>(new Lidar()));
    sensors.push_back(std::unique_ptr<Sensor>(new Radar()));

    // used to compute the RMSE later
    Tools tools;
    vector<VectorXd> estimations;
    vector<VectorXd> ground_truth;

    // set parameters
    ifstream config_file("./parm.config", std::ios::in);
    string line;
    while (std::getline(config_file >> std::ws, line)) {
	stringstream buffer(line);
	vector<string> cont;
	string ele;
	while (buffer >> ele) {
	    cont.emplace_back(ele);
	}

	if (cont[0] == "std_a") {
	    ukf.set_std_a(stof(cont[1]));
	}

	if (cont[0] == "std_yawdd") {
	    ukf.set_std_yawdd(stof(cont[1]));
	}

	if (cont[0] == "disable_lidar") {
	    if (1 == stoi(cont[1]))
		sensors[0]->disable();
	}
	if (cont[0] == "disable_radar") {
	    if (1 == stoi(cont[1]))
		sensors[1]->disable();
	}
	if (cont[0] == "std_px") {
	    dynamic_cast<Lidar*>(sensors[0].get())->set_std_px(stof(cont[1]));
	}
	if (cont[0] == "std_py") {
	    dynamic_cast<Lidar*>(sensors[0].get())->set_std_py(stof(cont[1]));
	}
	if (cont[0] == "std_radr") {
	    dynamic_cast<Radar*>(sensors[1].get())->set_std_radr(stof(cont[1]));
	}
	if (cont[0] == "std_radphi") {
	    dynamic_cast<Radar*>(sensors[1].get())->set_std_radphi(stof(cont[1]));
	}
	if (cont[0] == "std_radrd") {
	    dynamic_cast<Radar*>(sensors[1].get())->set_std_radrd(stof(cont[1]));
	}
    }

    h.onMessage([&sensors, &ukf,&tools,&estimations,&ground_truth](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
	// "42" at the start of the message means there's a websocket message event.
	// The 4 signifies a websocket message
	// The 2 signifies a websocket event

	if (length && length > 2 && data[0] == '4' && data[1] == '2')
	{

	    auto s = hasData(std::string(data));
	    if (s != "") {

		auto j = json::parse(s);

		std::string event = j[0].get<std::string>();

		if (event == "telemetry") {
		    // j[1] is the data JSON object

		    string sensor_measurment = j[1]["sensor_measurement"];

		    MeasurementPackage meas_package;
		    istringstream iss(sensor_measurment);
		    long long timestamp;

		    // reads first element from the current line
		    string sensor_type;
		    iss >> sensor_type;

		    if (sensor_type.compare("L") == 0) {
			meas_package.sensor_type_ = MeasurementPackage::LASER;
			meas_package.raw_measurements_ = VectorXd(2);
			float px;
			float py;
			iss >> px;
			iss >> py;
			meas_package.raw_measurements_ << px, py;
			iss >> timestamp;
			meas_package.timestamp_ = timestamp;
		    } else if (sensor_type.compare("R") == 0) {

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

		    //Call ProcessMeasurment(meas_package) for Kalman filter
		    ukf.ProcessMeasurement(meas_package, sensors);
		    ukf.filter(sensors);

		    //Push the current estimated x,y positon from the Kalman filter's state vector

		    VectorXd estimate(4);

		    double px = ukf.x(0);
		    double py = ukf.x(1);
		    double v  = ukf.x(2);
		    double yaw = ukf.x(3);

		    double v1 = cos(yaw)*v;
		    double v2 = sin(yaw)*v;

		    estimate(0) = px;
		    estimate(1) = py;
		    estimate(2) = v1;
		    estimate(3) = v2;

		    estimations.push_back(estimate);

		    VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);

		    json msgJson;
		    msgJson["estimate_x"] = px;
		    msgJson["estimate_y"] = py;
		    msgJson["rmse_x"] =  RMSE(0);
		    msgJson["rmse_y"] =  RMSE(1);
		    msgJson["rmse_vx"] = RMSE(2);
		    msgJson["rmse_vy"] = RMSE(3);
		    // cout << "(x, y) = " << px << "," << py << endl;
		    auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
		    // std::cout << msg << std::endl;
		    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

		}
	    } else {

		std::string msg = "42[\"manual\",{}]";
		ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
	std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
	std::cout << "Disconnected" << std::endl;
	ws.close();
    });

    int port = 4567;
    if (h.listen(port))
    {
	std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
	std::cerr << "Failed to listen to port" << std::endl;
	return -1;
    }
    h.run();
}























































































