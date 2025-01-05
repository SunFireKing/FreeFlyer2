#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

// Refer to: shazraz/Extended-Kalman-Filter
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include <math.h>
#include "FusionEKF.h"
#include "tools.h"

using namespace std;

// for convenience
using json = sunfireking::json;

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
  FusionEKF fusionEKF;

  // used to compute the RMSE later
  Tools tools;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  h.onMessage([&fusionEKF,&tools,&estimations,&ground_truth](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
    	  fusionEKF.ProcessMeasurement(meas_package);    	  

    	  //Push the current estimated x,y positon from the Kalman filter's state vector
	  
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
	  
    	  VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);

          json msgJson;
          msgJson["estimate_x"] = p_x;
          msgJson["estimate_y"] = p_y;
          msgJson["rmse_x"] =  RMSE(0);
          msgJson["rmse_y"] =  RMSE(1);
          msgJson["rmse_vx"] = RMSE(2);
          msgJson["rmse_vy"] = RMSE(3);
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
    ws.close();
    std::cout << "Disconnected" << std::endl;
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

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  //Initalize the F, P & Q matrices. 
  ekf_.F_ = MatrixXd(4,4);
  ekf_.P_ = MatrixXd(4,4);
  ekf_.Q_ = MatrixXd(4,4);
  
  //Assign the values for the laser sensor matrix, H_laser
  H_laser_ << 1,0,0,0,
	     0,1,0,0;
  
  //Set the values for acceleration noise
  noise_ax = 9;
  noise_ay = 9; 
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
   
    // first measurement
    cout << "EKF: \n";
    //cout << "Initializing state vector\n";
    ekf_.x_ = VectorXd(4);
    //cout << "Setting intial state...\n";
    ekf_.x_ << 1, 1, 0.5, 0.5; //remember to tweak the velocity values to adjust the RMSE in the beginning

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

      float rho = measurement_pack.raw_measurements_[0];
      float theta = measurement_pack.raw_measurements_[1];
      float rho_dot = measurement_pack.raw_measurements_[2];  
      //cout << "Radar measurement - extracting coordinates\n";
      ekf_.x_(0) = rho*cos(theta);
      ekf_.x_(1) = rho*sin(theta);
      //cout << "State" << ekf_.x_;
       
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      //cout << "Laser measurement - extracting coordinates.\n";
      float px = measurement_pack.raw_measurements_[0];
      float py = measurement_pack.raw_measurements_[1];
      
      ekf_.x_(0) = px;
      ekf_.x_(1) = py;
      //cout << "Initial state is: \n";
      //cout << ekf_.x_ << "\n";
    }
    //Capture the timestamp
    previous_timestamp_ = measurement_pack.timestamp_;
    //assign initial values to the state transition matrix, F
    ekf_.F_ << 1,0,1,0,
	       0,1,0,1,
	       0,0,1,0,
	       0,0,0,1;    
    //assign initial values to the covariance matrix, P. Adjust the variance values to reflect uncertainty in initial state
    ekf_.P_ << 1,0,0,0,
	       0,1,0,0,
	       0,0,500,0,
	       0,0,0,500;    

    // done initializing, no need to predict or update
    is_initialized_ = true;
    cout << "Completed initialization of FusionEKF.\n";
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  
  //Calculate deltaT
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  float dt2 = dt*dt;
  float dt3 = dt2*dt;
  float dt4 = dt3*dt;

  //Save the current timestamp for use in the next predict cycle
  previous_timestamp_ = measurement_pack.timestamp_;  

  //Update the state transition matrix, F
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;

  //Set the process covariance matrix, Q
  ekf_.Q_ << dt4/4*noise_ax, 0, dt3/2*noise_ax, 0,
	     0, dt4/4*noise_ay, 0, dt3/2*noise_ay,
	     dt3/2*noise_ax, 0, dt2*noise_ax, 0,
	     0, dt3/2*noise_ay, 0, dt2*noise_ay;
  //cout << "Process noise, Q is: \n";
  //cout << ekf_.Q_;
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    //Calculate the Jacobian matrix about the current predicted state and set the EKF state transition matrix, H
    //Tools Jacobian;
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    //cout << "Jacobian is:\n";
    //cout << Hj_;
    //Initialize the EKF object measurement covariance matrix, R, to the right size and assign the correct values
    ekf_.R_ = MatrixXd(3,3);
    ekf_.R_ = R_radar_; 
    ekf_.UpdateEKF(measurement_pack.raw_measurements_); //Comment this line to turn off radar updates   

  } else {
    //Set the EKF object to use the LASER sensor matrix, H
    ekf_.H_ = H_laser_;
    //Initialize the EKF object measurement covariance matrix, R, to the right size and assign the correct values
    ekf_.R_ = MatrixXd(2,2);
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_); //Comment this line to turn off LIDAR updates   
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
