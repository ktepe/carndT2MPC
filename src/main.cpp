#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

//switch debugging switch
#define ket_debug false
#define ket_debug_ false

// for convenience
using json = nlohmann::json;

//ket
using namespace std;


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
					double throttle=j[1]["throttle"];
					double steering_angle=j[1]["steering_angle"];
          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

					// ket, get the waypoints in car coordinate system, vehice is at (0,0)
					//vector<double> x_waypoints;
					//vector<double> y_waypoints;
					
					Eigen::VectorXd x_waypoints(ptsx.size());
					Eigen::VectorXd y_waypoints(ptsx.size());
					
					
					for (int i=0; i< ptsx.size(); i++){					
						double delta_x=ptsx[i]-px;
						double delta_y=ptsy[i]-py;						
						//x and y trnasitins of position of the car in coordiate system
						x_waypoints(i)=delta_x*cos(-psi)-delta_y*sin(-psi);
						y_waypoints(i)=delta_x*sin(-psi)+delta_y*cos(-psi);
					}
					
#if ket_debug         
					for (int i=0; i< ptsx.size(); i++) {
						cout << ptsx[i] << endl;
					}
#endif
					Eigen::VectorXd coeffs;
					
					coeffs=polyfit(x_waypoints, y_waypoints, 3);
#if ket_debug
					cout << coeffs << endl;
#endif
					double cte= polyeval(coeffs,0);
					double epsi=-atan(coeffs[1]);
					
					//construt the state vector
					Eigen::VectorXd state_vec(6);
					//calculate what would the predicted state in 0.1 sec. to compensate the delay
					double dt=0.1;
					double x_pre=v*dt;
					double y_pre=0;
					double psi_pre= v * -(steering_angle / 2.67) * dt;
					double v_pre=v+v*throttle*dt;
					double cte_pre = cte + v * sin(epsi) * dt;
					double epsi_pre= epsi + v * -(steering_angle / 2.67) * dt;	
						
//					state_vec << 0, 0, 0, v, cte, epsi;
					state_vec << x_pre, y_pre, psi_pre, v_pre, cte_pre, epsi_pre; 

#if ket_debug
					cout<< "before solver " << state_vec.size() << endl;
#endif
					
					//vector<double> actuation_vec=mpc.Solve(state_vec,coeffs);
					auto mpc_vec=mpc.Solve(state_vec,coeffs);
#if ket_debug
					cout<< "actuation vector " << mpc_vec[0] << " " << mpc_vec[1] << " " << mpc_vec.size()<<endl;
#endif

					double steer_value=mpc_vec[0];					
          double throttle_value=mpc_vec[1];
          
          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = -1.0*steer_value/deg2rad(25);
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;


          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line


					for (int i=2; i<mpc_vec.size()-4; i+=2){
						mpc_x_vals.push_back(mpc_vec[i]);
						mpc_y_vals.push_back(mpc_vec[i+1]);
					}

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line



					for (int i=0; i< ptsx.size(); i++){					
							//x and y trnasitins of position of the car in coordiate system
						next_x_vals.push_back(x_waypoints(i));
						
						next_y_vals.push_back(y_waypoints(i));
						}




          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
