#include <uWS/uWS.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "path_planner.h"
#include "sensor_fusion.h"
#include "world.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main()
{
    uWS::Hub h;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    World world{};

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

    string line;
    while (getline(in_map_, line))
    {
        std::istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        world.map_waypoints_x.push_back(x);
        world.map_waypoints_y.push_back(y);
        world.map_waypoints_s.push_back(s);
        world.map_waypoints_dx.push_back(d_x);
        world.map_waypoints_dy.push_back(d_y);
    }
    PathPlanner planner{world};
    SensorFusion sensor_fusion{};

    h.onMessage([&planner, &sensor_fusion](uWS::WebSocket<uWS::SERVER> ws, char *data,
                                           size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2')
        {
            auto s = hasData(data);

            if (s != "")
            {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry")
                {
                    // j[1] is the data JSON object

                    // Main car's localization Data
                    CarState car{};
                    car.x = j[1]["x"];
                    car.y = j[1]["y"];
                    car.s = j[1]["s"];
                    car.d = j[1]["d"];
                    car.yaw = j[1]["yaw"];
                    car.v = j[1]["speed"];

                    // Previous path data given to the Planner
                    ControlTrajectory old_trajectory{};
                    old_trajectory.x = j[1]["previous_path_x"].get<std::vector<double>>();
                    old_trajectory.y = j[1]["previous_path_y"].get<std::vector<double>>();
                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side
                    //   of the road.
                    auto sensor_fusion_raw = j[1]["sensor_fusion"];
                    sensor_fusion.Update(sensor_fusion_raw);
                    const auto object_in_front{sensor_fusion.GetObjectInFront(car)};

                    json msgJson;

                    const Lane lane{car.d};
                    double target_v{49.5};
                    if (object_in_front.has_value())
                    {
                        target_v = object_in_front.value().GetV() * 2.24 - 5.;
                    }
                    const auto trajectory{
                        planner.GetControlTrajectory(old_trajectory, car, lane, target_v)};

                    msgJson["next_x"] = trajectory.x;
                    msgJson["next_y"] = trajectory.y;

                    auto msg = "42[\"control\"," + msgJson.dump() + "]";

                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }  // end "telemetry" if
            }
            else
            {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }  // end websocket if
    });    // end h.onMessage

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
