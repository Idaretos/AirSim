// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include <iostream>
#include <chrono>

enum CommandType {
    Mono,
    Path,
    Crash,
    Random,
    Up,
    Half
};
/**
 * @brief
 * 
 * -n : number of drones
 * -m : number of drones to be controlled by the user
 * -t : command type
 * -i : iteration
*/
int main(int argc, char** argv)
{
    int n = 4;
    int m = 4;
    std::string t_s = "mono";
    int iter = 5;

    for (int j = 1; j < argc; j++) {
        if (std::string(argv[j]) == "-n") {
            n = std::stoi(argv[j + 1]);
        }
        else if (std::string(argv[j]) == "-m") {
            m = std::stoi(argv[j + 1]);
        }
        else if (std::string(argv[j]) == "-t") {
            t_s = argv[j + 1];
        }
        else if (std::string(argv[j]) == "-i") {
            iter = std::stoi(argv[j + 1]);
        }
    }

    int t = 0;
    if (t_s == "mono") {
        t = CommandType::Mono;
    }
    else if (t_s == "multi") {
        t = 1;
    }
    else if (t_s == "circle") {
        t = 2;
    }
    else if (t_s == "square") {
        t = 3;
    }
    else if (t_s == "random") {
        t = 4;
    }
    
    using namespace msr::airlib;
    std::string ip_addr = "147.46.125.127";

    msr::airlib::MultirotorRpcLibClient client(ip_addr);
    typedef ImageCaptureBase::ImageRequest ImageRequest;
    typedef ImageCaptureBase::ImageResponse ImageResponse;
    typedef ImageCaptureBase::ImageType ImageType;
    typedef common_utils::FileSystem FileSystem;

    try {
        client.confirmConnection();

        client.reset();
        
        std::vector<std::string> drone_names;
        for (int j = 0; j < n; j++) {
            drone_names.push_back("Drone" + std::to_string(j));
        }

        for (auto drone_name : drone_names) {
            client.enableApiControl(true, drone_name);
            client.armDisarm(true, drone_name);
        }

        std::this_thread::sleep_for(std::chrono::duration<double>(1));

        for (auto drone_name : drone_names) {
            client.takeoffAsync(5, drone_name);
        }

        std::this_thread::sleep_for(std::chrono::duration<double>(2));

        std::vector<msr::airlib::Vector3r> path;

        switch (t) {
            case CommandType::Mono:
                for (int j = 0; j < iter; j++){
                    for (auto drone_name : drone_names) {
                        client.moveToPositionAsync(0, 0, -7, 3, 3.4028235E38F, msr::airlib::DrivetrainType::MaxDegreeOfFreedom,msr::airlib::YawMode(),-1.0F, 1.0F, drone_name);
                    }
                    std::this_thread::sleep_for(std::chrono::duration<double>(2));
                    for (auto drone_name : drone_names) {
                        client.moveToPositionAsync(0, 0, -11, 3, 3.4028235E38F, msr::airlib::DrivetrainType::MaxDegreeOfFreedom,msr::airlib::YawMode(),-1.0F, 1.0F, drone_name);
                    }
                    std::this_thread::sleep_for(std::chrono::duration<double>(2));
                    for (auto drone_name : drone_names) {
                        client.moveToPositionAsync(0, 0, -15, 3, 3.4028235E38F, msr::airlib::DrivetrainType::MaxDegreeOfFreedom,msr::airlib::YawMode(),-1.0F, 1.0F, drone_name);
                    }
                    std::this_thread::sleep_for(std::chrono::duration<double>(2));
                    for (auto drone_name : drone_names) {
                        client.moveToPositionAsync(0, 0, -11, 3, 3.4028235E38F, msr::airlib::DrivetrainType::MaxDegreeOfFreedom,msr::airlib::YawMode(),-1.0F, 1.0F, drone_name);
                    }
                    std::this_thread::sleep_for(std::chrono::duration<double>(2));
                }
                break;
            case CommandType::Path:
                for (int i = 0; i < iter; i++) {
                    path.push_back(msr::airlib::Vector3r(0, 0, -7));
                    path.push_back(msr::airlib::Vector3r(0, 0, -11));
                    path.push_back(msr::airlib::Vector3r(0, 0, -15));
                    path.push_back(msr::airlib::Vector3r(0, 0, -11));
                }
                for (auto drone_name : drone_names)
                    client.moveOnPathAsync(path, 3, 3.4028235E38F, msr::airlib::DrivetrainType::MaxDegreeOfFreedom, msr::airlib::YawMode(), -1.0F, 1.0F, drone_name);
                break;
            case CommandType::Crash:
                client.moveByVelocityAsync(1, 0, 0, 5);
                break;
            case CommandType::Random:
                client.moveByVelocityAsync(1, 0, 0, 5);
                break;
            case CommandType::Up:
                client.moveToPositionAsync(0, 0, -7, 3, 3.4028235E38F, msr::airlib::DrivetrainType::MaxDegreeOfFreedom,msr::airlib::YawMode(),-1.0F, 1.0F);
                break;
            case CommandType::Half:
                // Do the same thing with mono, but only even drones will move first, and after a second odd drones will move
                for (int j = 0; j < iter; j++){
                    for (int k = 0; k < n; k+=2) {
                        client.moveToPositionAsync(0, 0, -7, 3, 3.4028235E38F, msr::airlib::DrivetrainType::MaxDegreeOfFreedom,msr::airlib::YawMode(),-1.0F, 1.0F, drone_names[k]);
                    }
                    std::this_thread::sleep_for(std::chrono::duration<double>(1));
                    for (int k = 1; k < n; k+=2) {
                            client.moveToPositionAsync(0, 0, -7, 3, 3.4028235E38F, msr::airlib::DrivetrainType::MaxDegreeOfFreedom,msr::airlib::YawMode(),-1.0F, 1.0F, drone_names[k]);
                    }
                    std::this_thread::sleep_for(std::chrono::duration<double>(1));
                    for (int k = 0; k < n; k++) {
                        if (k % 2 == 0) {
                            client.moveToPositionAsync(0, 0, -11, 3, 3.4028235E38F, msr::airlib::DrivetrainType::MaxDegreeOfFreedom,msr::airlib::YawMode(),-1.0F, 1.0F, drone_names[k]);
                        }
                    }
                    std::this_thread::sleep_for(std::chrono::duration<double>(1));
                    for (int k = 0; k < n; k++) {
                        if (k % 2 == 1) {
                            client.moveToPositionAsync(0, 0, -11, 3, 3.4028235E38F, msr::airlib::DrivetrainType::MaxDegreeOfFreedom,msr::airlib::YawMode(),-1.0F, 1.0F, drone_names[k]);
                        }
                    }
                    std::this_thread::sleep_for(std::chrono::duration<double>(1));
                    for (int k = 0; k < n; k++) {
                        if (k % 2 == 0) {
                            client.moveToPositionAsync(0, 0, -15, 3, 3.4028235E38F, msr::airlib::DrivetrainType::MaxDegreeOfFreedom,msr::airlib::YawMode(),-1.0F, 1.0F, drone_names[k]);
                        }
                    }
                    std::this_thread::sleep_for(std::chrono::duration<double>(1));
                    for (int k = 0; k < n; k++) {
                        if (k % 2 == 1) {
                            client.moveToPositionAsync(0, 0, -15, 3, 3.4028235E38F, msr::airlib::DrivetrainType::MaxDegreeOfFreedom,msr::airlib::YawMode(),-1.0F, 1.0F, drone_names[k]);
                        }
                    }
                    std::this_thread::sleep_for(std::chrono::duration<double>(1));
                    for (int k = 0; k < n; k++) {
                        if (k % 2 == 0) {
                            client.moveToPositionAsync(0, 0, -11, 3, 3.4028235E38F, msr::airlib::DrivetrainType::MaxDegreeOfFreedom,msr::airlib::YawMode(),-1.0F, 1.0F, drone_names[k]);
                        }
                    }
                    std::this_thread::sleep_for(std::chrono::duration<double>(1));
                    for (int k = 0; k < n; k++) {
                        if (k % 2 == 1) {
                            client.moveToPositionAsync(0, 0, -11, 3, 3.4028235E38F, msr::airlib::DrivetrainType::MaxDegreeOfFreedom,msr::airlib::YawMode(),-1.0F, 1.0F, drone_names[k]);
                        }
                    }
                    std::this_thread::sleep_for(std::chrono::duration<double>(1));
                }
        }

        // std::cout << "Press Enter to get FPV image" << std::endl;
        // std::cin.get();
        // const std::vector<ImageRequest> request{ ImageRequest("0", ImageType::Scene), ImageRequest("1", ImageType::DepthPlanar, true) };
        // const std::vector<ImageResponse>& response = client.simGetImages(request);
        // std::cout << "# of images received: " << response.size() << std::endl;

        // if (!response.size()) {
        //     std::cout << "Enter path with ending separator to save images (leave empty for no save)" << std::endl;
        //     std::string path;
        //     std::getline(std::cin, path);

        //     for (const ImageResponse& image_info : response) {
        //         std::cout << "Image uint8 size: " << image_info.image_data_uint8.size() << std::endl;
        //         std::cout << "Image float size: " << image_info.image_data_float.size() << std::endl;

        //         if (path != "") {
        //             std::string file_path = FileSystem::combine(path, std::to_string(image_info.time_stamp));
        //             if (image_info.pixels_as_float) {
        //                 Utils::writePFMfile(image_info.image_data_float.data(), image_info.width, image_info.height, file_path + ".pfm");
        //             }
        //             else {
        //                 std::ofstream file(file_path + ".png", std::ios::binary);
        //                 file.write(reinterpret_cast<const char*>(image_info.image_data_uint8.data()), image_info.image_data_uint8.size());
        //                 file.close();
        //             }
        //         }
        //     }
        // }

        // std::cout << "Press Enter to arm the drone" << std::endl;
        // std::cin.get();

        // client.enableApiControl(true);
        // client.armDisarm(true);

        // auto barometer_data = client.getBarometerData();
        // std::cout << "Barometer data \n"
        //           << "barometer_data.time_stamp \t" << barometer_data.time_stamp << std::endl
        //           << "barometer_data.altitude \t" << barometer_data.altitude << std::endl
        //           << "barometer_data.pressure \t" << barometer_data.pressure << std::endl
        //           << "barometer_data.qnh \t" << barometer_data.qnh << std::endl;

        // auto imu_data = client.getImuData();
        // std::cout << "IMU data \n"
        //           << "imu_data.time_stamp \t" << imu_data.time_stamp << std::endl
        //           << "imu_data.orientation \t" << imu_data.orientation << std::endl
        //           << "imu_data.angular_velocity \t" << imu_data.angular_velocity << std::endl
        //           << "imu_data.linear_acceleration \t" << imu_data.linear_acceleration << std::endl;

        // auto gps_data = client.getGpsData();
        // std::cout << "GPS data \n"
        //           << "gps_data.time_stamp \t" << gps_data.time_stamp << std::endl
        //           << "gps_data.gnss.time_utc \t" << gps_data.gnss.time_utc << std::endl
        //           << "gps_data.gnss.geo_point \t" << gps_data.gnss.geo_point << std::endl
        //           << "gps_data.gnss.eph \t" << gps_data.gnss.eph << std::endl
        //           << "gps_data.gnss.epv \t" << gps_data.gnss.epv << std::endl
        //           << "gps_data.gnss.velocity \t" << gps_data.gnss.velocity << std::endl
        //           << "gps_data.gnss.fix_type \t" << gps_data.gnss.fix_type << std::endl;

        // auto magnetometer_data = client.getMagnetometerData();
        // std::cout << "Magnetometer data \n"
        //           << "magnetometer_data.time_stamp \t" << magnetometer_data.time_stamp << std::endl
        //           << "magnetometer_data.magnetic_field_body \t" << magnetometer_data.magnetic_field_body << std::endl;
        // // << "magnetometer_data.magnetic_field_covariance" << magnetometer_data.magnetic_field_covariance // not implemented in sensor

        // std::cout << "Press Enter to takeoff" << std::endl;
        // std::cin.get();
        // float takeoff_timeout = 5;
        // client.takeoffAsync(takeoff_timeout);

        // // switch to explicit hover mode so that this is the fall back when
        // // move* commands are finished.
        // std::this_thread::sleep_for(std::chrono::duration<double>(5));
        // client.hoverAsync();

        // std::cout << "Press Enter to fly in a 10m box pattern at 3 m/s velocity" << std::endl;
        // std::cin.get();
        // // moveByVelocityZ is an offboard operation, so we need to set offboard mode.
        // client.enableApiControl(true);

        // auto position = client.getMultirotorState().getPosition();
        // float z = position.z(); // current position (NED coordinate system).
        // constexpr float speed = 3.0f;
        // constexpr float size = 10.0f;
        // constexpr float duration = size / speed;
        // DrivetrainType drivetrain = DrivetrainType::ForwardOnly;
        // YawMode yaw_mode(true, 0);

        // std::cout << "moveByVelocityZ(" << speed << ", 0, " << z << "," << duration << ")" << std::endl;
        // client.moveByVelocityZAsync(speed, 0, z, duration, drivetrain, yaw_mode);
        // std::this_thread::sleep_for(std::chrono::duration<double>(duration));
        // std::cout << "moveByVelocityZ(0, " << speed << "," << z << "," << duration << ")" << std::endl;
        // client.moveByVelocityZAsync(0, speed, z, duration, drivetrain, yaw_mode);
        // std::this_thread::sleep_for(std::chrono::duration<double>(duration));
        // std::cout << "moveByVelocityZ(" << -speed << ", 0, " << z << "," << duration << ")" << std::endl;
        // client.moveByVelocityZAsync(-speed, 0, z, duration, drivetrain, yaw_mode);
        // std::this_thread::sleep_for(std::chrono::duration<double>(duration));
        // std::cout << "moveByVelocityZ(0, " << -speed << "," << z << "," << duration << ")" << std::endl;
        // client.moveByVelocityZAsync(0, -speed, z, duration, drivetrain, yaw_mode);
        // std::this_thread::sleep_for(std::chrono::duration<double>(duration));

        // client.hoverAsync();

        // std::cout << "Press Enter to land" << std::endl;
        // std::cin.get();
        // client.landAsync();

        // std::cout << "Press Enter to disarm" << std::endl;
        // std::cin.get();
        // client.armDisarm(false);
    }
    catch (rpc::rpc_error& e) {
        const auto msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl
                  << msg << std::endl;
    }

    return 0;
}
