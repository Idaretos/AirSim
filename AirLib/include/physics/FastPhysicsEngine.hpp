// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_FastPhysicsEngine_hpp
#define airsim_core_FastPhysicsEngine_hpp

#include "common/Common.hpp"
#include "physics/PhysicsEngineBase.hpp"
#include <iostream>
#include <sstream>
#include <fstream>
#include <memory>
#include "common/CommonStructs.hpp"
#include "common/SteppableClock.hpp"

#include "common/common_utils/StrictMode.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"

#include <cinttypes>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <regex>
#define LISTEN_PORT 41543   // Port to receive the forwarded message from
#define BUFFER_SIZE 8192    // Buffer size for message
// #define UPDATE_PERIOD 1.7E9   // Update period for the client

namespace msr
{
namespace airlib
{
    class FastPhysicsEngine : public PhysicsEngineBase
    {
    public:
        FastPhysicsEngine(bool enable_ground_lock = true, Vector3r wind = Vector3r::Zero())
            : enable_ground_lock_(enable_ground_lock), wind_(wind)
        {
            setName("FastPhysicsEngine");

            // std::FILE* file = std::fopen("/home/idaretos/Control_AirSim/log/connectionLog.txt", "w");
            // std::fclose(file);
            // file = std::fopen("/home/idaretos/Control_AirSim/log/receiveMessage.txt", "w");
            // std::fclose(file);

            std::string ip_addr = "147.46.125.127";
            uint16_t port = 41451;
            float timeout_sec = 60;
            // client = new MultirotorRpcLibClient(ip_addr, port, timeout_sec);
            // client->confirmConnection();
            // client->reset();

            // read UPDATE_PERIOD from customsettings.txt
            std::ifstream file("/home/idaretos/Control_AirSim/customsettings.txt");
            std::string line;
            std::string key = "UPDATE_PERIOD";
            std::string value;
            while (std::getline(file, line)) {
                if (line.find(key) != std::string::npos) {
                    std::istringstream iss(line);
                    iss >> key >> value;
                    break;
                }
            }
            file.close();
            UPDATE_PERIOD = std::stoul(value);
            // convert milliseconds to nanoseconds
            UPDATE_PERIOD *= 1E6;
            
            std::FILE* logfile = std::fopen("/home/idaretos/Control_AirSim/log/updatePeriod.txt", "w");
            std::fprintf(logfile, "Update period: %lu\n", UPDATE_PERIOD);
            std::fclose(logfile);
            logfile = std::fopen("/home/idaretos/Control_AirSim/log/physics.txt", "w");
            std::fclose(logfile);
        }

        virtual ~FastPhysicsEngine()
        {
            // delete client;
            // clear thread
            msg_receiver.~thread();
            close(client_sock);
            close(listen_sock);
        }

        //*** Start: UpdatableState implementation ***//
        virtual void resetImplementation() override
        {
            for (PhysicsBody* body_ptr : *this) {
                update_[body_ptr->vehicle_name] = false;
                initPhysicsBody(body_ptr);
            }

            // create a thread for receiving the message from the controller
            // std::thread t1(&FastPhysicsEngine::receiveMessage, this);
            // t1.detach();
            msg_receiver = std::thread(&FastPhysicsEngine::receiveMessage, this);
            msg_receiver.detach();
        }

        virtual void insert(PhysicsBody* body_ptr) override
        {
            PhysicsEngineBase::insert(body_ptr);

            update_[body_ptr->vehicle_name] = false;
            initPhysicsBody(body_ptr);
        }

        virtual void update() override
        {
            PhysicsEngineBase::update();

            for (PhysicsBody* body_ptr : *this) {
                updatePhysics(*body_ptr);
            }
        }
        virtual void reportState(StateReporter& reporter) override
        {
            for (PhysicsBody* body_ptr : *this) {
                reporter.writeValue("Phys", debug_string_.str());
                reporter.writeValue("Is Grounded", body_ptr->isGrounded());
                reporter.writeValue("Force (world)", body_ptr->getWrench().force);
                reporter.writeValue("Torque (body)", body_ptr->getWrench().torque);
            }
            //call base
            UpdatableObject::reportState(reporter);
        }
        //*** End: UpdatableState implementation ***//

        // Set Wind, for API and Settings implementation
        void setWind(const Vector3r& wind) override
        {
            wind_ = wind;
        }

    private:
        void initPhysicsBody(PhysicsBody* body_ptr)
        {
            body_ptr->last_kinematics_time = clock()->nowNanos();
            // client->enableApiControl(true, body_ptr->vehicle_name);
            // client->armDisarm(true, body_ptr->vehicle_name);
            // // client->takeoffAsync(5, body_ptr->vehicle_name);
            last_client_update_time_[body_ptr->vehicle_name] = clock()->nowNanos();
            // client_state_[body_ptr->vehicle_name] = 0;
        }

        void updatePhysics(PhysicsBody& body)
        {

            TTimeDelta dt = clock()->updateSince(body.last_kinematics_time);

            body.lock();
            //get current kinematics state of the body - this state existed since last dt seconds
            const Kinematics::State& current = body.getKinematics();

            Eigen::Vector3f vehicle_pos = Eigen::Vector3f::Zero();
            Vector3r vehicle_vel = Vector3r::Zero();
            if (update_[body.vehicle_name]) {
                mtx.lock(); 
                std::string vehicle_name = body.vehicle_name;

                auto it = real_pos1.find(body.vehicle_name);
                if (it != real_pos1.end()) {
                    update_[vehicle_name] = false;
                    vehicle_pos = real_pos1.at(vehicle_name);
                    vehicle_vel = real_vel.at(vehicle_name);
                    real_pos1.erase(vehicle_name);
                    real_vel.erase(vehicle_name);
                }
                mtx.unlock();
                const_cast<Kinematics::State&>(current).pose.position = vehicle_pos;
                body.kinematics_->last_updated_twist.linear = vehicle_vel;
                body.setGrounded(false);
            }
#if 0 
            if (clock()->nowNanos() - last_client_update_time_[body.vehicle_name] > UPDATE_PERIOD) {
                if (mtx.try_lock()) {
                // swap real_pos_ptr and tmp_pos_ptr
                    // if (pos1) {
                    //    pos1 = false;
                    // } else {
                    //    pos1 = true;
                    // }

                    std::FILE* trylockfile = std::fopen("/home/idaretos/Control_AirSim/log/trylock.txt", "a");
                    std::fprintf(trylockfile, "Try lock\n");
                    std::fclose(trylockfile);
                    mtx.unlock();
                }
                // find the vehicle's position in the real_pos_ptr
                std::string vehicle_name = body.vehicle_name;
                
                Eigen::Vector3f vehicle_pos = Eigen::Vector3f::Zero();
                Vector3r vehicle_vel = Vector3r::Zero();

                mtx.lock();
                // check if table length is 0
                if (pos1) {
                    // std::cout << "Table size: " << real_pos1.size() << std::endl;
                    if (real_pos1.size() > 0) {
                        auto it = real_pos1.find(vehicle_name);
                        if (it != real_pos1.end()) {
                            // printf("Vehicle: %s\n", vehicle_name.c_str());
                            last_client_update_time_[body.vehicle_name] = clock()->nowNanos();
                            vehicle_pos = real_pos1.at(vehicle_name);
                            vehicle_vel = real_vel.at(vehicle_name);
                            real_pos1.erase(vehicle_name);
                            real_vel.erase(vehicle_name);
                        }
                    }
                } else {
                    if (real_pos2.size() > 0) {
                        auto it = real_pos2.find(vehicle_name);
                        // if (real_pos_ptr->find(vehicle_name) != real_pos_ptr->end()) {
                        if (it != real_pos2.end()) {
                            last_client_update_time_[body.vehicle_name] = clock()->nowNanos();
                            vehicle_pos = real_pos2.at(vehicle_name);
                            real_pos2.erase(vehicle_name);
                        }

                        // overwrite the vehicle's position
                        // body.setPose(Pose(vehicle_pos, Quaternionr::Identity()));
                    }
                }
                mtx.unlock();
                const_cast<Kinematics::State&>(current).pose.position = vehicle_pos;
                // const_cast<Kinematics::State&>(current).twist.linear = vehicle_vel;
                body.kinematics_->last_updated_twist.linear = vehicle_vel;
                // printf("Vehicle: %s, Velocity: %f %f %f\n", vehicle_name.c_str(), vehicle_vel.x(), vehicle_vel.y(), vehicle_vel.z());
                body.setGrounded(false);
            }
#endif
            const_cast<Kinematics::State&>(current).twist.linear = body.kinematics_->last_updated_twist.linear;

            Kinematics::State next;
            Wrench next_wrench;

            //first compute the response as if there was no collision
            //this is necessary to take in to account forces and torques generated by body
            getNextKinematicsNoCollision(dt, body, current, next, next_wrench, wind_);

            //if there is collision, see if we need collision response
            const CollisionInfo collision_info = body.getCollisionInfo();
            CollisionResponse& collision_response = body.getCollisionResponseInfo();
            //if collision was already responded then do not respond to it until we get updated information
            if (body.isGrounded() || (collision_info.has_collided && collision_response.collision_time_stamp != collision_info.time_stamp)) {
                bool is_collision_response = getNextKinematicsOnCollision(dt, collision_info, body, current, next, next_wrench, enable_ground_lock_);
                updateCollisionResponseInfo(collision_info, next, is_collision_response, collision_response);
                //throttledLogOutput("*** has collision", 0.1);
            }
            //else throttledLogOutput("*** no collision", 0.1);

            //Utils::log(Utils::stringf("T-VEL %s %" PRIu64 ": ",
            //    VectorMath::toString(next.twist.linear).c_str(), clock()->getStepCount()));

            body.setWrench(next_wrench);
            body.updateKinematics(next);
            body.unlock();

            //TODO: this is now being done in PawnSimApi::update. We need to re-think this sequence
            //with below commented out - Arducopter GPS may not work.
            //body.getEnvironment().setPosition(next.pose.position);
            //body.getEnvironment().update();
            // if (clock()->nowNanos() - last_client_update_time_[body.vehicle_name] > 2E9) {
            //     if (client_state_[body.vehicle_name] == 0) {
            //         client->moveToPositionAsync(0, 0, -7, 3, 3.4028235E38F, msr::airlib::DrivetrainType::MaxDegreeOfFreedom,msr::airlib::YawMode(),-1.0F, 1.0F, body.vehicle_name);
            //         client_state_[body.vehicle_name] = 1;
            //     } else if (client_state_[body.vehicle_name] == 1) {
            //         client->moveToPositionAsync(0, 1, -11, 3, 3.4028235E38F, msr::airlib::DrivetrainType::MaxDegreeOfFreedom,msr::airlib::YawMode(),-1.0F, 1.0F, body.vehicle_name);
            //         client_state_[body.vehicle_name] = 2;
            //     } else if (client_state_[body.vehicle_name] == 2) {
            //         client->moveToPositionAsync(0, 0, -15, 3, 3.4028235E38F, msr::airlib::DrivetrainType::MaxDegreeOfFreedom,msr::airlib::YawMode(),-1.0F, 1.0F, body.vehicle_name);
            //         client_state_[body.vehicle_name] = 3;
            //     } else if (client_state_[body.vehicle_name] == 3) {
            //         client->moveToPositionAsync(0, -1, -11, 3, 3.4028235E38F, msr::airlib::DrivetrainType::MaxDegreeOfFreedom,msr::airlib::YawMode(),-1.0F, 1.0F, body.vehicle_name);
            //         client_state_[body.vehicle_name] = 0;
            //     } else {
            //         client_state_[body.vehicle_name] = 0;
            //     }
            //     std::cout << body.vehicle_name << " moving to position" << std::endl;
            //     last_client_update_time_[body.vehicle_name] = clock()->nowNanos();
            // }

        }

        static void updateCollisionResponseInfo(const CollisionInfo& collision_info, const Kinematics::State& next,
                                                bool is_collision_response, CollisionResponse& collision_response)
        {
            collision_response.collision_time_stamp = collision_info.time_stamp;
            ++collision_response.collision_count_raw;

            //increment counter if we didn't collided with high velocity (like resting on ground)
            if (is_collision_response && next.twist.linear.squaredNorm() > kRestingVelocityMax * kRestingVelocityMax)
                ++collision_response.collision_count_non_resting;
        }

        //return value indicates if collision response was generated
        static bool getNextKinematicsOnCollision(TTimeDelta dt, const CollisionInfo& collision_info, PhysicsBody& body,
                                                 const Kinematics::State& current, Kinematics::State& next, Wrench& next_wrench, bool enable_ground_lock)
        {
            /************************* Collision response ************************/
            const real_T dt_real = static_cast<real_T>(dt);

            //are we going away from collision? if so then keep using computed next state
            if (collision_info.normal.dot(next.twist.linear) >= 0.0f)
                return false;

            /********** Core collision response ***********/
            //get avg current velocity
            const Vector3r vcur_avg = current.twist.linear + current.accelerations.linear * dt_real;

            //get average angular velocity
            const Vector3r angular_avg = current.twist.angular + current.accelerations.angular * dt_real;

            //contact point vector
            Vector3r r = collision_info.impact_point - collision_info.position;

            //see if impact is straight at body's surface (assuming its box)
            const Vector3r normal_body = VectorMath::transformToBodyFrame(collision_info.normal, current.pose.orientation);
            const bool is_ground_normal = Utils::isApproximatelyEqual(std::abs(normal_body.z()), 1.0f, kAxisTolerance);
            bool ground_collision = false;
            const float z_vel = vcur_avg.z();
            const bool is_landing = z_vel > std::abs(vcur_avg.x()) && z_vel > std::abs(vcur_avg.y());

            real_T restitution = body.getRestitution();
            real_T friction = body.getFriction();

            if (is_ground_normal && is_landing
                // So normal_body is the collision normal translated into body coords, why does an x==1 or y==1
                // mean we are coliding with the ground???
                // || Utils::isApproximatelyEqual(std::abs(normal_body.x()), 1.0f, kAxisTolerance)
                // || Utils::isApproximatelyEqual(std::abs(normal_body.y()), 1.0f, kAxisTolerance)
            ) {
                // looks like we are coliding with the ground.  We don't want the ground to be so bouncy
                // so we reduce the coefficient of restitution.  0 means no bounce.
                // TODO: it would be better if we did this based on the material we are landing on.
                // e.g. grass should be inelastic, but a hard surface like the road should be more bouncy.
                restitution = 0;
                // crank up friction with the ground so it doesn't try and slide across the ground
                // again, this should depend on the type of surface we are landing on.
                friction = 1;

                //we have collided with ground straight on, we will fix orientation later
                ground_collision = is_ground_normal;
            }

            //velocity at contact point
            const Vector3r vcur_avg_body = VectorMath::transformToBodyFrame(vcur_avg, current.pose.orientation);
            const Vector3r contact_vel_body = vcur_avg_body + angular_avg.cross(r);

            /*
            GafferOnGames - Collision response with columb friction
            http://gafferongames.com/virtual-go/collision-response-and-coulomb-friction/
            Assuming collision is with static fixed body,
            impulse magnitude = j = -(1 + R)V.N / (1/m + (I'(r X N) X r).N)
            Physics Part 3, Collision Response, Chris Hecker, eq 4(a)
            http://chrishecker.com/images/e/e7/Gdmphys3.pdf
            V(t+1) = V(t) + j*N / m
        */
            const real_T impulse_mag_denom = 1.0f / body.getMass() +
                                             (body.getInertiaInv() * r.cross(normal_body))
                                                 .cross(r)
                                                 .dot(normal_body);
            const real_T impulse_mag = -contact_vel_body.dot(normal_body) * (1 + restitution) / impulse_mag_denom;

            next.twist.linear = vcur_avg + collision_info.normal * (impulse_mag / body.getMass());
            next.twist.angular = angular_avg + r.cross(normal_body) * impulse_mag;

            //above would modify component in direction of normal
            //we will use friction to modify component in direction of tangent
            const Vector3r contact_tang_body = contact_vel_body - normal_body * normal_body.dot(contact_vel_body);
            const Vector3r contact_tang_unit_body = contact_tang_body.normalized();
            const real_T friction_mag_denom = 1.0f / body.getMass() +
                                              (body.getInertiaInv() * r.cross(contact_tang_unit_body))
                                                  .cross(r)
                                                  .dot(contact_tang_unit_body);
            const real_T friction_mag = -contact_tang_body.norm() * friction / friction_mag_denom;

            const Vector3r contact_tang_unit = VectorMath::transformToWorldFrame(contact_tang_unit_body, current.pose.orientation);
            next.twist.linear += contact_tang_unit * friction_mag;
            next.twist.angular += r.cross(contact_tang_unit_body) * (friction_mag / body.getMass());

            //TODO: implement better rolling friction
            next.twist.angular *= 0.9f;

            // there is no acceleration during collision response, this is a hack, but without it the acceleration cancels
            // the computed impulse response too much and stops the vehicle from bouncing off the collided object.
            next.accelerations.linear = Vector3r::Zero();
            next.accelerations.angular = Vector3r::Zero();

            next.pose = current.pose;
            if (enable_ground_lock && ground_collision) {
                float pitch, roll, yaw;
                VectorMath::toEulerianAngle(next.pose.orientation, pitch, roll, yaw);
                pitch = roll = 0;
                next.pose.orientation = VectorMath::toQuaternion(pitch, roll, yaw);

                //there is a lot of random angular velocity when vehicle is on the ground
                next.twist.angular = Vector3r::Zero();

                // also eliminate any linear velocity due to twist - since we are sitting on the ground there shouldn't be any.
                next.twist.linear = Vector3r::Zero();
                next.pose.position = collision_info.position;
                body.setGrounded(true);

                // but we do want to "feel" the ground when we hit it (we should see a small z-acc bump)
                // equal and opposite our downward velocity.
                next.accelerations.linear = -0.5f * body.getMass() * vcur_avg;

                //throttledLogOutput("*** Triggering ground lock", 0.1);
            }
            else {
                //else keep the orientation
                next.pose.position = collision_info.position + (collision_info.normal * collision_info.penetration_depth) + next.twist.linear * (dt_real * kCollisionResponseCycles);
            }
            next_wrench = Wrench::zero();

            //Utils::log(Utils::stringf("*** C-VEL %s: ", VectorMath::toString(next.twist.linear).c_str()));

            return true;
        }

        void throttledLogOutput(const std::string& msg, double seconds)
        {
            TTimeDelta dt = clock()->elapsedSince(last_message_time);
            const real_T dt_real = static_cast<real_T>(dt);
            if (dt_real > seconds) {
                Utils::log(msg);
                last_message_time = clock()->nowNanos();
            }
        }

        static Wrench getDragWrench(const PhysicsBody& body, const Quaternionr& orientation,
                                    const Vector3r& linear_vel, const Vector3r& angular_vel_body, const Vector3r& wind_world)
        {
            //add linear drag due to velocity we had since last dt seconds + wind
            //drag vector magnitude is proportional to v^2, direction opposite of velocity
            //total drag is b*v + c*v*v but we ignore the first term as b << c (pg 44, Classical Mechanics, John Taylor)
            //To find the drag force, we find the magnitude in the body frame and unit vector direction in world frame
            //http://physics.stackexchange.com/questions/304742/angular-drag-on-body
            //similarly calculate angular drag
            //note that angular velocity, acceleration, torque are already in body frame

            Wrench wrench = Wrench::zero();
            const real_T air_density = body.getEnvironment().getState().air_density;

            // Use relative velocity of the body wrt wind
            const Vector3r relative_vel = linear_vel - wind_world;
            const Vector3r linear_vel_body = VectorMath::transformToBodyFrame(relative_vel, orientation);

            for (uint vi = 0; vi < body.dragVertexCount(); ++vi) {
                const auto& vertex = body.getDragVertex(vi);
                const Vector3r vel_vertex = linear_vel_body + angular_vel_body.cross(vertex.getPosition());
                const real_T vel_comp = vertex.getNormal().dot(vel_vertex);
                //if vel_comp is -ve then we cull the face. If velocity too low then drag is not generated
                if (vel_comp > kDragMinVelocity) {
                    const Vector3r drag_force = vertex.getNormal() * (-vertex.getDragFactor() * air_density * vel_comp * vel_comp);
                    const Vector3r drag_torque = vertex.getPosition().cross(drag_force);

                    wrench.force += drag_force;
                    wrench.torque += drag_torque;
                }
            }

            //convert force to world frame, leave torque to local frame
            wrench.force = VectorMath::transformToWorldFrame(wrench.force, orientation);

            return wrench;
        }

        static Wrench getBodyWrench(const PhysicsBody& body, const Quaternionr& orientation)
        {
            //set wrench sum to zero
            Wrench wrench = Wrench::zero();

            //calculate total force on rigid body's center of gravity
            for (uint i = 0; i < body.wrenchVertexCount(); ++i) {
                //aggregate total
                const PhysicsBodyVertex& vertex = body.getWrenchVertex(i);
                const auto& vertex_wrench = vertex.getWrench();
                wrench += vertex_wrench;

                //add additional torque due to force applies farther than COG
                // tau = r X F
                wrench.torque += vertex.getPosition().cross(vertex_wrench.force);
            }

            //convert force to world frame, leave torque to local frame
            wrench.force = VectorMath::transformToWorldFrame(wrench.force, orientation);

            return wrench;
        }

        void getNextKinematicsNoCollision(TTimeDelta dt, PhysicsBody& body, const Kinematics::State& current,
                                                 Kinematics::State& next, Wrench& next_wrench, const Vector3r& wind)
        {
            const real_T dt_real = static_cast<real_T>(dt);

            Vector3r avg_linear = Vector3r::Zero();
            Vector3r avg_angular = Vector3r::Zero();

            /************************* Get force and torque acting on body ************************/
            //set wrench sum to zero
            const Wrench body_wrench = getBodyWrench(body, current.pose.orientation);

            if (body.isGrounded()) {
                // make it stick to the ground until the magnitude of net external force on body exceeds its weight.
                float external_force_magnitude = body_wrench.force.squaredNorm();
                Vector3r weight = body.getMass() * body.getEnvironment().getState().gravity;
                float weight_magnitude = weight.squaredNorm();
                if (external_force_magnitude >= weight_magnitude) {
                    //throttledLogOutput("*** Losing ground lock due to body_wrench " + VectorMath::toString(body_wrench.force), 0.1);
                    body.setGrounded(false);
                }
                next_wrench.force = Vector3r::Zero();
                next_wrench.torque = Vector3r::Zero();
                next.accelerations.linear = Vector3r::Zero();
            }
            else {
                //add linear drag due to velocity we had since last dt seconds + wind
                //drag vector magnitude is proportional to v^2, direction opposite of velocity
                //total drag is b*v + c*v*v but we ignore the first term as b << c (pg 44, Classical Mechanics, John Taylor)
                //To find the drag force, we find the magnitude in the body frame and unit vector direction in world frame
                avg_linear = current.twist.linear + current.accelerations.linear * (0.5f * dt_real);
                avg_angular = current.twist.angular + current.accelerations.angular * (0.5f * dt_real);
                const Wrench drag_wrench = getDragWrench(body, current.pose.orientation, avg_linear, avg_angular, wind);

                next_wrench = body_wrench + drag_wrench;

                //Utils::log(Utils::stringf("B-WRN %s: ", VectorMath::toString(body_wrench.force).c_str()));
                //Utils::log(Utils::stringf("D-WRN %s: ", VectorMath::toString(drag_wrench.force).c_str()));

                /************************* Update accelerations due to force and torque ************************/
                //get new acceleration due to force - we'll use this acceleration in next time step

                next.accelerations.linear = (next_wrench.force / body.getMass()) + body.getEnvironment().getState().gravity;
            }

            if (body.isGrounded()) {
                // this stops vehicle from vibrating while it is on the ground doing nothing.
                next.accelerations.angular = Vector3r::Zero();
                next.twist.linear = Vector3r::Zero();
                next.twist.angular = Vector3r::Zero();
            }
            else {
                //get new angular acceleration
                //Euler's rotation equation: https://en.wikipedia.org/wiki/Euler's_equations_(body_dynamics)
                //we will use torque to find out the angular acceleration
                //angular momentum L = I * omega
                const Vector3r angular_momentum = body.getInertia() * avg_angular;
                const Vector3r angular_momentum_rate = next_wrench.torque - avg_angular.cross(angular_momentum);
                //new angular acceleration - we'll use this acceleration in next time step
                next.accelerations.angular = body.getInertiaInv() * angular_momentum_rate;

                /************************* Update pose and twist after dt ************************/
                //Verlet integration: http://www.physics.udel.edu/~bnikolic/teaching/phys660/numerical_ode/node5.html
                next.twist.linear = current.twist.linear + (current.accelerations.linear + next.accelerations.linear) * (0.5f * dt_real);
                next.twist.angular = current.twist.angular + (current.accelerations.angular + next.accelerations.angular) * (0.5f * dt_real);

                //if controller has bug, velocities can increase idenfinitely
                //so we need to clip this or everything will turn in to infinity/nans

                if (next.twist.linear.squaredNorm() > EarthUtils::SpeedOfLight * EarthUtils::SpeedOfLight) { //speed of light
                    next.twist.linear /= (next.twist.linear.norm() / EarthUtils::SpeedOfLight);
                    next.accelerations.linear = Vector3r::Zero();
                }
                //
                //for disc of 1m radius which angular velocity translates to speed of light on tangent?
                if (next.twist.angular.squaredNorm() > EarthUtils::SpeedOfLight * EarthUtils::SpeedOfLight) { //speed of light
                    next.twist.angular /= (next.twist.angular.norm() / EarthUtils::SpeedOfLight);
                    next.accelerations.angular = Vector3r::Zero();
                }
            }
            // every 0.5 seconds, log the physics state
            // if (clock()->nowNanos() - last_client_update_time_[body.vehicle_name] > 1E8) {
            //     client->moveByVelocityAsync(next.twist.linear.x(), next.twist.linear.y(), next.twist.linear.z(), 0.1, msr::airlib::DrivetrainType::MaxDegreeOfFreedom, msr::airlib::YawMode(), body.vehicle_name);
            //     msr::airlib::TTimePoint current_tp = clock()->nowNanos();
            //     uint64_t chrono_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            //     last_client_update_time_[body.vehicle_name] = current_tp;
            //     std::FILE* logFile = std::fopen("/home/idaretos/Control_AirSim/log/connectionLog.txt", "a");
            //     std::fprintf(logFile, "moveByVelocityAsync, %s, %lu\n", body.vehicle_name.c_str(), chrono_time);
            //     std::fclose(logFile);
            // }

            computeNextPose(dt, current.pose, avg_linear, avg_angular, next);

        }

        static void computeNextPose(TTimeDelta dt, const Pose& current_pose, const Vector3r& avg_linear, const Vector3r& avg_angular, Kinematics::State& next)
        {
            real_T dt_real = static_cast<real_T>(dt);
            // printf("avg_linear: %f %f %f\n", avg_linear.x(), avg_linear.y(), avg_linear.z());

            next.pose.position = current_pose.position + avg_linear * dt_real;

            //use angular velocty in body frame to calculate angular displacement in last dt seconds
            real_T angle_per_unit = avg_angular.norm();
            if (Utils::isDefinitelyGreaterThan(angle_per_unit, 0.0f)) {
                //convert change in angle to unit quaternion
                AngleAxisr angle_dt_aa = AngleAxisr(angle_per_unit * dt_real, avg_angular / angle_per_unit);
                Quaternionr angle_dt_q = Quaternionr(angle_dt_aa);
                /*
            Add change in angle to previous orientation.
            Proof that this is q0 * q1:
            If rotated vector is qx*v*qx' then qx is attitude
            Initially we have q0*v*q0'
            Lets transform this to body coordinates to get
            q0'*(q0*v*q0')*q0
            Then apply q1 rotation on it to get
            q1(q0'*(q0*v*q0')*q0)q1'
            Then transform back to world coordinate
            q0(q1(q0'*(q0*v*q0')*q0)q1')q0'
            which simplifies to
            q0(q1(v)q1')q0'
            Thus new attitude is q0q1
            */
                next.pose.orientation = current_pose.orientation * angle_dt_q;
                if (VectorMath::hasNan(next.pose.orientation)) {
                    //Utils::DebugBreak();
                    Utils::log("orientation had NaN!", Utils::kLogLevelError);
                }

                //re-normalize quaternion to avoid accumulating error
                next.pose.orientation.normalize();
            }
            else //no change in angle, because angular velocity is zero (normalized vector is undefined)
                next.pose.orientation = current_pose.orientation;
        }

        void receiveMessage() {
            struct sockaddr_in listen_addr, client_addr;
            int loop_threshold = 100000;
            char buffer[BUFFER_SIZE];
            socklen_t client_addr_len = sizeof(client_addr);
            // Create a socket for receiving messages (TCP)
            if ((listen_sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
                // std::FILE *logFile = std::fopen("/home/idaretos/Control_AirSim/log/receiveMessage.txt", "a");
                // std::fprintf(logFile, "Socket creation failed\n");
                // std::fclose(logFile);
                perror("Socket creation failed");
                // exit(EXIT_FAILURE);
            }
            // Set up the listening address
            memset(&listen_addr, 0, sizeof(listen_addr));
            listen_addr.sin_family = AF_INET; // IPv4
            listen_addr.sin_addr.s_addr = INADDR_ANY; // Any incoming IP
            listen_addr.sin_port = htons(LISTEN_PORT); // Listening port
            // Bind the socket to the listening address
            if (bind(listen_sock, (const struct sockaddr *)&listen_addr, sizeof(listen_addr)) < 0) {
                // std::FILE *logFile = std::fopen("/home/idaretos/Control_AirSim/log/receiveMessage.txt", "a");
                // std::fprintf(logFile, "Bind failed\n");
                // std::fclose(logFile);
                perror("Bind failed");
                close(listen_sock);
                // exit(EXIT_FAILURE);
            }
            // Start listening for incoming connections
            if (listen(listen_sock, 5) < 0) {
                // std::FILE *logFile = std::fopen("/home/idaretos/Control_AirSim/log/receiveMessage.txt", "a");
                // std::fprintf(logFile, "Listen failed\n");
                // std::fclose(logFile);
                perror("Listen failed");
                close(listen_sock);
                // exit(EXIT_FAILURE);
            }
            printf("Listening on port %d for TCP connections...\n", LISTEN_PORT);
            // std::FILE *lf = std::fopen("/home/idaretos/Control_AirSim/log/receiveMessage.txt", "a");
            // std::fprintf(lf, "Listening on port %d for TCP connections...\n", LISTEN_PORT);
            // std::fclose(lf);
            // Accept an incoming connection
            client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &client_addr_len);
            if (client_sock < 0) {
                std::FILE *logFile = std::fopen("/home/idaretos/Control_AirSim/log/receiveMessage.txt", "a");
                perror("Connection accept failed");
                // continue;
            }

            std::string incomplete_message = "";

            while (1) {
                // Receive the message
                ssize_t len = recv(client_sock, buffer, BUFFER_SIZE, 0);
                // printf("Received message, len: %zd\n", len);
                if (len < 0) {
                    // std::FILE *logFile = std::fopen("/home/idaretos/Control_AirSim/log/receiveMessage.txt", "a");
                    perror("Receive failed");
                    close(client_sock);
                    if (loop_threshold-- < 0) {
                        break;
                    }
                    continue;
                }
                buffer[len] = '\0'; // Null-terminate the received data
                // printf("Received message: %s\n", buffer);
                // printf("message received\n");
                // break down the message using ";" as delimiter
                // std::FILE *lfile = std::fopen("/home/idaretos/Control_AirSim/log/receiveMessage.txt", "a");
                // std::fprintf(lfile, "Received message: %s\n", buffer);
                // std::fclose(lfile);
                std::string message(buffer);
                std::string delimiter = ";";
                size_t pos = 0;
                std::string token;
                
                mtx.lock();
                while ((pos = message.find(delimiter)) != std::string::npos) {
                    token = message.substr(0, pos);
                    // check if the token is following the format.
                    // use regex to check if the token is in the correct format
                    std::regex format("([a-zA-Z0-9]+:[+-]?([0-9]*[.])?[0-9]+,[+-]?([0-9]*[.])?[0-9]+,[+-]?([0-9]*[.])?[0-9]+,[+-]?([0-9]*[.])?[0-9]+,[+-]?([0-9]*[.])?[0-9]+,[+-]?([0-9]*[.])?[0-9]+)");
                    if (!std::regex_match(token, format)) {
                        if (incomplete_message.length() > 0) {
                            token = incomplete_message + token;
                            incomplete_message = "";
                            if (!std::regex_match(token, format)) {
                                continue;
                            }
                        } else {
                            incomplete_message = token;
                            message.erase(0, pos + delimiter.length());
                            continue;
                        }
                        // std::cout << "Invalid format: " << token << std::endl;
                        // // std::cout << "Message: " << message << std::endl;
                        // message.erase(0, pos + delimiter.length());
                        // continue;
                    }

                    std::string backup_token = token;
                    // the token will look like "vehicle_name:x,y,z"
                    // Save them in the tmp_pos_ptr
                    std::string vehicle_name = token.substr(0, token.find(":"));
                    token.erase(0, token.find(":") + 1);
                    std::string x = token.substr(0, token.find(","));
                    token.erase(0, token.find(",") + 1);
                    std::string y = token.substr(0, token.find(","));
                    token.erase(0, token.find(",") + 1);
                    std::string z = token;
                    token.erase(0, token.find(",") + 1);
                    std::string vx = token.substr(0, token.find(","));
                    token.erase(0, token.find(",") + 1);
                    std::string vy = token.substr(0, token.find(","));
                    token.erase(0, token.find(",") + 1);
                    
                    Eigen::Vector3f position(std::stof(x), std::stof(y), std::stof(z));
                    Vector3r velocity(std::stof(vx), std::stof(vy), std::stof(token));
                    
                    if (pos1) {
                        // std::cout << "Inserting into real_pos1, " << vehicle_name << ", " << position << std::endl;
                        real_pos1.insert({vehicle_name, position});
                        real_vel.insert({vehicle_name, velocity});
                    } else {
                        // std::cout << "Inserting into real_pos2, " << vehicle_name << ", " << position << std::endl;
                        real_pos2.insert({vehicle_name, position});
                        real_vel.insert({vehicle_name, velocity});
                    }
                    update_[vehicle_name] = true;
                    message.erase(0, pos + delimiter.length());
                }
                mtx.unlock();
                // std::FILE *logFile = std::fopen("/home/idaretos/Control_AirSim/log/receiveMessage.txt", "a");
                // std::fprintf(logFile, "Received message: %s\n", buffer);
                // std::fclose(logFile);
                // Close the connection
            }
            close(client_sock);
            close(listen_sock);
        }

    private:
        static constexpr uint kCollisionResponseCycles = 1;
        static constexpr float kAxisTolerance = 0.25f;
        static constexpr float kRestingVelocityMax = 0.1f;
        static constexpr float kDragMinVelocity = 0.1f;

        std::stringstream debug_string_;
        bool enable_ground_lock_;
        TTimePoint last_message_time;
        Vector3r wind_;

        // MultirotorRpcLibClient* client;
        MultirotorRpcLibClient* client;
        std::unordered_map<std::string, TTimePoint> last_client_update_time_;
        std::unordered_map<std::string, int> client_state_;

        std::mutex mtx;

        // std::unordered_map<std::string, Vector3r> real_pos1;
        // std::unordered_map<std::string, Vector3r> real_pos2;
        std::unordered_map<std::string, Eigen::Vector3f> real_pos1;
        std::unordered_map<std::string, Eigen::Vector3f> real_pos2;
        std::unordered_map<std::string, Vector3r> real_vel;
        std::unordered_map<std::string, bool> update_;
        bool pos1 = true;

        unsigned long UPDATE_PERIOD = 1.3E9;

        std::thread msg_receiver;

        int client_sock;
        int listen_sock;

        class ScopeLogger {
        public:
            ScopeLogger(std::stringstream& debug_string) : debug_string_(debug_string) {
                begin = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();                
            }
            ~ScopeLogger() {
                uint64_t end = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                std::FILE* logFile = std::fopen("/home/idaretos/Control_AirSim/log/physics.txt", "a");
                std::fprintf(logFile, "%lu, %lu, %s\n", begin, end, debug_string_.str().c_str());
                std::fclose(logFile);
            }
        private:
            uint64_t begin;
            std::stringstream& debug_string_;
        };
    };
}
} //namespace
#endif
