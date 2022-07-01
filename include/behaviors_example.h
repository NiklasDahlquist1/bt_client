

#ifndef BEHAVIORS_H
#define BEHAVIORS_H

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp_v3/blackboard.h"


#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"

//#include <chrono>
//#include <thread>


#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/LinearMath/Vector3.h"
//#include "tf/LinearMath/Matrix3x3.h"
//#include "tf/LinearMath/Quaternion.h"






#include "bt_client.hpp"


//#include <random>



/*
#define MAX_HEIGHT_MOVETO 1.8
#define TAKEOFF_HEIGHT 0.5
#define LAND_HEIGHT 0.5
#define UAV_AT_POINT_TOLERANCE 0.15
*/

/*

TODO:
add actions, return to home point, take off, land, safety checks
how to integrate CBFs?



*/


namespace behaviors
{
    class FollowPath : public BT::StatefulActionNode
    {
        private: 
            auction_ns::uav_state* statePtr;

            bool atPoint2D(geometry_msgs::Point point)
            {
                double error2 = pow(statePtr->currentPose.position.x - point.x, 2) + 
                                pow(statePtr->currentPose.position.y - point.y, 2);

                double tol = 0.85; //lookahead distance

                return error2 < pow(tol, 2);
            }

        public:

            void init(auction_ns::uav_state* statePtr)
            {
                if(statePtr == nullptr)
                {
                    //error
                }
                this->statePtr = statePtr;
            }

            FollowPath(const std::string& name, const BT::NodeConfiguration& config) : StatefulActionNode(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return{ };
            }

            BT::NodeStatus logic()
            {
                if(statePtr->path.poses.size() > statePtr->pathIndex)
                {
                    geometry_msgs::Point goalPoint = statePtr->path.poses[statePtr->pathIndex].pose.position;
                    while( atPoint2D(goalPoint))
                    {
                        if (statePtr->path.poses.size() > statePtr->pathIndex + 1)
                        {
                            statePtr->pathIndex++;
                            goalPoint = statePtr->path.poses[statePtr->pathIndex].pose.position;
                        }
                        else 
                        {
                            break;
                        }
                    }
                    goalPoint.z = 2; // set height



                    geometry_msgs::PoseStamped poseStamped;
                    poseStamped.header.stamp = ros::Time::now();
                    poseStamped.header.frame_id = "world";
                    poseStamped.pose.position = goalPoint;
                    statePtr->goalPoint_pub.publish(poseStamped);
                    return BT::NodeStatus::RUNNING;
                }
                else
                {
                    // no path/ path ended
                    return BT::NodeStatus::RUNNING;
                }

            }
            BT::NodeStatus onStart()
            {
                return logic();
            }

            BT::NodeStatus onRunning()
            {
                return logic();
            }

            void onHalted()
            {
                //
            }
    };

    class Follow3DPath : public BT::StatefulActionNode
    {
        private: 
            auction_ns::uav_state* statePtr;

            bool atPoint3D(geometry_msgs::Point point)
            {
                double error2 = pow(statePtr->currentPose.position.x - point.x, 2) + 
                                pow(statePtr->currentPose.position.y - point.y, 2) +
                                pow(statePtr->currentPose.position.z - point.z, 2);

                double tol = 1;

                return error2 < pow(tol, 2);
            }

        public:

            void init(auction_ns::uav_state* statePtr)
            {
                if(statePtr == nullptr)
                {
                    //error
                }
                this->statePtr = statePtr;
            }

            Follow3DPath(const std::string& name, const BT::NodeConfiguration& config) : StatefulActionNode(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return{ };
            }

            BT::NodeStatus logic()
            {
                
                if(statePtr->path.poses.size() > statePtr->pathIndex)
                {
                    geometry_msgs::Point goalPoint = statePtr->path.poses[statePtr->pathIndex].pose.position;
                    while( atPoint3D(goalPoint))
                    {
                        if (statePtr->path.poses.size() > statePtr->pathIndex + 1)
                        {
                            statePtr->pathIndex++;
                            goalPoint = statePtr->path.poses[statePtr->pathIndex].pose.position;
                        }
                        else 
                        {
                            break;
                        }
                    }
                    //goalPoint.z = 2; // set height


                    geometry_msgs::PoseStamped poseStamped;
                    poseStamped.header.stamp = ros::Time::now();
                    poseStamped.header.frame_id = "world";
                    poseStamped.pose.position = goalPoint;
                    statePtr->goalPoint_pub.publish(poseStamped);
                    return BT::NodeStatus::RUNNING;
                }
                else
                {
                    // no path/ path ended
                    return BT::NodeStatus::RUNNING;
                }

            }
            BT::NodeStatus onStart()
            {
                return logic();
            }

            BT::NodeStatus onRunning()
            {
                return logic();
            }

            void onHalted()
            {
                //
            }
    };





    // remove
    class MoveToGoalPoint : public BT::StatefulActionNode
    {
        private: 
            auction_ns::uav_state* statePtr;

            double lookAheadDistance = 3;
        public:

            void init(auction_ns::uav_state* statePtr)
            {
                if(statePtr == nullptr)
                {
                    //error
                }
                this->statePtr = statePtr;
            }

            MoveToGoalPoint(const std::string& name, const BT::NodeConfiguration& config) : StatefulActionNode(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return{ };
            }

            BT::NodeStatus logic()
            {
                
                geometry_msgs::PoseStamped poseStamped;
                poseStamped.header.stamp = ros::Time::now();
                poseStamped.header.frame_id = "world";

                tf::Vector3 currentPos;
                tf::Vector3 goalPos;
                tf::Vector3 dir;

                currentPos.setX(statePtr->currentPose.position.x);
                currentPos.setY(statePtr->currentPose.position.y);
                currentPos.setZ(statePtr->currentPose.position.z);

                goalPos.setX(statePtr->goalPoint.x);
                goalPos.setY(statePtr->goalPoint.y);
                goalPos.setZ(statePtr->goalPoint.z);

                dir = goalPos - currentPos;
                //std::cout << statePtr->goalPoint.z << std::endl;


                double length2 = dir.length2();

                if(length2 > pow(lookAheadDistance, 2))
                {
                    dir = dir.normalize() * lookAheadDistance;
                }
                //else if(length2 < pow(0.1, 2))
                //{
                //    dir = dir.normalize() * 0.1;
                //}

                dir = dir + currentPos;

                poseStamped.pose.position.x = dir.getX();
                poseStamped.pose.position.y = dir.getY();
                poseStamped.pose.position.z = dir.getZ();
                

                statePtr->goalPoint_pub.publish(poseStamped);
                return BT::NodeStatus::RUNNING;
            }
            BT::NodeStatus onStart()
            {
                return logic();
            }

            BT::NodeStatus onRunning()
            {
                return logic();
            }

            void onHalted()
            {
                //
            }
    };


    // check if the uav is close to a point
    class UAVAtPoint : public BT::SyncActionNode
    {
        private: 
            auction_ns::uav_state* statePtr;
        public:

            void init(auction_ns::uav_state* statePtr)
            {
                if(statePtr == nullptr)
                {
                    //error
                }
                this->statePtr = statePtr;
            }

            UAVAtPoint(const std::string& name, const BT::NodeConfiguration& config) : SyncActionNode(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return{ BT::InputPort<geometry_msgs::Point>("point"), BT::OutputPort<geometry_msgs::Point>("goalPoint") };
            }

            BT::NodeStatus tick() override
            {
                //std::cout << "curent x: " << statePtr->currentPose.position.x << std::endl;
                double error2 = pow(statePtr->currentPose.position.x - statePtr->goalPoint.x, 2) + 
                                pow(statePtr->currentPose.position.y - statePtr->goalPoint.y, 2) + 
                                pow(statePtr->currentPose.position.z - statePtr->goalPoint.z, 2);

                double tol = 0.5;
                //std::cout << "uav error: " << error2 << std::endl;

                if(error2 < pow(tol, 2))
                {

                    //publish uav waypoint once more to make sure that the uav has the exact goal
                    geometry_msgs::PoseStamped p;
                    p.header.stamp = ros::Time::now();
                    p.header.frame_id = "world";
                    p.pose.position = statePtr->goalPoint;
                    statePtr->goalPoint_pub.publish(p);
                    return BT::NodeStatus::SUCCESS;
                }
                else
                {
                    return BT::NodeStatus::FAILURE;
                }

                //
            }
    };


    // check if the uav is close to a point
    class UAVAtPoint2D : public BT::SyncActionNode
    {
        private: 
            auction_ns::uav_state* statePtr;
        public:

            void init(auction_ns::uav_state* statePtr)
            {
                if(statePtr == nullptr)
                {
                    //error
                }
                this->statePtr = statePtr;
            }

            UAVAtPoint2D(const std::string& name, const BT::NodeConfiguration& config) : SyncActionNode(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return{ BT::InputPort<geometry_msgs::Point>("point"), BT::OutputPort<geometry_msgs::Point>("goalPoint") };
            }

            BT::NodeStatus tick() override
            {
                //std::cout << "curent x: " << statePtr->currentPose.position.x << std::endl;
                double error2 = pow(statePtr->currentPose.position.x - statePtr->goalPoint.x, 2) + 
                                pow(statePtr->currentPose.position.y - statePtr->goalPoint.y, 2);

                double tol = 0.6;
                //std::cout << "uav error: " << error2 << std::endl;

                if(error2 < pow(tol, 2))
                {

                    //publish uav waypoint once more to make sure that the uav has the exact goal
                    geometry_msgs::PoseStamped p;
                    p.header.stamp = ros::Time::now();
                    p.header.frame_id = "world";
                    p.pose.position = statePtr->goalPoint;
                    statePtr->goalPoint_pub.publish(p);
                    return BT::NodeStatus::SUCCESS;
                }
                else
                {
                    return BT::NodeStatus::FAILURE;
                }

                //
            }
    };

/*

    class DelayOnce : public BT::StatefulActionNode
    {
        private: 
            bt_state::mav_state* state;

            ros::Time startTime;

            bool alreadyDelayed = false;

            BT::NodeStatus logic()
            {
                if(this->state == nullptr)
                {
                    ROS_INFO_STREAM("UAVAtHomePoint, pointer to state = null");
                }

                BT::Optional<double> msg = getInput<double>("delayTime");
                if (!msg)
                {
                    throw BT::RuntimeError("missing required input [message]: ", msg.error());
                }

                //ROS_INFO_STREAM("Time left" << startTime.toSec() - ros::Time::now().toSec() + msg.value());
                if(ros::Time::now().toSec() > startTime.toSec() + msg.value() || alreadyDelayed)
                {
                    alreadyDelayed = true;
                    return BT::NodeStatus::SUCCESS;
                }
                else
                {
                    return BT::NodeStatus::RUNNING;
                }
            }

        public:

            void init(bt_state::mav_state* statePtr)
            {
                if(statePtr == nullptr)
                {
                    //error
                }
                this->state = statePtr;
            }

            DelayOnce(const std::string& name, const BT::NodeConfiguration& config) : StatefulActionNode(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return{ BT::InputPort<double>("delayTime") };
            }

            BT::NodeStatus onStart()
            {
                startTime = ros::Time::now();
                return logic();
            }

            BT::NodeStatus onRunning()
            {
                return logic();
            }

            void onHalted()
            {
                //
            }



    };









    class Explore : public BT::StatefulActionNode
    {
        private: 
            bt_state::mav_state* state;


            BT::NodeStatus logic()
            {

                return BT::NodeStatus::RUNNING;
            }

        public:

            void init(bt_state::mav_state* statePtr)
            {
                if(statePtr == nullptr)
                {
                    //error
                }
                this->state = statePtr;
            }

            Explore(const std::string& name, const BT::NodeConfiguration& config) : StatefulActionNode(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return{  };
            }

            BT::NodeStatus onStart()
            {
                if(this->state == nullptr)
                {
                    ROS_INFO_STREAM("Explore, pointer to state = null");
                }
                return logic();
            }

            BT::NodeStatus onRunning()
            {
                return logic();
            }

            void onHalted()
            {
                //
            }
    };

*/

} // end namespace










// Template specialization to convert.
namespace BT
{

    template <> inline geometry_msgs::Pose convertFromString(StringView str)
    {
        // The next line should be removed...
        //printf("Converting string: \"%s\"\n", str.data() );

        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ';');
        if (parts.size() != 7)
        {
            throw RuntimeError("invalid input)");
        }
        else{
            geometry_msgs::Pose output;
            output.position.x    = convertFromString<double>(parts[0]);
            output.position.y    = convertFromString<double>(parts[1]);
            output.position.z    = convertFromString<double>(parts[2]);
            output.orientation.x = convertFromString<double>(parts[3]);
            output.orientation.y = convertFromString<double>(parts[4]);
            output.orientation.z = convertFromString<double>(parts[5]);
            output.orientation.w = convertFromString<double>(parts[6]);
            return output;
        }
    }

    template <> inline geometry_msgs::Point convertFromString(StringView str)
    {
        // The next line should be removed...
        //printf("Converting string: \"%s\"\n", str.data() );

        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ';');
        if (parts.size() != 3)
        {
            throw RuntimeError("invalid input)");
        }
        else{
            geometry_msgs::Point output;
            output.x = convertFromString<double>(parts[0]);
            output.y = convertFromString<double>(parts[1]);
            output.z = convertFromString<double>(parts[2]);
            return output;
        }
    }


    template <> inline bool convertFromString(StringView str)
    {
        bool output;
        if (str == "true")
        {
            output = true;
        }
        else if (str == "false")
        {
            output = false;
        }
        else
        {
            throw RuntimeError("invalid input)");
        }

        return output;
    }
} // end namespace BT








#endif





