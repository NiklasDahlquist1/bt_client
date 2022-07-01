






#ifndef BEHAVIORS_GROUND_H
#define BEHAVIORS_GROUND_H

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp_v3/blackboard.h"



#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/LinearMath/Vector3.h"



#include "client_groundAgent.hpp"









namespace behaviors
{
    class GroundFollowPath : public BT::StatefulActionNode
    {
        private: 
            auction_ns::ground_agent_state* statePtr;

            ros::Time lastServiceCall;
            double serviceUpdateIntervall = 1;


            bool atPoint2D(geometry_msgs::Point point)
            {
                double error2 = pow(statePtr->currentPose.position.x - point.x, 2) + 
                                pow(statePtr->currentPose.position.y - point.y, 2);

                double tol = 0.4; //lookahead distance

                return error2 < pow(tol, 2);
            }

        public:

            void init(auction_ns::ground_agent_state* statePtr)
            {
                if(statePtr == nullptr)
                {
                    //error
                }
                this->statePtr = statePtr;
            }

            GroundFollowPath(const std::string& name, const BT::NodeConfiguration& config) : StatefulActionNode(name, config)
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


                    if(ros::Time::now().toSec() > this->lastServiceCall.toSec() + this->serviceUpdateIntervall)
                    {
                        lastServiceCall = ros::Time::now();


                        // send current goal to controller
                        ros_turtlebot_control::MoveToPoint srv_moveTo;
                        srv_moveTo.request.x = goalPoint.x;
                        srv_moveTo.request.y = goalPoint.y;
                        
                        if (statePtr->setGoal_srv.call(srv_moveTo))
                        {

                        }
                        else
                        {
                            std::cout << "Failed to call service MoveToPoint for turtle control" << std::endl;
                        }
                    }

                    
                    //statePtr->goalPoint_pub.publish(poseStamped);
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












    // check if the robot is close to a point
    class GroundAtPoint2D : public BT::SyncActionNode
    {
        private: 
            auction_ns::ground_agent_state* statePtr;

            ros::Time lastServiceCall;
            double serviceUpdateIntervall = 2;

        public:

            void init(auction_ns::ground_agent_state* statePtr)
            {
                if(statePtr == nullptr)
                {
                    //error
                }
                this->statePtr = statePtr;
            }

            GroundAtPoint2D(const std::string& name, const BT::NodeConfiguration& config) : SyncActionNode(name, config)
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
                double tol = 0.2;
                //std::cout << "uav error: " << error2 << std::endl;

                if(error2 < pow(tol, 2))
                {
                    if(ros::Time::now().toSec() > this->lastServiceCall.toSec() + this->serviceUpdateIntervall)
                    {
                        lastServiceCall = ros::Time::now();


                        // send current goal to controller
                        ros_turtlebot_control::MoveToPoint srv_moveTo;
                        srv_moveTo.request.x = statePtr->goalPoint.x;
                        srv_moveTo.request.y = statePtr->goalPoint.y;
                        
                        if (statePtr->setGoal_srv.call(srv_moveTo))
                        {

                        }
                        else
                        {
                            std::cout << "Failed to call service MoveToPoint for turtle control" << std::endl;
                        }
                    }

                    return BT::NodeStatus::SUCCESS;
                }
                else
                {
                    return BT::NodeStatus::FAILURE;
                }

                //
            }
    };








    class GroundMoveToGoalPoint : public BT::StatefulActionNode
    {
        private: 
            auction_ns::ground_agent_state* statePtr;

            ros::Time lastServiceCall;
            double serviceUpdateIntervall = 1;

            double lookAheadDistance = 2;
        public:

            void init(auction_ns::ground_agent_state* statePtr)
            {
                if(statePtr == nullptr)
                {
                    //error
                }
                this->statePtr = statePtr;
            }

            GroundMoveToGoalPoint(const std::string& name, const BT::NodeConfiguration& config) : StatefulActionNode(name, config)
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
                

                if(ros::Time::now().toSec() > this->lastServiceCall.toSec() + this->serviceUpdateIntervall)
                {
                    lastServiceCall = ros::Time::now();


                    // send current goal to controller
                    ros_turtlebot_control::MoveToPoint srv_moveTo;
                    srv_moveTo.request.x = poseStamped.pose.position.x;
                    srv_moveTo.request.y = poseStamped.pose.position.y;
                    
                    if (statePtr->setGoal_srv.call(srv_moveTo))
                    {

                    }
                    else
                    {
                        std::cout << "Failed to call service MoveToPoint for turtle control" << std::endl;
                    }
                }

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











}








#endif