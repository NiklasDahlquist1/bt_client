






#ifndef BEHAVIORS_GROUND_H
#define BEHAVIORS_GROUND_H

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp_v3/blackboard.h"



#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/LinearMath/Vector3.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/LinearMath/Matrix3x3.h"
#include <math.h>    


#include "client_groundAgent.hpp"




#define TOLERANCE_AT_POINT 0.25//0.17
#define DSP_LOOK_AHEAD_DISTANCD 0.75//0.17





namespace behaviors
{
    class GroundFollowPath : public BT::StatefulActionNode
    {
        private: 
            auction_ns::turtlebot_state* statePtr;

            ros::Time lastServiceCall;
            double serviceUpdateIntervall = 0.9;
            bool moving_forward = true;
            bool direction_is_set = false;


            bool atPoint2D(geometry_msgs::Point point)
            {
                double error2 = pow(statePtr->currentPose.position.x - point.x, 2) + 
                                pow(statePtr->currentPose.position.y - point.y, 2);

                double tol = DSP_LOOK_AHEAD_DISTANCD;//1.75;//0.55; //lookahead distance

                return error2 < pow(tol, 2);
            }

        public:

            void init(auction_ns::turtlebot_state* statePtr)
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




                    // find goal yaw
                    tf::Vector3 current_path_goal;
                    tf::Vector3 path_next_point;
                    tf::Vector3 dir;
                    tf::Vector3 normal_yaw;
                    normal_yaw.setX(1);
                    normal_yaw.setY(0);
                    normal_yaw.setZ(0);


                    //std::cout << "TEST" << "\n";




                    if (statePtr->path.poses.size() > statePtr->pathIndex + 1)
                    {
                        current_path_goal.setX(statePtr->path.poses[statePtr->pathIndex].pose.position.x); 
                        current_path_goal.setY(statePtr->path.poses[statePtr->pathIndex].pose.position.y);
                        current_path_goal.setZ(statePtr->path.poses[statePtr->pathIndex].pose.position.z);

                        //current_path_goal.setX(statePtr->currentPose.position.x); 
                        //current_path_goal.setY(statePtr->currentPose.position.y);
                        //current_path_goal.setZ(statePtr->currentPose.position.z);


                        path_next_point.setX(statePtr->path.poses[statePtr->pathIndex + 1].pose.position.x + 0.001); // noise for later
                        path_next_point.setY(statePtr->path.poses[statePtr->pathIndex + 1].pose.position.y - 0.001);
                        path_next_point.setZ(statePtr->path.poses[statePtr->pathIndex + 1].pose.position.z);
                    }
                    else
                    {
                        int last_index = statePtr->path.poses.size() - 1;

                        // set direction between the two last points in the path
                        current_path_goal.setX(statePtr->path.poses[last_index - 1].pose.position.x);
                        current_path_goal.setY(statePtr->path.poses[last_index - 1].pose.position.y);
                        current_path_goal.setZ(statePtr->path.poses[last_index - 1].pose.position.z);

                        path_next_point.setX(statePtr->path.poses[last_index].pose.position.x + 0.001); // noise for later
                        path_next_point.setY(statePtr->path.poses[last_index].pose.position.y - 0.001);
                        path_next_point.setZ(statePtr->path.poses[last_index].pose.position.z);

                    }






                    dir = path_next_point - current_path_goal;
                    dir.setZ(0);
                    tf::Quaternion quat_goal;
                    tf::Quaternion quat_current;
                    quat_current.setX(statePtr->currentPose.orientation.x);
                    quat_current.setY(statePtr->currentPose.orientation.y);
                    quat_current.setZ(statePtr->currentPose.orientation.z);
                    quat_current.setW(statePtr->currentPose.orientation.w);
                    tf::Matrix3x3 rot_current(quat_current);
                    double roll, pitch, yaw;
                    rot_current.getRPY(roll, pitch, yaw);


                    tf::Vector3 current_pos;
                    tf::Vector3 relative_goal;
                    tf::Vector3 current_yaw;
                    current_pos.setX(statePtr->currentPose.position.x);
                    current_pos.setY(statePtr->currentPose.position.y);
                    current_pos.setZ(statePtr->currentPose.position.z);
                    relative_goal = path_next_point - current_pos;
                    current_yaw.setX(cos(yaw));
                    current_yaw.setY(sin(yaw));
                    //current_yaw.setY(0);

                    double angle_goal;

                    double test = current_yaw.dot(relative_goal);
                    //std::cout << "test " << test << "\n";
                    /*if(test < 0) // next point is behind turtlebot
                    {                        
                        //std::cout << "reversing" << "\n";
                        dir = -dir;
                        angle_goal = atan2(dir.getY(), dir.getX());
                        quat_goal.setRPY(0, 0, angle_goal);

                    }
                    else
                    {
                        angle_goal = atan2(dir.getY(), dir.getX());
                        quat_goal.setRPY(0, 0, angle_goal);
                    }*/


                    
                    if(test < 0) // next point is behind turtlebot
                    {                        
                        moving_forward = false;
                    }
                    else
                    {
                        moving_forward = true;
                    }


                    // dont update yaw angle if really close to goal TODO: check somewhere else
                    // here we check for the last point in the path
                    if(pow(statePtr->currentPose.position.x - statePtr->path.poses[statePtr->path.poses.size() - 1].pose.position.x, 2) + 
                       pow(statePtr->currentPose.position.y - statePtr->path.poses[statePtr->path.poses.size() - 1].pose.position.y, 2) < pow(1.5, 2))
                    {
                        //std::cout << "close\n";
                        if(direction_is_set == false)
                        {
                            direction_is_set = true;
                            if(test < 0) // next point is behind turtlebot
                            {         
                                //std::cout << "BACKING!!!\n";              
                                //std::cout << "reversing" << "\n";
                                dir = -dir;
                                angle_goal = atan2(dir.getY(), dir.getX());
                                quat_goal.setRPY(0, 0, angle_goal);

                            }
                            else
                            {
                                //std::cout << "FORWARD!!!!\n";    
                                angle_goal = atan2(dir.getY(), dir.getX());
                                quat_goal.setRPY(0, 0, angle_goal);
                            }
                            this->statePtr->yaw_from_path = angle_goal;
                            //std::cout << "setting current yaw: " << angle_goal << "\n";
                        }
                        else if(direction_is_set == true)
                        {
                            angle_goal = this->statePtr->yaw_from_path;
                        }
                    }
                    else
                    {
                        //std::cout << "far away\n";
                        direction_is_set = false;
                    }




                    if(direction_is_set == false)
                    {
                        if(moving_forward == false) // next point is behind turtlebot
                        {                        
                            //std::cout << "reversing" << "\n";
                            dir = -dir;
                            angle_goal = atan2(dir.getY(), dir.getX());
                            quat_goal.setRPY(0, 0, angle_goal);

                        }
                        else if(moving_forward == true)
                        {
                            angle_goal = atan2(dir.getY(), dir.getX());
                            quat_goal.setRPY(0, 0, angle_goal);
                        }
                        //this->statePtr->yaw_from_path = angle_goal;
                    }


                   



                    poseStamped.pose.orientation.x = quat_goal.getX();
                    poseStamped.pose.orientation.y = quat_goal.getY();
                    poseStamped.pose.orientation.z = quat_goal.getZ();
                    poseStamped.pose.orientation.w = quat_goal.getW();

                    // for vizualization
                    poseStamped.pose.position.z = 0;
                    poseStamped.pose.orientation.z = angle_goal;

                    //statePtr->setGoalPathPlanner_pub.publish(goalPoint);

                    statePtr->goalPoint_pub.publish(poseStamped);
                    //std::cout << "FOLLOW PATH PUB YAW: " << poseStamped.pose.orientation.z << "\n";
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
            auction_ns::turtlebot_state* statePtr;

            ros::Time lastServiceCall;
            double serviceUpdateIntervall = 2;

        public:

            void init(auction_ns::turtlebot_state* statePtr)
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
                double tol = TOLERANCE_AT_POINT;
                //std::cout << "uav error: " << error2 << std::endl;

                if(error2 < pow(tol, 2))
                {
                    /*
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
                    */
                    geometry_msgs::PoseStamped poseStamped;
                    poseStamped.header.stamp = ros::Time::now();
                    poseStamped.header.frame_id = "world";
                    poseStamped.pose.position = statePtr->goalPoint;

                    poseStamped.pose.orientation.z = this->statePtr->yaw_from_path;
                    //std::cout << "yaw at point: " << this->statePtr->yaw_from_path << "\n";

                    
                    statePtr->goalPoint_pub.publish(poseStamped);



                    return BT::NodeStatus::SUCCESS;
                }
                else
                {
                    return BT::NodeStatus::FAILURE;
                }

                //
            }
    };








    class Ground_at_pick_place : public BT::SyncActionNode
    {
        private: 
            auction_ns::turtlebot_state* statePtr;
            //bool completed = false;


        public:

            void init(auction_ns::turtlebot_state* statePtr)
            {
                if(statePtr == nullptr)
                {
                    //error
                }
                this->statePtr = statePtr;
            }

            Ground_at_pick_place(const std::string& name, const BT::NodeConfiguration& config) : SyncActionNode(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return{  };
            }

            BT::NodeStatus tick() override
            {
                //std::cout << "curent x: " << statePtr->currentPose.position.x << std::endl;
                double error2 = pow(statePtr->currentPose.position.x - statePtr->pick_goal.x, 2) + 
                                pow(statePtr->currentPose.position.y - statePtr->pick_goal.y, 2);
                double tol = TOLERANCE_AT_POINT;

                if(error2 < pow(tol, 2) || statePtr->pick_complete == true)
                {
                    statePtr->pick_complete = true;
                    /*
                    geometry_msgs::PoseStamped poseStamped;
                    poseStamped.header.stamp = ros::Time::now();
                    poseStamped.header.frame_id = "world";
                    poseStamped.pose.position = statePtr->pick_goal;

                    
                    statePtr->goalPoint_pub.publish(poseStamped);
                    */
                    statePtr->task_can_be_swapped = false;


                    return BT::NodeStatus::SUCCESS;
                }
                else
                {
                    // set goal for "follow path" behavior
                    statePtr->goalPoint = statePtr->pick_goal;
                    statePtr->setGoalPathPlanner_pub.publish(statePtr->goalPoint);

                    return BT::NodeStatus::FAILURE;
                }
                //
            }
    };


    class Ground_at_place_place : public BT::SyncActionNode
    {
        private: 
            auction_ns::turtlebot_state* statePtr;



        public:

            void init(auction_ns::turtlebot_state* statePtr)
            {
                if(statePtr == nullptr)
                {
                    //error
                }
                this->statePtr = statePtr;
            }

            Ground_at_place_place(const std::string& name, const BT::NodeConfiguration& config) : SyncActionNode(name, config)
            {
            }

            static BT::PortsList providedPorts()
            {
                return{  };
            }

            BT::NodeStatus tick() override
            {
                //std::cout << "curent x: " << statePtr->currentPose.position.x << std::endl;
                double error2 = pow(statePtr->currentPose.position.x - statePtr->place_goal.x, 2) + 
                                pow(statePtr->currentPose.position.y - statePtr->place_goal.y, 2);
                double tol = TOLERANCE_AT_POINT;

                if(error2 < pow(tol, 2))
                {
                    /*
                    geometry_msgs::PoseStamped poseStamped;
                    poseStamped.header.stamp = ros::Time::now();
                    poseStamped.header.frame_id = "world";
                    poseStamped.pose.position = statePtr->pick_goal;
*/
                    statePtr->pick_complete = false;


                    return BT::NodeStatus::SUCCESS;
                }
                else
                {
                    // set goal for "follow path" behavior
                    statePtr->goalPoint = statePtr->place_goal;
                    statePtr->setGoalPathPlanner_pub.publish(statePtr->goalPoint);

                    return BT::NodeStatus::FAILURE;
                }

                //
            }
    };











    class GroundMoveToGoalPoint : public BT::StatefulActionNode
    {
        private: 
            auction_ns::turtlebot_state* statePtr;

            ros::Time lastServiceCall;
            double serviceUpdateIntervall = 1;

            double lookAheadDistance = 0.7;
        public:

            void init(auction_ns::turtlebot_state* statePtr)
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
                //std::cout << "AAAAAAAAAAAAAAAAA " << statePtr->goalPoint << "\n";
                /*
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
*/


                //statePtr->goalPoint_pub.publish(poseStamped);
                geometry_msgs::PoseStamped poseStamped;
                poseStamped.header.stamp = ros::Time::now();
                poseStamped.header.frame_id = "world";
                poseStamped.pose.position = statePtr->goalPoint;

                
                statePtr->goalPoint_pub.publish(poseStamped);
                
/*
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
*/

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