/*
 *    Copyright (C) 2022 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"
#include <algorithm>
#include <utility>
#include <cppitertools/zip.hpp>
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/enumerate.hpp>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
    qRegisterMetaType<std::uint64_t>("std::uint64_t");
	this->startup_check_flag = startup_check;
	// Uncomment if there's too many debug messages
	// but it removes the possibility to see the messages
	// shown in the console with qDebug()
	QLoggingCategory::setFilterRules("*.debug=false\n");
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
//	G->write_to_json_file("./"+agent_name+".json");
	G.reset();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }





	agent_name = params["agent_name"].value;
	agent_id = stoi(params["agent_id"].value);

	tree_view = params["tree_view"].value == "true";
	graph_view = params["graph_view"].value == "true";
	qscene_2d_view = params["2d_view"].value == "true";
	osg_3d_view = params["3d_view"].value == "true";

	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		timer.start(Period);
		// create graph
		G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, ""); // Init nodes
		std::cout<< __FUNCTION__ << "Graph loaded" << std::endl;  

		//dsr update signals
		connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::modify_node_slot);
//		connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::modify_edge_slot);
//		connect(G.get(), &DSR::DSRGraph::update_node_attr_signal, this, &SpecificWorker::modify_attrs_slot);
//		connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
//		connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);

		// Graph viewer
		using opts = DSR::DSRViewer::view;
		int current_opts = 0;
		opts main = opts::none;
		if(tree_view)
		{
		    current_opts = current_opts | opts::tree;
		}
		if(graph_view)
		{
		    current_opts = current_opts | opts::graph;
		    main = opts::graph;
		}
		if(qscene_2d_view)
		{
		    current_opts = current_opts | opts::scene;
		}
		if(osg_3d_view)
		{
		    current_opts = current_opts | opts::osg;
		}
		graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
		setWindowTitle(QString::fromStdString(agent_name + "-") + QString::number(agent_id));

        //Set base speed reference to 0
//        if (auto robot = G->get_node(robot_name); robot.has_value())
//        {
//            G->add_or_modify_attrib_local<robot_ref_adv_speed_att>(robot.value(), (float) 0);
//            G->add_or_modify_attrib_local<robot_ref_rot_speed_att>(robot.value(), (float) 0);
//            G->add_or_modify_attrib_local<robot_ref_side_speed_att>(robot.value(), (float) 0);
//            G->update_node(robot.value());
//        }
//        hide();
        inner_eigen = G->get_inner_eigen_api();
        rt = G->get_rt_api();

		this->Period = period;
		timer.start(Period);
	}
	hide();
}

void SpecificWorker::compute()
{
    auto laser = read_laser_from_robot();
    update_robot_localization();
    try
    {
        auto rgbd = camerargbdsimple_proxy->getImage("");
        cv::Mat rgbd_frame (cv::Size(rgbd.width, rgbd.height), CV_8UC3, &rgbd.image[0]);
        cv::cvtColor(rgbd_frame, rgbd_frame, cv::COLOR_BGR2RGB);
        update_camera_rgbd(giraff_camera_realsense_name, rgbd_frame, rgbd.focalx, rgbd.focaly);
    }
    catch(const Ice::Exception &e) { /*std::cout << e.what() << std::endl;*/}
    // update_servo_position();
    update_laser(laser);

//    fps.print("FPS: ", [this](auto x){ graph_viewer->set_external_hz(x);});
}

std::vector<SpecificWorker::LaserPoint> SpecificWorker::read_laser_from_robot()
{
    std::vector<LaserPoint> laser_data;

    try {
        auto laser = laser_proxy->getLaserData();
        //for(auto &d : laser)
        //    qInfo() << d.angle << d.dist;
        std::transform(laser.begin(), laser.end(), std::back_inserter(laser_data), [](const auto &l) {return LaserPoint{l.dist, l.angle}; });
    }catch (const Ice::Exception &e){ /*std::cout << e.what() << " No laser_pioneer_data" << std::endl;*/ return {};}

    return laser_data;
}
void SpecificWorker::update_robot_localization()
{
    static RoboCompFullPoseEstimation::FullPoseEuler last_state;
    RoboCompFullPoseEstimation::FullPoseEuler pose;
    try
    {
        pose = fullposeestimation_proxy->getFullPoseEuler();
        qInfo() << "X:" << pose.x  << "// Y:" << pose.y << "// Z:" << pose.z << "// RX:" << pose.rx << "// RY:" << pose.ry << "// RZ:" << pose.rz;
    }
    catch(const Ice::Exception &e){ /*std::cout << e.what() <<  __FUNCTION__ << std::endl;*/};

    if( auto robot = G->get_node(robot_name); robot.has_value())
    {
        if( auto parent = G->get_parent_node(robot.value()); parent.has_value())
        {
            if (are_different(std::vector < float > {pose.x, pose.y, pose.rz},
                              std::vector < float > {last_state.x, last_state.y, last_state.rz},
                              std::vector < float > {1, 1, 0.05}))
            {
                auto edge = rt->get_edge_RT(parent.value(), robot->id()).value();

                qInfo() << "Real POS:" << pose.x << pose.y << pose.rz;
//                std::cout << convertion_matrix << std::endl;
//                Eigen::Vector2f converted_pos = (convertion_matrix * Eigen::Vector3f(pose.x, pose.y, 1.f)).head(2);

//                qInfo() << "CONVERTED POS:" << converted_pos.x() << converted_pos.y();
//                G->modify_attrib_local<rt_rotation_euler_xyz_att>(edge, std::vector < float > {0.0, 0.0, pose.rz});
//                G->modify_attrib_local<rt_translation_att>(edge, std::vector < float > {pose.x, pose.y, 0.0});
                if (auto grid_node = G->get_node(grid_type_name); grid_node.has_value()){
                    if (auto grid_active = G->get_attrib_by_name<grid_activated_att>(grid_node.value())){
                        if (grid_active.value())
                        {
                            qInfo() << "VALORES";
                            G->modify_attrib_local<rt_rotation_euler_xyz_att>(edge, std::vector < float > {0.0, 0.0, pose.rz});
                            G->modify_attrib_local<rt_translation_att>(edge, std::vector < float > {pose.x, pose.y, 0.0});
                        }
                        else
                        {
                            qInfo() << "A CERO";
                            G->modify_attrib_local<rt_rotation_euler_xyz_att>(edge, std::vector < float > {0.0, 0.0, 0.0});
                            G->modify_attrib_local<rt_translation_att>(edge, std::vector < float > {0.0, 0.0, 0.0});
                        }
                    }
                }


//                G->add_or_modify_attrib_local<rt_translation_grid_att>(edge, std::vector < float > {converted_pos.x(), converted_pos.y(), 0.0});
                // TODO: maybe it could be changed too
                G->modify_attrib_local<rt_translation_velocity_att>(edge, std::vector<float>{pose.vx, pose.vy, pose.vz});
                G->modify_attrib_local<rt_rotation_euler_xyz_velocity_att>(edge, std::vector<float>{pose.vrx, pose.vry, pose.vrz});
                // linear velocities are WRT world axes, so local speed has to be computed WRT to the robot's moving frame
                float side_velocity = -sin(pose.rz) * pose.vx + cos(pose.rz) * pose.vy;
                float adv_velocity = -cos(pose.rz) * pose.vx + sin(pose.rz) * pose.vy;
                G->insert_or_assign_edge(edge);
//                std::cout << "VELOCITIES: " << adv_velocity << " " << pose.vrz << std::endl;


//                G->add_or_modify_attrib_local<robot_local_linear_velocity_att>(robot.value(), std::vector<float>{adv_velocity, side_velocity, pose.rz});
//                G->add_or_modify_attrib_local<robot_ref_adv_speed_att>(robot.value(), adv_velocity);
//                G->add_or_modify_attrib_local<robot_ref_rot_speed_att>(robot.value(), pose.vrz);
//
                G->update_node(robot.value());
                last_state = pose;
            }
        }
        else  qWarning() << __FUNCTION__ << " No parent found for node " << QString::fromStdString(robot_name);
    }
    else    qWarning() << __FUNCTION__ << " No node " << QString::fromStdString(robot_name);
}
void SpecificWorker::update_camera_rgbd(std::string camera_name, const cv::Mat &v_image, float focalx, float focaly)
{

    if( auto node = G->get_node(camera_name); node.has_value())
    {
        std::vector<uint8_t> rgb; rgb.assign(v_image.data, v_image.data + v_image.total()*v_image.channels());
        G->add_or_modify_attrib_local<cam_rgb_att>(node.value(),  rgb);
        G->add_or_modify_attrib_local<cam_rgb_width_att>(node.value(), v_image.cols);
        G->add_or_modify_attrib_local<cam_rgb_height_att>(node.value(), v_image.rows);
        G->add_or_modify_attrib_local<cam_rgb_depth_att>(node.value(), v_image.depth());
        G->add_or_modify_attrib_local<cam_rgb_cameraID_att>(node.value(), 3);
        G->add_or_modify_attrib_local<cam_rgb_focalx_att>(node.value(), (int)focalx);
        G->add_or_modify_attrib_local<cam_rgb_focaly_att>(node.value(), (int)focaly);
        G->add_or_modify_attrib_local<cam_rgb_alivetime_att>(node.value(), (int)std::chrono::time_point_cast<std::chrono::milliseconds>(MyClock::now()).time_since_epoch().count());

        G->update_node(node.value());
    }
    else
        qWarning() << __FUNCTION__ << "Node camera_rgbd not found";
}
void SpecificWorker::update_servo_position()
{
    try
    {
        auto servo_data = this->jointmotorsimple_proxy->getMotorState("eye_motor");
        float servo_position = (float)servo_data.pos;
        float servo_vel = (float)servo_data.vel;
        bool moving = servo_data.isMoving;
        if( auto servo = G->get_node("servo"); servo.has_value())
        {
            G->add_or_modify_attrib_local<servo_pos_att>(servo.value(), servo_position);
            G->add_or_modify_attrib_local<servo_speed_att>(servo.value(), servo_vel);
            G->add_or_modify_attrib_local<servo_moving_att>(servo.value(), moving);
            G->update_node(servo.value());
        }
    }
    catch(const Ice::Exception &e){ /*std::cout << e.what() <<  __FUNCTION__ << std::endl;*/};
}
void SpecificWorker::update_laser(const std::vector<LaserPoint> &laser_data)
{
    if( auto node = G->get_node(laser_name); node.has_value())
    {
        // Transform laserData into two std::vector<float>
        std::vector<float> dists;
        std::transform(laser_data.begin(), laser_data.end(), std::back_inserter(dists), [](const auto &l) { return l.dist; });
        std::vector<float> angles;
        std::transform(laser_data.begin(), laser_data.end(), std::back_inserter(angles), [](const auto &l) { return l.angle; });

        // update laser in DSR
        G->add_or_modify_attrib_local<laser_dists_att>(node.value(), dists);
        G->add_or_modify_attrib_local<laser_angles_att>(node.value(), angles);
        G->update_node(node.value());
    }
    else
        qWarning() << __FUNCTION__ << "No laser node found";
}
bool SpecificWorker::are_different(const std::vector<float> &a, const std::vector<float> &b, const std::vector<float> &epsilon)
{
    for(auto &&[aa, bb, e] : iter::zip(a, b, epsilon))
        if (fabs(aa - bb) > e)
            return true;
    return false;
}
Eigen::Matrix3f SpecificWorker::get_new_grid_matrix(Eigen::Vector3d robot_position, Eigen::Vector3d robot_rotation)
{
    // build the matrix to transform from robot to local_grid, knowing robot and grid pose in world
    Eigen::Matrix3f r2w;
    qInfo() << __FUNCTION__;
    r2w << cos((float) robot_rotation.z()), -sin((float) robot_rotation.z()), (float) robot_position.x(),
            sin((float) robot_rotation.z()), cos((float) robot_rotation.z()), (float) robot_position.y(),
            0.f, 0.f, 1.f;
//    Eigen::Matrix2f w2g_2d_matrix;
//    w2g_2d_matrix <<  cos(grid_world_pose.get_ang()), sin(grid_world_pose.get_ang()),
//            -sin(grid_world_pose.get_ang()), cos(grid_world_pose.get_ang());
//    auto tr = w2g_2d_matrix * grid_world_pose.get_pos();
//    Eigen::Matrix3f w2g;
//    w2g << cos(grid_world_pose.get_ang()), sin(grid_world_pose.get_ang()), -tr.x(),
//            -sin(grid_world_pose.get_ang()), cos(grid_world_pose.get_ang()), -tr.y(),
//            0.f, 0.f, 1.f;
//    Eigen::Matrix3f r2g = w2g * r2w;  // from r to world and then from world to grid
    return r2w.inverse();
}
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}
//////////////////// SLOTS ////////////////////
void SpecificWorker::modify_node_slot(const std::uint64_t id, const std::string &type)
{
//    std::cout << type << std::endl;
    if (type == "grid")
    {
        qInfo() << __FUNCTION__ << "GRID";
        if(auto grid_node = G->get_node(id); grid_node.has_value())
        {
            if(auto grid_activated = G->get_attrib_by_name<grid_activated_att>(grid_node.value()); grid_activated.has_value())
            {
                qInfo() << "PYREP SEND INITIAL POSE";
                this->fullposeestimation_proxy->setInitialPose(0.0,0.0,0.0,0.0,0.0,0.0);
//                if(auto robot_pose = inner_eigen->transform(world_name, robot_name); robot_pose.has_value())
//                {
//                    if(auto robot_rotation_3d = inner_eigen->get_euler_xyz_angles(world_name, robot_name); robot_rotation_3d.has_value())
//                    {
//                        convertion_matrix = get_new_grid_matrix(robot_pose.value(), robot_rotation_3d.value());
//                    }
//                }
            }
        }
    }

//     if (type == "servo")
//     {
//        qInfo() << __FUNCTION__ << "SERVO";
//         //            // servo
//         if (auto servo = G->get_node("servo"); servo.has_value())
//         {
// //            std::cout << "ENTERING IN SERVO" << std::endl;
//             if(auto servo_send_pos = G->get_attrib_by_name<servo_ref_pos_att>(servo.value()); servo_send_pos.has_value())
//             {
//                 qInfo() << "1";
//                 float servo_pos = servo_send_pos.value();
//                 if(auto servo_send_speed = G->get_attrib_by_name<servo_ref_speed_att>(servo.value()); servo_send_speed.has_value())
//                 {
//                     qInfo() << "1";
//                     float servo_speed = servo_send_speed.value();
//                     servo_pos_anterior = servo_pos;
//                     servo_speed_anterior =  servo_speed;

//                     try {
//                         std::cout << "SENDING POSITION" << std::endl;
//                         RoboCompJointMotorSimple::MotorGoalPosition goal;
//                         goal.maxSpeed = (float)servo_speed;
//                         goal.position = (float)servo_pos;
//                         this->jointmotorsimple_proxy->setPosition("", goal);
//                     }
//                     catch (const RoboCompGenericBase::HardwareFailedException &re) {
// //                        std::cout << __FUNCTION__ << "Exception setting base speed " << re << '\n';
// //                        std::cout << __FUNCTION__ << "Exception setting base speed " << re << '\n';
//                     }
//                     catch (const Ice::Exception &e) {
//                         //std::cout << e.what() << '\n';
//                     }
//                 }
//             }

//         }
//     }

    if (type == omnirobot_type_name)   // pasar al SLOT the change attrib
    {
//        qInfo() << __FUNCTION__ << "ROBOT";
//        qInfo() << __FUNCTION__  << " Dentro " << id << QString::fromStdString(type);
        if (auto robot = G->get_node(robot_name); robot.has_value())
        {
            // speed
            auto ref_adv_speed = G->get_attrib_by_name<robot_ref_adv_speed_att>(robot.value());
            auto ref_rot_speed = G->get_attrib_by_name<robot_ref_rot_speed_att>(robot.value());
            auto ref_side_speed = G->get_attrib_by_name<robot_ref_side_speed_att>(robot.value());
//            qInfo() << __FUNCTION__ << ref_adv_speed.has_value() << ref_rot_speed.has_value();
            if (ref_adv_speed.has_value() and ref_rot_speed.has_value() and ref_side_speed.has_value())
            {
                //comprobar si la velocidad ha cambiado y el cambio es mayor de 10mm o algo así, entonces entra y tiene que salir estos mensajes
                std::cout << __FUNCTION__  <<endl;
                // Check de values are within robot's accepted range. Read them from config
                //const float lowerA = -10, upperA = 10, lowerR = -10, upperR = 5, lowerS = -10, upperS = 10;
                //std::clamp(ref_adv_speed.value(), lowerA, upperA);
                float adv = ref_adv_speed.value();
                float rot = ref_rot_speed.value();
                float side = ref_side_speed.value();
                //float inc = 10.0;
//                cout << __FUNCTION__ << "adv " << adv << " rot " << rot << endl;
                if ( adv != av_anterior or rot != rot_anterior or side != side_anterior)
                {
                    std::cout<< "..................................."<<endl;
//                    std::cout << __FUNCTION__ << " " << ref_adv_speed.value() << " " << ref_rot_speed.value()
//                              << std::endl;
                    av_anterior = adv;
                    rot_anterior = rot;
                    side_anterior = side;
                    try {
//                        differentialrobot_proxy->setSpeedBase(ref_adv_speed.value(), ref_rot_speed.value());
                        omnirobot_proxy->setSpeedBase(ref_side_speed.value(), ref_adv_speed.value(), ref_rot_speed.value());
                        std::cout << "VELOCIDADES: " << ref_adv_speed.value() << " " << ref_rot_speed.value() << " " << ref_side_speed.value() << std::endl;
                    }
                    catch (const RoboCompGenericBase::HardwareFailedException &re) {
                        std::cout << __FUNCTION__ << "Exception setting base speed " << re << '\n';
                    }
                    catch (const Ice::Exception &e) {
                        std::cout << e.what() << '\n';
                    }
                }
            }
        }
    }
}

/**************************************/
// From the RoboCompCameraRGBDSimple you can call this methods:
// this->camerargbdsimple_proxy-f>getAll(...)
// this->camerargbdsimple_proxy->getDepth(...)
// this->camerargbdsimple_proxy->getImage(...)

/**************************************/
// From the RoboCompCameraRGBDSimple you can use this types:
// RoboCompCameraRGBDSimple::TImage
// RoboCompCameraRGBDSimple::TDepth
// RoboCompCameraRGBDSimple::TRGBD

/**************************************/
// From the RoboCompCameraSimple you can call this methods:
// this->camerasimple_proxy->getImage(...)

/**************************************/
// From the RoboCompCameraSimple you can use this types:
// RoboCompCameraSimple::TImage

/**************************************/
// From the RoboCompCoppeliaUtils you can call this methods:
// this->coppeliautils_proxy->addOrModifyDummy(...)
// this->coppeliautils_proxy->setDummySpeed(...)

/**************************************/
// From the RoboCompCoppeliaUtils you can use this types:
// RoboCompCoppeliaUtils::PoseType
// RoboCompCoppeliaUtils::SpeedType

/**************************************/
// From the RoboCompFullPoseEstimation you can call this methods:
// this->fullposeestimation_proxy->getFullPoseEuler(...)
// this->fullposeestimation_proxy->getFullPoseMatrix(...)
// this->fullposeestimation_proxy->setInitialPose(...)

/**************************************/
// From the RoboCompFullPoseEstimation you can use this types:
// RoboCompFullPoseEstimation::FullPoseMatrix
// RoboCompFullPoseEstimation::FullPoseEuler

/**************************************/
// From the RoboCompJointMotorSimple you can call this methods:
// this->jointmotorsimple_proxy->getMotorParams(...)
// this->jointmotorsimple_proxy->getMotorState(...)
// this->jointmotorsimple_proxy->setPosition(...)
// this->jointmotorsimple_proxy->setVelocity(...)
// this->jointmotorsimple_proxy->setZeroPos(...)

/**************************************/
// From the RoboCompJointMotorSimple you can use this types:
// RoboCompJointMotorSimple::MotorState
// RoboCompJointMotorSimple::MotorParams
// RoboCompJointMotorSimple::MotorGoalPosition
// RoboCompJointMotorSimple::MotorGoalVelocity

/**************************************/
// From the RoboCompLaser you can call this methods:
// this->laser_proxy->getLaserAndBStateData(...)
// this->laser_proxy->getLaserConfData(...)
// this->laser_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

/**************************************/
// From the RoboCompLaser you can call this methods:
// this->laser1_proxy->getLaserAndBStateData(...)
// this->laser1_proxy->getLaserConfData(...)
// this->laser1_proxy->getLaserData(...)

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

/**************************************/
// From the RoboCompOmniRobot you can call this methods:
// this->omnirobot_proxy->correctOdometer(...)
// this->omnirobot_proxy->getBasePose(...)
// this->omnirobot_proxy->getBaseState(...)
// this->omnirobot_proxy->resetOdometer(...)
// this->omnirobot_proxy->setOdometer(...)
// this->omnirobot_proxy->setOdometerPose(...)
// this->omnirobot_proxy->setSpeedBase(...)
// this->omnirobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompOmniRobot you can use this types:
// RoboCompOmniRobot::TMechParams

/**************************************/
// From the RoboCompRealSenseFaceID you can call this methods:
// this->realsensefaceid_proxy->authenticate(...)
// this->realsensefaceid_proxy->enroll(...)
// this->realsensefaceid_proxy->eraseAll(...)
// this->realsensefaceid_proxy->eraseUser(...)
// this->realsensefaceid_proxy->getQueryUsers(...)
// this->realsensefaceid_proxy->startPreview(...)
// this->realsensefaceid_proxy->stopPreview(...)

/**************************************/
// From the RoboCompRealSenseFaceID you can use this types:
// RoboCompRealSenseFaceID::UserData

