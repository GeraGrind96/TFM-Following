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

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include "dsr/api/dsr_api.h"
#include "dsr/gui/dsr_gui.h"
#include <doublebuffer/DoubleBuffer.h>
#include "/home/robocomp/robocomp/components/robocomp-shadow/etc/graph_names.h"
#include "/home/robocomp/robocomp/components/robocomp-shadow/etc/plan.h"

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	void AgenteConversacional_asynchronousIntentionReceiver(int intention);
	void AgenteConversacional_componentState(int state);
	int AgenteConversacional_situationChecking();


public slots:
	void compute();
	int startup_check();
	void initialize(int period);
private:
	// DSR graph
	std::shared_ptr<DSR::DSRGraph> G;

	//DSR params
	std::string agent_name;
	int agent_id;

	bool tree_view;
	bool graph_view;
	bool qscene_2d_view;
	bool osg_3d_view;

	// DSR graph viewer
	std::unique_ptr<DSR::DSRViewer> graph_viewer;
	QHBoxLayout mainLayout;
	void modify_node_slot(std::uint64_t id, const std::string &type);
	void modify_attrs_slot(std::uint64_t id, const std::vector<std::string>& att_names){};
	void modify_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type);

	void del_edge_slot(std::uint64_t from, std::uint64_t to, const std::string &edge_tag);
	void del_node_slot(std::uint64_t from);
	bool startup_check_flag;

    // Functions for planing
    void create_ask_for_follow_plan();
    void create_ask_for_stop_following_plan();
	void create_talking_plan();
	void create_ask_for_stop_talking_plan();
    void start_mission(int intention);
    void stop_mission();
    void insert_intention_node(const Plan &plan);
    uint64_t node_string2id(Plan currentPlan);

	bool first_follow = true; // To avoid say "te sigo" every time a mission is created

	// Functions for creating nodes
	void create_waiting_person_node();
	void remove_waiting_person_node();

    // Person data
    std::string interest_person_name;
    uint64_t interest_person_node_id;
    uint64_t interest_robot_intention_node_id;

    // Missions
    DoubleBuffer<Plan, Plan> plan_buffer;
    Plan temporary_plan;
    Plan current_plan;
};

#endif
