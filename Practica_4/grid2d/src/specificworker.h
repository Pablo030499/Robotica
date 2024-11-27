/*
 *    Copyright (C) 2024 by YOUR NAME HERE
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

#define HIBERNATION_ENABLED

#include <genericworker.h>
#include "Lidar3D.h"
#include <Eigen/Dense>
#include "abstract_graphic_viewer/abstract_graphic_viewer.h"
#include <expected>
#include <random>
#include <doublebuffer_sync/doublebuffer_sync.h>
#include <locale>
#include <qcustomplot/qcustomplot.h>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	RoboCompGrid2D::Result Grid2D_getPaths(RoboCompGrid2D::TPoint source, RoboCompGrid2D::TPoint target);


public slots:
	void initialize();
	void compute();
	void emergency();
	void restore();
	int startup_check();
private:
	bool startup_check_flag;
	struct Params
	{
		float ROBOT_WIDTH = 460;  // mm
		float ROBOT_LENGTH = 480;  // mm
		float MAX_ADV_SPEED = 1900; // mm/s
		float MAX_ROT_SPEED = 2; // rad/s
		float SEARCH_ROT_SPEED = 0.9; // rad/s
		float STOP_THRESHOLD = 700; // mm
		float ADVANCE_THRESHOLD = ROBOT_WIDTH * 3; // mm
		float LIDAR_FRONT_SECTION = 0.2; // rads, aprox 12 degrees
		// person
		float PERSON_MIN_DIST = 800; // mm
		int MAX_DIST_POINTS_TO_SHOW = 300; // points to show in plot
		// lidar
		std::string LIDAR_NAME_LOW = "bpearl";
		std::string LIDAR_NAME_HIGH = "helios";
		QRectF GRID_MAX_DIM{-5000, 2500, 10000, -5000};
		// control track
		float acc_distance_factor = 2;
		float k1 = 1.1;  // proportional gain for the angle error;
		float k2 = 0.5; // proportional gain for derivative of the angle error;
	};
	Params params;

	//Casillas

	enum class State
	{
		OCUPADA, LIBRE, DESCONOCIDO
	};
	struct TCell {State state; QGraphicsRectItem *item;  /* other fields */};

	constexpr static int DIMENSION = 5000;
	static constexpr  int CELLS = 50;
	static constexpr int CELL_SIZE = DIMENSION/CELLS;

	using TGrid = std::array<std::array<TCell, CELLS>, CELLS>;
	TGrid grid;

	// Given two array coordinates (i,j), returns world coordinates (x,y)
	QPointF grid_to_world(int i, int j);

	// Given two world coordinates (x,y), returns world coordinates (i,j)
	QPointF world_to_grid(const int &x, const int &y);

	// lidar
	std::vector<Eigen::Vector2f> read_lidar_bpearl();


	//draw
	AbstractGraphicViewer *viewer;
	void draw_lidar(auto &filtered_points, QGraphicsScene *scene);
	QGraphicsPolygonItem *robot_draw;

	// QCustomPlot object
	QCustomPlot *plot;
};




#endif
