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
#include "specificworker.h"
#include <cppitertools/enumerate.hpp>
#include <cppitertools/range.hpp>
#include <QMouseEvent>
#include <QApplication>
#include <QWidget>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
	// Uncomment if there's too many debug messages
	// but it removes the possibility to see the messages
	// shown in the console with qDebug()
//	QLoggingCategory::setFilterRules("*.debug=false\n");
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
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
	

	return true;
}

/**
 * Draws LIDAR points onto a QGraphicsScene.
 *
 * This method clears any existing graphical items from the scene, then iterates over the filtered
 * LIDAR points to add new items. Each LIDAR point is represented as a colored rectangle. The point
 * with the minimum distance is highlighted in red, while the other points are drawn in green.
 *
 * @param filtered_points A collection of filtered points to be drawn, each containing the coordinates
 *                        and distance.
 * @param scene A pointer to the QGraphicsScene where the points will be drawn.
 */
void SpecificWorker::draw_lidar(auto &filtered_points, QGraphicsScene *scene)
{
	static std::vector<QGraphicsItem*> items;   // store items so they can be shown between iterations

	// remove all items drawn in the previous iteration
	for(auto i: items)
	{
		scene->removeItem(i);
		delete i;
	}
	items.clear();

	auto color = QColor(Qt::darkGreen);
	auto brush = QBrush(QColor(Qt::darkGreen));
	for(const auto &p : filtered_points)
	{
		auto item = scene->addRect(-50, -50, 100, 100, color, brush);
		item->setPos(p.x(), p.y());
		items.push_back(item);
	}
}

void SpecificWorker::initialize()
{
	std::cout << "Initialize worker" << std::endl;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		// Viewer
		viewer = new AbstractGraphicViewer(this->frame, params.GRID_MAX_DIM);
		auto [r, e] = viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
		robot_draw = r;
		viewer->setStyleSheet("background-color: lightGray;");
		this->resize(800, 700);
		viewer->show();

		//grid
		QPen pen(Qt::blue, 20);
		for ( const auto &[i, row] : grid | iter::enumerate)
			for ( const auto &[j, cell] : row | iter::enumerate)
			{
				cell.item = viewer->scene.addRect(-CELLS/2, -CELLS/2, CELLS, CELLS, pen);
				cell.item->setPos(grid_to_world(i, j));
				cell.state = State::DESCONOCIDO;
			}
		//connect
		connect(viewer, SIGNAL(new_mouse_coordinates(QPointF)),this ,SLOT(mouseClick(QPointF)));

		this->setPeriod(STATES::Compute, 100);
		//this->setPeriod(STATES::Emergency, 500);
	}
}

void SpecificWorker::compute()
{
	//TODO Recorre todos los puntos LiDAR y, para cada uno, calcula la ecuación de la recta desde el centro (robot) hasta el punto.
	try {
		const auto points = read_lidar_bpearl();
		compute_cells(points);
	}

	catch(const Ice::Exception &e) { std::cout << e.what() << std::endl; }
}

void SpecificWorker::compute_cells(auto points) {

	remove_cells_draw();

	qDebug() << "Se ha clicado el raton en la posicion " << mouse_pos.x() << ", " << mouse_pos.y();

	for(const auto &p: points)
	{
		auto saltos = p.norm() / CELL_SIZE;
		for(const auto &s: iter::range(0.f, 1.f, 1.f / saltos))
		{
			auto step = p * s;
			auto celda = world_to_grid(step.x(), step.y());
			int x_index = static_cast<int>(celda.x());
			//std::cout << "x: " << x_index << endl;
			int y_index = static_cast<int>(celda.y());
			//std::cout << "y: " << y_index << endl;

			if (not (x_index >= 0 && x_index < CELLS && y_index >= 0 && y_index < CELLS)) continue;
			grid[x_index][y_index].item->setBrush(QColor(Qt::green));
			grid[x_index][y_index].state = State::LIBRE;
		}

		auto celda = world_to_grid(p.x(), p.y());
		int x_index = static_cast<int>(celda.x());
		int y_index = static_cast<int>(celda.y());

		if (not (x_index >= 0 && x_index < CELLS && y_index >= 0 && y_index < CELLS)) continue;
		grid[x_index][y_index].item->setBrush(QColor(Qt::red));
		grid[x_index][y_index].state = State::OCUPADA;
	}
}

void SpecificWorker::mouseClick(QPointF p) {
	qDebug() << "Mouse click at " << p.x() << ", " << p.y();
	mouse_pos.x() = QPointF(p.x(), p.y());
}

void SpecificWorker::remove_cells_draw() {
	for(auto i: iter::range(0, CELLS)) {
		for(auto j: iter::range(0, CELLS)) {
			grid[i][j].item->setBrush(QColor(Qt::white));
		}
	}
}

QPointF SpecificWorker::grid_to_world(int i, int j) {
	int x  = (DIMENSION/CELL_SIZE)*i - DIMENSION/2;
	int y  = (-DIMENSION/CELL_SIZE)*j + DIMENSION/2;

	return QPointF(x, y);
}
/*
QPointF SpecificWorker::world_to_grid(int x, int y) {
	int i = CELL_SIZE/(DIMENSION)*x + CELLS;
	int j = -CELL_SIZE/(DIMENSION)*y + CELLS;

	return QPointF(i, j);
}
*/

QPointF SpecificWorker::world_to_grid(int x, int y) {
	int i = (x + DIMENSION/2) / CELL_SIZE;
	int j = (-y + DIMENSION/2) / CELL_SIZE;

	i = std::max(0, std::min(i, CELLS - 1));
	j = std::max(0, std::min(j, CELLS - 1));

	return QPointF(i, j);
}
/*
QPointF SpecificWorker::grid_to_world(int i, int j) {
	int x = (i - CELLS / 2) * CELL_SIZE;
	int y = (CELLS / 2 - j) * CELL_SIZE;

	return QPointF(x, y);
}
*/

std::vector<Eigen::Vector2f> SpecificWorker::read_lidar_bpearl()
{
	try
	{
		auto ldata =  lidar3d_proxy->getLidarData("bpearl", 0, 2*M_PI, 1);
		// filter points according to height and distance
		std::vector<Eigen::Vector2f>  p_filter;
		for(const auto &a: ldata.points)
		{
			if(a.z < 500 and a.distance2d > 200)
				p_filter.emplace_back(a.x, a.y);
		}
		return p_filter;
	}
	catch(const Ice::Exception &e){std::cout << e << std::endl;}
	return {};
}

void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
	//computeCODE
	//
	//if (SUCCESSFUL)
    //  emmit goToRestore()
}

//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
	//computeCODE
	//Restore emergency component

}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

RoboCompGrid2D::Result SpecificWorker::Grid2D_getPaths(RoboCompGrid2D::TPoint source, RoboCompGrid2D::TPoint target)
{
#ifdef HIBERNATION_ENABLED
	hibernation = true;
#endif
//implementCODE

}

/**************************************/
// From the RoboCompLidar3D you can call this methods:
// this->lidar3d_proxy->getLidarData(...)
// this->lidar3d_proxy->getLidarDataArrayProyectedInImage(...)
// this->lidar3d_proxy->getLidarDataProyectedInImage(...)
// this->lidar3d_proxy->getLidarDataWithThreshold2d(...)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData

/**************************************/
// From the RoboCompGrid2D you can use this types:
// RoboCompGrid2D::TPoint
// RoboCompGrid2D::Result

