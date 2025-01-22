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
#include <queue>
#include <vector>
#include <limits>
#include <algorithm>

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
		Grid2D_getPaths(RoboCompGrid2D::TPoint(0.f, 0.f), RoboCompGrid2D::TPoint(mouse_pos.x(), mouse_pos.y()));
	}

	catch(const Ice::Exception &e) { std::cout << e.what() << std::endl; }
}

void SpecificWorker::set_ocuped_cells(int x_index, int y_index) {
	grid[x_index][y_index].item->setBrush(QColor(Qt::red));
	grid[x_index][y_index+1].state = State::OCUPADA;
	grid[x_index][y_index-1].item->setBrush(QColor(Qt::red));
	grid[x_index-1][y_index].state = State::OCUPADA;
	grid[x_index+1][y_index].item->setBrush(QColor(Qt::red));
	grid[x_index-1][y_index-1].state = State::OCUPADA;
	grid[x_index+1][y_index+1].item->setBrush(QColor(Qt::red));
	grid[x_index+1][y_index-1].state = State::OCUPADA;
	grid[x_index-1][y_index+1].item->setBrush(QColor(Qt::red));
	//rellenar_cells(x_index, y_index);
}

void SpecificWorker::rellenar_cells(int x_index, int y_index) {
		if(grid[x_index][y_index+1].state == State::OCUPADA && grid[x_index][y_index-1].state == State::OCUPADA
			&& grid[x_index-1][y_index].state == State::OCUPADA && grid[x_index+1][y_index].state == State::OCUPADA){
			grid[x_index][y_index+1].state = State::OCUPADA;
		}
}

void SpecificWorker::compute_cells(auto points) {

	remove_cells_draw();

	//qDebug() << "Se ha clicado el raton en la posicion " << mouse_pos.x() << ", " << mouse_pos.y();

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

		if (not (x_index > 0 && x_index < CELLS-1 && y_index > 0 && y_index < CELLS-1)) continue;
		set_ocuped_cells(x_index, y_index);
	}
}

void SpecificWorker::mouseClick(QPointF p) {
	qDebug() << "Mouse click at (" << p.x() << ", " << p.y() << ")";

	// Dibuja un marcador en el punto donde se hizo clic
	auto item = viewer->scene.addEllipse(p.x() - 5, p.y() - 5, 10, 10, QPen(Qt::blue), QBrush(Qt::blue));
	viewer->scene.update();
	mouse_pos.setX(p.x());
	mouse_pos.setY(p.y());

	qDebug() << "Mouse_pos click at (" << mouse_pos.x() << ", " << mouse_pos.y() << ")";
}

void SpecificWorker::remove_cells_draw() {
	for(auto i: iter::range(0, CELLS)) {
		for(auto j: iter::range(0, CELLS)) {
			grid[i][j].item->setBrush(QColor(Qt::white));
			grid[i][j].state = State::DESCONOCIDO;
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

QPointF SpecificWorker::world_to_grid(float x, float y) {
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

float calculate_euclidean_distance(int x1, int y1, int x2, int y2) {
	float dx = static_cast<float>(x2 - x1);
	float dy = static_cast<float>(y2 - y1);
	return std::sqrt(dx * dx + dy * dy);
}
struct Coordenada {
    int x, y;
    float distancia;

    bool operator>(const Coordenada& other) const {
        return distancia > other.distancia;  // Usamos '>' para la cola de prioridad (min-heap)
    }
};

std::vector<QPointF> SpecificWorker::dijkstra(int startX, int startY, int endX, int endY) {
    // Direcciones posibles: arriba, abajo, izquierda, derecha
    std::vector<std::pair<int, int>> direcciones = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {1, 1},{-1, 1},{-1, -1},{1, -1}};

    // Cola de prioridad para almacenar las celdas por distancia (usamos la estructura Coordenada)
    std::priority_queue<Coordenada, std::vector<Coordenada>, std::greater<Coordenada>> pq;

    // Inicializamos la celda de inicio
    grid[startX][startY].distancia = 0.0f;
    pq.push({startX, startY, 0.0f});

    // Inicializamos todas las celdas como no visitadas y con distancia infinita
    for (int i = 0; i < CELLS; ++i) {
        for (int j = 0; j < CELLS; ++j) {
            grid[i][j].visitada = false;
            grid[i][j].distancia = std::numeric_limits<float>::infinity();
        }
    }
    grid[startX][startY].distancia = 0.0f;

    // Procesamos las celdas mientras haya celdas en la cola
    while (!pq.empty()) {
        // Extraemos la celda con la menor distancia
        Coordenada actual = pq.top();
        pq.pop();

        int x = actual.x;
        int y = actual.y;

        // Si ya visitamos esta celda, la ignoramos
        if (grid[x][y].visitada) continue;
        grid[x][y].visitada = true;

        // Si hemos llegado al destino, terminamos
        if (x == endX && y == endY) {
            break;
        }

        // Explorar las celdas vecinas
        for (const auto& direccion : direcciones) {
            int nx = x + direccion.first;
            int ny = y + direccion.second;

            // Verificamos si la celda está dentro de los límites y es accesible
            if (nx >= 0 && nx < CELLS && ny >= 0 && ny < CELLS && grid[nx][ny].state != State::OCUPADA) {
                // Calculamos la nueva distancia usando la distancia euclidiana al punto final
                float nuevaDistancia = grid[x][y].distancia + calculate_euclidean_distance(x, y, nx, ny);

                // Si encontramos una distancia más corta para esta celda, la actualizamos
                if (nuevaDistancia < grid[nx][ny].distancia) {
                    grid[nx][ny].distancia = nuevaDistancia;
                    pq.push({nx, ny, nuevaDistancia});
                }
            }
        }
    }

    // Si encontramos un camino, reconstruimos el camino desde el destino hacia el origen
    std::vector<QPointF> path;
    int x = endX;
    int y = endY;

    while (!(x == startX && y == startY)) {
        // Añadimos la celda actual al camino usando world_to_grid para convertirla en QPointF
        path.push_back(QPointF(x, y));

        // Buscamos la celda anterior en el camino (la que tiene la menor distancia)
        float minDistancia = std::numeric_limits<float>::infinity();
        int prevX = x, prevY = y;

        for (const auto& direccion : direcciones) {
            int nx = x + direccion.first;
            int ny = y + direccion.second;

            if (nx >= 0 && nx < CELLS && ny >= 0 && ny < CELLS && grid[nx][ny].distancia < minDistancia) {
                minDistancia = grid[nx][ny].distancia;
                prevX = nx;
                prevY = ny;
            }
        }

        // Actualizamos la celda
        x = prevX;
        y = prevY;
    }

    // Añadimos el punto inicial al camino
    path.push_back(grid_to_world(startX, startY));

    // El camino está en orden inverso, por lo que lo revertimos
    std::reverse(path.begin(), path.end());

    return path;
}

void SpecificWorker::paintPath(const std::vector<QPointF>& path)
{
	for (int i = 0; i < path.size(); ++i)
	{
		grid[path[i].x()][path[i].y()].item->setBrush(QColor(Qt::yellow));
	}
}

void SpecificWorker::printPath(const std::vector<QPointF>& path)
{
	for (const auto& point : path)
	{
		qDebug() << "Punto en el camino:" << point.x() << point.y();
	}
}

bool SpecificWorker::validGoal(int goalX, int goalY){

	if (grid[goalX][goalY].state != State::OCUPADA) {
		return true;
	}
	return false;
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

	qDebug() << "Coordenadas meta: " << target.x << target.y;
	const QPointF index = world_to_grid(target.x, target.y);
	int goalX = index.x();
	int goalY = index.y();

	if (goalX < 0 || goalX >= CELLS || goalY < 0 || goalY >= CELLS || !(validGoal(goalX, goalY)))
	{
		qDebug() << "Punto fuera del grid. ";
		return {};
	}

	qDebug() << "Indices de la meta: " << goalX << goalY;
	int startX = CELLS / 2;
	int startY = CELLS / 2;

	auto path = dijkstra(startX, startY, goalX, goalY);
	printPath(path);
	paintPath(path);
	RoboCompGrid2D::Result result;
	std::ranges::transform(path, std::back_inserter(result.path), [](const auto& p)
			{ return RoboCompGrid2D::TPoint{p.x(), p.y()}; });
	result.timestamp = QDateTime::currentMSecsSinceEpoch();
	result.valid = !path.empty();
	return result;
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

