#pragma once
#include <utility>
#include <map>
#include <vector>
#include "PointMapCell.hpp"


/*!
    ObstructionMap:
    Creates a 2D grid where LiDAR data points are segregated into cells of the grid.
    the cells contain meta data describing the points enclosed. For example maximum height,
    average intensity or whatever is useful. The purpose being to create a 2D map which is
    computationaly inexpensive to operate on.
*/
class PointMap {
public:
    PointMap();

    void SetSize(const double& x, const double& y);
    std::pair<double, double> GetSize();

    void SetNumCells(const size_t& x_cells, const size_t& y_cells);
    std::pair<int, int> GetNumCells();

    std::pair<double, double> GetCellSize();

    void SetHeightCutOff(const double& z) {z_cut_off = z;}

    int AddLIDARPoints(const std::vector<Point>& points);

    void RemoveUpdate(int ID);

    void ClearMap();

    std::map< std::pair< int, int> , PointMapCell> cell_map;

    void UpdateCellDimensions();

private:
    

    double x_size, y_size;
    int x_num_cells, y_num_cells;
    double x_cell_size, y_cell_size;
    double z_cut_off;
    

    unsigned int update_counter;
    
};