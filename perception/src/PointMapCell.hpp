#include <vector>
#include "Point.hpp"


struct MetaData
{
    bool occupied;
    float min_z;
    float max_z;
};

struct PointUpdate
{
    int id;
    std::vector<Point> points;
    MetaData data;
};

class PointMapCell{
public:

    void AddUpdate(const int& update_id, const std::vector<Point>& points);
    void RemoveUpdate(const int& id);
    void UpdateMetaData();
    void SetSize(const double& x, const double& y);
    void SetAngularWindow(const double& left, const double& right) { a1 = left; a2 = right;}
    void SetDistance(const double& distance){r = distance;}
    double GetDistance(){return r;}
    std::pair<double, double> GetAngularWindow(){return std::pair<double, double> (a1,a2);}

    std::vector<PointUpdate> updates;
    MetaData meta_data;

    double a1;
    double a2;
    
private:
    double r;
    double x_size;
    double y_size;

};