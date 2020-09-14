#include "PointMapCell.hpp"

/*!
Removes a set of points from the cell based on the update ID.
@param id The ID of the update to be removed.
*/
void PointMapCell::RemoveUpdate(const int& id)
{
    auto it = updates.begin();
    while( it != updates.end() )
    {
        if(it->id == id)
        {
            updates.erase(it);
            return;
        }
    }
    UpdateMetaData();
    
}
void PointMapCell::UpdateMetaData()
{
    bool first = true;
    meta_data.max_z = 0;
    meta_data.min_z = 0;

    //for every point in the updates.
    for( auto update : updates )
    {
        if(first)
        {
            meta_data.max_z = update.data.max_z;
            meta_data.min_z = update.data.min_z;
            first = false;
            continue;
        }

        if(update.data.max_z > meta_data.max_z)
            meta_data.max_z = update.data.max_z;
        if(update.data.min_z < meta_data.min_z)
            meta_data.min_z = update.data.min_z;
    }
}
void PointMapCell::SetSize(const double& x, const double& y)
{
    x_size = x;
    y_size = y;

    
}

void PointMapCell::AddUpdate(const int& update_id, const std::vector<Point>& points)
{
    PointUpdate update;
    update.id = update_id;
    MetaData& update_meta_data = update.data;
    if(points.size() == 0)
        return;

    update_meta_data.occupied = true;

    //grab the first point in the cell and take it's z value as the minimum and maximum.
    update_meta_data.min_z = points[0].z;
    update_meta_data.max_z = points[0].z;

    //for every point in the updates.
    for( auto point : points)
    {
        if(point.z < update_meta_data.min_z)
            update_meta_data.min_z = point.z;
        if(point.z > update_meta_data.max_z)
            update_meta_data.max_z = point.z;
    }

    updates.push_back(update);
    UpdateMetaData();
}