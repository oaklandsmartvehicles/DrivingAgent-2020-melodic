#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "Perception.h"

namespace perception
{

class PerceptionNodelet : public nodelet::Nodelet
{
public:
  PerceptionNodelet()
  {
  }
  ~PerceptionNodelet()
  {
  }

  void onInit(void)
  {
    node_.reset(new Perception(getNodeHandle(), getPrivateNodeHandle()));
  }

private:
  boost::shared_ptr<Perception> node_;
};

}

#if ROS_VERSION_MINOR > 12
PLUGINLIB_EXPORT_CLASS(perception::PerceptionNodelet, nodelet::Nodelet);
#else
PLUGINLIB_DECLARE_CLASS(perception, PerceptionNodelet, perception::PerceptionNodelet, nodelet::Nodelet);
#endif
