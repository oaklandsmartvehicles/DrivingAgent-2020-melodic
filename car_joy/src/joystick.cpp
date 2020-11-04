#include "joystick.hpp"

namespace Joystick
{
  joystick::joystick(ros::NodeHandle& n, ros::NodeHandle& pn)
  {
    
    joy_sub_ = n.subscribe<sensor_msgs::Joy>("joy", 10, &joystick::recvjoydata, this);

  }
  void joystick::recvjoydata(const sensor_msgs::Joy::ConstPtr& joy)
        {
          // std::cout<< joy->axes[0] <<"\n";
          // ######################################################################____________ up left gear _____________________________##############################################

          
          // ############################################ left side returns +1 while right side -1 the scale is from (1 ---> -1)
          double left_Gear_X_axis = joy->axes[0];

                        // canelling some noise from (-0.1 ----> 0.1)
                        if (left_Gear_X_axis > -0.1 && left_Gear_X_axis < 0.1)
                          {
                            left_Gear_X_axis = 0 ;
                          }
   ROS_INFO_STREAM( "left_Gear_X_axis is :: "<< left_Gear_X_axis );
            // ############################################ up side returns +1 while down side -1 the scale is from (1 ----> -1)
          double left_Gear_Y_axis = joy->axes[1];

                        // canelling some noise from (-0.1 ----> 0.1)
                        if (left_Gear_Y_axis > -0.1 && left_Gear_Y_axis < 0.1)
                          {
                            left_Gear_Y_axis = 0 ;
                          }
    ROS_INFO_STREAM( "left_Gear_Y_axis is::   " << left_Gear_Y_axis);
            // ############################################ LT = +1  if it not pressed and goes gradually to -1 if it fully pressed    scale is (+1 -----> -1 ) 
          double lt = joy->axes[2];

          ROS_INFO_STREAM( "left_Gear_X_axis is :: "<< left_Gear_X_axis << "left_Gear_Y_axis is::   " << left_Gear_Y_axis) ;



            // ######################################################################____________ down right gear _____________________________##############################################


          // ############################################ left side returns +1 while right side -1 the scale is from (1 __  -1)
          double right_Gear_X_axis = joy->axes[3];
            // ############################################ up side returns +1 while down side -1 the scale is from (1 __  -1)
          double right_Gear_Y_axis = joy->axes[4];
            // ############################################ LT = +1  if it not pressed and goes gradually to -1 if it fully pressed    scale is (+1 __ -1 ) 
          double rt = joy->axes[5];
          

          // ######################################################################____________ plus  _____________________________##############################################


          // ############################################ left side returns +1 while right side -1 the scale is from (1 __  -1)
          double plus_X_axis = joy->axes[6];
            // ############################################ up side returns +1 while down side -1 the scale is from (1 __  -1)
          double plus_Y_axis = joy->axes[7];
            

          

          // ######################################################################____________ Buttoms _____________________________##############################################
          // ############################################ when its pressed = 1   else = 0




          double buttom_A                     = joy->buttons[0];
          double buttom_B                     = joy->buttons[1];
          double buttom_X                     = joy->buttons[2];
          double buttom_Y                     = joy->buttons[3];
          double buttom_LB                    = joy->buttons[4];
          double buttom_RB                    = joy->buttons[5];
          double buttom_BACK                  = joy->buttons[6];
          double buttom_START                 = joy->buttons[7];
          double buttom_XBOX                  = joy->buttons[8];
          double buttom_left_gear_button      = joy->buttons[9];
          double buttom_right_gear_button     = joy->buttons[10];
          

          // plus 

          double arrow_x_l = joy->buttons[11];
          double arrow_x_r = joy->buttons[12];
          double arrow_y_u = joy->buttons[13];
          double arrow_y_d = joy->buttons[14];
          
          
            

          

          


  }


  
} // namespace name
