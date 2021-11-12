#ifndef QRVIZ_HPP
#define QRVIZ_HPP

#include <QObject>
#include <ros/ros.h>
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>
#include <rviz/tool.h>
#include <QVBoxLayout>
//设置光标的图标
#include <rviz/load_resource.h>
#include <QCursor>


class QRviz
{
public:
  QRviz(QVBoxLayout* layout);
  void Set_FixedFrame(QString Frame_name);
  void Display_Grid(int Cell_Count,QColor color,bool enable);
  void Display_TF(bool enable);
  void Display_LaserScan(QString laser_topic,bool enable);
  void Display_Path(QString path_topic,bool enable);
  void Display_Lpath(QString path_topic,bool enable);
  void Display_Map(QString topic,QString color_scheme, bool enable);
  void Display_Marker(QString marker_topic,bool enable);
  void Set_Start_Pose();
  void Set_single_nav_Pose();
  void Set_back_nav_Pose();
  void Set_points_nav_Pose();
  void Display_Robot(QString topic);
  void Display_GCostMap(QString topic, QString color_scheme);
  void Display_CostMap(QString topic, QString color_scheme);
  void Display_PoseArray(QString posearray_topic, bool enable);
  void Set_FoucsCamera();
  void Set_Select();
  void Display_RobotModel();
private:
  //首先需要创建一个rviz的显示容器：
  rviz::RenderPanel* render_panel_;
  //创建rviz控制对象
  rviz::VisualizationManager* manager_;
  rviz::ToolManager* tool_manager_;
  rviz::Display* Grid_=NULL;
  rviz::Display* TF_=NULL;
  rviz::Display* LaserScan_=NULL;
  rviz::Display* Path_=NULL;
  rviz::Display* Lpath_=NULL;
  rviz::Display* Map_=NULL;
  rviz::Display* Marker_=NULL;
  rviz::Display* Robot_=NULL;
  rviz::Display* RobotModel_=NULL;
  rviz::Display* CostMap_=NULL;
  rviz::Display* GCostMap_=NULL;
  rviz::Display* PoseArray_=NULL;
  enum CursorType
  {
    Default,
    Rotate2D,
    Rotate3D,
    MoveXY
  };
  QMap<CursorType, QCursor> my_standard_cursors_;

};

#endif // QRVIZ_HPP
