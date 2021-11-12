/***********************************************************************************************************

（1）新建地图、保存地图、编辑地图
（2）设置单点导航、多点巡航、保存设置内容
（3）类rviz设计，可视化操作
（4）键盘控制节点
（5）自定义功能、自定义单点导航名字

***********************************************************************************************************/

#include "../include/robot_one/qrviz.hpp"

#include <QDebug>
#include <QException>

QRviz::QRviz(QVBoxLayout* layout)
{
  //创建rviz panel
  render_panel_=new rviz::RenderPanel();
  //向layout添加
  layout->addWidget(render_panel_);
  //创建rviz控制对象
  manager_=new rviz::VisualizationManager(render_panel_);
  tool_manager_=manager_->getToolManager();
  ROS_ASSERT(manager_!=NULL);
  //初始化render_panel 实现放大缩小等操作
  render_panel_->initialize(manager_->getSceneManager(),manager_);

  manager_->setFixedFrame("map");
  //初始化rviz控制对象
  manager_->initialize();
  manager_->startUpdate();
  manager_->removeAllDisplays();
  //全部打开
  Display_Grid(100,QColor(160,160,164),true);
//  Display_TF();
  Display_LaserScan(QString("scan"),true);
  Display_Map(QString("map"),QString("map"),true);

  Display_Marker(QString("/marker"),true);
  Display_Path(QString("/move_base/NavfnROS/plan"),true);
  Display_Lpath(QString("/move_base/DWAPlannerROS/local_plan"),true);
  Display_Robot(QString("/move_base/local_costmap/footprint"));
  Display_CostMap(QString("/move_base/local_costmap/costmap"),QString("costmap"));
  Display_RobotModel();
  //容易导致应用卡顿，所以注释掉了
// Display_GCostMap(QString("/move_base/global_costmap/costmap"),QString("costmap"));
//  Display_PoseArray(QString("/move_base/TebLocalPlannerROS/teb_poses"),true);

}

void QRviz::Set_FixedFrame(QString Frame_name)
{
    manager_->setFixedFrame(Frame_name);
    qDebug()<<manager_->getFixedFrame();
}

void QRviz::Display_Grid(int Cell_Count,QColor color,bool enable)
{
    if(Grid_!=NULL)
    {
        delete Grid_;
        Grid_=NULL;
    }
    Grid_=manager_->createDisplay("rviz/Grid","myGrid",enable);
    //设置cell Count
    Grid_->subProp("Plane Cell Count")->setValue(Cell_Count);
    //设置颜色
    Grid_->subProp("Color")->setValue(color);


    ROS_ASSERT(Grid_!=NULL);
}

void QRviz::Display_TF(bool enable)
{
    if(TF_!=NULL)
    {
        delete TF_;
        TF_=NULL;
    }
    TF_=manager_->createDisplay("rviz/TF","myTF",enable);
    ROS_ASSERT(TF_!=NULL);
}

void QRviz::Display_LaserScan(QString laser_topic,bool enable)
{
    if(LaserScan_!=NULL)
    {
        delete LaserScan_;
        LaserScan_=NULL;
    }
    LaserScan_=manager_->createDisplay("rviz/LaserScan","myLaser",enable);
    LaserScan_->subProp("Topic")->setValue(laser_topic);
    LaserScan_->subProp("Color Transformer")->setValue("FlatColor");
    LaserScan_->subProp("Color")->setValue(QColor(255,0,0));
    LaserScan_->subProp("Size (m)")->setValue(0.1);
    ROS_ASSERT(LaserScan_!=NULL);
}

void QRviz::Display_Map(QString topic,QString color_scheme,bool enable)
{
    if(Map_!=NULL)
    {
        delete Map_;
        Map_=NULL;
    }
    Map_=manager_->createDisplay("rviz/Map","myMap",enable);
    ROS_ASSERT(Map_!=NULL);
    Map_->subProp("Topic")->setValue(topic);
    Map_->subProp("Color Scheme")->setValue(color_scheme);
}

void QRviz::Display_Marker(QString marker_topic,bool enable)
{

    if((Marker_)!=NULL)
    {
        delete Marker_;
        Marker_=NULL;
    }
    Marker_=manager_->createDisplay("rviz/Marker","myMarker",enable);
    ROS_ASSERT(Marker_!=NULL);
    (Marker_)->subProp("Marker Topic")->setValue(marker_topic);
//    qDebug()<< "test Display_Marker";
}

//**************************后续添加的**********************************//

void QRviz::Display_Path(QString path_topic,bool enable)
{
    if(Path_!=NULL)
    {
        delete Path_;
        Path_=NULL;
    }
    Path_=manager_->createDisplay("rviz/Path","mypath",enable);
    Path_->subProp("Topic")->setValue(path_topic);
    ROS_ASSERT(Path_!=NULL);
}
void QRviz::Display_Lpath(QString path_topic,bool enable)
{
    if(Lpath_!=NULL)
    {
        delete Lpath_;
        Lpath_=NULL;
    }
    Lpath_=manager_->createDisplay("rviz/Path","myLpath",enable);
    Lpath_->subProp("Topic")->setValue(path_topic);
    Lpath_->subProp("Color")->setValue(QColor(0, 12, 255));
    ROS_ASSERT(Lpath_!=NULL);
}
void QRviz::Display_Robot(QString topic)
{
    if(Robot_!=NULL)
    {
        delete Robot_;
        Robot_=NULL;
    }
    Robot_=manager_->createDisplay("rviz/Polygon","myPolygon",true);
    Robot_->subProp("Topic")->setValue(topic);
    ROS_ASSERT(Robot_!=NULL);
}
void QRviz::Display_CostMap(QString topic,QString color_scheme)
{
    if(CostMap_!=NULL)
    {
        delete CostMap_;
        CostMap_=NULL;
    }
    CostMap_=manager_->createDisplay("rviz/Map","myCostMap",true);
    ROS_ASSERT(CostMap_!=NULL);
    CostMap_->subProp("Topic")->setValue(topic);
    CostMap_->subProp("Color Scheme")->setValue(color_scheme);
}
void QRviz::Display_GCostMap(QString topic,QString color_scheme)
{
    if(GCostMap_!=NULL)
    {
        delete GCostMap_;
        GCostMap_=NULL;
    }
    GCostMap_=manager_->createDisplay("rviz/Map","myGcostMap",true);

    GCostMap_->subProp("Topic")->setValue(topic);
    GCostMap_->subProp("Color Scheme")->setValue(color_scheme);
    GCostMap_->subProp("Draw Behind")->setValue(true);
    ROS_ASSERT(GCostMap_!=NULL);
}

void QRviz::Display_PoseArray(QString posearray_topic,bool enable)
{
    if(PoseArray_!=NULL)
    {
        delete PoseArray_;
        PoseArray_=NULL;
    }
    PoseArray_=manager_->createDisplay("rviz/PoseArray","myPoseArray",enable);
    PoseArray_->subProp("Topic")->setValue(posearray_topic);
    ROS_ASSERT(PoseArray_!=NULL);
}
void QRviz::Display_RobotModel()
{
    if(RobotModel_!=NULL)
    {
        delete RobotModel_;
        RobotModel_=NULL;
    }
    RobotModel_=manager_->createDisplay("rviz/RobotModel","myRobotModel",true);
    ROS_ASSERT(RobotModel_!=NULL);
}

//********************************************************************//
void QRviz::Set_Start_Pose()
{
    rviz::Tool* current_tool_=tool_manager_->addTool("rviz/SetInitialPose");
    current_tool_->setCursor(Qt::ArrowCursor);

    //设置当前使用的工具
    tool_manager_->setCurrentTool(current_tool_);
}

void QRviz::Set_single_nav_Pose()
{
     rviz::Tool* current_tool_=tool_manager_->addTool("rviz/SetGoal");
     current_tool_->setCursor(Qt::ArrowCursor);
     //获取属性容器
     rviz::Property* pro=current_tool_->getPropertyContainer();
     //
     pro->subProp("Topic")->setValue("/single_goal");
     //设置当前使用的工具
     tool_manager_->setCurrentTool(current_tool_);
}

void QRviz::Set_points_nav_Pose()
{
    rviz::Tool* current_tool_=tool_manager_->addTool("rviz/SetGoal");
    current_tool_->setCursor(Qt::ArrowCursor);
    //获取属性容器
    rviz::Property* pro=current_tool_->getPropertyContainer();
    //
    pro->subProp("Topic")->setValue("/points_goal");
    //设置当前使用的工具
    tool_manager_->setCurrentTool(current_tool_);
}

void QRviz::Set_back_nav_Pose()
{
    rviz::Tool* current_tool_=tool_manager_->addTool("rviz/SetGoal");
    current_tool_->setCursor(Qt::ArrowCursor);
    //获取属性容器
    rviz::Property* pro=current_tool_->getPropertyContainer();
    //
    pro->subProp("Topic")->setValue("/back_pose");
    //设置当前使用的工具
    tool_manager_->setCurrentTool(current_tool_);
}
void QRviz::Set_FoucsCamera()
{
    //获取设置Pos的工具
    //添加工具

    rviz::Tool* current_tool_=tool_manager_->addTool("rviz/FocusCamera");
    current_tool_->setCursor(Qt::ArrowCursor);
    //设置当前使用的工具为SetInitialPose（实现在地图上标点）
    tool_manager_->setCurrentTool( current_tool_ );
     manager_->startUpdate();
}
//此为select工具，非Measure工具
void QRviz::Set_Select()
{
    //获取设置Pos的工具
    //添加工具

    rviz::Tool* current_tool_=tool_manager_->addTool("rviz/Select");
    current_tool_->setCursor(Qt::ArrowCursor);
    //设置当前使用的工具为SetInitialPose（实现在地图上标点）
    tool_manager_->setCurrentTool( current_tool_ );
     manager_->startUpdate();
}
