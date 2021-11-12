#include "../include/robot_one/qstopthread.hpp"
#include <QDebug>
#include "../include/robot_one/qnode.hpp"
namespace  robot_one{
QStopThread::QStopThread(QNode *qnode)
{
    qnode1=qnode;
}

void QStopThread::run()
{
    qnode1->set_stop_points_nav(true);
}

}
