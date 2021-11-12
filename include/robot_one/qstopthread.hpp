#ifndef robot_one_QSTOPTHREAD_HPP
#define robot_one_QSTOPTHREAD_HPP
#include <QThread>
#include "qnode.hpp"
namespace robot_one {
class QStopThread : public QThread
{
public:
     QStopThread(QNode* qnode);
    void run() override;
private:
    QNode* qnode1;

};
 }
#endif // SIMPLETHREADONE_H
