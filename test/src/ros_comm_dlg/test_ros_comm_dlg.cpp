
#include <QtTest/qtest.h>
#include "../../include/commont_test.hpp"

#include "../../../include/motion_manager/roscommdialog.hpp"
#include "../../../include/motion_manager/qnode.hpp"

using namespace motion_manager;
using namespace Qt;


class TestRosComm : public QObject
{
    Q_OBJECT

public:
    TestRosComm();

private Q_SLOTS:
    void test_master_url();
};

TestRosComm::TestRosComm()
{

}

void TestRosComm::test_master_url()
{
    RosCommDialog rc;
    QString str = "Master url";
    rc.setMasterUrl(str);
    QCOMPARE(rc.getMasterUrl(), str);

}


QTEST_MAIN(TestRosComm)

#include "test_ros_comm_dlg.moc"
