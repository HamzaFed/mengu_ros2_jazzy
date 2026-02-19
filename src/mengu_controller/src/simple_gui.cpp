#include <QApplication>
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QSlider>
#include <QPushButton>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <thread>


int main(int argc, char *argv[])
{
    
    QApplication app(argc, argv);

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mengu_gui_node");

    auto arm_pub =
        node->create_publisher<trajectory_msgs::msg::JointTrajectory>("/arm_controller/joint_trajectory", 10);

    // Ana pencere
    QWidget window;
    window.setWindowTitle("Mengu Simple GUI");
    window.resize(600, 300);  

    // Ana dikey layout
    QVBoxLayout *mainLayout = new QVBoxLayout();
    window.setLayout(mainLayout);

    // joint_1 için yatay layout
    QHBoxLayout *jointLayout = new QHBoxLayout();

    QLabel *jointLabel = new QLabel("joint_1: 0");
    jointLabel->setMinimumWidth(80);

    QSlider *jointSlider = new QSlider(Qt::Horizontal);
    jointSlider->setRange(-180, 180);
    jointSlider->setValue(0);

    // joint_2 için yatay layout
    QHBoxLayout *jointLayout2 = new QHBoxLayout();

    QLabel *jointLabel2 = new QLabel("joint_2: 0");
    jointLabel2->setMinimumWidth(80);

    QSlider *jointSlider2 = new QSlider(Qt::Horizontal);
    jointSlider2->setRange(-180, 180);
    jointSlider2->setValue(0);

    // joint_3 için yatay layout
    QHBoxLayout *jointLayout3 = new QHBoxLayout();

    QLabel *jointLabel3 = new QLabel("joint_3: 0");
    jointLabel3->setMinimumWidth(80);

    QSlider *jointSlider3 = new QSlider(Qt::Horizontal);
    jointSlider3->setRange(-180, 180);
    jointSlider3->setValue(0);

    // reset butonu

    QHBoxLayout *resetLayout = new QHBoxLayout();
    QPushButton *resetButton = new QPushButton("Reset All");

    resetLayout->addStretch();
    resetLayout->addWidget(resetButton);
    resetLayout->addStretch();
    
    // layout içine ekle
    jointLayout->addWidget(jointLabel);
    jointLayout->addWidget(jointSlider);

    jointLayout2->addWidget(jointLabel2);
    jointLayout2->addWidget(jointSlider2);

    jointLayout3->addWidget(jointLabel3);
    jointLayout3->addWidget(jointSlider3);


    // ana layout'a ekle
    mainLayout->addLayout(jointLayout);
    mainLayout->addLayout(jointLayout2);
    mainLayout->addLayout(jointLayout3);
    mainLayout->addLayout(resetLayout);

    // slider → label bağlantısı
    QObject::connect(
        jointSlider,
        &QSlider::valueChanged,
        [=](int value) {
            jointLabel->setText(QString("joint_1: %1").arg(value));
        }
    );

    QObject::connect(
        jointSlider2,
        &QSlider::valueChanged,
        [=](int value) {
            jointLabel2->setText(QString("joint_2: %1").arg(value));
        }
    );

    QObject::connect(
        jointSlider3,
        &QSlider::valueChanged,
        [=](int value) {
            jointLabel3->setText(QString("joint_3: %1").arg(value));
        }
    );

    QObject::connect(
        resetButton,
        &QPushButton::clicked,
        [=](){
            jointSlider->setValue(0);
            jointSlider2->setValue(0);
            jointSlider3->setValue(0);
        }
    );

    

   

    // hız slider

    QHBoxLayout *speedLayout = new QHBoxLayout();
    QLabel *speedLabel = new QLabel("Hız Değeri(%): 50");
    speedLabel->setMinimumWidth(80);


    QSlider *speedSlider = new QSlider(Qt::Horizontal);
    speedSlider->setRange(1, 100);
    speedSlider->setValue(50);
    
    speedLayout->addWidget(speedLabel);
    speedLayout->addWidget(speedSlider);

    mainLayout->addLayout(speedLayout);

    QObject::connect(
        speedSlider,
        &QSlider::valueChanged,
        [=](int value) {
            speedLabel->setText(QString("Hız Degeri(%): %1").arg(value));
        }
    );

    QHBoxLayout *hızResetLayout = new QHBoxLayout();
    QPushButton *hızResetButton = new QPushButton("Reset Hız");

    hızResetLayout->addStretch();
    hızResetLayout->addWidget(hızResetButton);
    hızResetLayout->addStretch();

    mainLayout->addLayout(hızResetLayout);

    QObject::connect(
        hızResetButton,
        &QPushButton::clicked,
        [=](){
            speedSlider->setValue(50);
            
        }
    );
    
    //TIMER

    QTimer *timer1 = new QTimer(&window);
    
    QObject::connect(
        timer1,
        &QTimer::timeout,
        [jointSlider, jointSlider2, jointSlider3, speedSlider, arm_pub](){

            int j1 = jointSlider->value();
            int j2 = jointSlider2->value();
            int j3 = jointSlider3->value();
            int s1 = speedSlider->value();

            qDebug("j1= %d  j2= %d  j3= %d speed= %d", j1, j2, j3, s1);
            
            trajectory_msgs::msg::JointTrajectory msg;
            
            msg.joint_names = {"joint_1", "joint_2", "joint_3"};

            trajectory_msgs::msg::JointTrajectoryPoint point;

            double j1r = j1 * M_PI / 180.0;
            double j2r = j2 * M_PI / 180.0;
            double j3r = j3 * M_PI / 180.0;

            point.positions ={j1r, j2r, j3r};

            double speed = static_cast<double>(s1); // slider değeri
            double time_sec = 10.0 - (speed - 1.0) * (10.0 - 0.5) / (100.0 - 1.0);

            point.time_from_start = rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(time_sec * 1e9));


            msg.points.push_back(point);
            arm_pub->publish(msg);

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "publish ediliyor");
        }
    );

    timer1->start(100); //ms

    auto ros_thread = std::thread([&](){
        rclcpp::spin(node);
    }

    );

    window.show();
    window.activateWindow();

    int ret = app.exec();
    rclcpp::shutdown();
    ros_thread.join();
    return ret;
    

}
