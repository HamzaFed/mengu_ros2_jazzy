#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <QApplication>
#include <QWidget>
#include <QSlider>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QRadioButton>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QLabel>
#include <QTimer>

enum class ControlMode
{
  MANUAL,
  AUTO
};

class MenguGui : public QWidget
{
public:
  MenguGui(rclcpp::Node::SharedPtr node)
  : node_(node)
  {
    pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/joint_trajectory", 10);

    mode_ = ControlMode::MANUAL;

    setup_ui();
    setup_timer();
    update_ui_state();
  }

private:
  /* ================= UI ================= */

  void setup_ui()
  {
    auto *main = new QVBoxLayout(this);

    // ---- MODE ----
    auto *mode_layout = new QHBoxLayout();
    manual_btn_ = new QRadioButton("Manual");
    auto_btn_   = new QRadioButton("Auto");
    manual_btn_->setChecked(true);

    mode_layout->addWidget(manual_btn_);
    mode_layout->addWidget(auto_btn_);
    main->addLayout(mode_layout);

    connect(manual_btn_, &QRadioButton::clicked, this, [&]() {
      mode_ = ControlMode::MANUAL;
      update_ui_state();
    });

    connect(auto_btn_, &QRadioButton::clicked, this, [&]() {
      mode_ = ControlMode::AUTO;
      update_ui_state();
    });

    // ---- SLIDERS ----
    slider_j1_ = create_slider("J1", main);
    slider_j2_ = create_slider("J2", main);
    slider_j3_ = create_slider("J3", main);

    // ---- INPUT BOX ----
    input_j1_ = create_input("J1 target", main);
    input_j2_ = create_input("J2 target", main);
    input_j3_ = create_input("J3 target", main);

    speed_box_ = new QSpinBox();
    speed_box_->setRange(0, 100);
    speed_box_->setSuffix(" %");
    main->addWidget(new QLabel("Speed"));
    main->addWidget(speed_box_);

    publish_btn_ = new QPushButton("Publish");
    main->addWidget(publish_btn_);

    connect(publish_btn_, &QPushButton::clicked,
            this, &MenguGui::publish_auto);
  }

  QSlider* create_slider(const QString &name, QVBoxLayout *layout)
  {
    layout->addWidget(new QLabel(name));
    auto *s = new QSlider(Qt::Horizontal);
    s->setRange(-180, 180);
    layout->addWidget(s);
    return s;
  }

  QDoubleSpinBox* create_input(const QString &name, QVBoxLayout *layout)
  {
    layout->addWidget(new QLabel(name));
    auto *b = new QDoubleSpinBox();
    b->setRange(-180.0, 180.0);
    b->setDecimals(2);
    layout->addWidget(b);
    return b;
  }

  void update_ui_state()
  {
    bool manual = (mode_ == ControlMode::MANUAL);

    slider_j1_->setEnabled(manual);
    slider_j2_->setEnabled(manual);
    slider_j3_->setEnabled(manual);

    input_j1_->setEnabled(!manual);
    input_j2_->setEnabled(!manual);
    input_j3_->setEnabled(!manual);
    speed_box_->setEnabled(!manual);
    publish_btn_->setEnabled(!manual);
  }

  /* ================= TIMER ================= */

  void setup_timer()
  {
    timer_ = new QTimer(this);
    timer_->setInterval(50); // 20 Hz
    connect(timer_, &QTimer::timeout,
            this, &MenguGui::publish_manual);
    timer_->start();
  }

  /* ================= PUBLISH ================= */

  void publish_manual()
  {
    if (mode_ != ControlMode::MANUAL)
      return;

    publish(
      slider_j1_->value(),
      slider_j2_->value(),
      slider_j3_->value(),
      0.5
    );
  }

  void publish_auto()
  {
    if (mode_ != ControlMode::AUTO)
      return;

    double speed = speed_box_->value();

    double min_t = 0.2;
    double max_t = 3.0;
    double t = max_t - (speed / 100.0) * (max_t - min_t);

    publish(
      input_j1_->value(),
      input_j2_->value(),
      input_j3_->value(),
      t
    );
  }

  void publish(double j1, double j2, double j3, double time)
  {
    trajectory_msgs::msg::JointTrajectory msg;
    msg.joint_names = {"joint_1", "joint_2", "joint_3"};

    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = {
      deg2rad(j1),
      deg2rad(j2),
      deg2rad(j3)
    };

    p.time_from_start.sec = static_cast<int>(time);
    p.time_from_start.nanosec =
      (time - p.time_from_start.sec) * 1e9;

    msg.points.push_back(p);
    pub_->publish(msg);
  }

  double deg2rad(double d)
  {
    return d * M_PI / 180.0;
  }

  /* ================= MEMBERS ================= */

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;

  ControlMode mode_;

  QSlider *slider_j1_, *slider_j2_, *slider_j3_;
  QDoubleSpinBox *input_j1_, *input_j2_, *input_j3_;
  QSpinBox *speed_box_;
  QPushButton *publish_btn_;
  QRadioButton *manual_btn_, *auto_btn_;
  QTimer *timer_;
};

/* ================= MAIN ================= */

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  auto node = rclcpp::Node::make_shared("mengu_gui_node");
  MenguGui gui(node);
  gui.show();

  std::thread ros_thread([&]() {
    rclcpp::spin(node);
  });

  int ret = app.exec();
  rclcpp::shutdown();
  ros_thread.join();
  return ret;
}
