#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <QApplication>
#include <QWidget>
#include <QVBoxLayout>
#include <QLabel>
#include <QTextEdit>
#include <QPushButton>
#include <QString>
#include <QThread>
#include <QObject>

#include <memory>
#include <string>
#include <thread>

// Qt窗口，显示接收到字符串
class QtSubscriberWidget : public QWidget
{
  Q_OBJECT
public:
  explicit QtSubscriberWidget(QWidget *parent = nullptr)
  : QWidget(parent)
  {
    auto *layout = new QVBoxLayout(this);
    auto *title = new QLabel("ROS2 String Subscriber (Qt Visualization)", this);
    layout->addWidget(title);

    text_edit_ = new QTextEdit(this);
    // 将文本框设为只读
    text_edit_->setReadOnly(true);
    layout->addWidget(text_edit_);

    auto *quit_btn = new QPushButton("Quit", this);
    layout->addWidget(quit_btn);
    // 点击按钮时关闭窗口
    connect(quit_btn, &QPushButton::clicked, this, &QWidget::close);
    setLayout(layout);
    setWindowTitle("String Subscriber GUI");
    resize(600, 400);
  }

public slots:
  void appendDataSlot(const QString &text)
  {
    text_edit_->append(text);
  }

private:
  QTextEdit *text_edit_;
};

class ROS2NodeWrapper : public QObject
{
  Q_OBJECT
public:
  ROS2NodeWrapper()
  {
    // 在 main 中初始化 rclcpp
  }

  void start()
  {
    // 在此封装类中创建节点和订阅器
    node_ = rclcpp::Node::make_shared("string_subscriber_qt_node");
    sub_ = node_->create_subscription<std_msgs::msg::String>(
      "string_topic", 10,
      [this](std_msgs::msg::String::SharedPtr msg) {
        // 将收到的消息转换为 QString 并发射信号到 Qt 主线程
        QString qtxt = QString::fromStdString(msg->data);
        emit newData(qtxt);
      }
    );

    // 在这个线程中启动 executor
    exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    exec_->add_node(node_);
    spinning_thread_ = std::thread([this]() {
      exec_->spin();
    });
  }

  void stop()
  {
    if (exec_) {
      // 取消执行器的 spin
      exec_->cancel();
    }
    if (spinning_thread_.joinable()) {
      // 等待后台线程退出
      spinning_thread_.join();
    }
    node_.reset();
    exec_.reset();
  }

signals:
  // 当接收到新数据时发出的信号
  void newData(const QString &text);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec_;
  std::thread spinning_thread_;
};

int main(int argc, char **argv)
{
  // 先初始化 ROS（避免与 Qt argc/argv 冲突）
  rclcpp::init(argc, argv);

  int qt_argc = 0;
  QApplication app(qt_argc, nullptr);

  QtSubscriberWidget widget;
  widget.show();

  // 创建 ROS2 封装并在后台线程开始 spin
  ROS2NodeWrapper ros_wrapper;
  // 将 ROS 封装的信号连接到 Qt 窗口的槽，以便跨线程更新 UI
  QObject::connect(&ros_wrapper, &ROS2NodeWrapper::newData,
                   &widget, &QtSubscriberWidget::appendDataSlot);

  ros_wrapper.start();

  // 运行 Qt 事件循环；当 GUI 关闭时，停止 ROS
  int ret = app.exec();

  // 停止 ROS executor 并 shutdown
  ros_wrapper.stop();
  rclcpp::shutdown();

  return ret;
}

#include "sub_node.moc"