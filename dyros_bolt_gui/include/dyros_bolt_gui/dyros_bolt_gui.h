#ifndef RQT_DYROS_H
#define RQT_DYROS_H

#include <rqt_gui_cpp/plugin.h>
#include <dyros_bolt_gui/ui_dyros_bolt_gui.h>
#include <QWidget>
#include <std_msgs/Float32.h>
#include <iostream>

#include <dyros_bolt_msgs/JointCommand.h>

namespace dyros_bolt_gui {

class DyrosBoltGui
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  DyrosBoltGui();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

  ros::Publisher jointCmd_pub;
  dyros_bolt_msgs::JointCommand jointCmd_msgs;

protected slots:
  virtual void jointCommand();

//signals:
  // Comment in to signal that the plugin has a way to configure it
  //bool hasConfiguration() const;
  //void triggerConfiguration();
private:
  Ui::DyrosBoltGuiWidget ui_;
  QWidget* widget_;

  QTimer *timer_;

  std::vector<QLabel *> left1;
  std::vector<QLabel *> left2;
  std::vector<QLabel *> left3;
  std::vector<QLabel *> right1;
  std::vector<QLabel *> right2;
  std::vector<QLabel *> right3;
  
  ros::NodeHandle nh_;

};
} // namespace

#endif // my_namespace__my_plugin_H