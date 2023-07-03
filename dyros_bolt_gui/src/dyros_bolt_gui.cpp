#include "dyros_bolt_gui/dyros_bolt_gui.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>

namespace dyros_bolt_gui {

DyrosBoltGui::DyrosBoltGui()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("DyrosBoltGui");

  jointCmd_pub = nh_.advertise<dyros_bolt_msgs::JointCommand>("/dyros_bolt/joint_command", 10);
}

void DyrosBoltGui::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // add widget to the user interface
  context.addWidget(widget_);

  connect(ui_.fl_haa, SIGNAL(pressed()), this, SLOT(jointCtrlSetClicked()));
  connect(ui_.fl_hfe, SIGNAL(pressed()), this, SLOT(jointCtrlSetClicked()));
  connect(ui_.fl_kfe, SIGNAL(pressed()), this, SLOT(jointCtrlSetClicked()));
  connect(ui_.fr_haa, SIGNAL(pressed()), this, SLOT(jointCtrlSetClicked()));
  connect(ui_.fr_hfe, SIGNAL(pressed()), this, SLOT(jointCtrlSetClicked()));
  connect(ui_.fr_kfe, SIGNAL(pressed()), this, SLOT(jointCtrlSetClicked()));
}

void DyrosBoltGui::shutdownPlugin()
{
  // TODO unregister all publishers here
}

void DyrosBoltGui::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void DyrosBoltGui::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

void DyrosBoltGui::jointCtrlSetClicked()
{
    string joint_obj = sender().objectName();
}

void DyrosBoltGui::sendjointCommand()
{
    
}

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

} // namespace

PLUGINLIB_EXPORT_CLASS(rqt_dyros_gui::DyrosBoltGui, rqt_gui_cpp::Plugin)