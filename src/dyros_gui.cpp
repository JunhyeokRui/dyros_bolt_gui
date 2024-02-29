#include "dyros_gui/dyros_gui.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>

namespace dyros_gui {

RqtDyrosPlugin::RqtDyrosPlugin()
    : rqt_gui_cpp::Plugin()
    , widget_(0)
{
    // Constructor is called first before initPlugin function, needless to say.

    // give QObjects reasonable names
    setObjectName("RqtDyrosPlugin");
}

void RqtDyrosPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget_ = new QWidget();
    // extend the widget with all attributes and children from UI file
    ui_.setupUi(widget_);
    // add widget to the user interface
    context.addWidget(widget_);

    axis_state_pub = getNodeHandle().advertise<std_msgs::Int16>("/odrv_axis_request_states",10);
    joint_ctrl_pub = getNodeHandle().advertise<dyros_bolt_msgs::JointCommand>("/dyros_bolt/joint_command",10);
    custom_ctrl_pub = getNodeHandle().advertise<dyros_bolt_msgs::CustomCommand>("/dyros_bolt/custom_command",10);
    axis_current_state_sub = getNodeHandle().subscribe("/odrv_axis_current_states", 10, &RqtDyrosPlugin::axisStateCallback, this);
    joint_state_sub = getNodeHandle().subscribe("/joint_states", 10, &RqtDyrosPlugin::jointStateCallback, this);

    ui_.RebootBT->setShortcut(QKeySequence(Qt::Key_E));

    connect(ui_.idlestateButton,SIGNAL(clicked()),this,SLOT(idlebutton()));
    connect(ui_.mcalibrationButton,SIGNAL(clicked()),this,SLOT(motorcalibutton()));
    connect(ui_.caliButton,SIGNAL(clicked()),this,SLOT(encodercalibutton()));
    connect(ui_.CLButton,SIGNAL(clicked()),this,SLOT(clbutton()));
    connect(ui_.EstopButton,SIGNAL(clicked()),this,SLOT(estopbutton()));
    connect(ui_.RebootBT,SIGNAL(clicked()),this,SLOT(rebootbutton()));
    connect(ui_.resetEncoderButton,SIGNAL(clicked()),this,SLOT(encoderbutton()));
    connect(ui_.CcCmdButton,SIGNAL(clicked()),this,SLOT(ccCmdbutton()));
    connect(ui_.gravButton,SIGNAL(clicked()),this,SLOT(gravCmdbutton()));
    
    

    connect(ui_.jointButton,SIGNAL(clicked()),this,SLOT(jointCommandClicked()));
    connect(ui_.jointButton_2,SIGNAL(clicked()),this,SLOT(jointCommandClicked()));
    connect(ui_.jointButton_3,SIGNAL(clicked()),this,SLOT(jointCommandClicked()));
    connect(ui_.jointButton_4,SIGNAL(clicked()),this,SLOT(jointCommandClicked()));
    connect(ui_.jointButton_5,SIGNAL(clicked()),this,SLOT(jointCommandClicked()));
    connect(ui_.jointButton_6,SIGNAL(clicked()),this,SLOT(jointCommandClicked()));

    
    joint_cmd_msgs.name.resize(8);
    joint_cmd_msgs.position.resize(8);
    joint_cmd_msgs.duration.resize(8);

    jointStates.push_back(ui_.jointState_1);
    jointStates.push_back(ui_.jointState_2);
    jointStates.push_back(ui_.jointState_3);
    jointStates.push_back(ui_.jointState_4);
    jointStates.push_back(ui_.jointState_5);
    jointStates.push_back(ui_.jointState_6);
    
    // joint_cmd_msgs.name = jointName;
}

void RqtDyrosPlugin::estopbutton()
{
    
}
void RqtDyrosPlugin::gravCmdbutton()
{
    
}
void RqtDyrosPlugin::ccCmdbutton()
{
    if(ui_.custom_start->isChecked())
    {
        custom_cmd_msgs.custom_mode = 1;
        custom_cmd_msgs.first_foot_step = ui_.isRightFootStep->isChecked();
        
        custom_cmd_msgs.x=ui_.direction_x->value();
        custom_cmd_msgs.y=ui_.direction_y->value();
        custom_cmd_msgs.z=ui_.direction_z->value();
        custom_cmd_msgs.theta=ui_.direction_theta->value();

        custom_cmd_msgs.step_length_x = ui_.step_length_x->value();
        custom_cmd_msgs.step_length_y = ui_.step_length_y->value();
    }
    if(ui_.custom_stop->isChecked())
    {
        custom_cmd_msgs.custom_mode = 0;
    }
    custom_ctrl_pub.publish(custom_cmd_msgs);
}

void RqtDyrosPlugin::encoderbutton()
{
    axis_state_msgs.data = 19;
    axis_state_pub.publish(axis_state_msgs);
}

void RqtDyrosPlugin::rebootbutton()
{
    axis_state_msgs.data = 16;
    axis_state_pub.publish(axis_state_msgs);
}

void RqtDyrosPlugin::idlebutton()
{
    axis_state_msgs.data = 1;
    axis_state_pub.publish(axis_state_msgs);
}

void RqtDyrosPlugin::motorcalibutton()
{
    axis_state_msgs.data = 4;
    axis_state_pub.publish(axis_state_msgs);
}

void RqtDyrosPlugin::encodercalibutton()
{
    axis_state_msgs.data = 7;
    axis_state_pub.publish(axis_state_msgs);
}

void RqtDyrosPlugin::clbutton()
{
    axis_state_msgs.data = 8;
    axis_state_pub.publish(axis_state_msgs);
}

void RqtDyrosPlugin::jointCommandClicked()
{
    if(sender()->objectName() == "jointButton")
    {
        send_joint_ctrl(0,ui_.doubleSpinBox->value());
    }
    else if(sender()->objectName() == "jointButton_2")
    {
        send_joint_ctrl(1,ui_.doubleSpinBox_2->value());
    }
    else if(sender()->objectName() == "jointButton_3")
    {
        send_joint_ctrl(2,ui_.doubleSpinBox_3->value());
    }
    else if(sender()->objectName() == "jointButton_4")
    {
        send_joint_ctrl(4,ui_.doubleSpinBox_4->value());
    }
    else if(sender()->objectName() == "jointButton_5")
    {
        send_joint_ctrl(5,ui_.doubleSpinBox_5->value());
    }
    else if(sender()->objectName() == "jointButton_6")
    {
        send_joint_ctrl(6,ui_.doubleSpinBox_6->value());
    }
}

void RqtDyrosPlugin::send_joint_ctrl(int id, double q_)
{
    joint_cmd_msgs.name[id] = jointName[id];
    joint_cmd_msgs.position[id] = q_;
    if(q_ > 0) joint_cmd_msgs.duration[id] = 0.5 * (1 + q_);
    else joint_cmd_msgs.duration[id] = 0.5 * (1 - q_);

    joint_ctrl_pub.publish(joint_cmd_msgs);

    joint_cmd_msgs.name[id] = "";
}

void RqtDyrosPlugin::jointStateCallback(const sensor_msgs::JointStateConstPtr &msg)
{
    ui_.jointName_1->setText(QString::fromStdString(msg->name[0]));
    ui_.jointName_2->setText(QString::fromStdString(msg->name[1]));
    ui_.jointName_3->setText(QString::fromStdString(msg->name[2]));
    ui_.jointName_4->setText(QString::fromStdString(msg->name[4]));
    ui_.jointName_5->setText(QString::fromStdString(msg->name[5]));
    ui_.jointName_6->setText(QString::fromStdString(msg->name[6]));


    ui_.jointValue_1->setText(QString::number(msg->position[0]));
    ui_.jointValue_2->setText(QString::number(msg->position[1]));
    ui_.jointValue_3->setText(QString::number(msg->position[2]));
    ui_.jointValue_4->setText(QString::number(msg->position[4]));
    ui_.jointValue_5->setText(QString::number(msg->position[5]));
    ui_.jointValue_6->setText(QString::number(msg->position[6]));
}

void RqtDyrosPlugin::axisStateCallback(const std_msgs::Int16MultiArrayConstPtr &msg)
{

    for(int i = 0; i < 6; i++)
    {
        switch (msg->data[i])
        {
            case 8:
                // jointStates[i]->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
                jointStates[i]->setText(QString::fromUtf8("ON"));
                break;
            case 0:
            case 1:
                // jointStates[i]->setStyleSheet("QLabel { background-color : rgb(239, 41, 41) ; color : black; }");
                jointStates[i]->setText(QString::fromUtf8("OFF"));
                break;
            case 3:
            case 4:
            case 7:
                // jointStates[i]->setStyleSheet("QLabel { background-color : rgb(252, 175, 62) ; color : black; }");
                jointStates[i]->setText(QString::fromUtf8("CALI"));
                break;
            default:
                // jointStates[i]->setStyleSheet("QLabel { background-color : rgb(239, 41, 41) ; color : black; }");
                jointStates[i]->setText(QString::fromUtf8("OFF"));
                break;
        }
    }
    // ui_.jointState_1->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
    // ui_.jointState_1->setText(QString::fromUtf8("ON"));

    // ui_.jointState_2->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
    // ui_.jointState_2->setText(QString::fromUtf8("ON"));
   
    // ui_.jointState_3->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
    // ui_.jointState_3->setText(QString::fromUtf8("ON"));
    
    // ui_.jointState_4->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
    // ui_.jointState_4->setText(QString::fromUtf8("ON")); 
    
    // ui_.jointState_5->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
    // ui_.jointState_5->setText(QString::fromUtf8("ON")); 
    
    // ui_.jointState_6->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
    // ui_.jointState_6->setText(QString::fromUtf8("ON")); 
}

void RqtDyrosPlugin::shutdownPlugin()
{
    // TODO unregister all publishers here
}

void RqtDyrosPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
    // TODO save intrinsic configuration, usually using:
    // instance_settings.setValue(k, v)
}

void RqtDyrosPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
    // TODO restore intrinsic configuration, usually using:
    // v = instance_settings.value(k)
}

/* bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
    // Usually used to open a dialog to offer the user a set of configuration
}
*/

} // namespace

PLUGINLIB_EXPORT_CLASS(dyros_gui::RqtDyrosPlugin, rqt_gui_cpp::Plugin)