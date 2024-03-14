// #include "dyros_gui/dyros_gui.h"
// #include <pluginlib/class_list_macros.h>
// #include <QStringList>

// namespace dyros_gui {

// RqtDyrosPlugin::RqtDyrosPlugin()
//     : rqt_gui_cpp::Plugin()
//     , widget_(0)
// {
//     // Constructor is called first before initPlugin function, needless to say.

//     // give QObjects reasonable names
//     setObjectName("RqtDyrosPlugin");
// }

// void RqtDyrosPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
// {
//     // access standalone command line arguments
//     QStringList argv = context.argv();
//     // create QWidget
//     widget_ = new QWidget();
//     // extend the widget with all attributes and children from UI file
//     ui_.setupUi(widget_);
//     // add widget to the user interface
//     context.addWidget(widget_);

//     axis_state_pub = getNodeHandle().advertise<std_msgs::Int16>("/odrv_axis_request_states",10);
//     joint_ctrl_pub = getNodeHandle().advertise<dyros_bolt_msgs::JointCommand>("/dyros_bolt/joint_command",10);
//     axis_current_state_sub = getNodeHandle().subscribe("/odrv_axis_current_states", 10, &RqtDyrosPlugin::axisStateCallback, this);
//     joint_state_sub = getNodeHandle().subscribe("/joint_states", 10, &RqtDyrosPlugin::jointStateCallback, this);


//     connect(ui_.idlestateButton,SIGNAL(clicked()),this,SLOT(idlebutton()));
//     connect(ui_.mcalibrationButton,SIGNAL(clicked()),this,SLOT(motorcalibutton()));
//     connect(ui_.caliButton,SIGNAL(clicked()),this,SLOT(encodercalibutton()));
//     connect(ui_.CLButton,SIGNAL(clicked()),this,SLOT(clbutton()));
//     connect(ui_.EstopButton,SIGNAL(clicked()),this,SLOT(estopbutton()));
//     connect(ui_.RebootBT,SIGNAL(clicked()),this,SLOT(rebootbutton()));
//     connect(ui_.resetEncoderButton,SIGNAL(clicked()),this,SLOT(encoderbutton()));
    
    

//     connect(ui_.jointButton,SIGNAL(clicked()),this,SLOT(jointCommandClicked()));
//     connect(ui_.jointButton_2,SIGNAL(clicked()),this,SLOT(jointCommandClicked()));
//     connect(ui_.jointButton_3,SIGNAL(clicked()),this,SLOT(jointCommandClicked()));
//     connect(ui_.jointButton_4,SIGNAL(clicked()),this,SLOT(jointCommandClicked()));
//     connect(ui_.jointButton_5,SIGNAL(clicked()),this,SLOT(jointCommandClicked()));
//     connect(ui_.jointButton_6,SIGNAL(clicked()),this,SLOT(jointCommandClicked()));

    
//     joint_cmd_msgs.name.resize(8);
//     joint_cmd_msgs.position.resize(8);
//     joint_cmd_msgs.duration.resize(8);

//     jointStates.push_back(ui_.jointState_1);
//     jointStates.push_back(ui_.jointState_2);
//     jointStates.push_back(ui_.jointState_3);
//     jointStates.push_back(ui_.jointState_4);
//     jointStates.push_back(ui_.jointState_5);
//     jointStates.push_back(ui_.jointState_6);
    
//     // joint_cmd_msgs.name = jointName;
// }

// void RqtDyrosPlugin::estopbutton()
// {
    
// }

// void RqtDyrosPlugin::encoderbutton()
// {
//     axis_state_msgs.data = 19;
//     axis_state_pub.publish(axis_state_msgs);
// }

// void RqtDyrosPlugin::rebootbutton()
// {
//     axis_state_msgs.data = 16;
//     axis_state_pub.publish(axis_state_msgs);
// }

// void RqtDyrosPlugin::idlebutton()
// {
//     axis_state_msgs.data = 1;
//     axis_state_pub.publish(axis_state_msgs);
// }

// void RqtDyrosPlugin::motorcalibutton()
// {
//     axis_state_msgs.data = 4;
//     axis_state_pub.publish(axis_state_msgs);
// }

// void RqtDyrosPlugin::encodercalibutton()
// {
//     axis_state_msgs.data = 7;
//     axis_state_pub.publish(axis_state_msgs);
// }

// void RqtDyrosPlugin::clbutton()
// {
//     axis_state_msgs.data = 8;
//     axis_state_pub.publish(axis_state_msgs);
// }

// void RqtDyrosPlugin::jointCommandClicked()
// {
//     if(sender()->objectName() == "jointButton")
//     {
//         send_joint_ctrl(0,ui_.doubleSpinBox->value());
//     }
//     else if(sender()->objectName() == "jointButton_2")
//     {
//         send_joint_ctrl(1,ui_.doubleSpinBox_2->value());
//     }
//     else if(sender()->objectName() == "jointButton_3")
//     {
//         send_joint_ctrl(2,ui_.doubleSpinBox_3->value());
//     }
//     else if(sender()->objectName() == "jointButton_4")
//     {
//         send_joint_ctrl(4,ui_.doubleSpinBox_4->value());
//     }
//     else if(sender()->objectName() == "jointButton_5")
//     {
//         send_joint_ctrl(5,ui_.doubleSpinBox_5->value());
//     }
//     else if(sender()->objectName() == "jointButton_6")
//     {
//         send_joint_ctrl(6,ui_.doubleSpinBox_6->value());
//     }
// }

// void RqtDyrosPlugin::send_joint_ctrl(int id, double q_)
// {
//     joint_cmd_msgs.name[id] = jointName[id];
//     joint_cmd_msgs.position[id] = q_;
//     if(q_ > 0) joint_cmd_msgs.duration[id] = 0.5 * (1 + q_);
//     else joint_cmd_msgs.duration[id] = 0.5 * (1 - q_);

//     joint_ctrl_pub.publish(joint_cmd_msgs);

//     joint_cmd_msgs.name[id] = "";
// }

// void RqtDyrosPlugin::jointStateCallback(const sensor_msgs::JointStateConstPtr &msg)
// {
//     ui_.jointName_1->setText(QString::fromStdString(msg->name[0]));
//     ui_.jointName_2->setText(QString::fromStdString(msg->name[1]));
//     ui_.jointName_3->setText(QString::fromStdString(msg->name[2]));
//     ui_.jointName_4->setText(QString::fromStdString(msg->name[4]));
//     ui_.jointName_5->setText(QString::fromStdString(msg->name[5]));
//     ui_.jointName_6->setText(QString::fromStdString(msg->name[6]));


//     ui_.jointValue_1->setText(QString::number(msg->position[0]));
//     ui_.jointValue_2->setText(QString::number(msg->position[1]));
//     ui_.jointValue_3->setText(QString::number(msg->position[2]));
//     ui_.jointValue_4->setText(QString::number(msg->position[4]));
//     ui_.jointValue_5->setText(QString::number(msg->position[5]));
//     ui_.jointValue_6->setText(QString::number(msg->position[6]));
// }

// void RqtDyrosPlugin::axisStateCallback(const std_msgs::Int16MultiArrayConstPtr &msg)
// {

//     for(int i = 0; i < 6; i++)
//     {
//         switch (msg->data[i])
//         {
//             case 8:
//                 // jointStates[i]->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
//                 jointStates[i]->setText(QString::fromUtf8("ON"));
//                 break;
//             case 0:
//             case 1:
//                 // jointStates[i]->setStyleSheet("QLabel { background-color : rgb(239, 41, 41) ; color : black; }");
//                 jointStates[i]->setText(QString::fromUtf8("OFF"));
//                 break;
//             case 3:
//             case 4:
//             case 7:
//                 // jointStates[i]->setStyleSheet("QLabel { background-color : rgb(252, 175, 62) ; color : black; }");
//                 jointStates[i]->setText(QString::fromUtf8("CALI"));
//                 break;
//             default:
//                 // jointStates[i]->setStyleSheet("QLabel { background-color : rgb(239, 41, 41) ; color : black; }");
//                 jointStates[i]->setText(QString::fromUtf8("OFF"));
//                 break;
//         }
//     }
//     // ui_.jointState_1->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
//     // ui_.jointState_1->setText(QString::fromUtf8("ON"));

//     // ui_.jointState_2->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
//     // ui_.jointState_2->setText(QString::fromUtf8("ON"));
   
//     // ui_.jointState_3->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
//     // ui_.jointState_3->setText(QString::fromUtf8("ON"));
    
//     // ui_.jointState_4->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
//     // ui_.jointState_4->setText(QString::fromUtf8("ON")); 
    
//     // ui_.jointState_5->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
//     // ui_.jointState_5->setText(QString::fromUtf8("ON")); 
    
//     // ui_.jointState_6->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
//     // ui_.jointState_6->setText(QString::fromUtf8("ON")); 
// }

// void RqtDyrosPlugin::shutdownPlugin()
// {
//     // TODO unregister all publishers here
// }

// void RqtDyrosPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
// {
//     // TODO save intrinsic configuration, usually using:
//     // instance_settings.setValue(k, v)
// }

// void RqtDyrosPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
// {
//     // TODO restore intrinsic configuration, usually using:
//     // v = instance_settings.value(k)
// }

// /* bool hasConfiguration() const
// {
//   return true;
// }

// void triggerConfiguration()
// {
//     // Usually used to open a dialog to offer the user a set of configuration
// }
// */

// } // namespace

// PLUGINLIB_EXPORT_CLASS(dyros_gui::RqtDyrosPlugin, rqt_gui_cpp::Plugin)


#include "dyros_bolt_gui/dyros_bolt_gui.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <vector>
#include <QDebug>

#ifdef COMPILE_MELODIC
#include "qxtglobalshortcut.h"
#endif

int elng[33] = {0, 1, 16, 17, 9, 8, 4, 5, 13, 12, 14, 15, 7, 6, 2, 3, 11, 10, 18, 19, 27, 28, 29, 30, 31, 32, 20, 21, 22, 23, 24, 25, 26};
int elng2[33] = {23, 24, 15, 16, 17, 18, 19, 20, 21, 22, 25, 26, 27, 28, 29, 30, 31, 32, 12, 13, 14, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
int mo2g[33] = {21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 20, 19, 18, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 10, 11, 12, 13, 14, 15, 16, 17};

namespace dyros_bolt_gui
{
    MyQGraphicsScene::MyQGraphicsScene(QWidget *parent) : QGraphicsScene(parent)
    {
    }

    MyQGraphicsView::MyQGraphicsView(QWidget *parent) : QGraphicsView(parent)
    {
    }
    /*
void MyQGraphicsScene::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    std::cout << "press" << std::endl;
}*/
    void MyQGraphicsScene::wheelEvent(QGraphicsSceneWheelEvent *event)
    {
        // std::cout << parent()->findChild<QObject *>("graphicsViewCustom")->objectName().toStdString() << std::endl;

        QGraphicsView *view_ = parent()->findChild<QGraphicsView *>("graphicsViewCustom");

        // view_->setViewport();

        double scaleFactor = 0.1;
        const qreal minFactor = 1.0;
        const qreal maxFactor = 10.0;
        static qreal h11 = 1.0;
        static qreal h22 = 1.0;

        if (event->delta() == 120)
        {
            h11 = (h11 >= maxFactor) ? h11 : (h11 + scaleFactor);
            h22 = (h22 >= maxFactor) ? h22 : (h22 + scaleFactor);
        }
        else if (event->delta() == -120)
        {
            h11 = (h11 <= minFactor) ? minFactor : (h11 - scaleFactor);
            h22 = (h22 <= minFactor) ? minFactor : (h22 - scaleFactor);
        }
        view_->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
        view_->setTransform(QTransform(h11, 0, 0, 0, h22, 0, 0, 0, 1));
    }

    DyrosBoltGui::DyrosBoltGui()
        : rqt_gui_cpp::Plugin(), widget_(0)
    {
        qRegisterMetaType<std_msgs::StringConstPtr>();
        qRegisterMetaType<geometry_msgs::PolygonStampedConstPtr>();
        qRegisterMetaType<std_msgs::Float32ConstPtr>();
        qRegisterMetaType<std_msgs::Float32MultiArrayConstPtr>();
        qRegisterMetaType<sensor_msgs::ImuConstPtr>();
        qRegisterMetaType<sensor_msgs::JointStateConstPtr>();
        qRegisterMetaType<std_msgs::Int32MultiArrayConstPtr>();
        qRegisterMetaType<std_msgs::Int8MultiArrayConstPtr>();
        setObjectName("DyrosBoltGui");

        // initPlugin()
        pointsub = nh_.subscribe("/dyros_bolt/point", 3, &DyrosBoltGui::pointCallback, this, ros::TransportHints().tcpNoDelay(true));
        timesub = nh_.subscribe("/dyros_bolt/time", 3, &DyrosBoltGui::timerCallback, this, ros::TransportHints().tcpNoDelay(true));
        sysstatesub = nh_.subscribe("/dyros_bolt/systemstate", 10, &DyrosBoltGui::sysstateCallback, this, ros::TransportHints().tcpNoDelay(true));
        com_pub = nh_.advertise<std_msgs::String>("/dyros_bolt/command", 10);
        guilogsub = nh_.subscribe("/dyros_bolt/guilog", 10, &DyrosBoltGui::guiLogCallback, this);
        gain_pub = nh_.advertise<std_msgs::Float32MultiArray>("/dyros_bolt/gain_command", 10);
        imusub = nh_.subscribe("/dyros_bolt/imu", 10, &DyrosBoltGui::imuCallback, this, ros::TransportHints().tcpNoDelay(true));
        task_pub = nh_.advertise<dyros_bolt_msgs::TaskCommand>("/dyros_bolt/taskcommand", 10);
        task_que_pub = nh_.advertise<dyros_bolt_msgs::TaskCommandQue>("/dyros_bolt/taskquecommand", 10);
        taskgain_pub = nh_.advertise<dyros_bolt_msgs::TaskGainCommand>("/dyros_bolt/taskgaincommand", 10);
        velcommand_pub = nh_.advertise<dyros_bolt_msgs::VelocityCommand>("/dyros_bolt/velcommand", 10);
        poscom_pub = nh_.advertise<dyros_bolt_msgs::positionCommand>("/dyros_bolt/positioncommand", 10);
        jointsub = nh_.subscribe("/dyros_bolt/jointstates", 10, &DyrosBoltGui::jointstateCallback, this);

        arm_gain_pub = nh_.advertise<std_msgs::Float32MultiArray>("/dyros_bolt/dg/armpdgain", 10);

        dyros_bolt_starter_pub = nh_.advertise<std_msgs::String>("/dyros_bolt/starter", 10);
        dyros_bolt_stopper_pub = nh_.advertise<std_msgs::String>("/dyros_bolt/stopper", 10);
        q_.resize(33);
        q_dot_.resize(33);
        torque_desired_.resize(33);

        ecat_sub = nh_.subscribe("/dyros_bolt/ecatstates", 10, &DyrosBoltGui::ecatstateCallback, this);
        ecat_comstate_sub = nh_.subscribe("/dyros_bolt/comstates", 10, &DyrosBoltGui::comstateCallback, this);

        // dg
        //  walkingspeed_pub = nh_.advertise<std_msgs::Float32>("/dyros_bolt/walkingspeedcommand", 100);
        //  walkingduration_pub = nh_.advertise<std_msgs::Float32>("/dyros_bolt/walkingdurationcommand", 100);
        //  walkingangvel_pub = nh_.advertise<std_msgs::Float32>("/dyros_bolt/walkingangvelcommand", 100);
        //  kneetargetangle_pub = nh_.advertise<std_msgs::Float32>("/dyros_bolt/kneetargetanglecommand", 100);
        //  footheight_pub = nh_.advertise<std_msgs::Float32>("/dyros_bolt/footheightcommand", 100);

        // avatar
        upperbodymode_pub = nh_.advertise<std_msgs::Int8>("/dyros_bolt/avatar/upperbodymodecommand", 10);
        pose_calibration_pub = nh_.advertise<std_msgs::Int8>("/dyros_bolt/avatar/pose_calibration_flag", 10);
        vr_slider_pub = nh_.advertise<std_msgs::Float32MultiArray>("/dyros_bolt/avatar/vr_caliabration_param", 10);

        taskgain_msg.pgain.resize(6);
        taskgain_msg.dgain.resize(6);

        gain_msg.data.resize(33);

        arm_gain_msg.data.resize(16);
        // ecatlabels = {ui_.}
    }

    void DyrosBoltGui::QTimerCallback()
    {
        static double stored_timer = 0;

        const double time_err = 0.1;

        static bool connected = false;

        if (connected)
        {

            if (stored_timer == robot_time)
            {
                std::cout << "Disconnected at " << robot_time << std::endl;
                ui_.currenttime->setText(QString::fromUtf8("DISCON"));
                connected = false;
            }
        }
        else
        {
            if (robot_time > stored_timer)
            {
                std::cout << "Connected at" << robot_time << std::endl;
                connected = true;
            }
        }

        stored_timer = robot_time;
    }

    void DyrosBoltGui::sendCommand(QString str)
    {
        com_msg.data = str.toStdString();
        com_pub.publish(com_msg);
    }

    void DyrosBoltGui::DyrosBoltStarter()
    {
        std_msgs::String msg_starter;

        msg_starter.data = "start_dyros_bolt";

        dyros_bolt_starter_pub.publish(msg_starter);
    }
    void DyrosBoltGui::DyrosBoltStopper()
    {
        std_msgs::String msg_stopper;

        msg_stopper.data = "stop_dyros_bolt";

        dyros_bolt_stopper_pub.publish(msg_stopper);
    }

    void DyrosBoltGui::initPlugin(qt_gui_cpp::PluginContext &context)
    {
        widget_ = new QWidget();
        ui_.setupUi(widget_);

        if (context.serialNumber() > 1)
        {
            widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
        }
        context.addWidget(widget_);

        bool sim_mode = false;
        nh_.param("/dyros_bolt_controller/sim_mode", sim_mode, true);

        ui_.torqueon_button->setShortcut(QKeySequence(Qt::Key_E));
        ui_.torqueoff_button->setShortcut(QKeySequence(Qt::Key_C));
        ui_.safetyresetbtn->setShortcut(QKeySequence(Qt::Key_R));
        // ui_.emergencyoff_button->setShortcut(QKeySequence(Qt::Key_Escape));
        // ui_.emergencyoff_button_2->setShortcut(QkeySequence(Qr))

        timer_ = new QTimer();

        connect(timer_, SIGNAL(timeout()), this, SLOT(QTimerCallback()));

        timer_->start(50);

        QSignalMapper *signalMapper = new QSignalMapper(this);

        connect(ui_.torqueon_button, SIGNAL(pressed()), signalMapper, SLOT(map()));
        signalMapper->setMapping(ui_.torqueon_button, "torqueon");

        connect(ui_.torqueoff_button, SIGNAL(pressed()), signalMapper, SLOT(map()));
        signalMapper->setMapping(ui_.torqueoff_button, "torqueoff");

        connect(ui_.emergencyoff_button, SIGNAL(pressed()), signalMapper, SLOT(map()));
        signalMapper->setMapping(ui_.emergencyoff_button, "emergencyoff");

        connect(ui_.qdotest, SIGNAL(pressed()), signalMapper, SLOT(map()));
        signalMapper->setMapping(ui_.qdotest, "qdot_est");

        connect(ui_.locklower, SIGNAL(pressed()), signalMapper, SLOT(map()));
        signalMapper->setMapping(ui_.locklower, "locklower");

        connect(ui_.vjbtn, SIGNAL(pressed()), signalMapper, SLOT(map()));
        signalMapper->setMapping(ui_.vjbtn, "simvirtualjoint");

        connect(ui_.IgIMUbtn, SIGNAL(pressed()), signalMapper, SLOT(map()));
        signalMapper->setMapping(ui_.IgIMUbtn, "imuignore");

        connect(ui_.imuresetbtn, SIGNAL(pressed()), signalMapper, SLOT(map()));
        signalMapper->setMapping(ui_.imuresetbtn, "imureset");

        connect(ui_.encoderresetbtn, SIGNAL(pressed()), signalMapper, SLOT(map()));
        signalMapper->setMapping(ui_.encoderresetbtn, "encoderreset");

        connect(ui_.encodercalibbtn, SIGNAL(pressed()), signalMapper, SLOT(map()));
        signalMapper->setMapping(ui_.encodercalibbtn, "encodercalib");

        connect(ui_.motorcalibbtn, SIGNAL(pressed()), signalMapper, SLOT(map()));
        signalMapper->setMapping(ui_.motorcalibbtn, "motorcalib");

        connect(ui_.sebyftbtn, SIGNAL(pressed()), signalMapper, SLOT(map()));
        signalMapper->setMapping(ui_.sebyftbtn, "sebyft");

        connect(ui_.disablelower, SIGNAL(pressed()), signalMapper, SLOT(map()));
        signalMapper->setMapping(ui_.disablelower, "disablelower");

        connect(ui_.emergencyoff_button_2, SIGNAL(pressed()), signalMapper, SLOT(map()));
        signalMapper->setMapping(ui_.emergencyoff_button_2, "E1");

        connect(ui_.emergencyoff_button_3, SIGNAL(pressed()), signalMapper, SLOT(map()));
        signalMapper->setMapping(ui_.emergencyoff_button_3, "terminate");

        connect(ui_.initializebtn, SIGNAL(pressed()), signalMapper, SLOT(map()));
        signalMapper->setMapping(ui_.initializebtn, "ecatinit");

        connect(ui_.initializebtn_2, SIGNAL(pressed()), signalMapper, SLOT(map()));
        signalMapper->setMapping(ui_.initializebtn_2, "ecatinitwaist");

        connect(ui_.initializebtn_3, SIGNAL(pressed()), signalMapper, SLOT(map()));
        signalMapper->setMapping(ui_.initializebtn_3, "ecatinitlower");

        connect(ui_.sebutton, SIGNAL(pressed()), signalMapper, SLOT(map()));
        signalMapper->setMapping(ui_.sebutton, "stateestimation");

        connect(ui_.torqueredis, SIGNAL(pressed()), signalMapper, SLOT(map()));
        signalMapper->setMapping(ui_.torqueredis, "torqueredis");

        // connect(ui_.qp2nd, SIGNAL(pressed()), signalMapper, SLOT(map()));
        // signalMapper->setMapping(ui_.qp2nd, "qp2nd");

        connect(ui_.gravity_button_4, SIGNAL(pressed()), signalMapper, SLOT(map()));
        signalMapper->setMapping(ui_.gravity_button_4, "gravity");

        connect(ui_.task_button_4, SIGNAL(pressed()), signalMapper, SLOT(map()));
        signalMapper->setMapping(ui_.task_button_4, "positioncontrol");

        connect(ui_.task_button_5, SIGNAL(pressed()), signalMapper, SLOT(map()));
        signalMapper->setMapping(ui_.task_button_5, "positiongravcontrol");

        connect(ui_.task_button_6, SIGNAL(pressed()), signalMapper, SLOT(map()));
        signalMapper->setMapping(ui_.task_button_6, "positiondobcontrol");

        connect(ui_.ftcalibbtn, SIGNAL(pressed()), signalMapper, SLOT(map()));
        signalMapper->setMapping(ui_.ftcalibbtn, "ftcalib");

        connect(ui_.handftbtn, SIGNAL(pressed()), signalMapper, SLOT(map()));
        signalMapper->setMapping(ui_.handftbtn, "handftcalib");

        connect(ui_.data_button_4, SIGNAL(pressed()), signalMapper, SLOT(map()));
        signalMapper->setMapping(ui_.data_button_4, "showdata");

        connect(ui_.iybtn, SIGNAL(pressed()), signalMapper, SLOT(map()));
        signalMapper->setMapping(ui_.iybtn, "inityaw");

        connect(ui_.printdatabutton, SIGNAL(pressed()), signalMapper, SLOT(map()));
        signalMapper->setMapping(ui_.printdatabutton, "printdata");

        connect(ui_.qdot_lpf, SIGNAL(pressed()), signalMapper, SLOT(map()));
        signalMapper->setMapping(ui_.qdot_lpf, "enablelpf");

        connect(ui_.forceloadbtn, SIGNAL(pressed()), signalMapper, SLOT(map()));
        signalMapper->setMapping(ui_.forceloadbtn, "forceload");

        connect(ui_.systemon_button, SIGNAL(pressed()), this, SLOT(DyrosBoltStarter()));
        connect(ui_.systemoff_button, SIGNAL(pressed()), this, SLOT(DyrosBoltStopper()));

        // connect(ui_.contact_button_4, SIGNAL(pressed()), this, SLOT(fixedgravcb()));
#ifdef COMPILE_MELODIC
        QxtGlobalShortcut *sc_E0 = new QxtGlobalShortcut(this);
        sc_E0->setShortcut(QKeySequence("F1"));
        connect(sc_E0, SIGNAL(activated()), signalMapper, SLOT(map()));
        signalMapper->setMapping(sc_E0, "E0");

        QxtGlobalShortcut *sc_E1 = new QxtGlobalShortcut(this);
        sc_E1->setShortcut(QKeySequence("F2"));
        connect(sc_E1, SIGNAL(activated()), signalMapper, SLOT(map()));
        signalMapper->setMapping(sc_E1, "E1");

        QxtGlobalShortcut *sc_E2 = new QxtGlobalShortcut(this);
        sc_E2->setShortcut(QKeySequence("F3"));
        connect(sc_E2, SIGNAL(activated()), signalMapper, SLOT(map()));
        signalMapper->setMapping(sc_E2, "E2");

        QxtGlobalShortcut *sc_grav = new QxtGlobalShortcut(this);
        sc_grav->setShortcut(QKeySequence("F4"));
        connect(sc_grav, SIGNAL(activated()), signalMapper, SLOT(map()));
        signalMapper->setMapping(sc_grav, "positioncontrol");

#endif
        connect(signalMapper, SIGNAL(mapped(QString)), this, SLOT(sendCommand(QString)));

        // Sending command end!

        // Select tabs..
        connect(ui_.ecat_btn, SIGNAL(pressed()), this, SLOT(ecatpbtn()));
        connect(ui_.stat_btn, SIGNAL(pressed()), this, SLOT(statpbtn()));
        connect(ui_.command_btn, SIGNAL(pressed()), this, SLOT(commandpbtn()));
        connect(ui_.mtunebtn, SIGNAL(pressed()), this, SLOT(mtunebtn()));
        connect(ui_.walkingbtn, SIGNAL(pressed()), this, SLOT(walkingbtn()));
        connect(ui_.avatar_btn, SIGNAL(pressed()), this, SLOT(avatarbtn()));

        connect(ui_.sendtunebtn, SIGNAL(pressed()), this, SLOT(sendtunebtn()));
        connect(ui_.resettunebtn, SIGNAL(pressed()), this, SLOT(resettunebtn()));

        connect(ui_.pcSendCommand, SIGNAL(pressed()), this, SLOT(positionCommand()));
        connect(ui_.pcTorqueStandard, SIGNAL(pressed()), this, SLOT(positionPreset1()));
        connect(ui_.getcurrentq, SIGNAL(pressed()), this, SLOT(getCurrentPos()));
        connect(ui_.pc4ConStandard, SIGNAL(pressed()), this, SLOT(positionPreset2()));
        connect(ui_.pcTorque3con, SIGNAL(pressed()), this, SLOT(positionPreset4()));
        connect(ui_.pcTorquepos3, SIGNAL(pressed()), this, SLOT(positionPreset3()));
        connect(ui_.poscomrelative, SIGNAL(stateChanged(int)), this, SLOT(positionRelative(int)));

        connect(ui_.customtaskgain, SIGNAL(stateChanged(int)), this, SLOT(customtaskgaincb(int)));
        connect(ui_.solver_mode, SIGNAL(currentIndexChanged(int)), this, SLOT(solvermode_cb(int)));

        ui_.stackedWidget->setCurrentIndex(0);

        ui_.ecat_btn->setShortcut(QKeySequence(Qt::Key_1));
        ui_.stat_btn->setShortcut(QKeySequence(Qt::Key_2));
        ui_.command_btn->setShortcut(QKeySequence(Qt::Key_3));
        ui_.mtunebtn->setShortcut(QKeySequence(Qt::Key_4));
        ui_.walkingbtn->setShortcut(QKeySequence(Qt::Key_5));
        ui_.avatar_btn->setShortcut(QKeySequence(Qt::Key_6));
        ui_.task_button_4->setShortcut(QKeySequence(Qt::Key_P));

        connect(this, &DyrosBoltGui::timerCallback, this, &DyrosBoltGui::timercb);
        connect(this, &DyrosBoltGui::guiLogCallback, this, &DyrosBoltGui::plainTextEditcb);
        connect(this, &DyrosBoltGui::pointCallback, this, &DyrosBoltGui::pointcb);
        connect(this, &DyrosBoltGui::imuCallback, this, &DyrosBoltGui::imucb);
        connect(this, &DyrosBoltGui::sysstateCallback, this, &DyrosBoltGui::sysstatecb);
        connect(this, &DyrosBoltGui::ecatstateCallback, this, &DyrosBoltGui::ecatstatecb);
        connect(this, &DyrosBoltGui::comstateCallback, this, &DyrosBoltGui::comstatecb);
        connect(this, &DyrosBoltGui::jointstateCallback, this, &DyrosBoltGui::jointstatecb);

        // connect(ui_)
        connect(ui_.safetyresetbtn, SIGNAL(pressed()), this, SLOT(safetyresetbtncb()));
        connect(ui_.safetyresetbtn_2, SIGNAL(pressed()), this, SLOT(safety2btncb()));

        connect(ui_.task_send_button, SIGNAL(pressed()), this, SLOT(tasksendcb()));
        connect(ui_.walkinginit_btn, SIGNAL(pressed()), this, SLOT(walkinginitbtncb()));
        connect(ui_.walkingstart_btn, SIGNAL(pressed()), this, SLOT(walkingstartbtncb()));

        connect(ui_.task_mode, SIGNAL(currentIndexChanged(int)), this, SLOT(taskmodecb(int)));

        // connect(ui_.)

        connect(ui_.que_add, SIGNAL(pressed()), this, SLOT(que_addquebtn()));
        connect(ui_.que_delete, SIGNAL(pressed()), this, SLOT(que_deletebtn()));
        connect(ui_.que_down, SIGNAL(pressed()), this, SLOT(que_downbtn()));
        connect(ui_.que_up, SIGNAL(pressed()), this, SLOT(que_upbtn()));
        connect(ui_.que_reset, SIGNAL(pressed()), this, SLOT(que_resetbtn()));
        connect(ui_.que_send, SIGNAL(pressed()), this, SLOT(que_sendbtn()));

        connect(ui_.taskgain_sendbtn, SIGNAL(pressed()), this, SLOT(sendtaskgaincommand()));
        connect(ui_.taskgain_resetbtn, SIGNAL(pressed()), this, SLOT(resettaskgaincommand()));

        connect(ui_.horizontalSlider, SIGNAL(valueChanged(int)), this, SLOT(slidervelcommand()));
        connect(ui_.horizontalSlider_2, SIGNAL(valueChanged(int)), this, SLOT(slidervelcommand()));
        connect(ui_.horizontalSlider_3, SIGNAL(valueChanged(int)), this, SLOT(slidervelcommand()));

        connect(ui_.horizontalSlider, SIGNAL(sliderReleased()), this, SLOT(sliderrel1()));
        connect(ui_.horizontalSlider_2, SIGNAL(sliderReleased()), this, SLOT(sliderrel2()));
        connect(ui_.horizontalSlider_3, SIGNAL(sliderReleased()), this, SLOT(sliderrel3()));

        // dg
        //  connect(ui_.walking_speed_slider_2, SIGNAL(valueChanged(int)), this, SLOT(walkingspeedcb(int)));
        //  connect(ui_.walking_duration_slider_2, SIGNAL(valueChanged(int)), this, SLOT(walkingdurationcb(int)));
        //  connect(ui_.walking_angvel_slider_2, SIGNAL(valueChanged(int)), this, SLOT(walkingangvelcb(int)));
        //  connect(ui_.knee_target_angle_slider_2, SIGNAL(valueChanged(int)), this, SLOT(kneetargetanglecb(int)));
        //  connect(ui_.foot_height_slider_2, SIGNAL(valueChanged(int)), this, SLOT(footheightcb(int)));

        // avatar
        // JHRui edit
        connect(ui_.upperbody_mode_init_pose, SIGNAL(pressed()), this, SLOT(sendupperbodymodecb_1()));
        connect(ui_.upperbody_mode_zero_pose, SIGNAL(pressed()), this, SLOT(sendupperbodymodecb_2()));
        connect(ui_.upperbody_mode_freezing, SIGNAL(pressed()), this, SLOT(sendupperbodymodecb_3()));
        connect(ui_.upperbody_mode_ready_pose, SIGNAL(pressed()), this, SLOT(sendupperbodymodecb_4()));
        connect(ui_.upperbody_mode_head_only_ret, SIGNAL(pressed()), this, SLOT(sendupperbodymodecb_5()));
        connect(ui_.upperbody_mode_motion_ret_1, SIGNAL(pressed()), this, SLOT(sendupperbodymodecb_6()));
        connect(ui_.upperbody_mode_motion_ret_2, SIGNAL(pressed()), this, SLOT(sendupperbodymodecb_7()));
        connect(ui_.upperbody_mode_abs_hand_pos, SIGNAL(pressed()), this, SLOT(sendupperbodymodecb_8()));
        connect(ui_.upperbody_mode_prop_hand_pos, SIGNAL(pressed()), this, SLOT(sendupperbodymodecb_9()));
        // connect(ui_.upperbody_mode_NA, SIGNAL(pressed()), this, SLOT(sendupperbodymodecb()));
        // JHRui edit
        
        connect(ui_.send_upperbody_mode_button, SIGNAL(pressed()), this, SLOT(sendupperbodymodecb()));

        connect(ui_.still_pose_button, SIGNAL(pressed()), this, SLOT(sendstillposecalibration()));
        connect(ui_.T_pose_button, SIGNAL(pressed()), this, SLOT(sendtposecalibration()));
        connect(ui_.forward_pose_button, SIGNAL(pressed()), this, SLOT(sendforwardposecalibration()));
        connect(ui_.reset_pose_button, SIGNAL(pressed()), this, SLOT(sendresetposecalibration()));
        connect(ui_.load_saved_cali_button, SIGNAL(pressed()), this, SLOT(sendloadsavedcalibration()));

        connect(ui_.vr_eye_distance_slider, SIGNAL(valueChanged(int)), this, SLOT(vr_eye_distance_cb(int)));
        connect(ui_.vr_eye_depth_slider, SIGNAL(valueChanged(int)), this, SLOT(vr_eye_depth_cb(int)));

        connect(ui_.ArmGainSendBtn, SIGNAL(pressed()), this, SLOT(armGainSend()));

        connect(ui_.callStiffGainsButton, SIGNAL(pressed()), this, SLOT(setArmStiffPDGain()));
        connect(ui_.callSoftGainsButton, SIGNAL(pressed()), this, SLOT(setArmSoftPDGain()));

        if (sim_mode)
        {
            // ui_.label_zpstatus->setStyleSheet("QLabel { background-color : yellow; color : black; }");
            ui_.label_imustatus->setStyleSheet("QLabel { background-color : yellow; color : black; }");
            // ui_.label_ecatstatus->setStyleSheet("QLabel { background-color : yellow; color : black; }");
            ui_.label_ftstatus->setStyleSheet("QLabel { background-color : yellow; color : black; }");

            // ui_.label_zpstatus->setText(QString::fromUtf8("SIM MODE"));
            ui_.label_imustatus->setText(QString::fromUtf8("SIM MODE"));
            // ui_.label_ecatstatus->setText(QString::fromUtf8("SIM MODE"));
            ui_.label_ftstatus->setText(QString::fromUtf8("SIM MODE"));

            ui_.stackedWidget->setCurrentIndex(2);
        }
        else
        {
            ui_.vjbtn->setDisabled(true);

            // ui_.label_zpstatus->setStyleSheet("QLabel { background-color : red; color : white; }");
            ui_.label_imustatus->setStyleSheet("QLabel { background-color : red; color : white; }");
            // ui_.label_ecatstatus->setStyleSheet("QLabel { background-color : red; color : white; }");
            ui_.label_ftstatus->setStyleSheet("QLabel { background-color : red; color : white; }");
            ui_.stackedWidget->setCurrentIndex(0);
        }

        ecatlabels.resize(33);
        safetylabels.resize(33);
        zplabels.resize(33);

        positionlabels.resize(15);
        torquelayout_.resize(15);
        torquelabels.resize(15);
        torquelabels2.resize(15);
        vellabels.resize(15);

        for (int i = 0; i < 15; i++)
        {
            positionlabels[i] = new QLabel(ui_.position_layout->parentWidget());
            ui_.position_layout->addWidget(positionlabels[i]);
            positionlabels[i]->setFrameShape(QFrame::Panel);

            vellabels[i] = new QLabel(ui_.velocity_layout->parentWidget());
            ui_.velocity_layout->addWidget(vellabels[i]);
            vellabels[i]->setFrameShape(QFrame::Panel);

            torquelayout_[i] = new QVBoxLayout();
            ui_.torque_layout->addLayout(torquelayout_[i]);

            torquelayout_[i]->setSpacing(0);

            // ui_.torque_layout2->setSpacing(0);
            // ui_.torque_layout->setSpacing(0);

            torquelabels2[i] = new QLabel(torquelayout_[i]->parentWidget());
            torquelayout_[i]->addWidget(torquelabels2[i]);
            torquelabels2[i]->setFrameShape(QFrame::Panel);

            torquelabels[i] = new QLabel(torquelayout_[i]->parentWidget());
            torquelayout_[i]->addWidget(torquelabels[i]);
            torquelabels[i]->setFrameShape(QFrame::NoFrame);
            // int top, bottom, left, right;
            // torquelabels2[i]->getContentsMargins(&left, &top, &right, &bottom);

            // torquelabels2[i]->setContentsMargins(left, top, right, 0);

            // torquelabels[i] = new QLabel(ui_.torque_layout->parentWidget());
            // ui_.torque_layout->addWidget(torquelabels[i]);
            // torquelabels[i]->setFrameShape(QFrame::Panel);

            // torquelabels[i]->setContentsMargins(left, 0, right, bottom);
        }

        // head
        for (int i = 0; i < 2; i++)
        {
            ecatlabels[i] = new QLabel(ui_.head_layout->parentWidget());
            ui_.head_layout->addWidget(ecatlabels[i]);
            ecatlabels[i]->setFrameShape(QFrame::Panel);

            safetylabels[i] = new QLabel(ui_.head_safety->parentWidget());
            ui_.head_safety->addWidget(safetylabels[i]);
            safetylabels[i]->setFrameShape(QFrame::Panel);

            zplabels[i] = new QLabel(ui_.head_elmo->parentWidget());
            ui_.head_elmo->addWidget(zplabels[i]);
            zplabels[i]->setFrameShape(QFrame::Panel);
        }

        for (int i = 2; i < 10; i++)
        {
            ecatlabels[i] = new QLabel(ui_.leftarm_layout->parentWidget());
            ui_.leftarm_layout->addWidget(ecatlabels[i]);
            ecatlabels[i]->setFrameShape(QFrame::Panel);

            safetylabels[i] = new QLabel(ui_.leftarm_safety->parentWidget());
            ui_.leftarm_safety->addWidget(safetylabels[i]);
            safetylabels[i]->setFrameShape(QFrame::Panel);

            zplabels[i] = new QLabel(ui_.leftarm_elmo->parentWidget());
            ui_.leftarm_elmo->addWidget(zplabels[i]);
            zplabels[i]->setFrameShape(QFrame::Panel);
        }

        for (int i = 10; i < 18; i++)
        {
            ecatlabels[i] = new QLabel(ui_.rightarm_layout->parentWidget());
            ui_.rightarm_layout->addWidget(ecatlabels[i]);
            ecatlabels[i]->setFrameShape(QFrame::Panel);

            safetylabels[i] = new QLabel(ui_.rightarm_safety->parentWidget());
            ui_.rightarm_safety->addWidget(safetylabels[i]);
            safetylabels[i]->setFrameShape(QFrame::Panel);

            zplabels[i] = new QLabel(ui_.rightarm_elmo->parentWidget());
            ui_.rightarm_elmo->addWidget(zplabels[i]);
            zplabels[i]->setFrameShape(QFrame::Panel);
        }

        for (int i = 18; i < 21; i++)
        {
            ecatlabels[i] = new QLabel(ui_.waist_layout->parentWidget());
            ui_.waist_layout->addWidget(ecatlabels[i]);
            ecatlabels[i]->setFrameShape(QFrame::Panel);

            safetylabels[i] = new QLabel(ui_.waist_safety->parentWidget());
            ui_.waist_safety->addWidget(safetylabels[i]);
            safetylabels[i]->setFrameShape(QFrame::Panel);

            zplabels[i] = new QLabel(ui_.waist_elmo->parentWidget());
            ui_.waist_elmo->addWidget(zplabels[i]);
            zplabels[i]->setFrameShape(QFrame::Panel);
        }

        for (int i = 21; i < 27; i++)
        {
            ecatlabels[i] = new QLabel(ui_.leftleg_layout->parentWidget());
            ui_.leftleg_layout->addWidget(ecatlabels[i]);
            ecatlabels[i]->setFrameShape(QFrame::Panel);

            safetylabels[i] = new QLabel(ui_.leftleg_safety->parentWidget());
            ui_.leftleg_safety->addWidget(safetylabels[i]);
            safetylabels[i]->setFrameShape(QFrame::Panel);

            zplabels[i] = new QLabel(ui_.leftleg_elmo->parentWidget());
            ui_.leftleg_elmo->addWidget(zplabels[i]);
            zplabels[i]->setFrameShape(QFrame::Panel);
        }

        for (int i = 27; i < 33; i++)
        {
            ecatlabels[i] = new QLabel(ui_.rightleg_layout->parentWidget());
            ui_.rightleg_layout->addWidget(ecatlabels[i]);
            ecatlabels[i]->setFrameShape(QFrame::Panel);

            safetylabels[i] = new QLabel(ui_.rightleg_safety->parentWidget());
            ui_.rightleg_safety->addWidget(safetylabels[i]);
            safetylabels[i]->setFrameShape(QFrame::Panel);

            zplabels[i] = new QLabel(ui_.rightleg_elmo->parentWidget());
            ui_.rightleg_elmo->addWidget(zplabels[i]);
            zplabels[i]->setFrameShape(QFrame::Panel);
        }

        // ecat constant tune
        ecattexts.resize(33);
        for (int i = 0; i < 2; i++)
        {
            ecattexts[i] = new QLineEdit(ui_.head_layout_2->parentWidget());
            ui_.head_layout_2->addWidget(ecattexts[i]);
        }
        for (int i = 2; i < 10; i++)
        {
            ecattexts[i] = new QLineEdit(ui_.leftarm_layout_2->parentWidget());
            ui_.leftarm_layout_2->addWidget(ecattexts[i]);
        }
        for (int i = 10; i < 18; i++)
        {
            ecattexts[i] = new QLineEdit(ui_.rightarm_layout_2->parentWidget());
            ui_.rightarm_layout_2->addWidget(ecattexts[i]);
        }
        for (int i = 18; i < 21; i++)
        {
            ecattexts[i] = new QLineEdit(ui_.waist_layout_2->parentWidget());
            ui_.waist_layout_2->addWidget(ecattexts[i]);
        }
        for (int i = 21; i < 27; i++)
        {
            ecattexts[i] = new QLineEdit(ui_.leftleg_layout_2->parentWidget());
            ui_.leftleg_layout_2->addWidget(ecattexts[i]);
        }
        for (int i = 27; i < 33; i++)
        {
            ecattexts[i] = new QLineEdit(ui_.rightleg_layout_2->parentWidget());
            ui_.rightleg_layout_2->addWidget(ecattexts[i]);
        }

        for (int i = 0; i < 33; i++)
        {
            ecatlabels[i]->setAlignment(Qt::AlignCenter);
            ecattexts[i]->setText(QString::fromUtf8("0.0"));
            ecattexts[i]->setValidator(new QDoubleValidator(-1000, 1000, 3, this));

            safetylabels[i]->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
        }
        ui_.taskgain->setDisabled(true);

        ui_.graphicsView->setDisabled(true);
        ui_.graphicsView->setHidden(true);

        view = new MyQGraphicsView(widget_);
        view->setObjectName(QStringLiteral("graphicsViewCustom"));
        view->setGeometry(ui_.graphicsView->geometry());
        view->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        view->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        view->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContentsOnFirstShow);
        view->setInteractive(true);
        view->setDragMode(QGraphicsView::ScrollHandDrag);
        view->setTransformationAnchor(QGraphicsView::NoAnchor);
        view->setResizeAnchor(QGraphicsView::NoAnchor);

        scene = new MyQGraphicsScene(widget_);
        // scene->setSceneRect(0, 0, view->width(), view->height());
        view->setScene(scene);

        scene2 = new QGraphicsScene(widget_);

        ui_.graphicsView_2->setScene(scene2);

        pbar = new QGraphicsRectItem(QRectF(-30.5, -140.5, 61, 281));

        scene2->addItem(pbar);

        // std::cout << widget_->findChild<QObject *>("graphicsViewCustom")->objectName().toStdString() << std::endl;

        QBrush redbrush(Qt::red);
        QBrush bluebrush(Qt::blue);
        QBrush yellowbrush(Qt::yellow);
        QBrush blackbrush(Qt::black);

        pbar->setBrush(QBrush(Qt::yellow));

        QPen blackpen(Qt::black);
        Pelv = new QGraphicsRectItem(QRectF(-150 / 4, -40 / 4, 300 / 4, 80 / 4));
        scene->addItem(Pelv);

        lfoot_d = new QGraphicsRectItem(QRectF(-85 / 4, -120 / 4, 170 / 4, 300 / 4));
        scene->addItem(lfoot_d);

        rfoot_d = new QGraphicsRectItem(QRectF(-85 / 4, -120 / 4, 170 / 4, 300 / 4));
        scene->addItem(rfoot_d);

        com_d = scene->addEllipse(-10, -10, 20, 20, blackpen, yellowbrush);
        rfoot_c = scene->addEllipse(-2, -2, 4, 4, blackpen, blackbrush);
        lfoot_c = scene->addEllipse(-2, -2, 4, 4, blackpen, blackbrush);

        rhand_c = scene->addEllipse(-2, -2, 4, 4, blackpen, blackbrush);
        lhand_c = scene->addEllipse(-2, -2, 4, 4, blackpen, blackbrush);

        zmp = scene->addEllipse(-5, -5, 10, 10, blackpen, redbrush);

        lfoot_zmp = scene->addEllipse(-2, -2, 4, 4, blackpen, redbrush);
        rfoot_zmp = scene->addEllipse(-2, -2, 4, 4, blackpen, redbrush);

        scene->addLine(-20, 0, 40, 0, blackpen);
        scene->addLine(0, -20, 0, 40, blackpen);

        QGraphicsTextItem *front = scene->addText("front");
        front->setPos(0, 50);

        // ui_.graphicsView->scale(10, 10);

        // ui_.graphicsView->setSceneRect(-210, -260, 421, 521);

        // for(int i=0;i<)

        //    label_40 = new QLabel(verticalLayoutWidget);
        //    label_40->setObjectName(QStringLiteral("label_40"));

        //    leftarm_layout->addWidget(label_40);

        // widget_->s
        // connect(ui_.log_btn,SIGNAL(pressed()),this,SLOR(ui_.))
        /*
    line = new QLineSeries();
    chart = new QChart();
    QFont labelsFont;
    labelsFont.setPixelSize(8);

    chart->legend()->hide();
    chart->addSeries(line);
    chart->createDefaultAxes();
    chart->setContentsMargins(-20, -20, -20, -20);
    chart->layout()->setContentsMargins(0, 0, 0, 0);
    chart->setBackgroundRoundness(0);
    chart->axisX()->setLabelsFont(labelsFont);
    chart->axisY()->setLabelsVisible(false);
    chart->axisY()->setLabelsFont(labelsFont);

    line_roll = new QLineSeries();
    chart_roll = new QChart();
    line_pitch = new QLineSeries();
    chart_pitch = new QChart();
    line_yaw = new QLineSeries();
    chart_yaw = new QChart();

    chart_roll->legend()->hide();
    chart_roll->addSeries(line_roll);
    chart_roll->createDefaultAxes();
    chart_roll->setContentsMargins(-20, -20, -20, -20);
    chart_roll->layout()->setContentsMargins(0, 0, 0, 0);
    chart_roll->setBackgroundRoundness(0);
    chart_roll->axisX()->setLabelsFont(labelsFont);
    chart_roll->axisY()->setLabelsFont(labelsFont);

    chart_pitch->legend()->hide();
    chart_pitch->addSeries(line_pitch);
    chart_pitch->createDefaultAxes();
    chart_pitch->setContentsMargins(-20, -20, -20, -20);
    chart_pitch->layout()->setContentsMargins(0, 0, 0, 0);
    chart_pitch->setBackgroundRoundness(0);
    chart_pitch->axisX()->setLabelsFont(labelsFont);
    chart_pitch->axisY()->setLabelsFont(labelsFont);

    chart_yaw->legend()->hide();
    chart_yaw->addSeries(line_yaw);
    chart_yaw->createDefaultAxes();
    chart_yaw->setContentsMargins(-20, -20, -20, -20);
    chart_yaw->layout()->setContentsMargins(0, 0, 0, 0);
    chart_yaw->setBackgroundRoundness(0);
    chart_yaw->axisX()->setLabelsFont(labelsFont);
    chart_yaw->axisY()->setLabelsFont(labelsFont);

    ui_.imu_roll->setChart(chart_roll);
    ui_.imu_roll->setRenderHint(QPainter::Antialiasing);

    ui_.imu_pitch->setChart(chart_pitch);
    ui_.imu_pitch->setRenderHint(QPainter::Antialiasing);

    ui_.imu_yaw->setChart(chart_yaw);
    ui_.imu_yaw->setRenderHint(QPainter::Antialiasing);*/

        // QChartView *chartView = new QChartView(chart, ui_.widget);
        // chartView->setRenderHint(QPainter::Antialiasing);
    }
    void DyrosBoltGui::shutdownPlugin()
    {
    }

    void DyrosBoltGui::gs_test()
    {
        std::cout << "Hello World!" << std::endl;
    }

    void DyrosBoltGui::slidervelcommand()
    {
        velcmd_msg.des_vel.resize(6);
        int gui_selected = ui_.comboBox->currentIndex();
        int start_index = 0;
        if (gui_selected == 0)
        {
            velcmd_msg.task_link = 0;
            start_index = 0;
        }
        else if (gui_selected == 1)
        {
            velcmd_msg.task_link = 0;
            start_index = 3;
        }
        else if (gui_selected == 2)
        {
            velcmd_msg.task_link = 1;
            start_index = 3;
        }
        else if (gui_selected == 3)
        {
            velcmd_msg.task_link = 2;
            start_index = 0;
        }
        else if (gui_selected == 4)
        {
            velcmd_msg.task_link = 2;
            start_index = 3;
        }
        else if (gui_selected == 5)
        {
            velcmd_msg.task_link = 3;
            start_index = 0;
        }
        else if (gui_selected == 6)
        {
            velcmd_msg.task_link = 3;
            start_index = 3;
        }
        velcmd_msg.des_vel[start_index + 0] = (ui_.horizontalSlider->value() - 50) / 200.0 * ui_.horizontalSlider_4->value();
        velcmd_msg.des_vel[start_index + 1] = (ui_.horizontalSlider_2->value() - 50) / 200.0 * ui_.horizontalSlider_4->value();
        velcmd_msg.des_vel[start_index + 2] = (ui_.horizontalSlider_3->value() - 50) / 200.0 * ui_.horizontalSlider_4->value();

        velcommand_pub.publish(velcmd_msg);
    }

    void DyrosBoltGui::sliderrel1()
    {
        ui_.horizontalSlider->setValue(50);
    }

    void DyrosBoltGui::sliderrel2()
    {
        ui_.horizontalSlider_2->setValue(50);
    }
    void DyrosBoltGui::sliderrel3()
    {
        ui_.horizontalSlider_3->setValue(50);
    }

    void DyrosBoltGui::saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const
    {
    }

    void DyrosBoltGui::restoreSettings(const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings)
    {
    }

    void DyrosBoltGui::taskmodecb(int index)
    {
        static int index_before = -1;

        if ((index_before == 2) || (index_before == 3))
        {
        }
        else
        {
            if (index == 7)
            {
                ui_.label_83->setText(QString::fromUtf8("X Axis"));
                ui_.label_86->setText(QString::fromUtf8("Y Axis"));
                ui_.com_pos->setText(QString::number(0.0));
                ui_.com_height->setText(QString::number(0.0));
                ui_.pelv_pitch->setText(QString::number(pelv_pitch));
                ui_.com_pitch->setText(QString::number(upper_pitch));
            }
            else
            {
                ui_.label_83->setText(QString::fromUtf8("COM Pos"));
                ui_.label_86->setText(QString::fromUtf8("Height"));
                ui_.com_pos->setText(QString::number(0.5));
                ui_.com_height->setText(QString::number(com_height));
                ui_.pelv_pitch->setText(QString::number(0));
                ui_.com_pitch->setText(QString::number(0));
            }
        }

        index_before = index;
    }

    void DyrosBoltGui::solvermode_cb(int state)
    {
        if (state == 0)
        {
            ui_.cr_mode->setEnabled(true);
            ui_.cr_mode->setCurrentIndex(0);
        }
        else
        {
            ui_.cr_mode->setDisabled(true);
            ui_.cr_mode->setCurrentIndex(2);
        }
    }

    void DyrosBoltGui::jointstatecb(const sensor_msgs::JointStateConstPtr &msg)
    {
        for (int i = 0; i < 33; i++)
        {
            q_[i] = msg->position[i];
            q_dot_[i] = msg->velocity[i];
            torque_desired_[i] = msg->effort[i];
        }

        if (ui_.stackedWidget->currentIndex() == 1)
        {
            for (int i = 0; i < 15; i++)
            {
                positionlabels[i]->setText(QString::number(msg->position[i], 'f', 4));
                vellabels[i]->setText(QString::number(msg->velocity[i], 'f', 4));

                int h = torquelabels[i]->height();
                int w = torquelabels[i]->width();

                torquelabels[i]->setMaximumHeight(5);
                torquelabels[i]->setMaximumWidth(w);

                // torquelabels[i]->setFixedHeight(h);

                QPixmap pix(w, 5);
                QPainter paint(&pix);
                pix.fill(Qt::lightGray);

                double max_per = abs(msg->effort[i]) / (1000 / NM2CNT_J[i]);

                int bw = w * max_per;

                paint.setPen(QPen(QColor(0, 0, 0, 0)));

                if (max_per > 1)
                {
                    max_per = 1.0;
                }

                int red; // = 255 * max_per;

                int green; // = 255 * (1 - max_per);
                if (max_per < 0.5)
                {
                    red = 255 * max_per * 2;
                    green = 255;
                }
                else if (max_per >= 0.5 && max_per < 1)
                {
                    red = 255;
                    green = 255 - 255 * (max_per - 0.5) * 2;
                }
                else if (max_per >= 1)
                {
                    red = 255;
                    green = 0;
                }

                paint.setBrush(QColor(red, green, 0));

                paint.drawRect(QRect(0, 0, bw, 5));

                torquelabels[i]->setPixmap(pix);
                torquelabels2[i]->setText(QString::number(msg->effort[i], 'f', 4));
            }
        }
    }

    void DyrosBoltGui::sysstatecb(const std_msgs::Int8MultiArrayConstPtr &msg)
    {
        if (msg->data[0] == 0) // imu
        {
            ui_.label_imustatus->setStyleSheet("QLabel { background-color : red ; color : white; }");
            ui_.label_imustatus->setText(QString::fromUtf8("NOT OK"));
        }
        else if (msg->data[0] == 1)
        {
            ui_.label_imustatus->setStyleSheet("QLabel { background-color : yellow; color : black; }");
            ui_.label_imustatus->setText(QString::fromUtf8("waiting"));
        }
        else if (msg->data[0] == 2)
        {
            ui_.label_imustatus->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
            ui_.label_imustatus->setText(QString::fromUtf8("OK"));
        }
        else if (msg->data[0] == 3)
        {
            ui_.label_imustatus->setStyleSheet("QLabel { background-color : yellow; color : black; }");
            ui_.label_imustatus->setText(QString::fromUtf8("SIM MODE"));
        }

        if (msg->data[1] == 0) // zp
        {
            ui_.label_ecat1status->setStyleSheet("QLabel { background-color : red ; color : white; }");
            ui_.label_ecat1status->setText(QString::fromUtf8("NOT OK"));
        }
        else if (msg->data[1] == 1)
        {
            ui_.label_ecat1status->setStyleSheet("QLabel { background-color : yellow ; color : black; }");
            ui_.label_ecat1status->setText(QString::fromUtf8("INITIAL"));
        }
        else if (msg->data[1] == 2)
        {
            ui_.label_ecat1status->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
            ui_.label_ecat1status->setText(QString::fromUtf8("CONTROL"));
        }
        else if (msg->data[1] == 3)
        {
            ui_.label_ecat1status->setStyleSheet("QLabel { background-color : yellow; color : black; }");
            ui_.label_ecat1status->setText(QString::fromUtf8("SIM MODE"));
        }

        if (msg->data[2] == 0) // ft
        {
            ui_.label_ftstatus->setStyleSheet("QLabel { background-color : red ; color : white; }");
            ui_.label_ftstatus->setText(QString::fromUtf8("NOT OK"));
        }
        else if (msg->data[2] == 1)
        {
            ui_.label_ftstatus->setStyleSheet("QLabel { background-color : yellow ; color : black; }");
            ui_.label_ftstatus->setText(QString::fromUtf8("INIT REQ"));
        }
        else if (msg->data[2] == 2)
        {
            ui_.label_ftstatus->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
            ui_.label_ftstatus->setText(QString::fromUtf8("OK"));
        }
        else if (msg->data[2] == 3)
        {
            ui_.label_ftstatus->setStyleSheet("QLabel { background-color : yellow; color : black; }");
            ui_.label_ftstatus->setText(QString::fromUtf8("SIM MODE"));
        }

        if (msg->data[3] == 0) // ecat
        {
            ui_.label_ecat2status->setStyleSheet("QLabel { background-color : red ; color : white; }");
            ui_.label_ecat2status->setText(QString::fromUtf8("NOT OK"));
        }
        else if (msg->data[3] == 1)
        {
            ui_.label_ecat2status->setStyleSheet("QLabel { background-color : yellow; color : black; }");
            ui_.label_ecat2status->setText(QString::fromUtf8("INITIAL"));
        }
        else if (msg->data[3] == 2)
        {
            ui_.label_ecat2status->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
            ui_.label_ecat2status->setText(QString::fromUtf8("CONTROL"));
        }
        else if (msg->data[3] == 3)
        {
            ui_.label_ecat2status->setStyleSheet("QLabel { background-color : yellow; color : black; }");
            ui_.label_ecat2status->setText(QString::fromUtf8("SIM MODE"));
        }

        if (msg->data[4] == 1) // se
        {
            ui_.label_sestatus->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
            ui_.label_sestatus->setText(QString::fromUtf8("ON"));
        }
        else if (msg->data[4] == 0)
        {
            ui_.label_sestatus->setStyleSheet("QLabel { background-color : red ; color : black; }");
            ui_.label_sestatus->setText(QString::fromUtf8("OFF"));
        }

        if (msg->data[5] == 0) // tc
        {
            ui_.label_tcstatus->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
            ui_.label_tcstatus->setText(QString::fromUtf8("ON"));
        }
        else if (msg->data[5] == 1)
        {
            ui_.label_tcstatus->setStyleSheet("QLabel { background-color : yellow ; color : black; }");
            ui_.label_tcstatus->setText(QString::fromUtf8("WARN"));
        }
        else if (msg->data[5] == 2)
        {
            ui_.label_tcstatus->setStyleSheet("QLabel { background-color : red ; color : black; }");
            ui_.label_tcstatus->setText(QString::fromUtf8("ERROR"));
        }
        else if (msg->data[5] == 3)
        {
            ui_.label_tcstatus->setStyleSheet("QLabel { background-color : rgba(0, 0, 0, 0) ; color : black; }");
            ui_.label_tcstatus->setText(QString::fromUtf8("OFF"));
        }

        if (msg->data[6] == -2)
        {
        }
        else if (msg->data[6] == -1)
        {
            for (int i = 15; i < 23; i++)
            {
                ecatlabels[mo2g[i]]->setText(QString::fromUtf8(""));
                ecatlabels[mo2g[i]]->setStyleSheet("QLabel { background-color : transparent ; color : black; }");
            }
        }
        else
        {
            int jn = msg->data[6];
            if ((jn >= 15) && (jn < 23))
            {
                int num = mo2g[jn];
                for (int i = 15; i < 23; i++)
                {
                    ecatlabels[mo2g[i]]->setText(QString::fromUtf8(""));
                    ecatlabels[mo2g[i]]->setStyleSheet("QLabel { background-color : transparent ; color : black; }");
                }
                ecatlabels[num]->setText(QString::fromUtf8("CONTACT"));
                ecatlabels[num]->setStyleSheet("QLabel { background-color : yellow ; color : black; }");
            }
        }

        if (msg->data[7] == -2)
        {
        }
        else if (msg->data[7] == -1)
        {
            for (int i = 25; i < 33; i++)
            {
                ecatlabels[mo2g[i]]->setText(QString::fromUtf8(""));
                ecatlabels[mo2g[i]]->setStyleSheet("QLabel { background-color : transparent ; color : black; }");
            }
        }
        else
        {
            int jn = msg->data[7];
            if ((jn >= 25) && (jn < 33))
            {
                int num = mo2g[jn];
                for (int i = 25; i < 33; i++)
                {
                    ecatlabels[mo2g[i]]->setText(QString::fromUtf8(""));
                    ecatlabels[mo2g[i]]->setStyleSheet("QLabel { background-color : transparent ; color : black; }");
                }
                ecatlabels[num]->setText(QString::fromUtf8("CONTACT"));
                ecatlabels[num]->setStyleSheet("QLabel { background-color : yellow ; color : black; }");
            }
        }
    }

    void DyrosBoltGui::que_downbtn()
    {
    }
    void DyrosBoltGui::que_upbtn()
    {
    }
    void DyrosBoltGui::que_deletebtn()
    {
        std::cout << ui_.que_listwidget->currentIndex().row() << std::endl;
        QListWidgetItem *item = ui_.que_listwidget->takeItem(ui_.que_listwidget->currentIndex().row());

        ui_.que_listwidget->removeItemWidget(item);
    }
    void DyrosBoltGui::que_resetbtn()
    {
        ui_.que_listwidget->clear();
        tq_.clear();
    }

    void DyrosBoltGui::que_sendbtn()
    {
        task_que_msg.tque.resize(tq_.size());
        for (int i = 0; i < tq_.size(); i++)
        {
            task_que_msg.tque[i] = tq_[i].tc_;
        }
        task_que_pub.publish(task_que_msg);
    }

    void DyrosBoltGui::customtaskgaincb(int state)
    {
        if (ui_.customtaskgain->isChecked())
        {
            ui_.taskgain->setEnabled(true);
        }
        else
        {
            ui_.taskgain->setDisabled(true);
        }
    }

    void DyrosBoltGui::safetyresetbtncb()
    {
        // for (int i = 0; i < 33; i++)
        // {
        //     safetylabels[i]->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
        // }
        com_msg.data = std::string("safetyreset");
        com_pub.publish(com_msg);
    }

    void DyrosBoltGui::safety2btncb()
    {
        // for (int i = 0; i < 33; i++)
        // {
        //     safetylabels[i]->setStyleSheet("QLabel { background-color : yellow ; color : black; }");
        // }
        com_msg.data = std::string("safetydisable");
        com_pub.publish(com_msg);
    }

    void DyrosBoltGui::timercb(const std_msgs::Float32ConstPtr &msg)
    {
        robot_time = msg->data;
        ui_.currenttime->setText(QString::number(msg->data, 'f', 3));
    }

    void DyrosBoltGui::ecatpbtn()
    {
        ui_.stackedWidget->setCurrentIndex(0);
    }
    void DyrosBoltGui::statpbtn()
    {
        ui_.stackedWidget->setCurrentIndex(1);
    }
    void DyrosBoltGui::commandpbtn()
    {
        ui_.stackedWidget->setCurrentIndex(2);
    }
    void DyrosBoltGui::mtunebtn()
    {
        ui_.stackedWidget->setCurrentIndex(3);
    }
    void DyrosBoltGui::walkingbtn()
    {
        ui_.stackedWidget->setCurrentIndex(4);
    }
    void DyrosBoltGui::avatarbtn()
    {
        ui_.stackedWidget->setCurrentIndex(5);
    }

    void DyrosBoltGui::comstatecb(const std_msgs::Float32MultiArrayConstPtr &msg)
    {
        ui_.label_e1latavg->setText(QString::number(msg->data[0], 'f', 3));
        ui_.label_e1latmax->setText(QString::number(msg->data[1], 'f', 3));
        ui_.label_e1comavg->setText(QString::number(msg->data[2], 'f', 3));
        ui_.label_e1commax->setText(QString::number(msg->data[3], 'f', 3));
        ui_.label_e1ovf->setText(QString::number((int)msg->data[4]));

        ui_.label_e2latavg->setText(QString::number(msg->data[5], 'f', 3));
        ui_.label_e2latmax->setText(QString::number(msg->data[6], 'f', 3));
        ui_.label_e2comavg->setText(QString::number(msg->data[7], 'f', 3));
        ui_.label_e2commax->setText(QString::number(msg->data[8], 'f', 3));
        ui_.label_e2ovf->setText(QString::number((int)msg->data[9]));

        ui_.label_e1cnt->setText(QString::number((int)msg->data[10]));
        ui_.label_e2cnt->setText(QString::number((int)msg->data[11]));

        ui_.label_e1wcnt->setText(QString::number((int)msg->data[12]));
        ui_.label_e2wcnt->setText(QString::number((int)msg->data[13]));

        int prg_val = (int)(100.0 * msg->data[14]);

        // ui_.torqueStatus->setValue(prg_val);

        pbar->setY(140.5 + 140.5 - 281.0 * msg->data[14]);

        QString str_percentage_;

        str_percentage_.sprintf("%d %%", (int)(100 * msg->data[14]));

        ui_.label_torquerat->setText(str_percentage_);
    }

    void DyrosBoltGui::ecatstatecb(const std_msgs::Int8MultiArrayConstPtr &msg)
    {

        for (int i = 0; i < 33; i++)
        {
            int num_ecat = msg->data[i];
            int num_zp = msg->data[i + 33];
            int num_safety = msg->data[i + 66];

            // safety->elmo->zp

            if (num_safety == 0)
            {
                safetylabels[mo2g[i]]->setText(QString::fromUtf8("OK"));
                safetylabels[mo2g[i]]->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
            }
            else if (num_safety == 1)
            {
                safetylabels[mo2g[i]]->setText(QString::fromUtf8("JL"));
                safetylabels[mo2g[i]]->setStyleSheet("QLabel { background-color : red ; color : white; }");
            }
            else if (num_safety == 2)
            {
                safetylabels[mo2g[i]]->setText(QString::fromUtf8("VL"));
                safetylabels[mo2g[i]]->setStyleSheet("QLabel { background-color : red ; color : white; }");
            }
            else if (num_safety == 3)
            {
                safetylabels[mo2g[i]]->setText(QString::fromUtf8("TL"));
                safetylabels[mo2g[i]]->setStyleSheet("QLabel { background-color : red ; color : white; }");
            }
            else if (num_safety == 4)
            {
                safetylabels[mo2g[i]]->setText(QString::fromUtf8("CL"));
                safetylabels[mo2g[i]]->setStyleSheet("QLabel { background-color : red ; color : white; }");
            }
            else if (num_safety == 6) //No Torque
            {
                safetylabels[mo2g[i]]->setText(QString::fromUtf8("NOT"));
                safetylabels[mo2g[i]]->setStyleSheet("QLabel { background-color : black ; color : white; }");
            }
            else if (num_safety == 7) //Lock Lower
            {
                safetylabels[mo2g[i]]->setText(QString::fromUtf8("LL"));
                safetylabels[mo2g[i]]->setStyleSheet("QLabel { background-color : orange ; color : black; }");
            }
            else if (num_safety == 9)
            {
                safetylabels[mo2g[i]]->setText(QString::fromUtf8("NOS"));
                safetylabels[mo2g[i]]->setStyleSheet("QLabel { background-color : gray ; color : white; }");
            }

            if (num_ecat == 0) // zp started
            {
                ecatlabels[mo2g[i]]->setText(QString::fromUtf8("0"));
                ecatlabels[mo2g[i]]->setStyleSheet("QLabel { background-color : yellow ; color : black; }");
            }
            else if (num_ecat == 1)
            {
                ecatlabels[mo2g[i]]->setText(QString::fromUtf8("1"));
                ecatlabels[mo2g[i]]->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
            }
            else if (num_ecat == 2)
            {
                ecatlabels[mo2g[i]]->setText(QString::fromUtf8("2"));
                ecatlabels[mo2g[i]]->setStyleSheet("QLabel { background-color : orange ; color : black; }");
            }
            else if (num_ecat == 3)
            {
                ecatlabels[mo2g[i]]->setText(QString::fromUtf8("3"));
                ecatlabels[mo2g[i]]->setStyleSheet("QLabel { background-color : red ; color : white; }");
            }
            else if (num_ecat == 4)
            {
                ecatlabels[mo2g[i]]->setText(QString::fromUtf8("4"));
                ecatlabels[mo2g[i]]->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
            }

            if (num_zp == 0) // zp started
            {
                zplabels[mo2g[i]]->setText(QString::fromUtf8("zp"));
                zplabels[mo2g[i]]->setStyleSheet("QLabel { background-color : yellow ; color : black; }");
            }
            else if (num_zp == 1)
            {
                zplabels[mo2g[i]]->setText(QString::fromUtf8("sc"));
                zplabels[mo2g[i]]->setStyleSheet("QLabel { background-color : yellow ; color : black; }");
            }
            else if (num_zp == 2)
            {
                zplabels[mo2g[i]]->setText(QString::fromUtf8("man"));
                zplabels[mo2g[i]]->setStyleSheet("QLabel { background-color : orange ; color : black; }");
            }
            else if (num_zp == 3)
            {
                zplabels[mo2g[i]]->setText(QString::fromUtf8("length"));
                zplabels[mo2g[i]]->setStyleSheet("QLabel { background-color : red ; color : white; }");
            }
            else if (num_zp == 4)
            {
                zplabels[mo2g[i]]->setText(QString::fromUtf8("go0"));
                zplabels[mo2g[i]]->setStyleSheet("QLabel { background-color : yellow ; color : black; }");
            }
            else if (num_zp == 5)
            {
                zplabels[mo2g[i]]->setText(QString::fromUtf8("OK"));
                zplabels[mo2g[i]]->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
            }
        }
    }

    void DyrosBoltGui::plainTextEditcb(const std_msgs::StringConstPtr &msg)
    {
        // std::cout << msg->data << std::endl;
        std::string rcv_msg;
        rcv_msg = msg->data;
        std::string word;
        std::vector<std::string> words;
        for (auto x : rcv_msg)
        {
            if (x == ' ')
            {
                words.push_back(word);
                word.erase();
            }
            else
                word = word + x;
        }
        words.push_back(word);
        if (words[0] == "jointzp")
        {
            int num = elng[atoi(words[1].c_str())];
            if (atoi(words[2].c_str()) == 0) // zp started
            {
                ecatlabels[num]->setText(QString::fromUtf8("zp"));
                ecatlabels[num]->setStyleSheet("QLabel { background-color : yellow ; color : black; }");
            }
            else if (atoi(words[2].c_str()) == 1) // zp success
            {
                ecatlabels[num]->setText(QString::fromUtf8("ok"));
                ecatlabels[num]->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
            }
            else if (atoi(words[2].c_str()) == 2) // zp manual
            {
                ecatlabels[num]->setText(QString::fromUtf8("manual"));
                ecatlabels[num]->setStyleSheet("QLabel { background-color : orange ; color : black; }");
            }
            else if (atoi(words[2].c_str()) == 3) // failed
            {
                ecatlabels[num]->setText(QString::fromUtf8("fail"));
                ecatlabels[num]->setStyleSheet("QLabel { background-color : red ; color : white; }");
            }
        }
        else if (words[0] == "Lock")
        {
            int num = elng[atoi(words[1].c_str())];
            safetylabels[num]->setStyleSheet("QLabel { background-color : red ; color : white; }");
        }
        else if (words[0] == "DOB")
        {
            int num = mo2g[atoi(words[1].c_str())];
            if ((num < 0) || (num > 32))
            {
                // std::cout << "Joint Number Exceed" << std::endl;
            }
            else
            {
                //    ecatlabels[num]->setText(QString::fromUtf8("CONTACT"));
                //  ecatlabels[num]->setStyleSheet("QLabel { background-color : yellow ; color : black; }");
            }
        }
        else if (msg->data == "imuvalid")
        {
            ui_.label_imustatus->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
            ui_.label_imustatus->setText(QString::fromUtf8("OK"));
        }
        else if (msg->data == "imunotvalid")
        {
            ui_.label_imustatus->setStyleSheet("QLabel { background-color : red; color : white; }");
            ui_.label_imustatus->setText(QString::fromUtf8("NOT OK"));
        }
        else if (msg->data == "zpgood")
        {
            // ui_.label_zpstatus->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
            // ui_.label_zpstatus->setText(QString::fromUtf8("OK"));
        }
        else if (msg->data == "initreq")
        {
            ui_.label_ftstatus->setStyleSheet("QLabel { background-color : yellow ; color : black; }");
            ui_.label_ftstatus->setText(QString::fromUtf8("INIT REQ"));
        }
        else if (msg->data == "ftgood")
        {
            ui_.label_ftstatus->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
            ui_.label_ftstatus->setText(QString::fromUtf8("OK"));
        }
        else if (msg->data == "ecatgood")
        {
            // ui_.label_ecatstatus->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
            // ui_.label_ecatstatus->setText(QString::fromUtf8("OK"));
        }
        else if (msg->data == "zpnotgood")
        {
            // ui_.label_zpstatus->setStyleSheet("QLabel { background-color : red; color : white; }");
            // ui_.label_zpstatus->setText(QString::fromUtf8("NOT OK"));
        }
        else if (msg->data == "ecatcommutationdone")
        {
            // ui_.label_zpstatus->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
            // ui_.label_zpstatus->setText(QString::fromUtf8("OK"));
        }
        else if (msg->data == "ecatcommutation")
        {
            // ui_.label_zpstatus->setStyleSheet("QLabel { background-color : red; color : white; }");
            // ui_.label_zpstatus->setText(QString::fromUtf8("COMMUTATION"));
        }
        else
        {
            ui_.plainTextEdit->appendPlainText(QString::fromStdString(msg->data));
        }
    }

    void DyrosBoltGui::pointcb(const geometry_msgs::PolygonStampedConstPtr &msg)
    {
        // msg->polygon.points[0].x;
        // msg->polygon.points[0].y;

        // std::cout<<msg->polygon.points[0].x*25<<std::endl;

        com_d->setPos(QPointF(msg->polygon.points[0].y * 250, msg->polygon.points[0].x * 250));

        ui_.label->setText(QString::number(msg->polygon.points[0].x, 'f', 5));
        ui_.label_2->setText(QString::number(msg->polygon.points[0].y, 'f', 5));
        com_height = msg->polygon.points[0].z;
        rfoot_d->setPos(QPointF(msg->polygon.points[1].y * 250, msg->polygon.points[1].x * 250));
        rfoot_d->setRotation(msg->polygon.points[9].z * -180.0 / 3.141592);
        rfoot_c->setPos(QPointF(msg->polygon.points[1].y * 250, msg->polygon.points[1].x * 250));

        ui_.label_73->setText(QString::number(msg->polygon.points[1].x, 'f', 5));
        ui_.label_74->setText(QString::number(msg->polygon.points[1].y, 'f', 5));

        lfoot_d->setPos(QPointF(msg->polygon.points[2].y * 250, msg->polygon.points[2].x * 250));
        lfoot_d->setRotation(msg->polygon.points[8].z * -180.0 / 3.141592);
        lfoot_c->setPos(QPointF(msg->polygon.points[2].y * 250, msg->polygon.points[2].x * 250));

        lhand_c->setPos(QPointF(msg->polygon.points[6].y * 250, msg->polygon.points[6].x * 250));
        rhand_c->setPos(QPointF(msg->polygon.points[5].y * 250, msg->polygon.points[5].x * 250));

        lfoot_zmp->setPos(QPointF(msg->polygon.points[13].y * 250, msg->polygon.points[13].x * 250));

        rfoot_zmp->setPos(QPointF(msg->polygon.points[14].y * 250, msg->polygon.points[14].x * 250));

        float ang_pelv = msg->polygon.points[4].z;

        Pelv->setPos(QPointF((msg->polygon.points[3].y + 0.07 * sin(ang_pelv)) * 250, (msg->polygon.points[3].x + 0.07 * cos(ang_pelv)) * 250));
        Pelv->setRotation(ang_pelv * -180.0 / 3.141592);

        zmp->setPos(QPointF(msg->polygon.points[7].y * 250, msg->polygon.points[7].x * 250));

        ui_.label_64->setText(QString::number(msg->polygon.points[2].x, 'f', 5));
        ui_.label_65->setText(QString::number(msg->polygon.points[2].y, 'f', 5));

        // pelvis rpy

        pelv_pitch = msg->polygon.points[4].y * 180.0 / 3.141592;
        upper_pitch = msg->polygon.points[14].y * 180.0 / 3.141592;

        ui_.label_14->setText(QString::number(msg->polygon.points[4].x * 180.0 / 3.141592, 'f', 5));
        ui_.label_15->setText(QString::number(msg->polygon.points[4].y * 180.0 / 3.141592, 'f', 5));
        ui_.label_16->setText(QString::number(msg->polygon.points[4].z * 180.0 / 3.141592, 'f', 5));

        ui_.label_13->setText(QString::number(msg->polygon.points[3].x, 'f', 5));
        ui_.label_21->setText(QString::number(msg->polygon.points[3].y, 'f', 5));
        ui_.label_91->setText(QString::number(msg->polygon.points[3].z, 'f', 5));

        // zmp by ft

        ui_.label_22->setText(QString::number(msg->polygon.points[7].x, 'f', 5));
        ui_.label_23->setText(QString::number(msg->polygon.points[7].y, 'f', 5));

        // Right hand
        ui_.label_126->setText(QString::number(msg->polygon.points[5].x, 'f', 5));
        ui_.label_127->setText(QString::number(msg->polygon.points[5].y, 'f', 5));
        ui_.label_128->setText(QString::number(msg->polygon.points[5].z, 'f', 5));

        // Left hand
        ui_.label_119->setText(QString::number(msg->polygon.points[6].x, 'f', 5));
        ui_.label_120->setText(QString::number(msg->polygon.points[6].y, 'f', 5));
        ui_.label_121->setText(QString::number(msg->polygon.points[6].z, 'f', 5));

        // LF orient
        ui_.label_105->setText(QString::number(msg->polygon.points[8].x * 180.0 / 3.141592, 'f', 5));
        ui_.label_106->setText(QString::number(msg->polygon.points[8].y * 180.0 / 3.141592, 'f', 5));
        ui_.label_107->setText(QString::number(msg->polygon.points[8].z * 180.0 / 3.141592, 'f', 5));

        // RF orient
        ui_.label_112->setText(QString::number(msg->polygon.points[9].x * 180.0 / 3.141592, 'f', 5));
        ui_.label_113->setText(QString::number(msg->polygon.points[9].y * 180.0 / 3.141592, 'f', 5));
        ui_.label_114->setText(QString::number(msg->polygon.points[9].z * 180.0 / 3.141592, 'f', 5));

        // Upper orient
        ui_.label_96->setText(QString::number(msg->polygon.points[10].x * 180.0 / 3.141592, 'f', 5));
        ui_.label_97->setText(QString::number(msg->polygon.points[10].y * 180.0 / 3.141592, 'f', 5));
        ui_.label_98->setText(QString::number(msg->polygon.points[10].z * 180.0 / 3.141592, 'f', 5));

        // ft display

        ui_.ft0->setText(QString::number(msg->polygon.points[15].x, 'f', 5));
        ui_.ft1->setText(QString::number(msg->polygon.points[15].y, 'f', 5));
        ui_.ft2->setText(QString::number(msg->polygon.points[15].z, 'f', 5));
        ui_.ft3->setText(QString::number(msg->polygon.points[16].x, 'f', 5));
        ui_.ft4->setText(QString::number(msg->polygon.points[16].y, 'f', 5));
        ui_.ft5->setText(QString::number(msg->polygon.points[16].z, 'f', 5));

        ui_.ft6->setText(QString::number(msg->polygon.points[17].x, 'f', 5));
        ui_.ft7->setText(QString::number(msg->polygon.points[17].y, 'f', 5));
        ui_.ft8->setText(QString::number(msg->polygon.points[17].z, 'f', 5));
        ui_.ft9->setText(QString::number(msg->polygon.points[18].x, 'f', 5));
        ui_.ft10->setText(QString::number(msg->polygon.points[18].y, 'f', 5));
        ui_.ft11->setText(QString::number(msg->polygon.points[18].z, 'f', 5));

        double com_x = msg->polygon.points[0].x;
        double com_y = msg->polygon.points[0].y;

        double left_x = msg->polygon.points[2].x;
        double left_y = msg->polygon.points[2].y;

        double right_x = msg->polygon.points[1].x;
        double right_y = msg->polygon.points[1].y;

        double a, b, c;

        a = (right_y - left_y) / (right_x - left_x);
        b = -1;
        c = -(a * left_x + b * left_y);

        double dis = ((a * com_x + b * com_y + c)) / sqrt(a * a + b * b);

        // com distance from both foot

        dis = msg->polygon.points[0].z;
        ui_.label_3->setText(QString::number(dis, 'f', 5));

        com_x = msg->polygon.points[12].x;
        com_y = msg->polygon.points[12].y;

        a = (right_y - left_y) / (right_x - left_x);
        b = -1;
        c = -(a * left_x + b * left_y);

        dis = ((a * com_x + b * com_y + c)) / sqrt(a * a + b * b);

        ui_.label_42->setText(QString::number(dis, 'f', 5));

        // ui_.graphicsView->setSceneRect(0, 0, 0, 0);
    }

    void DyrosBoltGui::sendtunebtn()
    {
        for (int i = 0; i < 33; i++)
        {
            gain_msg.data[i] = ecattexts[elng[i]]->text().toFloat();
        }
        gain_pub.publish(gain_msg);
    }

    void DyrosBoltGui::resettunebtn()
    {
        for (int i = 0; i < 33; i++)
        {
            ecattexts[elng[i]]->setText(QString::number(NM2CNT[i], 'f', 3));
        }
    }

    void DyrosBoltGui::handletaskmsg()
    {
        task_msg.pelv_pitch = ui_.pelv_pitch->text().toFloat();
        task_msg.roll = ui_.com_roll->text().toFloat();
        task_msg.pitch = ui_.com_pitch->text().toFloat();
        task_msg.yaw = ui_.com_yaw->text().toFloat();
        task_msg.ratio = ui_.com_pos->text().toFloat();
        task_msg.height = ui_.com_height->text().toFloat();

        task_msg.left_foot = ui_.cb_lf->isChecked();
        task_msg.right_foot = ui_.cb_rf->isChecked();

        task_msg.l_x = ui_.text_l_x->text().toFloat();
        task_msg.l_y = ui_.text_l_y->text().toFloat();
        task_msg.l_z = ui_.text_l_z->text().toFloat();
        task_msg.l_roll = ui_.text_l_roll->text().toFloat();
        task_msg.l_pitch = ui_.text_l_pitch->text().toFloat();
        task_msg.l_yaw = ui_.text_l_yaw->text().toFloat();

        task_msg.r_x = ui_.text_r_x->text().toFloat();
        task_msg.r_y = ui_.text_r_y->text().toFloat();
        task_msg.r_z = ui_.text_r_z->text().toFloat();
        task_msg.r_roll = ui_.text_r_roll->text().toFloat();
        task_msg.r_pitch = ui_.text_r_pitch->text().toFloat();
        task_msg.r_yaw = ui_.text_r_yaw->text().toFloat();

        task_msg.time = ui_.text_traj_time->text().toFloat();
        task_msg.mode = ui_.task_mode->currentIndex();

        task_msg.customTaskGain = ui_.customtaskgain->isChecked();

        task_msg.solver = ui_.solver_mode->currentIndex();

        task_msg.contactredis = ui_.cr_mode->currentIndex();

        task_msg.acc_p = ui_.accgain->text().toFloat();

        task_msg.maintain_lc = ui_.checkBox->isChecked();

        if (task_msg.customTaskGain)
        {
            task_msg.pos_p = ui_.pospgain->text().toFloat();
            task_msg.pos_d = ui_.posdgain->text().toFloat();
            task_msg.ang_p = ui_.angpgain->text().toFloat();
            task_msg.ang_d = ui_.angdgain->text().toFloat();
        }

        task_msg.x = ui_.text_walking_x->text().toFloat();
        task_msg.y = ui_.text_walking_y->text().toFloat();
        task_msg.z = ui_.text_walking_z->text().toFloat();
        task_msg.walking_height = ui_.text_walking_height->text().toFloat();
        task_msg.theta = ui_.text_walking_theta->text().toFloat();
        task_msg.step_length_x = ui_.text_walking_steplengthx->text().toFloat();
        task_msg.step_length_y = ui_.text_walking_steplengthy->text().toFloat();
    }

    void DyrosBoltGui::que_addquebtn()
    {
        task_que tq_temp;

        handletaskmsg();
        tq_temp.tc_ = task_msg;
        // ui_.text_que->text().toStdString();

        std::stringstream ss;
        ss << "task " << tq_.size() + 1 << " : ";

        ui_.que_listwidget->addItem(ss.str().c_str() + ui_.text_que->text());

        tq_.push_back(tq_temp);

        // list.append(ui_.text_que->text());

        // ui_.que_list->set
    }

    void DyrosBoltGui::tasksendcb()
    {
        handletaskmsg();

        task_pub.publish(task_msg);

        if ((ui_.task_mode->currentIndex() == 2) || (ui_.task_mode->currentIndex() == 3))
        {
        }
        else
        {

            ui_.text_l_x->setText(QString::number(0.0, 'f', 1));
            ui_.text_l_y->setText(QString::number(0.0, 'f', 1));
            ui_.text_l_z->setText(QString::number(0.0, 'f', 1));
            ui_.text_l_roll->setText(QString::number(0.0, 'f', 1));
            ui_.text_l_pitch->setText(QString::number(0.0, 'f', 1));
            ui_.text_l_yaw->setText(QString::number(0.0, 'f', 1));

            ui_.text_r_x->setText(QString::number(0.0, 'f', 1));
            ui_.text_r_y->setText(QString::number(0.0, 'f', 1));
            ui_.text_r_z->setText(QString::number(0.0, 'f', 1));
            ui_.text_r_roll->setText(QString::number(0.0, 'f', 1));
            ui_.text_r_pitch->setText(QString::number(0.0, 'f', 1));
            ui_.text_r_yaw->setText(QString::number(0.0, 'f', 1));
        }
    }

    void DyrosBoltGui::imucb(const sensor_msgs::ImuConstPtr &msg)
    { /*
    //std::cout<<robot_time<<"msg->linacc"<<msg->linear_acceleration.x<<std::endl;
    line_roll->append(robot_time, msg->linear_acceleration.x);
    chart_roll->axisY()->setRange(-5, 5);
    chart_roll->axisX()->setRange(robot_time - 5, robot_time);
    if (line_roll->at(0).x() < (robot_time - 5))
        line_roll->remove(0);

    line_pitch->append(robot_time, msg->linear_acceleration.y);
    chart_pitch->axisY()->setRange(-5, 5);
    chart_pitch->axisX()->setRange(robot_time - 5, robot_time);
    if (line_pitch->at(0).x() < (robot_time - 5))
        line_pitch->remove(0);

    line_yaw->append(robot_time, msg->linear_acceleration.z);
    chart_yaw->axisY()->setRange(-5, 5);
    chart_yaw->axisX()->setRange(robot_time - 5, robot_time);
    if (line_yaw->at(0).x() < (robot_time - 5))
        line_yaw->remove(0);*/
    }

    void DyrosBoltGui::walkinginitbtncb()
    {
        task_msg.walking_enable = 3.0;
        task_msg.ik_mode = ui_.ik_mode->currentIndex();

        if (ui_.walking_pattern->currentIndex() == 0)
        {
            task_msg.pattern = 0;
        }
        else if (ui_.walking_pattern->currentIndex() == 1)
        {
            task_msg.pattern = 1;
        }
        else
        {
            task_msg.pattern = 2;
        }

        if (ui_.controlmode->currentIndex() == 0)
        {
            task_msg.comcontrol = 0;
        }
        else if (ui_.controlmode->currentIndex() == 1)
        {
            task_msg.comcontrol = 1;
        }
        else
        {
            task_msg.comcontrol = 2;
        }

        if (ui_.checkBox_dob->isChecked() == true)
        {
            task_msg.dob = true;
        }
        else
        {
            task_msg.dob = false;
        }

        if (ui_.checkBox_IMU->isChecked() == true)
        {
            task_msg.imu = true;
        }
        else
        {
            task_msg.imu = false;
        }

        if (ui_.checkBox_mom->isChecked() == true)
        {
            task_msg.mom = true;
        }
        else
        {
            task_msg.mom = false;
        }

        task_msg.first_foot_step = ui_.step_mode->currentIndex();

        task_msg.x = ui_.text_walking_x->text().toFloat();
        task_msg.y = ui_.text_walking_y->text().toFloat();
        task_msg.z = ui_.text_walking_z->text().toFloat();
        task_msg.walking_height = ui_.text_walking_height->text().toFloat();
        task_msg.theta = ui_.text_walking_theta->text().toFloat();
        task_msg.step_length_x = ui_.text_walking_steplengthx->text().toFloat();
        task_msg.step_length_y = ui_.text_walking_steplengthy->text().toFloat();

        task_pub.publish(task_msg);
    }

    void DyrosBoltGui::walkingstartbtncb()
    {
        task_msg.walking_enable = 1.0;
        task_msg.ik_mode = ui_.ik_mode->currentIndex();

        if (ui_.walking_pattern->currentIndex() == 0)
        {
            task_msg.pattern = 0;
        }
        else if (ui_.walking_pattern->currentIndex() == 1)
        {
            task_msg.pattern = 1;
        }
        else
        {
            task_msg.pattern = 2;
        }

        if (ui_.controlmode->currentIndex() == 0)
        {
            task_msg.comcontrol = 0;
        }
        else
        {
            task_msg.comcontrol = 1;
        }

        if (ui_.walking_pattern_2->currentIndex() == 0)
        {
            task_msg.pattern2 = 0;
        }
        else if (ui_.walking_pattern_2->currentIndex() == 1)
        {
            task_msg.pattern2 = 1;
        }

        if (ui_.checkBox_dob->isChecked() == true)
        {
            task_msg.dob = true;
        }
        else
        {
            task_msg.dob = false;
        }

        if (ui_.checkBox_IMU->isChecked() == true)
        {
            task_msg.imu = true;
        }
        else
        {
            task_msg.imu = false;
        }

        if (ui_.checkBox_mom->isChecked() == true)
        {
            task_msg.mom = true;
        }
        else
        {
            task_msg.mom = false;
        }

        task_msg.first_foot_step = ui_.step_mode->currentIndex();

        task_msg.x = ui_.text_walking_x->text().toFloat();
        task_msg.y = ui_.text_walking_y->text().toFloat();
        task_msg.z = ui_.text_walking_z->text().toFloat();
        task_msg.walking_height = ui_.text_walking_height->text().toFloat();
        task_msg.theta = ui_.text_walking_theta->text().toFloat();
        task_msg.step_length_x = ui_.text_walking_steplengthx->text().toFloat();
        task_msg.step_length_y = ui_.text_walking_steplengthy->text().toFloat();

        task_pub.publish(task_msg);
    }

    /*
void DyrosBoltGui::wheelEvent(QWheelEvent *event)
{
    std::cout << "wheel event" << std::endl;
}*/

    void DyrosBoltGui::sendtaskgaincommand()
    {
        taskgain_msg.mode = ui_.taskgain_combo->currentIndex() + 1;

        taskgain_msg.pgain[0] = ui_.gainx->text().toFloat();
        taskgain_msg.pgain[1] = ui_.gainy->text().toFloat();
        taskgain_msg.pgain[2] = ui_.gainz->text().toFloat();
        taskgain_msg.pgain[3] = ui_.gainr->text().toFloat();
        taskgain_msg.pgain[4] = ui_.gainp->text().toFloat();
        taskgain_msg.pgain[5] = ui_.gainyaw->text().toFloat();

        taskgain_msg.dgain[0] = ui_.gainx_2->text().toFloat();
        taskgain_msg.dgain[1] = ui_.gainy_2->text().toFloat();
        taskgain_msg.dgain[2] = ui_.gainz_2->text().toFloat();
        taskgain_msg.dgain[3] = ui_.gainr_2->text().toFloat();
        taskgain_msg.dgain[4] = ui_.gainp_2->text().toFloat();
        taskgain_msg.dgain[5] = ui_.gainyaw_2->text().toFloat();

        taskgain_pub.publish(taskgain_msg);
    }
    void DyrosBoltGui::resettaskgaincommand()
    {
        taskgain_msg.mode = 0;

        for (int i = 0; i < 6; i++)
        {
            taskgain_msg.pgain[i] = 0.0;
            taskgain_msg.dgain[i] = 0.0;
        }

        taskgain_pub.publish(taskgain_msg);
    }

    void DyrosBoltGui::getCurrentPos()
    {
        for (int i = 0; i < 33; i++)
            ecattexts[i]->setText(QString::number(q_[elng2[i]], 'f', 3));
    }

    void DyrosBoltGui::positionCommand()
    {
        poscom_msg.traj_time = ui_.poscom_trajtime->text().toFloat();
        for (int i = 0; i < 33; i++)
        {
            poscom_msg.position[elng2[i]] = ecattexts[i]->text().toFloat();
        }
        poscom_msg.gravity = ui_.poscomgravity->isChecked();
        poscom_msg.relative = ui_.poscomrelative->isChecked();
        poscom_pub.publish(poscom_msg);
    }

    void DyrosBoltGui::positionPreset1()
    {
        for (int i = 0; i < 33; i++)
        {
            ecattexts[i]->setText(QString::number(posStandard[elng2[i]], 'f', 3));
        }
    }

    void DyrosBoltGui::positionPreset2()
    {
        for (int i = 0; i < 33; i++)
        {
            ecattexts[i]->setText(QString::number(posStandard2[elng2[i]], 'f', 3));
        }
    }

    void DyrosBoltGui::positionPreset3()
    {
        for (int i = 0; i < 33; i++)
        {
            ecattexts[i]->setText(QString::number(posStandard3[elng2[i]], 'f', 3));
        }
    }

    void DyrosBoltGui::positionPreset4()
    {
        for (int i = 0; i < 33; i++)
        {
            ecattexts[i]->setText(QString::number(posStandard4[elng2[i]], 'f', 3));
        }
    }

    void DyrosBoltGui::positionRelative(int index)
    {
        if (ui_.poscomrelative->isChecked())
        {
            for (int i = 0; i < 33; i++)
            {
                ecattexts[i]->setText(QString::number(0));
            }
        }
        else
        {
        }
    }

    void DyrosBoltGui::armGainSend()
    {
        arm_gain_msg.data[0] = ui_.arm_P_1->text().toFloat();
        arm_gain_msg.data[1] = ui_.arm_P_2->text().toFloat();
        arm_gain_msg.data[2] = ui_.arm_P_3->text().toFloat();
        arm_gain_msg.data[3] = ui_.arm_P_4->text().toFloat();
        arm_gain_msg.data[4] = ui_.arm_P_5->text().toFloat();
        arm_gain_msg.data[5] = ui_.arm_P_6->text().toFloat();
        arm_gain_msg.data[6] = ui_.arm_P_7->text().toFloat();
        arm_gain_msg.data[7] = ui_.arm_P_8->text().toFloat();

        arm_gain_msg.data[8] = ui_.arm_D_1->text().toFloat();
        arm_gain_msg.data[9] = ui_.arm_D_2->text().toFloat();
        arm_gain_msg.data[10] = ui_.arm_D_3->text().toFloat();
        arm_gain_msg.data[11] = ui_.arm_D_4->text().toFloat();
        arm_gain_msg.data[12] = ui_.arm_D_5->text().toFloat();
        arm_gain_msg.data[13] = ui_.arm_D_6->text().toFloat();
        arm_gain_msg.data[14] = ui_.arm_D_7->text().toFloat();
        arm_gain_msg.data[15] = ui_.arm_D_8->text().toFloat();

        arm_gain_pub.publish(arm_gain_msg);
    }
    // dg
    //  void DyrosBoltGui::walkingspeedcb(int value)
    //  {
    //      double max_speed = 0.6;
    //      double min_speed = -0.4;
    //      double scale = value;

    //     walkingspeed_msg.data = scale / 100 * (max_speed - min_speed) + min_speed;
    //     walkingspeed_pub.publish(walkingspeed_msg);
    // }

    // void DyrosBoltGui::walkingdurationcb(int value)
    // {
    //     double max_duration = 1;
    //     double min_duration = 0.2;
    //     double scale = value;

    //     walkingduration_msg.data = scale / 100 * (max_duration - min_duration) + min_duration;
    //     walkingduration_pub.publish(walkingduration_msg);
    // }

    // void DyrosBoltGui::walkingangvelcb(int value)
    // {
    //     double max_angvel = 1;
    //     double min_angvel = -1;
    //     double scale = value;

    //     walkingangvel_msg.data = scale / 100 * (max_angvel - min_angvel) + min_angvel;
    //     walkingangvel_pub.publish(walkingangvel_msg);
    // }

    // void DyrosBoltGui::kneetargetanglecb(int value)
    // {
    //     double max_knee = M_PI / 2;
    //     double min_knee = 0;
    //     double scale = value;

    //     // kneetargetangle_msg.data = scale/100*(max_knee - min_knee) + min_knee;
    //     kneetargetangle_msg.data = scale / 180 * M_PI;
    //     kneetargetangle_pub.publish(kneetargetangle_msg);
    // }

    // void DyrosBoltGui::footheightcb(int value)
    // {
    //     double max_footz = 0.1;
    //     double min_footz = 0.005;
    //     double scale = value;

    //     // footheight_msg.data = scale/100*(max_footz - min_footz) + min_footz;
    //     footheight_msg.data = scale / 100;
    //     footheight_pub.publish(footheight_msg);
    // }

    // avatar
    // JHRui edit
    void DyrosBoltGui::sendupperbodymodecb_1()
    {
        upperbodymode_msg.data = 1;
        upperbodymode_pub.publish(upperbodymode_msg);
        // std::cout << upperbodymode_msg.data << std::endl;
    }
    void DyrosBoltGui::sendupperbodymodecb_2()
    {
        upperbodymode_msg.data = 2;
        upperbodymode_pub.publish(upperbodymode_msg);
        // std::cout << upperbodymode_msg.data << std::endl;
    }
    void DyrosBoltGui::sendupperbodymodecb_3()
    {
        upperbodymode_msg.data = 3;
        upperbodymode_pub.publish(upperbodymode_msg);
        // std::cout << upperbodymode_msg.data << std::endl;
    }
    void DyrosBoltGui::sendupperbodymodecb_4()
    {
        upperbodymode_msg.data = 4;
        upperbodymode_pub.publish(upperbodymode_msg);
        // std::cout << upperbodymode_msg.data << std::endl;
    }
    void DyrosBoltGui::sendupperbodymodecb_5()
    {
        upperbodymode_msg.data = 5;
        upperbodymode_pub.publish(upperbodymode_msg);
        // std::cout << upperbodymode_msg.data << std::endl;
    }
    void DyrosBoltGui::sendupperbodymodecb_6()
    {
        upperbodymode_msg.data = 6;
        upperbodymode_pub.publish(upperbodymode_msg);
        // std::cout << upperbodymode_msg.data << std::endl;
    }
    void DyrosBoltGui::sendupperbodymodecb_7()
    {
        upperbodymode_msg.data = 7;
        upperbodymode_pub.publish(upperbodymode_msg);
        // std::cout << upperbodymode_msg.data << std::endl;
    }
    void DyrosBoltGui::sendupperbodymodecb_8()
    {
        upperbodymode_msg.data = 8;
        upperbodymode_pub.publish(upperbodymode_msg);
        // std::cout << upperbodymode_msg.data << std::endl;
    }
    void DyrosBoltGui::sendupperbodymodecb_9()
    {
        upperbodymode_msg.data = 9;
        upperbodymode_pub.publish(upperbodymode_msg);
        // std::cout << upperbodymode_msg.data << std::endl;
    }
    // JHRui edit


    void DyrosBoltGui::sendupperbodymodecb()
    {
        upperbodymode_msg.data = ui_.upperbody_mode->currentIndex() + 1;
        upperbodymode_pub.publish(upperbodymode_msg);
    }

    void DyrosBoltGui::sendstillposecalibration()
    {
        pose_calibration_msg.data = 1;

        pose_calibration_pub.publish(pose_calibration_msg);
    }
    void DyrosBoltGui::sendtposecalibration()
    {
        pose_calibration_msg.data = 2;

        pose_calibration_pub.publish(pose_calibration_msg);
    }
    void DyrosBoltGui::sendforwardposecalibration()
    {
        pose_calibration_msg.data = 3;

        pose_calibration_pub.publish(pose_calibration_msg);
    }
    void DyrosBoltGui::sendresetposecalibration()
    {
        pose_calibration_msg.data = 4;

        pose_calibration_pub.publish(pose_calibration_msg);
    }
    void DyrosBoltGui::sendloadsavedcalibration()
    {
        pose_calibration_msg.data = 5;

        pose_calibration_pub.publish(pose_calibration_msg);
    }

    void DyrosBoltGui::vr_eye_depth_cb(int value)
    {
        double scale = value;

        vr_slider_msg.data[0] = scale / 100;
        vr_slider_pub.publish(vr_slider_msg);
    }

    void DyrosBoltGui::vr_eye_distance_cb(int value)
    {
        double scale = value;

        vr_slider_msg.data[1] = scale / 100;
        vr_slider_pub.publish(vr_slider_msg);
    }
    void DyrosBoltGui::setArmStiffPDGain()
    {
        ui_.arm_P_1->setText(QString::number(armPDgainStiff[0], 'f', 2));
        ui_.arm_P_2->setText(QString::number(armPDgainStiff[1], 'f', 2));
        ui_.arm_P_3->setText(QString::number(armPDgainStiff[2], 'f', 2));
        ui_.arm_P_4->setText(QString::number(armPDgainStiff[3], 'f', 2));
        ui_.arm_P_5->setText(QString::number(armPDgainStiff[4], 'f', 2));
        ui_.arm_P_6->setText(QString::number(armPDgainStiff[5], 'f', 2));
        ui_.arm_P_7->setText(QString::number(armPDgainStiff[6], 'f', 2));
        ui_.arm_P_8->setText(QString::number(armPDgainStiff[7], 'f', 2));

        ui_.arm_D_1->setText(QString::number(armPDgainStiff[8], 'f', 2));
        ui_.arm_D_2->setText(QString::number(armPDgainStiff[9], 'f', 2));
        ui_.arm_D_3->setText(QString::number(armPDgainStiff[10], 'f', 2));
        ui_.arm_D_4->setText(QString::number(armPDgainStiff[11], 'f', 2));
        ui_.arm_D_5->setText(QString::number(armPDgainStiff[12], 'f', 2));
        ui_.arm_D_6->setText(QString::number(armPDgainStiff[13], 'f', 2));
        ui_.arm_D_7->setText(QString::number(armPDgainStiff[14], 'f', 2));
        ui_.arm_D_8->setText(QString::number(armPDgainStiff[15], 'f', 2));
    }
    void DyrosBoltGui::setArmSoftPDGain()
    {
        ui_.arm_P_1->setText(QString::number(armPDgainSoft[0], 'f', 2));
        ui_.arm_P_2->setText(QString::number(armPDgainSoft[1], 'f', 2));
        ui_.arm_P_3->setText(QString::number(armPDgainSoft[2], 'f', 2));
        ui_.arm_P_4->setText(QString::number(armPDgainSoft[3], 'f', 2));
        ui_.arm_P_5->setText(QString::number(armPDgainSoft[4], 'f', 2));
        ui_.arm_P_6->setText(QString::number(armPDgainSoft[5], 'f', 2));
        ui_.arm_P_7->setText(QString::number(armPDgainSoft[6], 'f', 2));
        ui_.arm_P_8->setText(QString::number(armPDgainSoft[7], 'f', 2));

        ui_.arm_D_1->setText(QString::number(armPDgainSoft[8], 'f', 2));
        ui_.arm_D_2->setText(QString::number(armPDgainSoft[9], 'f', 2));
        ui_.arm_D_3->setText(QString::number(armPDgainSoft[10], 'f', 2));
        ui_.arm_D_4->setText(QString::number(armPDgainSoft[11], 'f', 2));
        ui_.arm_D_5->setText(QString::number(armPDgainSoft[12], 'f', 2));
        ui_.arm_D_6->setText(QString::number(armPDgainSoft[13], 'f', 2));
        ui_.arm_D_7->setText(QString::number(armPDgainSoft[14], 'f', 2));
        ui_.arm_D_8->setText(QString::number(armPDgainSoft[15], 'f', 2));
    }

} // namespace dyros_bolt_gui

PLUGINLIB_EXPORT_CLASS(dyros_bolt_gui::DyrosBoltGui, rqt_gui_cpp::Plugin)
