#include "controlgui.h"
#include "ui_controlgui.h"
#include <QString>
#include <QTimer>
#include <cstring>
#include <math.h>
#include <stdio.h>
ControlGUI::ControlGUI(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ControlGUI)
{
    ui->setupUi(this);
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(on_Timer())); // connect timeout signal to on_Timer slot
    timer->start(20); // on_Timer is called every 20 ms

    _ecatCmdLcm = new ecat_command_t;
    _ecatDataLcm = new ecat_data_t;
    _ecatADLstatus = new ecat_adl_status_t;
    _ecathardenable = new ecat_hard_enable_t;
}


ControlGUI::~ControlGUI()
{
    std::memset(_ecathardenable,0,sizeof(ecat_hard_enable_t));
    if (!_ecatCmdLcm)
    {
        delete _ecatCmdLcm;
        _ecatCmdLcm = nullptr;
    }
    if (!_ecatDataLcm)
    {
        delete _ecatDataLcm;
        _ecatDataLcm = nullptr;
    }
    if (!_ecatADLstatus)
    {
        delete _ecatADLstatus;
        _ecatADLstatus = nullptr;
    }
    if (!_ecathardenable)
    {
        delete _ecathardenable;
        _ecathardenable = nullptr;
    }
    if (!timer)
    {
        delete timer;
        timer = nullptr;
    }
    delete ui;
}

void ControlGUI::init()
{

    /* Initialize lcm */
    if (!lcm.good())
    {
        printf("Failed to intialize lcm1 \n");
        return;
    }

    /* Subcsribe to channel "ecat_data" */
    lcm.subscribe("ecat_data", &ControlGUI::ecatData_handler, this);

    /* Subcsribe to channel "ecat_adl_stauts" */
    lcm.subscribe("ecat_adl_status", &ControlGUI::ecatADLstatus_handler, this);

    /* Create thread to run lcm_handle*()*/
    lcm_thread = std::thread(&ControlGUI::handleLCM, this);

}

void ControlGUI::ecatData_handler(const lcm::ReceiveBuffer *rbuf, const std::string& chan, const ecat_data_t *msg)
{
    (void)rbuf;
    (void)chan;
    std::memcpy(_ecatDataLcm,msg,sizeof(ecat_data_t));
}

void ControlGUI::ecatADLstatus_handler(const lcm::ReceiveBuffer *rbuf, const std::string& chan, const ecat_adl_status_t *msg)
{
    (void)rbuf;
    (void)chan;
    std::memcpy(_ecatADLstatus, msg, sizeof(ecat_adl_status_t));
}

void ControlGUI::handleLCM()
{
    while(1)
    {
        if(0!=lcm.handle()) // success 0. wrong -1
        {
            printf("Error receiving lcm data \n");
            break;
        }
    }
    std::terminate();

}

void ControlGUI::ZeroEcatdata()
{
    /* Initializes _ecatCmdLcm to zeros */
    std::memset(_ecatCmdLcm,0,sizeof(ecat_command_t));

    /* Initializes _ecathardenable to zeros */
    std::memset(_ecathardenable,0,sizeof(ecat_hard_enable_t));

    /* Initializes _ecatADLstatus to zeros */
    std::memset(_ecatADLstatus,0,sizeof(ecat_adl_status_t));

    /* Initializes _ecatDataLcm to zeros */
    std::memset(_ecatDataLcm,0,sizeof(ecat_data_t));
}


void ControlGUI::updateOutput()
{
    ui->OU_ESTOP->setText((QString::number((static_cast<double>(_ecatADLstatus->estop_status)))));

    ui->OU_ECAT_FR->setText((QString::number(static_cast<uint8_t>(_ecatADLstatus->ecat_status[0]))));
    ui->OU_ECAT_FL->setText((QString::number(static_cast<uint8_t>(_ecatADLstatus->ecat_status[1]))));
    ui->OU_ECAT_HR->setText((QString::number(static_cast<uint8_t>(_ecatADLstatus->ecat_status[2]))));
    ui->OU_ECAT_HL->setText((QString::number(static_cast<uint8_t>(_ecatADLstatus->ecat_status[3]))));


    ui->OU_x_FR->setText((QString::number(static_cast<double>(_ecatDataLcm->x[0]))));
    ui->OU_x_FL->setText((QString::number(static_cast<double>(_ecatDataLcm->x[1]))));
    ui->OU_x_HR->setText((QString::number(static_cast<double>(_ecatDataLcm->x[2]))));
    ui->OU_x_HL->setText((QString::number(static_cast<double>(_ecatDataLcm->x[3]))));

    ui->OU_y_FR->setText((QString::number(static_cast<double>(_ecatDataLcm->y[0]))));
    ui->OU_y_FL->setText((QString::number(static_cast<double>(_ecatDataLcm->y[1]))));
    ui->OU_y_HR->setText((QString::number(static_cast<double>(_ecatDataLcm->y[2]))));
    ui->OU_y_HL->setText((QString::number(static_cast<double>(_ecatDataLcm->y[3]))));

    ui->OU_z_FR->setText((QString::number(static_cast<double>(_ecatDataLcm->z[0]))));
    ui->OU_z_FL->setText((QString::number(static_cast<double>(_ecatDataLcm->z[1]))));
    ui->OU_z_HR->setText((QString::number(static_cast<double>(_ecatDataLcm->z[2]))));
    ui->OU_z_HL->setText((QString::number(static_cast<double>(_ecatDataLcm->z[3]))));

    ui->OU_dx_FR->setText((QString::number(static_cast<double>(_ecatDataLcm->dx[0]))));
    ui->OU_dx_FL->setText((QString::number(static_cast<double>(_ecatDataLcm->dx[1]))));
    ui->OU_dx_HR->setText((QString::number(static_cast<double>(_ecatDataLcm->dx[2]))));
    ui->OU_dx_HL->setText((QString::number(static_cast<double>(_ecatDataLcm->dx[3]))));

    ui->OU_dy_FR->setText((QString::number(static_cast<double>(_ecatDataLcm->dy[0]))));
    ui->OU_dy_FL->setText((QString::number(static_cast<double>(_ecatDataLcm->dy[1]))));
    ui->OU_dy_HR->setText((QString::number(static_cast<double>(_ecatDataLcm->dy[2]))));
    ui->OU_dy_HL->setText((QString::number(static_cast<double>(_ecatDataLcm->dy[3]))));

    ui->OU_dz_FR->setText((QString::number(static_cast<double>(_ecatDataLcm->dz[0]))));
    ui->OU_dz_FL->setText((QString::number(static_cast<double>(_ecatDataLcm->dz[1]))));
    ui->OU_dz_HR->setText((QString::number(static_cast<double>(_ecatDataLcm->dz[2]))));
    ui->OU_dz_HL->setText((QString::number(static_cast<double>(_ecatDataLcm->dz[3]))));

    ui->OU_qa_FR->setText((QString::number(static_cast<double>(_ecatDataLcm->q_abad[0]))));
    ui->OU_qa_FL->setText((QString::number(static_cast<double>(_ecatDataLcm->q_abad[1]))));
    ui->OU_qa_HR->setText((QString::number(static_cast<double>(_ecatDataLcm->q_abad[2]))));
    ui->OU_qa_HL->setText((QString::number(static_cast<double>(_ecatDataLcm->q_abad[3]))));

    ui->OU_qh_FR->setText((QString::number(static_cast<double>(_ecatDataLcm->q_hip[0]))));
    ui->OU_qh_FL->setText((QString::number(static_cast<double>(_ecatDataLcm->q_hip[1]))));
    ui->OU_qh_HR->setText((QString::number(static_cast<double>(_ecatDataLcm->q_hip[2]))));
    ui->OU_qh_HL->setText((QString::number(static_cast<double>(_ecatDataLcm->q_hip[3]))));

    ui->OU_qk_FR->setText((QString::number(static_cast<double>(_ecatDataLcm->q_knee[0]))));
    ui->OU_qk_FL->setText((QString::number(static_cast<double>(_ecatDataLcm->q_knee[1]))));
    ui->OU_qk_HR->setText((QString::number(static_cast<double>(_ecatDataLcm->q_knee[2]))));
    ui->OU_qk_HL->setText((QString::number(static_cast<double>(_ecatDataLcm->q_knee[3]))));

    ui->OU_dqa_FR->setText((QString::number(static_cast<double>(_ecatDataLcm->dq_abad[0]))));
    ui->OU_dqa_FL->setText((QString::number(static_cast<double>(_ecatDataLcm->dq_abad[1]))));
    ui->OU_dqa_HR->setText((QString::number(static_cast<double>(_ecatDataLcm->dq_abad[2]))));
    ui->OU_dqa_HL->setText((QString::number(static_cast<double>(_ecatDataLcm->dq_abad[3]))));

    ui->OU_dqh_FR->setText((QString::number(static_cast<double>(_ecatDataLcm->dq_hip[0]))));
    ui->OU_dqh_FL->setText((QString::number(static_cast<double>(_ecatDataLcm->dq_hip[1]))));
    ui->OU_dqh_HR->setText((QString::number(static_cast<double>(_ecatDataLcm->dq_hip[2]))));
    ui->OU_dqh_HL->setText((QString::number(static_cast<double>(_ecatDataLcm->dq_hip[3]))));

    ui->OU_dqk_FR->setText((QString::number(static_cast<double>(_ecatDataLcm->dq_knee[0]))));
    ui->OU_dqk_FL->setText((QString::number(static_cast<double>(_ecatDataLcm->dq_knee[1]))));
    ui->OU_dqk_HR->setText((QString::number(static_cast<double>(_ecatDataLcm->dq_knee[2]))));
    ui->OU_dqk_HL->setText((QString::number(static_cast<double>(_ecatDataLcm->dq_knee[3]))));

    ui->OU_taua_FR->setText((QString::number(static_cast<double>(_ecatDataLcm->tau_abad[0]))));
    ui->OU_taua_FL->setText((QString::number(static_cast<double>(_ecatDataLcm->tau_abad[1]))));
    ui->OU_taua_HR->setText((QString::number(static_cast<double>(_ecatDataLcm->tau_abad[2]))));
    ui->OU_taua_HL->setText((QString::number(static_cast<double>(_ecatDataLcm->tau_abad[3]))));

    ui->OU_tauh_FR->setText((QString::number(static_cast<double>(_ecatDataLcm->tau_hip[0]))));
    ui->OU_tauh_FL->setText((QString::number(static_cast<double>(_ecatDataLcm->tau_hip[1]))));
    ui->OU_tauh_HR->setText((QString::number(static_cast<double>(_ecatDataLcm->tau_hip[2]))));
    ui->OU_tauh_HL->setText((QString::number(static_cast<double>(_ecatDataLcm->tau_hip[3]))));

    ui->OU_tauk_FR->setText((QString::number(static_cast<double>(_ecatDataLcm->tau_knee[0]))));
    ui->OU_tauk_FL->setText((QString::number(static_cast<double>(_ecatDataLcm->tau_knee[1]))));
    ui->OU_tauk_HR->setText((QString::number(static_cast<double>(_ecatDataLcm->tau_knee[2]))));
    ui->OU_tauk_HL->setText((QString::number(static_cast<double>(_ecatDataLcm->tau_knee[3]))));

    ui->OU_fx_FR->setText((QString::number(static_cast<double>(_ecatDataLcm->fx[0]))));
    ui->OU_fx_FL->setText((QString::number(static_cast<double>(_ecatDataLcm->fx[1]))));
    ui->OU_fx_HR->setText((QString::number(static_cast<double>(_ecatDataLcm->fx[2]))));
    ui->OU_fx_HL->setText((QString::number(static_cast<double>(_ecatDataLcm->fx[3]))));

    ui->OU_fy_FR->setText((QString::number(static_cast<double>(_ecatDataLcm->fy[0]))));
    ui->OU_fy_FL->setText((QString::number(static_cast<double>(_ecatDataLcm->fy[1]))));
    ui->OU_fy_HR->setText((QString::number(static_cast<double>(_ecatDataLcm->fy[2]))));
    ui->OU_fy_HL->setText((QString::number(static_cast<double>(_ecatDataLcm->fy[3]))));

    ui->OU_fz_FR->setText((QString::number(static_cast<double>(_ecatDataLcm->fz[0]))));
    ui->OU_fz_FL->setText((QString::number(static_cast<double>(_ecatDataLcm->fz[1]))));
    ui->OU_fz_HR->setText((QString::number(static_cast<double>(_ecatDataLcm->fz[2]))));
    ui->OU_fz_HL->setText((QString::number(static_cast<double>(_ecatDataLcm->fz[3]))));

}

void ControlGUI::on_Timer()
{
    updateOutput();
    if(_ecatADLstatus->estop_status) // if ESTOP
    {
        ui->FR_HE->setChecked(false);
        ui->FL_HE->setChecked(false);
        ui->HR_HE->setChecked(false);
        ui->HL_HE->setChecked(false);
    }
    _ecathardenable->hard_enable[0] = ui->FR_HE->isChecked();
    _ecathardenable->hard_enable[1] = ui->FL_HE->isChecked();
    _ecathardenable->hard_enable[2] = ui->HR_HE->isChecked();
    _ecathardenable->hard_enable[3] = ui->HL_HE->isChecked();
    _ecathardenable->control_mode = static_cast<int8_t>(ui->Control_mode->text().toInt());
    _ecathardenable->h_max = ui->Stand_height->text().toFloat();    
    lcm.publish("ecat_hard_enable", _ecathardenable);
}

void ControlGUI::on_Send_released()
{
    for (int leg(0); leg<4; leg++)
    {
        _ecatCmdLcm->x_des[leg] = ui->INxDes->text().toFloat()*pow(-1,leg);
        _ecatCmdLcm->y_des[leg] = ui->INyDes->text().toFloat();
        _ecatCmdLcm->z_des[leg] = ui->INzDes->text().toFloat();
        _ecatCmdLcm->dx_des[leg] = ui->INdxDes->text().toFloat()*pow(-1,leg);
        _ecatCmdLcm->dy_des[leg] = ui->INdyDes->text().toFloat();
        _ecatCmdLcm->dz_des[leg] = ui->INdzDes->text().toFloat();
        _ecatCmdLcm->kpx[leg] = ui->INkpX->text().toFloat();
        _ecatCmdLcm->kpy[leg] = ui->INkpY->text().toFloat();
        _ecatCmdLcm->kpz[leg] = ui->INkpZ->text().toFloat();
        _ecatCmdLcm->kdx[leg] = ui->INkdx->text().toFloat();
        _ecatCmdLcm->kdy[leg] = ui->INkdy->text().toFloat();
        _ecatCmdLcm->kdz[leg] = ui->INkdz->text().toFloat();

        _ecatCmdLcm->fx_ff[leg] = ui->INfxff->text().toFloat();
        _ecatCmdLcm->fy_ff[leg] = ui->INfyff->text().toFloat();
        _ecatCmdLcm->fz_ff[leg] = ui->INfzff->text().toFloat();
        _ecatCmdLcm->tau_abad_ff[leg] = ui->INtauAbadff->text().toFloat();
        _ecatCmdLcm->tau_hip_ff[leg] = ui->INtauHipff->text().toFloat();
        _ecatCmdLcm->tau_knee_ff[leg] = ui->INtauKneeff->text().toFloat();
        _ecatCmdLcm->q_des_abad[leg] = ui->INqDesAbad->text().toFloat();
        _ecatCmdLcm->q_des_hip[leg] = ui->INqDesHip->text().toFloat();
        _ecatCmdLcm->q_des_knee[leg] = ui->INqDesKnee->text().toFloat();
        _ecatCmdLcm->qd_des_abad[leg] = ui->INqdDesAbad->text().toFloat();
        _ecatCmdLcm->qd_des_hip[leg] = ui->INqdDesHip->text().toFloat();
        _ecatCmdLcm->qd_des_knee[leg] = ui->INqdDesKnee->text().toFloat();
        _ecatCmdLcm->kp_joint_abad[leg] = ui->INkpAbad->text().toFloat();
        _ecatCmdLcm->kp_joint_hip[leg] = ui->INkpHip->text().toFloat();
        _ecatCmdLcm->kp_joint_knee[leg] = ui->INkpKnee->text().toFloat();
        _ecatCmdLcm->kd_joint_abad[leg] = ui->INkdAbad->text().toFloat();
        _ecatCmdLcm->kd_joint_hip[leg] = ui->INkdHip->text().toFloat();
        _ecatCmdLcm->kd_joint_knee[leg] = ui->INkdKnee->text().toFloat();
    }

    _ecatCmdLcm->zero_joints[0] = ui->IN_ZJ_FR->text().toInt();
    _ecatCmdLcm->zero_joints[1] = ui->IN_ZJ_FL->text().toInt();
    _ecatCmdLcm->zero_joints[2] = ui->IN_ZJ_HR->text().toInt();
    _ecatCmdLcm->zero_joints[3] = ui->IN_ZJ_HL->text().toInt();

    _ecatCmdLcm->enable[0] = ui->IN_SE_FR->text().toInt();
    _ecatCmdLcm->enable[1] = ui->IN_SE_FL->text().toInt();
    _ecatCmdLcm->enable[2] = ui->IN_SE_HR->text().toInt();
    _ecatCmdLcm->enable[3] = ui->IN_SE_HL->text().toInt();


    _ecatCmdLcm->max_torque[0] = ui->IN_TauMax_FR->text().toFloat();
    _ecatCmdLcm->max_torque[1] = ui->IN_TauMax_FL->text().toFloat();
    _ecatCmdLcm->max_torque[2] = ui->IN_TauMax_HR->text().toFloat();
    _ecatCmdLcm->max_torque[3] = ui->IN_TauMax_HL->text().toFloat();


    lcm.publish("ecat_cmd_GUI", _ecatCmdLcm);
}

