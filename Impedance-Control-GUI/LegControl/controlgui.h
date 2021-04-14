#ifndef CONTROLGUI_H
#define CONTROLGUI_H

#include <QWidget>
#include <QString>
#include <QTimer>
#include <thread>
#include "lcm-types/ecat_command_t.hpp"
#include "lcm-types/ecat_data_t.hpp"
#include "lcm-types/ecat_hard_enable_t.hpp"
#include "lcm-types/ecat_adl_status_t.hpp"
#include <lcm/lcm-cpp.hpp>

namespace Ui {
class ControlGUI;
}

class ControlGUI : public QWidget
{
    Q_OBJECT

public:
    explicit ControlGUI(QWidget *parent = nullptr);
    ~ControlGUI();

private slots:

    void on_Timer();    

    void on_Send_released();


private:
    Ui::ControlGUI *ui;

    /* cpp implementation */
    lcm::LCM lcm;

    ecat_command_t *_ecatCmdLcm = nullptr;
    ecat_data_t *_ecatDataLcm = nullptr;
    ecat_adl_status_t *_ecatADLstatus = nullptr;
    ecat_hard_enable_t *_ecathardenable = nullptr;

    std::thread lcm_thread;

    QString tempString;
    QTimer *timer = nullptr;

    void ZeroEcatdata();
    void handleLCM();
    void updateOutput();
    void ecatData_handler(const lcm::ReceiveBuffer *rbuf, const std::string& chan, const ecat_data_t *msg);
    void ecatADLstatus_handler(const lcm::ReceiveBuffer *rbuf, const std::string& chan, const ecat_adl_status_t *msg);

public:
    void init();
};


#endif // CONTROLGUI_H
