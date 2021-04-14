#include "controlgui.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ControlGUI w;
    w.init();
    w.show();

    return a.exec();
}
