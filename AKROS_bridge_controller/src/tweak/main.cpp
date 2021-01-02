#include <QApplication>
#include <QDialog>
#include "tweak.h"

int main(int argc, char** argv){
    QApplication app(argc, argv);
    QWidget* window = new QWidget;
    tweak_controller* dialog = new tweak_controller(window);
    dialog->show();
    return app.exec();
}