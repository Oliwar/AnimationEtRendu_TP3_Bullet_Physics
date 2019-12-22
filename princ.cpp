// Basé sur :
// CC-BY Edouard.Thiel@univ-amu.fr - 22/01/2019

#include "princ.h"
#include <QColorDialog>
#include <QDebug>

Princ::Princ(QWidget *parent) : QMainWindow(parent)
{
    setupUi(this);
}

void Princ::on_pushButton_reset_clicked()
{
    glarea->destroy_simulation();
    glarea->init_simulation();
}
