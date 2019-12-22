// Basé sur :
// CC-BY Edouard.Thiel@univ-amu.fr - 22/01/2019

#include "princ.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    /* initialize random seed: */
    srand(static_cast<unsigned>(time(nullptr)));
    QApplication a(argc, argv);
    Princ w;
    w.show();

    return a.exec();
}
