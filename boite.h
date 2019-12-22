#ifndef BOITE_H
#define BOITE_H

#include "btBulletDynamicsCommon.h"

#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <QVector3D>
#include <stdio.h>

class Boite
{
public:
    Boite();
    Boite(QVector3D position, float longueur, float largeur, float hauteur, int id);

    QVector3D position;
    float longueur;
    float largeur;
    float profondeur;
    int id;

    void affiche(QOpenGLShaderProgram *sp, int start);
    void anime(btDiscreteDynamicsWorld* dynamicsWorld);

    QMatrix4x4 modelMatrix;
};

#endif // BOITE_H
