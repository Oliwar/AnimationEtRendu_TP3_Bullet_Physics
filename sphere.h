#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "btBulletDynamicsCommon.h"

#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <QVector3D>
#include <stdio.h>

class Sphere
{
public:
    Sphere(float rayon, int id);
    Sphere(QVector3D position, float rayon, int id);

    QVector3D position;
    float rayon;
    int id;

    void affiche(QOpenGLShaderProgram *sp, int start);
    void anime(btDiscreteDynamicsWorld* dynamicsWorld);

    QMatrix4x4 modelMatrix;
};

#endif // OBSTACLE_H
