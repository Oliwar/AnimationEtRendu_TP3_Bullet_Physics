#include "sphere.h"

Sphere::Sphere(float rayon, int id)
{
    this->rayon = rayon;
    this->id = id;
}

Sphere::Sphere(QVector3D position, float rayon, int id)
{
    this->position = position;
    this->rayon = rayon;
    this->id = id;
}

void Sphere::anime(btDiscreteDynamicsWorld* dynamicsWorld){
    dynamicsWorld->stepSimulation(1.f / 60.f, 10);

    btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[this->id];
    btRigidBody* body = btRigidBody::upcast(obj);
    btTransform trans;
    if (body && body->getMotionState())
    {
        body->getMotionState()->getWorldTransform(trans);
    }
    else
    {
        trans = obj->getWorldTransform();
    }

    this->position.setX(trans.getOrigin().getX());
    this->position.setY(trans.getOrigin().getY());
    this->position.setZ(trans.getOrigin().getZ());
}

void Sphere::affiche(QOpenGLShaderProgram *sp, int start){
    this->modelMatrix.setToIdentity();
    this->modelMatrix.translate(position);

    sp->setUniformValue("modelMatrix", this->modelMatrix);

    glDrawArrays(GL_TRIANGLE_STRIP, start, 3362);
}
