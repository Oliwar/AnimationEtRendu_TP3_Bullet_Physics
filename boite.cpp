#include "boite.h"

Boite::Boite(QVector3D position, float longueur, float largeur, float profondeur, int id)
{
    this->position = position;
    this->longueur = longueur;
    this->largeur = largeur;
    this->profondeur = profondeur;
    this->id = id;
}

void Boite::anime(btDiscreteDynamicsWorld* dynamicsWorld){
    dynamicsWorld->stepSimulation(1.f / 500.f, 10);

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

void Boite::affiche(QOpenGLShaderProgram *sp, int start){
    this->modelMatrix.setToIdentity();
    this->modelMatrix.translate(position);

    sp->setUniformValue("modelMatrix", this->modelMatrix);

    glDrawArrays(GL_TRIANGLES, start, 36);
}
