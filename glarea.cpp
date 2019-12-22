// Basé sur :
// CC-BY Edouard.Thiel@univ-amu.fr - 22/01/2019

#include "glarea.h"
#include <QDebug>
#include <QSurfaceFormat>
#include <QMatrix4x4>
#include <cmath>

GLArea::GLArea(QWidget *parent) :
    QOpenGLWidget(parent)
{
    QSurfaceFormat sf;
    sf.setDepthBufferSize(24);
    sf.setSamples(16);
    setFormat(sf);

    setEnabled(true);                   // événements clavier et souris
    setFocusPolicy(Qt::StrongFocus);    // accepte focus
    setFocus();                         // donne le focus

    timer = new QTimer(this);
    timer->setInterval(50);           // msec
    connect (timer, SIGNAL(timeout()), this, SLOT(onTimeout()));
    timer->start();
    elapsedTimer.start();

    init_simulation();
}


GLArea::~GLArea()
{
    delete timer;

    destroy_simulation();

    // Contrairement aux méthodes virtuelles initializeGL, resizeGL et repaintGL,
    // dans le destructeur le contexte GL n'est pas automatiquement rendu courant.
    makeCurrent();
    tearGLObjects();
    doneCurrent();
}


void GLArea::initializeGL()
{
    initializeOpenGLFunctions();
    glClearColor(0.5f,0.5f,1.0f,1.0f);
    glEnable(GL_DEPTH_TEST);

    makeGLObjects();

    // shader de la sphere
    program_sphere = new QOpenGLShaderProgram(this);
    program_sphere->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/sphere.vsh");
    program_sphere->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/sphere.fsh");
    if (! program_sphere->link()) {  // édition de lien des shaders dans le shader program
        qWarning("Failed to compile and link shader program:");
        qWarning() << program_sphere->log();
    }
}


void GLArea::makeGLObjects()
{
    // Création du cube sol
    GLfloat vertices_cube[] = {
        -100.0f,-50.0f,-100.0f,
        -100.0f,-50.0f, 100.0f,
        -100.0f, 50.0f, 100.0f,
         100.0f, 50.0f,-100.0f,
        -100.0f,-50.0f,-100.0f,
        -100.0f, 50.0f,-100.0f,
         100.0f,-50.0f, 100.0f,
        -100.0f,-50.0f,-100.0f,
         100.0f,-50.0f,-100.0f,
         100.0f, 50.0f,-100.0f,
         100.0f,-50.0f,-100.0f,
        -100.0f,-50.0f,-100.0f,
        -100.0f,-50.0f,-100.0f,
        -100.0f, 50.0f, 100.0f,
        -100.0f, 50.0f,-100.0f,
         100.0f,-50.0f, 100.0f,
        -100.0f,-50.0f, 100.0f,
        -100.0f,-50.0f,-100.0f,
        -100.0f, 50.0f, 100.0f,
        -100.0f,-50.0f, 100.0f,
         100.0f,-50.0f, 100.0f,
         100.0f, 50.0f, 100.0f,
         100.0f,-50.0f,-100.0f,
         100.0f, 50.0f,-100.0f,
         100.0f,-50.0f,-100.0f,
         100.0f, 50.0f, 100.0f,
         100.0f,-50.0f, 100.0f,
         100.0f, 50.0f, 100.0f,
         100.0f, 50.0f,-100.0f,
        -100.0f, 50.0f,-100.0f,
         100.0f, 50.0f, 100.0f,
        -100.0f, 50.0f,-100.0f,
        -100.0f, 50.0f, 100.0f,
         100.0f, 50.0f, 100.0f,
        -100.0f, 50.0f, 100.0f,
         100.0f,-50.0f, 100.0f
    };

    GLfloat vertices_cube_colors[] = {
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 1.0f, 0.0f
    };

    QVector<GLfloat> vertData_cube;
    for (int i = 0; i < 36; ++i) {
        // coordonnées sommets
        for (int j = 0; j < 3; j++)
            vertData_cube.append(vertices_cube[i*3+j]);
        for (int j = 0; j < 3; j++)
            vertData_cube.append(vertices_cube_colors[i*3+j]);
    }

    vbo_cube.create();
    vbo_cube.bind();
    vbo_cube.allocate(vertData_cube.constData(), vertData_cube.count() * int(sizeof(GLfloat)));

    // Création des spheres
    QVector<GLfloat> vertices_sphere;
    QVector<GLfloat> vertices_sphere_color;
    float rayon;
    float rouge;
    float vert;
    float bleue;
    float pi = static_cast<float>(M_PI);
    for(Sphere sphere : spheres){
        rayon = sphere.rayon;
        rouge = 1.0f;
        vert = 0.0f;
        bleue = 0.0f;
        for(int i = 0; i <= 40; i++)
        {
            float lat0 = pi * (-0.5f + (i - 1) / 40.0f);
            float z0  = sin(lat0);
            float zr0 = cos(lat0);

            float lat1 = pi * (-0.5f + i / 40.0f);
            float z1 = sin(lat1);
            float zr1 = cos(lat1);

            for(int j = 0; j <= 40; j++)
            {
                float lng = 2 * pi * (j - 1) / 40.0f;
                float x = cos(lng);
                float y = sin(lng);

                vertices_sphere.push_back(x * zr0 * rayon); //X
                vertices_sphere.push_back(y * zr0 * rayon); //Y
                vertices_sphere.push_back(z0 * rayon);      //Z
                vertices_sphere_color.push_back(rouge);
                vertices_sphere_color.push_back(vert);
                vertices_sphere_color.push_back(bleue);

                vertices_sphere.push_back(x * zr1 * rayon); //X
                vertices_sphere.push_back(y * zr1 * rayon); //Y
                vertices_sphere.push_back(z1 * rayon);      //Z
                vertices_sphere_color.push_back(rouge);
                vertices_sphere_color.push_back(vert);
                vertices_sphere_color.push_back(bleue);
            }
        }
    }

    QVector<GLfloat> vertData_sphere;
    for (int i = 0; i < 3362*static_cast<int>(spheres.size()); ++i) {
        // coordonnées sommets
        for (int j = 0; j < 3; j++)
            vertData_sphere.append(vertices_sphere[i*3+j]);
        for (int j = 0; j < 3; j++)
            vertData_sphere.append(vertices_sphere_color[i*3+j]);
    }

    vbo_sphere.create();
    vbo_sphere.bind();
    vbo_sphere.allocate(vertData_sphere.constData(), vertData_sphere.count() * int(sizeof(GLfloat)));

    // Création des boites
    QVector<GLfloat> vertData_boite;
    float x;
    float y;
    float z;
    for(Boite b : boites){
        x = b.longueur/2.0f;
        y = b.largeur/2.0f;
        z = b.profondeur/2.0f;
        rouge = rand_float(0.0f, 1.0f);
        vert = rand_float(0.0f, 1.0f);
        bleue = rand_float(0.0f, 1.0f);

        GLfloat vertices_boite[] = {
            -x,-y,-z,
            -x,-y, z,
            -x, y, z,
             x, y,-z,
            -x,-y,-z,
            -x, y,-z,
             x,-y, z,
            -x,-y,-z,
             x,-y,-z,
             x, y,-z,
             x,-y,-z,
            -x,-y,-z,
            -x,-y,-z,
            -x, y, z,
            -x, y,-z,
             x,-y, z,
            -x,-y, z,
            -x,-y,-z,
            -x, y, z,
            -x,-y, z,
             x,-y, z,
             x, y, z,
             x,-y,-z,
             x, y,-z,
             x,-y,-z,
             x, y, z,
             x,-y, z,
             x, y, z,
             x, y,-z,
            -x, y,-z,
             x, y, z,
            -x, y,-z,
            -x, y, z,
             x, y, z,
            -x, y, z,
             x,-y, y
        };

        GLfloat vertices_boite_colors[] = {
            rouge, vert, bleue,
            rouge, vert, bleue,
            rouge, vert, bleue,
            rouge, vert, bleue,
            rouge, vert, bleue,
            rouge, vert, bleue,
            rouge, vert, bleue,
            rouge, vert, bleue,
            rouge, vert, bleue,
            rouge, vert, bleue,
            rouge, vert, bleue,
            rouge, vert, bleue,
            rouge, vert, bleue,
            rouge, vert, bleue,
            rouge, vert, bleue,
            rouge, vert, bleue,
            rouge, vert, bleue,
            rouge, vert, bleue,
            rouge, vert, bleue,
            rouge, vert, bleue,
            rouge, vert, bleue,
            rouge, vert, bleue,
            rouge, vert, bleue,
            rouge, vert, bleue,
            rouge, vert, bleue,
            rouge, vert, bleue,
            rouge, vert, bleue,
            rouge, vert, bleue,
            rouge, vert, bleue,
            rouge, vert, bleue,
            rouge, vert, bleue,
            rouge, vert, bleue,
            rouge, vert, bleue,
            rouge, vert, bleue,
            rouge, vert, bleue,
            rouge, vert, bleue
        };

        for (int i = 0; i < 36; ++i) {
            // coordonnées sommets
            for (int j = 0; j < 3; j++)
                vertData_boite.append(vertices_boite[i*3+j]);
            for (int j = 0; j < 3; j++)
                vertData_boite.append(vertices_boite_colors[i*3+j]);
        }
    }

    vbo_boite.create();
    vbo_boite.bind();
    vbo_boite.allocate(vertData_boite.constData(), vertData_boite.count() * int(sizeof(GLfloat)));
}


void GLArea::tearGLObjects()
{
    vbo_cube.destroy();
    vbo_sphere.destroy();
    vbo_boite.destroy();
}


void GLArea::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
    windowRatio = float(w) / h;
}


void GLArea::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Matrice de projection
    QMatrix4x4 projectionMatrix;
    projectionMatrix.perspective(45.0f, windowRatio, 1.0f, 1000.0f);

    // Matrice de vue (caméra)
    QMatrix4x4 viewMatrix;
    viewMatrix.translate(xPos, yPos, zPos);
    viewMatrix.rotate(xRot, 1, 0, 0);
    viewMatrix.rotate(yRot, 0, 1, 0);
    viewMatrix.rotate(zRot, 0, 0, 1);

    /* CUBE */
    vbo_cube.bind();
    program_sphere->bind();

    QMatrix4x4 modelMatrixCube;
    modelMatrixCube.setToIdentity();
    modelMatrixCube.translate(0.0f, -56.0f, 0.0f);
    program_sphere->setUniformValue("projectionMatrix", projectionMatrix);
    program_sphere->setUniformValue("viewMatrix", viewMatrix);
    program_sphere->setUniformValue("modelMatrix", modelMatrixCube);

    program_sphere->setAttributeBuffer("in_position", GL_FLOAT, 0, 3, 6 * sizeof(GLfloat));
    program_sphere->setAttributeBuffer("colAttr", GL_FLOAT, 3 * sizeof(GLfloat), 3, 6 * sizeof(GLfloat));
    program_sphere->enableAttributeArray("in_position");
    program_sphere->enableAttributeArray("colAttr");

    glDrawArrays(GL_TRIANGLES, 0, 36);

    program_sphere->disableAttributeArray("in_position");
    program_sphere->disableAttributeArray("colAttr");
    program_sphere->release();
    vbo_cube.release();
    /* FIN CUBE */

    /* SPHERE */
    vbo_sphere.bind();
    program_sphere->bind();

    program_sphere->setUniformValue("projectionMatrix", projectionMatrix);
    program_sphere->setUniformValue("viewMatrix", viewMatrix);

    program_sphere->setAttributeBuffer("in_position", GL_FLOAT, 0, 3, 6 * sizeof(GLfloat));
    program_sphere->setAttributeBuffer("colAttr", GL_FLOAT, 3 * sizeof(GLfloat), 3, 6 * sizeof(GLfloat));
    program_sphere->enableAttributeArray("in_position");
    program_sphere->enableAttributeArray("colAttr");

    int start = 0;
    for(Sphere &s : spheres){
        s.affiche(program_sphere, start);
        start += 3362;
    }

    program_sphere->disableAttributeArray("in_position");
    program_sphere->disableAttributeArray("colAttr");
    program_sphere->release();
    vbo_sphere.release();
    /* FIN SPHERE */

    /* BOITE */
    vbo_boite.bind();
    program_sphere->bind();

    program_sphere->setUniformValue("projectionMatrix", projectionMatrix);
    program_sphere->setUniformValue("viewMatrix", viewMatrix);

    program_sphere->setAttributeBuffer("in_position", GL_FLOAT, 0, 3, 6 * sizeof(GLfloat));
    program_sphere->setAttributeBuffer("colAttr", GL_FLOAT, 3 * sizeof(GLfloat), 3, 6 * sizeof(GLfloat));
    program_sphere->enableAttributeArray("in_position");
    program_sphere->enableAttributeArray("colAttr");

    start = 0;
    for(Boite &b : boites){
        b.affiche(program_sphere, start);
        start += 36;
    }

    program_sphere->disableAttributeArray("in_position");
    program_sphere->disableAttributeArray("colAttr");
    program_sphere->release();
    vbo_boite.release();
    /* FIN BOITE */
}


void GLArea::keyPressEvent(QKeyEvent *ev)
{
    float da = 1.0f;

    switch(ev->key()) {
        case Qt::Key_A :
            xRot -= da;
            update();
            break;

        case Qt::Key_Q :
            xRot += da;
            update();
            break;

        case Qt::Key_Z :
            yRot -= da;
            update();
            break;

        case Qt::Key_S :
            yRot += da;
            update();
            break;

        case Qt::Key_E :
            zRot -= da;
            update();
            break;

        case Qt::Key_D :
            zRot += da;
            update();
            break;
    }
}


void GLArea::keyReleaseEvent(QKeyEvent *ev)
{
    qDebug() << __FUNCTION__ << ev->text();
}


void GLArea::mousePressEvent(QMouseEvent *ev)
{
    lastPos = ev->pos();
}


void GLArea::mouseReleaseEvent(QMouseEvent *ev)
{
    qDebug() << __FUNCTION__ << ev->x() << ev->y() << ev->button();
}


void GLArea::mouseMoveEvent(QMouseEvent *ev)
{
    int dx = ev->x() - lastPos.x();
    int dy = ev->y() - lastPos.y();

    if (ev->buttons() & Qt::LeftButton) {
        xRot += dy;
        yRot += dx;
        update();
    } else if (ev->buttons() & Qt::RightButton) {
        xPos += dx/10.0f;
        yPos -= dy/10.0f;
        update();
    } else if (ev->buttons() & Qt::MidButton) {
        xPos += dx/10.0f;
        zPos += dy;
        update();
    }

    lastPos = ev->pos();
}

float GLArea::rand_float(float min, float max){
    return min + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max-min)));
}

void GLArea::init_simulation(){
    ///-----initialization_start-----

    ///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
    collisionConfiguration = new btDefaultCollisionConfiguration();

    ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    dispatcher = new btCollisionDispatcher(collisionConfiguration);

    ///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
    overlappingPairCache = new btDbvtBroadphase();

    ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    solver = new btSequentialImpulseConstraintSolver;

    dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);

    dynamicsWorld->setGravity(btVector3(0, -10, 0));

    ///-----initialization_end-----

    ///create a few basic rigid bodies
    int id = 0;
    //the ground is a cube of side 100 at position y = -56.
    //the sphere will hit it at y = -6, with center at -5
    {
        btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(100.), btScalar(50.), btScalar(100.)));

        collisionShapes.push_back(groundShape);

        btTransform groundTransform;
        groundTransform.setIdentity();
        groundTransform.setOrigin(btVector3(0, -56, 0));

        btScalar mass(0.);

        //rigidbody is dynamic if and only if mass is non zero, otherwise static
        bool isDynamic = (mass != 0.f);

        btVector3 localInertia(0, 0, 0);
        if (isDynamic)
            groundShape->calculateLocalInertia(mass, localInertia);

        //using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
        btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
        btRigidBody* body = new btRigidBody(rbInfo);

        //add the body to the dynamics world
        dynamicsWorld->addRigidBody(body);

    }

    spheres.clear();
    {
        //create a dynamic rigidbody

        //btCollisionShape* colShape = new btBoxShape(btVector3(1,1,1));
        float rayon = 2.0f;

        btCollisionShape* colShape = new btSphereShape(btScalar(rayon));
        collisionShapes.push_back(colShape);

        /// Create Dynamic Objects
        btTransform startTransform;
        startTransform.setIdentity();

        btScalar mass(1.f);

        //rigidbody is dynamic if and only if mass is non zero, otherwise static
        bool isDynamic = (mass != 0.f);

        btVector3 localInertia(0, 0, 0);
        if (isDynamic)
            colShape->calculateLocalInertia(mass, localInertia);

        float posX = 0;
        float posY = 7;
        float posZ = 40;
        Sphere s = Sphere(QVector3D(posX, posY, posZ), rayon, ++id);
        spheres.push_back(s);

        startTransform.setOrigin(btVector3(posX, posY, posZ));

        //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
        btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
        btRigidBody* body = new btRigidBody(rbInfo);
        body->setLinearVelocity(btVector3(0, 0, -100));

        dynamicsWorld->addRigidBody(body);
    }

    boites.clear();
    float longueur = 1.0f;
    float largeur = 1.0f;
    float profondeur = 1.0f;
    for(float k = -5.5f; k < 14.5f; ++k){
        for(float i = 0; i < 1; ++i){
            for(float j = -4.5f; j < 5.5f; ++j){
                {
                    Boite b = Boite(QVector3D(j, k, i), longueur, largeur, profondeur, ++id);
                    boites.push_back(b);

                    btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(longueur/2), btScalar(largeur/2), btScalar(profondeur/2)));

                    collisionShapes.push_back(groundShape);

                    btTransform groundTransform;
                    groundTransform.setIdentity();
                    groundTransform.setOrigin(btVector3(j, k, i));

                    btScalar mass(0.5f);

                    //rigidbody is dynamic if and only if mass is non zero, otherwise static
                    bool isDynamic = (mass != 0.f);

                    btVector3 localInertia(0, 0, 0);
                    if (isDynamic)
                        groundShape->calculateLocalInertia(mass, localInertia);

                    //using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
                    btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
                    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
                    btRigidBody* body = new btRigidBody(rbInfo);

                    //add the body to the dynamics world
                    dynamicsWorld->addRigidBody(body);
                }
            }
        }
    }
}

void GLArea::destroy_simulation(){
    //cleanup in the reverse order of creation/initialization

    ///-----cleanup_start-----

    //remove the rigidbodies from the dynamics world and delete them
    for (int i = dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
    {
        btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);
        if (body && body->getMotionState())
        {
            delete body->getMotionState();
        }
        dynamicsWorld->removeCollisionObject(obj);
        delete obj;
    }

    //delete collision shapes
    for (int j = 0; j < collisionShapes.size(); j++)
    {
        btCollisionShape* shape = collisionShapes[j];
        collisionShapes[j] = nullptr;
        delete shape;
    }

    //delete dynamics world
    delete dynamicsWorld;

    //delete solver
    delete solver;

    //delete broadphase
    delete overlappingPairCache;

    //delete dispatcher
    delete dispatcher;

    delete collisionConfiguration;

    //next line is optional: it will be cleared by the destructor when the array goes out of scope
    collisionShapes.clear();
}

void GLArea::onTimeout()
{
    static qint64 old_chrono = elapsedTimer.elapsed(); // static : initialisation la première fois et conserve la dernière valeur
    qint64 chrono = elapsedTimer.elapsed();
    dt = (chrono - old_chrono) / 1000.0f;
    old_chrono = chrono;

    for(Sphere &s : spheres){
        s.anime(dynamicsWorld);
    }

    for(Boite &b : boites){
        b.anime(dynamicsWorld);
    }

    update();
}
