/****************************************************************************
**
** Copyright (C) 2016 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the QtCore module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include "mainwidget.h"

#include <QMouseEvent>

#include <math.h>

MainWidget::MainWidget(QWidget *parent) :
    QOpenGLWidget(parent),
    geometries(0),
    textureGrass(0),
    textureRock(0),
    textureSnow(0),
    textureMap(0),
    angularSpeed(0)
{
    xOffset = 0;
    yOffset = -1;
    zOffset = 0;
}

MainWidget::~MainWidget()
{
    // Make sure the context is current when deleting the texture
    // and the buffers.
    makeCurrent();
    delete textureGrass;
    delete textureRock;
    delete textureSnow;
    delete textureMap;
    delete geometries;
    doneCurrent();
}

void MainWidget::keyPressEvent(QKeyEvent *e){
    if(freeView){
        if(e->key() == Qt::Key_W){zOffset+=0.1;}
        if(e->key() == Qt::Key_S){zOffset-=0.1;}
        if(e->key() == Qt::Key_A){xOffset+=0.1;}
        if(e->key() == Qt::Key_D){xOffset-=0.1;}
        if(e->key() == Qt::Key_Space){yOffset-=0.1;}
        if(e->key() == Qt::Key_Shift){yOffset+=0.1;}
    }
    if(e->key() == Qt::Key_C){freeView = !freeView;}
    update();
}


//! [0]
void MainWidget::mousePressEvent(QMouseEvent *e)
{
    // Save mouse press position
    mousePressPosition = QVector2D(e->localPos());
}

void MainWidget::mouseReleaseEvent(QMouseEvent *e)
{
    if(freeView){
        // Mouse release position - mouse press position
        QVector2D diff = QVector2D(e->localPos()) - mousePressPosition;

        // Rotation axis is perpendicular to the mouse position difference
        // vector
        QVector3D n = QVector3D(diff.y(), diff.x(), 0.0).normalized();

        // Accelerate angular speed relative to the length of the mouse sweep
        qreal acc = diff.length() / 10.0;

        // Calculate new rotation axis as weighted sum
        rotationAxis = ( rotationAxis * angularSpeed + n * acc).normalized();
        rotationAxis[0] = 0;
        rotationAxis[2] = 0;
        // Increase angular speed
        angularSpeed += acc;
    }
}
//! [0]

//! [1]
void MainWidget::timerEvent(QTimerEvent *)
{
    if(freeView){
        // Decrease angular speed (friction)
        angularSpeed *= 0.70;

        // Stop rotation when speed goes below threshold
        if (angularSpeed < 0.01) {
            angularSpeed = 0.0;
        } else {
            // Update rotation
            rotation = QQuaternion::fromAxisAndAngle(rotationAxis, angularSpeed) * rotation;

            // Request an update
            update();
        }
    }else{
        rotation = QQuaternion::fromAxisAndAngle(0,1,0, 0.5) * rotation;
        update();
    }
}
//! [1]

void MainWidget::initializeGL()
{
    initializeOpenGLFunctions();

    glClearColor(0, 0, 0, 1);

    initShaders();
    initTextures();

//! [2]
    // Enable depth buffer
    glEnable(GL_DEPTH_TEST);

    // Enable back face culling
    //glEnable(GL_CULL_FACE);
//! [2]

    geometries = new GeometryEngine;

    // Use QBasicTimer because its faster than QTimer
    timer.start(12, this);
}

//! [3]
void MainWidget::initShaders()
{
    // Compile vertex shader
    if (!program.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/vshader.glsl"))
        close();

    // Compile fragment shader
    if (!program.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/fshader.glsl"))
        close();

    // Link shader pipeline
    if (!program.link())
        close();

    // Bind shader pipeline for use
    if (!program.bind())
        close();
}
//! [3]

//! [4]
void MainWidget::initTextures()
{
    // Load image
    textureGrass = new QOpenGLTexture(QImage(":/grass.png").mirrored());
    textureRock = new QOpenGLTexture(QImage(":/rock.png").mirrored());
    textureSnow = new QOpenGLTexture(QImage(":/snowrocks.png").mirrored());
    textureMap = new QOpenGLTexture(QImage(":/heightmap-1024x1024.png").mirrored());

    // Set nearest filtering mode for texture minification
    textureGrass->setMinificationFilter(QOpenGLTexture::Nearest);
    textureRock->setMinificationFilter(QOpenGLTexture::Nearest);
    textureSnow->setMinificationFilter(QOpenGLTexture::Nearest);
    textureMap->setMinificationFilter(QOpenGLTexture::Nearest);

    // Set bilinear filtering mode for texture magnification
    textureGrass->setMagnificationFilter(QOpenGLTexture::Linear);
    textureRock->setMagnificationFilter(QOpenGLTexture::Linear);
    textureSnow->setMagnificationFilter(QOpenGLTexture::Linear);
    textureMap->setMagnificationFilter(QOpenGLTexture::Linear);

    // Wrap texture coordinates by repeating
    // f.ex. texture coordinate (1.1, 1.2) is same as (0.1, 0.2)
    textureGrass->setWrapMode(QOpenGLTexture::Repeat);
    textureRock->setWrapMode(QOpenGLTexture::Repeat);
    textureSnow->setWrapMode(QOpenGLTexture::Repeat);
    textureMap->setWrapMode(QOpenGLTexture::Repeat);
}
//! [4]

//! [5]
void MainWidget::resizeGL(int w, int h)
{
    // Calculate aspect ratio
    qreal aspect = qreal(w) / qreal(h ? h : 1);

    // Set near plane to 3.0, far plane to 7.0, field of view 45 degrees
    const qreal zNear = 1.0, zFar = 70.0, fov = 45.0;

    // Reset projection
    projection.setToIdentity();

    // Set perspective projection
    projection.perspective(fov, aspect, zNear, zFar);
}
//! [5]

void MainWidget::paintGL()
{
    // Clear color and depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.53,0.81,0.92,0.8);
    textureGrass->bind(0);
    textureRock->bind(1);
    textureSnow->bind(2);
    textureMap->bind(3);

//! [6]
    // Calculate model view transformation
    QMatrix4x4 matrix;
    if(freeView){
        matrix.rotate(rotation);
        matrix.translate(xOffset, yOffset, zOffset);
    }else{


        matrix.rotate( QQuaternion::fromAxisAndAngle(1,0,0,45));

        matrix.rotate(rotation);

        matrix.translate(0, -7, -0);

    }
    // Set modelview-projection matrix
    program.setUniformValue("mvp_matrix", projection * matrix);
//! [6]

    // Use texture unit 0 which contains cube.png
    program.setUniformValue("textureGrass", 0);
    program.setUniformValue("textureRock", 1);
    program.setUniformValue("textureSnow", 2);
    program.setUniformValue("textureMap", 3);

    // Draw cube geometry
    //geometries->drawCubeGeometry(&program);
    geometries->drawPlaneGeometry(&program);
}
