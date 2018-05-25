#ifndef OPENGLDISPLAY_H
#define OPENGLDISPLAY_H

#include <QOpenGLWidget>
#include <QWidget>
#include <QOpenGLFunctions>
#include <QOpenGLBuffer>
#include <GL/glu.h>
#include <GL/gl.h>
#include "opencv/cv.h"

QT_FORWARD_DECLARE_CLASS(QOpenGLShaderProgram);
QT_FORWARD_DECLARE_CLASS(QOpenGLTexture)

class openglDisplay : public QOpenGLWidget, protected QOpenGLFunctions
{
public:
    openglDisplay(QWidget *parent) : QOpenGLWidget(parent) { }
    //~openglDisplay();
    void setDisplayTexture(cv::Mat&);
    GLuint matToTexture(cv::Mat, GLenum , GLenum , GLenum );
    void makeObject();
    void makeGui();
    QImage mat_to_qimage(cv::Mat&, QImage::Format);
    QImage Mat2QImage(const cv::Mat_<double>&);

protected:
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();

private:
  QOpenGLTexture* texture;
  QOpenGLShaderProgram* program;
  GLuint matTexture;
  QOpenGLBuffer glbuffer;

};

#endif // OPENGLDISPLAY_H
