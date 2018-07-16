#include "opengldisplay.h"
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>

//openglDisplay::openglDisplay()
//{

//}

//openglDisplay::~openglDisplay()
//{

//}

void openglDisplay::initializeGL()
{
    initializeOpenGLFunctions();


    makeObject();

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

#define PROGRAM_VERTEX_ATTRIBUTE 0
#define PROGRAM_TEXCOORD_ATTRIBUTE 1

    QOpenGLShader *vshader = new QOpenGLShader(QOpenGLShader::Vertex, this);
    const char *vsrc =
        "attribute highp vec4 vertex;\n"
        "attribute mediump vec4 texCoord;\n"
        "varying mediump vec4 texc;\n"
        "uniform mediump mat4 matrix;\n"
        "void main(void)\n"
        "{\n"
        "    gl_Position = matrix * vertex;\n"
        "    texc = texCoord;\n"
        "}\n";
    vshader->compileSourceCode(vsrc);

    QOpenGLShader *fshader = new QOpenGLShader(QOpenGLShader::Fragment, this);
    const char *fsrc =
        "uniform sampler2D texture;\n"
        "varying mediump vec4 texc;\n"
        "void main(void)\n"
        "{\n"
        "    gl_FragColor = texture2D(texture, texc.st);\n"
        "}\n";
    fshader->compileSourceCode(fsrc);

    program = new QOpenGLShaderProgram;
    program->addShader(vshader);
    program->addShader(fshader);
    program->bindAttributeLocation("vertex", PROGRAM_VERTEX_ATTRIBUTE);
    program->bindAttributeLocation("texCoord", PROGRAM_TEXCOORD_ATTRIBUTE);
    program->link();

    program->bind();
    program->setUniformValue("texture", 0);
}

void openglDisplay::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    //control orientation
    QMatrix4x4 m;
    m.ortho(-1.0f, +1.0f, +1.0f, -1.0f, 4.0f, 15.0f);
    m.translate(0.0f, 0.0f, -10.0f);

    program->setUniformValue("matrix", m);
    program->enableAttributeArray(PROGRAM_VERTEX_ATTRIBUTE);
    program->enableAttributeArray(PROGRAM_TEXCOORD_ATTRIBUTE);
    program->setAttributeBuffer(PROGRAM_VERTEX_ATTRIBUTE, GL_FLOAT, 0, 3, 5 * sizeof(GLfloat));
    program->setAttributeBuffer(PROGRAM_TEXCOORD_ATTRIBUTE, GL_FLOAT, 3 * sizeof(GLfloat), 2, 5 * sizeof(GLfloat));

    texture->bind();
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
}

void openglDisplay::resizeGL(int width, int height)
{
    int side = qMin(width, height);
    glViewport((width - side) / 2, (height - side) / 2, side, side);
}

//this function currently not in used -- probably will be deprecate
GLuint openglDisplay::matToTexture(cv::Mat mat, GLenum minFilter, GLenum magFilter, GLenum wrapFilter)
{
    // Generate a number for our textureID's unique handle
    GLuint textureID;
    glGenTextures(1, &textureID);

    // Bind to our texture handle
    glBindTexture(GL_TEXTURE_2D, textureID);

    // Catch silly-mistake texture interpolation method for magnification
    if (magFilter == GL_LINEAR_MIPMAP_LINEAR  ||
        magFilter == GL_LINEAR_MIPMAP_NEAREST ||
        magFilter == GL_NEAREST_MIPMAP_LINEAR ||
        magFilter == GL_NEAREST_MIPMAP_NEAREST)
    {
        //cout << "You can't use MIPMAPs for magnification - setting filter to GL_LINEAR" << endl;
        magFilter = GL_LINEAR;
    }

    // Set texture interpolation methods for minification and magnification
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, minFilter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, magFilter);

    // Set texture clamping method
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrapFilter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrapFilter);

    // Set incoming texture format to:
    // GL_BGR       for CV_CAP_OPENNI_BGR_IMAGE,
    // GL_LUMINANCE for CV_CAP_OPENNI_DISPARITY_MAP,
    // Work out other mappings as required ( there's a list in comments in main() )
    GLenum inputColourFormat = GL_BGR;
    if (mat.channels() == 1)
    {
        inputColourFormat = GL_LUMINANCE;
    }

    // Create the texture
    glTexImage2D(GL_TEXTURE_2D,     // Type of texture
                 0,                 // Pyramid level (for mip-mapping) - 0 is the top level
                 GL_RGB,            // Internal colour format to convert to
                 mat.cols,          // Image width  i.e. 640 for Kinect in standard mode
                 mat.rows,          // Image height i.e. 480 for Kinect in standard mode
                 0,                 // Border width in pixels (can either be 1 or 0)
                 inputColourFormat, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
                 GL_UNSIGNED_BYTE,  // Image data type
                 mat.ptr());        // The actual image data itself

    return textureID;
}

QImage openglDisplay::mat_to_qimage(cv::Mat &mat, QImage::Format format)
{
    QImage matImage;
    if(!mat.empty())
        //matImage = Mat2QImage(mat);
        matImage =  QImage(mat.data, mat.cols, mat.rows, mat.step, format);
    else
        matImage = QImage(QString(":/images/side2.png")).mirrored();
  return matImage;
}


QImage openglDisplay::Mat2QImage(const cv::Mat_<double> &src)
{
        double scale = 255.0;
        QImage dest(src.cols, src.rows, QImage::Format_ARGB32);
        for (int y = 0; y < src.rows; ++y) {
                const double *srcrow = src[y];
                QRgb *destrow = (QRgb*)dest.scanLine(y);
                for (int x = 0; x < src.cols; ++x) {
                        unsigned int color = srcrow[x] * scale;
                        destrow[x] = qRgba(color, color, color, 255);
                }
        }
        return dest;
}

void openglDisplay::setDisplayTexture(cv::Mat &frame){
    //matTexture = matToTexture(frame, GL_LINEAR_MIPMAP_LINEAR, GL_LINEAR, GL_CLAMP);
    if(!frame.empty()){
        QImage matImage = mat_to_qimage(frame,QImage::Format_RGB888);  //opencv videocapture mat rgb is rgb888 format

        //recreate texture to set new data
        texture->destroy();
        texture->create();
        if(!matImage.isNull()) texture->setData( matImage.mirrored());
        else
            texture->setData( QImage(QString(":/images/side1.png")));
        update();
        texture->release();
    }
}

void openglDisplay::makeObject()
{
    //showing default display
    texture = new QOpenGLTexture(QImage(QString(":/images/side1.png")).mirrored());

    float xl = 1;
    float yl = 1;
    float zl = 1;

    ////drawing box frame and placing the texture coordinate in vertex points
    /// vertices data structure
    /// vectices in 3D space
    /// 1. x vertex point
    /// 2. y vertex point
    /// 3. z vertex point
    /// Texture points in 2D
    /// 4. x texture point
    /// 5. y texture point
    ///----------------------------------------------------------------------
    //point vertices array
    QVector<GLfloat> vertData;

    vertData.append(xl * 1);
    vertData.append(yl * -1);
    vertData.append(zl * -1);
    vertData.append(1);
    vertData.append(1);

    vertData.append(xl * -1);
    vertData.append(yl * -1);
    vertData.append(zl * -1);
    vertData.append(0);
    vertData.append(1);

    vertData.append(xl * -1);
    vertData.append(yl * 1);
    vertData.append(zl * -1);
    vertData.append(0);
    vertData.append(0);

    vertData.append(xl * 1);
    vertData.append(yl * 1);
    vertData.append(zl * -1);
    vertData.append(1);
    vertData.append(0);

    glbuffer.create();
    glbuffer.bind();
    glbuffer.allocate(vertData.constData(), vertData.count() * sizeof(GLfloat));
}

void openglDisplay::makeGui(){
    float xl = 1;
    float yl = 1;
    float zl = 1;
    //texture = new QOpenGLTexture(QImage(QString(":/images/side3.png")).mirrored());
    ////drawing box frame and placing the texture coordinate in vertex points
    /// vertices data structure
    /// vectices in 3D space
    /// 1. x vertex point
    /// 2. y vertex point
    /// 3. z vertex point
    /// Texture points in 2D
    /// 4. x texture point
    /// 5. y texture point
    ///----------------------------------------------------------------------
    //point vertices array
    QVector<GLfloat> vertData;

    vertData.append(xl * 1);
    vertData.append(yl * -1);
    vertData.append(zl * -1);
    vertData.append(1);
    vertData.append(1);

    vertData.append(xl * -1);
    vertData.append(yl * -1);
    vertData.append(zl * -1);
    vertData.append(0);
    vertData.append(1);

    vertData.append(xl * -1);
    vertData.append(yl * 1);
    vertData.append(zl * -1);
    vertData.append(0);
    vertData.append(0);

    vertData.append(xl * 1);
    vertData.append(yl * 1);
    vertData.append(zl * -1);
    vertData.append(1);
    vertData.append(0);

    glbuffer.create();
    glbuffer.bind();
    glbuffer.allocate(vertData.constData(), vertData.count() * sizeof(GLfloat));
}
