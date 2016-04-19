#define GL_GLEXT_PROTOTYPES

#include <QtCore/QSignalMapper>
#include <QtGui/QApplication>
#include <QtGui/QKeyEvent>
#include <QtGui/QMenuBar>
#include <QtGui/QMessageBox>
#include <QtGui/QFileDialog>
#include <QtGui/QColorDialog>
#include <QtGui/QVector3D>

#include <boost/program_options.hpp>

#define NOMINMAX
#include <OpenEXR/ImathVec.h>
#include <OpenEXR/ImathMatrix.h>

#include "object_3d_benchmark/surfel_renderer.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace benchmark_retrieval {

inline void glTranslate(const Imath::V3f& v)
{
    glTranslatef(v.x, v.y, v.z);
}

inline void glVertex(const Imath::V3f& v)
{
    glVertex3f(v.x, v.y, v.z);
}

inline void glVertex(const Imath::V2f& v)
{
    glVertex2f(v.x, v.y);
}

inline void glColor(const Imath::C3f& c)
{
    glColor3f(c.x, c.y, c.z);
}

inline void glLoadMatrix(const Imath::M44f& m)
{
    glLoadMatrixf((GLfloat*)m[0]);
}


//----------------------------------------------------------------------
inline float rad2deg(float r)
{
    return r*180/M_PI;
}

inline QVector3D exr2qt(const Imath::V3f& v)
{
    return QVector3D(v.x, v.y, v.z);
}

inline Imath::V3f qt2exr(const QVector3D& v)
{
    return Imath::V3f(v.x(), v.y(), v.z());
}

inline Imath::M44f qt2exr(const QMatrix4x4& m)
{
    Imath::M44f mOut;
    for(int j = 0; j < 4; ++j)
    for(int i = 0; i < 4; ++i)
        mOut[j][i] = m.constData()[4*j + i];
    return mOut;
}

PointArrayModel::PointArrayModel()
    : m_npoints(0)
{ }

bool PointArrayModel::loadCloud(SurfelCloudT::Ptr& cloud)
{
    if(cloud->empty())
    {
        QMessageBox::critical(0, tr("Error"),
            tr("Couldn't open cloud!"));
        return false;
    }
    //m_fileName = fileName;

    m_npoints = cloud->size();
    m_P.reset(new V3f[m_npoints]);
    m_N.reset(new V3f[m_npoints]);
    m_r.reset(new float[m_npoints]);
    m_color.reset(new C3f[m_npoints]);

    m_colorChannelNames.push_back(QString("RGB"));

    // Iterate over all particles & pull in the data.
    V3f* outP = m_P.get();
    V3f* outN = m_N.get();
    float* outr = m_r.get();
    C3f* outc = m_color.get();
    for(size_t i = 0; i < cloud->size(); ++i)
    {
        SurfelType point = cloud->at(i);
        *outP++ = V3f(point.x, point.y, point.z);
        *outN++ = V3f(point.normal_x, point.normal_y, point.normal_z);
        *outr++ = point.radius;
        uint8_t* rgb = (uint8_t*)(&point.rgba);
        *outc++ = C3f(float(rgb[2])/255.0f, float(rgb[1])/255.0f, float(rgb[0])/255.0f);
    }
    return true;
}

bool PointArrayModel::loadPCDFile(const QString& fileName)
{
    SurfelCloudT::Ptr cloud(new SurfelCloudT);
    pcl::io::loadPCDFile(fileName.toStdString(), *cloud);
    return loadCloud(cloud);
}

V3f PointArrayModel::centroid() const
{
    V3f sum(0);
    const V3f* P = m_P.get();
    for(size_t i = 0; i < m_npoints; ++i, ++P)
        sum += *P;
    return (1.0f/m_npoints) * sum;
}

//------------------------------------------------------------------------------
GLfloat PointView::m_lightingPos[4];

PointView::PointView(QWidget *parent)
    : QGLWidget(parent),
    //m_camera(true),
    m_lastPos(0,0),
    m_zooming(false),
    m_backgroundColor(255, 255, 255),
    m_visMode(Vis_Disks),
    m_drawAxes(false),
    m_lighting(true),
    m_points(),
    m_cloudCenter(0)
{
    setFocusPolicy(Qt::StrongFocus);

    //connect(&m_camera, SIGNAL(projectionChanged()), this, SLOT(updateGL()));
    //connect(&m_camera, SIGNAL(viewChanged()), this, SLOT(updateGL()));
}


void PointView::loadPointFiles(const QStringList& fileNames)
{
    m_points.clear();
    for(int i = 0; i < fileNames.size(); ++i)
    {
        boost::shared_ptr<PointArrayModel> points(new PointArrayModel());
        if(points->loadPCDFile(fileNames[i]) && !points->empty())
            m_points.push_back(points);
    }
    if(m_points.empty())
        return;
    emit colorChannelsChanged(m_points[0]->colorChannels());
    m_cloudCenter = m_points[0]->centroid();
    //m_camera.setCenter(exr2qt(m_cloudCenter));
#if 0
    // Debug
    PointArray a;
    loadPointFile(a, fileNames[0].toStdString());
    m_pointTree.reset(); // free up memory
    m_pointTree = boost::shared_ptr<DiffusePointOctree>(new DiffusePointOctree(a));
#endif
    updateGL();
}

void PointView::loadConfigureCloud(SurfelCloudT::Ptr& cloud)
{
    m_points.clear();
    boost::shared_ptr<PointArrayModel> points(new PointArrayModel());
    if(points->loadCloud(cloud) && !points->empty())
    {
        m_points.push_back(points);
    }
    if(m_points.empty())
    {
        return;
    }
    emit colorChannelsChanged(m_points[0]->colorChannels());
    m_cloudCenter = m_points[0]->centroid();
    //m_camera.setCenter(exr2qt(m_cloudCenter));
    updateGL();
}


void PointView::setProjectionMatrix(const QMatrix4x4& projectionMatrix)
{
    m_projectionMatrix = projectionMatrix;
}


void PointView::setViewMatrix(const QMatrix4x4& viewMatrix)
{
    m_viewMatrix = viewMatrix;
}


void PointView::setLightingPos(const QVector3D& lightingPos)
{
    m_lightingPos[0] = lightingPos.x();
    m_lightingPos[1] = lightingPos.y();
    m_lightingPos[2] = lightingPos.z();
    m_lightingPos[3] = 1;
}


PointView::VisMode PointView::visMode() const
{
    return m_visMode;
}


void PointView::setBackground(QColor col)
{
    m_backgroundColor = col;
    updateGL();
}


void PointView::setVisMode(VisMode mode)
{
    m_visMode = mode;
    updateGL();
}


void PointView::toggleDrawAxes()
{
    m_drawAxes = !m_drawAxes;
}


QSize PointView::sizeHint() const
{
    // Size hint, mainly for getting the initial window size right.
    // setMinimumSize() also sort of works for this, but doesn't allow the
    // user to later make the window smaller.
    return QSize(320,240);
}


void PointView::initializeGL()
{
    //glEnable(GL_MULTISAMPLE);
}


void PointView::resizeGL(int w, int h)
{
    // Draw on full window
    glViewport(0, 0, w, h);
    //m_camera.setViewport(geometry());
}


void PointView::paintGL()
{
    //--------------------------------------------------
    // Draw main scene
    // Set camera projection
    glMatrixMode(GL_PROJECTION);
    glLoadMatrix(qt2exr(m_projectionMatrix));
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrix(qt2exr(m_viewMatrix));

    glClearDepth(1.0f);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glClearColor(m_backgroundColor.redF(), m_backgroundColor.greenF(),
                 m_backgroundColor.blueF(), 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Draw geometry

    if(m_drawAxes)
        drawAxes();
    for(size_t i = 0; i < m_points.size(); ++i)
        drawPoints(*m_points[i], m_visMode, m_lighting);
}

/// Draw a set of axes
void PointView::drawAxes()
{
    glEnable(GL_LINE_SMOOTH);
    glLineWidth(1);
    glColor3f(1.0, 0.0, 0.0);
    glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(1, 0, 0);
    glEnd();
    glColor3f(0.0, 1.0, 0.0);
    glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 1, 0);
    glEnd();
    glColor3f(0.0, 0.0, 1.0);
    glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, 1);
    glEnd();
}

cv::Mat PointView::drawImage()
{
    paintGL();
    glFlush();
    QImage buffer = grabFrameBuffer(false);
    cv::Mat image(buffer.height(), buffer.width(), CV_8UC4, buffer.bits());
    //boost::filesystem::path cloud_path(fileName.toStdString());
    //boost::filesystem::path image_path = cloud_path.parent_path() / "surfel_image.png";
    //cv::imwrite(image_path.string(), image);

    //cv::imshow("Buffer", image);
    //cv::waitKey();

    cv::Mat rgb_image;
    cv::cvtColor(image, rgb_image, CV_RGBA2RGB);

    return rgb_image;
}

/// Draw point cloud using OpenGL
void PointView::drawPoints(const PointArrayModel& points, VisMode visMode,
                           bool useLighting)
{
    if(points.empty())
        return;
    switch(visMode)
    {
        case Vis_Points:
        {
            glDisable(GL_COLOR_MATERIAL);
            glDisable(GL_LIGHTING);
            // Draw points
            glPointSize(1);
            glColor3f(1,1,1);
            // Set distance attenuation for points, following the usual 1/z
            // law.
            //GLfloat attenParams[3] = {0, 0, 1};
            //glPointParameterfv(GL_POINT_DISTANCE_ATTENUATION, attenParams);
            //glPointParameterf(GL_POINT_SIZE_MIN, 0);
            //glPointParameterf(GL_POINT_SIZE_MAX, 100);
            // Draw all points at once using vertex arrays.
            glEnableClientState(GL_VERTEX_ARRAY);
            glVertexPointer(3, GL_FLOAT, 3*sizeof(float),
                            reinterpret_cast<const float*>(points.P()));
            const float* col = reinterpret_cast<const float*>(points.color());
            if(col)
            {
                glEnableClientState(GL_COLOR_ARRAY);
                glColorPointer(3, GL_FLOAT, 3*sizeof(float), col);
            }
            glDrawArrays(GL_POINTS, 0, points.size());
            glDisableClientState(GL_VERTEX_ARRAY);
            glDisableClientState(GL_COLOR_ARRAY);
        }
        break;
        case Vis_Disks:
        {
            // Materials
            glShadeModel(GL_SMOOTH);
            if(useLighting)
            {
                glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
                glEnable(GL_COLOR_MATERIAL);
                // Lighting
                // light0
                //GLfloat lightPos[] = {0, 0, 0, 1}; // lighting pos should be configurable, 5m right of viewing direction?
                GLfloat whiteCol[] = {0.005, 0.005, 0.005, 1};
                glLightfv(GL_LIGHT0, GL_POSITION, m_lightingPos);
                glLightfv(GL_LIGHT0, GL_DIFFUSE, whiteCol);
                glEnable(GL_LIGHT0);
                // whole-scene ambient intensity scaling
                GLfloat ambientCol[] = {0.5, 0.5, 0.5, 1};
                glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambientCol);
                // whole-scene diffuse intensity
                //GLfloat diffuseCol[] = {0.05, 0.05, 0.05, 1};
                //glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuseCol);
                // two-sided lighting.
                // TODO: Why doesn't this work?? (handedness problems?)
                //glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
                // rescaling of normals
                //glEnable(GL_RESCALE_NORMAL);
            }
            //glCullFace(GL_BACK);
            //glEnable(GL_CULL_FACE);
            // Compile radius 1 disk primitive
            V3f p0(0);
            V3f t1(1,0,0);
            V3f t2(0,1,0);
            V3f diskNormal(0,0,1);
            GLuint disk = glGenLists(1);
            glNewList(disk, GL_COMPILE);
                glBegin(GL_TRIANGLE_FAN);
                const int nSegs = 10;
                // Scale so that _min_ radius of polygon approx is 1.
                float scale = 1/cos(M_PI/nSegs);
                glVertex3f(0,0,0);
                for(int i = 0; i <= nSegs; ++i)
                {
                    float angle = 2*M_PI*float(i)/nSegs;
                    glVertex3f(scale*cos(angle), scale*sin(angle), 0);
                }
                glEnd();
                // Disk normal
                //glBegin(GL_LINES);
                //    glVertex(V3f(0));
                //    glVertex(diskNormal);
                //glEnd();
            glEndList();
            // Draw points as disks.
            // TODO: Doing this in immediate mode is really slow!  Using
            // vertex shaders would probably be a much better method, or
            // perhaps just compile into a display list?
            if(useLighting)
                glEnable(GL_LIGHTING);
            const V3f* P = points.P();
            const V3f* N = points.N();
            const float* r = points.r();
            const C3f* col = points.color();
            for(size_t i = 0; i < points.size(); ++i, ++P, ++N, ++r)
            {
                glColor(col ? *col++ : C3f(1));
                if(N->length2() == 0)
                {
                    // For zero-length normals, we don't know the disk
                    // orientation, so draw as a point instead.
                    glPointSize(1);
                    glBegin(GL_POINTS);
                        glVertex(*P);
                    glEnd();
                    continue;
                }
                glPushMatrix();
                // Translate disk to point location and scale
                glTranslate(*P);
                glScalef(*r, *r, *r);
                // Transform the disk normal (0,0,1) into the correct normal
                // direction for the current point.  The appropriate transform
                // is a rotation about a direction perpendicular to both
                // normals,
                V3f v = diskNormal % *N;
                if(v.length2() > 1e-10)
                {
                    // And via the angle given by the dot product.  (If the
                    // length of v is very small we don't do the rotation for
                    // numerical stability.)
                    float angle = rad2deg(acosf(diskNormal.dot(*N)));
                    glRotatef(angle, v.x, v.y, v.z);
                }
                // Instance the disk
                glCallList(disk);
                glPopMatrix();
            }
            glDeleteLists(disk, 1);
            if(useLighting)
                glDisable(GL_LIGHTING);
        }
        break;
    }
}

cv::Mat render_surfel_image(SurfelCloudT::Ptr& cloud, const Eigen::Matrix4f& T,
                            const Eigen::Matrix3f& K, size_t height, size_t width)
{
    int argc = 1;
    char* argv = "render_surfels";
    //QApplication::setDesktopSettingsAware(false);
    QApplication::setStyle("cde");
    QApplication app(argc, &argv);

    PointView pview;
    //pview.loadConfigureCloud(cloud);
    float f = 10.0f; // far plane distance
    float n = 0.5f; // near plane distance
    QMatrix4x4 proj(2.0f*K(0, 0)/float(width), 0, (float(width) - 2.0f*K(0, 2) + 0*float(width))/float(width), 0,
                    0, -2.0f*K(1, 1)/float(height), (-float(height) + 2.0f*K(1, 2) + 0*float(height))/float(height), 0,
                    0, 0, -(f+n)/(f-n), -2.0f*f*n/(f-n),
                    0, 0, -1.0f, 0);
    glViewport(0, 0, width, height);
    pview.setProjectionMatrix(proj);
    QMatrix4x4 view(T(0, 0), T(0, 1), T(0, 2), T(0, 3),
                    T(1, 0), T(1, 1), T(1, 2), T(1, 3),
                    -T(2, 0), -T(2, 1), -T(2, 2), -T(2, 3),
                    T(3, 0), T(3, 1), T(3, 2), T(3, 3));
    QVector3D lighting_pos(5.0f*T(0, 0), 5.0f*T(0, 1), 5.0f*T(0, 2));
    pview.setLightingPos(lighting_pos);
    pview.setGeometry(0, 0, width, height);
    pview.setViewMatrix(view);
    pview.updateGeometry();
    //pview.updateGL();
    pview.loadConfigureCloud(cloud);
    cv::Mat image = pview.drawImage();

    return image;
}

} // namespace benchmark_retrieval
