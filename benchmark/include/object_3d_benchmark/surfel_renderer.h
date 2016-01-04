#ifndef SURFEL_RENDERER_H
#define SURFEL_RENDERER_H

#include <cmath>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/shared_array.hpp>

#include <QtGui/QMainWindow>
#include <QtGui/QMatrix4x4>
#include <QtOpenGL/QGLWidget>

#include <OpenEXR/ImathVec.h>
#include <OpenEXR/ImathColor.h>

#include <opencv2/core/core.hpp>

#include "object_3d_benchmark/surfel_type.h"

using SurfelT = SurfelType;
using SurfelCloudT = pcl::PointCloud<SurfelT>;

namespace benchmark_retrieval {

using Imath::V3f;
using Imath::V2f;
using Imath::C3f;

//------------------------------------------------------------------------------
/// Container for points to be displayed in the PointView interface
class PointArrayModel : public QObject
{
    Q_OBJECT

public:
    PointArrayModel();

    /// Load points from a file
    //bool loadPointFile(const QString& fileName);

    /// Load points from a PCD point cloud file
    bool loadPCDFile(const QString& fileName);
    bool loadCloud(SurfelCloudT::Ptr& cloud);

    /// Return the number of points
    size_t size() const { return m_npoints; }
    /// Return true when there are zero points
    bool empty() const { return m_npoints == 0; }

    /// Return point position
    const V3f* P() const { return m_P.get(); }
    /// Return point normals
    const V3f* N() const { return m_N.get(); }
    /// Return point radii
    const float* r() const { return m_r.get(); }
    /// Return point color, or NULL if no color channel is present
    const C3f* color() const { return m_color.get(); }

    /// Get a list of channel names which look like color channels
    QStringList colorChannels() { return m_colorChannelNames; }

    /// Set the channel name which the color() function returns data for
    //void setColorChannel(const QString& name);

    /// Compute the centroid of the P data
    V3f centroid() const;

private:
    /// Look through the channels to find all which look like color data
    //static QStringList findColorChannels(const Partio::ParticlesInfo* ptFile);

    QString m_fileName;
    QStringList m_colorChannelNames;
    size_t m_npoints;
    boost::shared_array<V3f> m_P;
    boost::shared_array<V3f> m_N;
    boost::shared_array<float> m_r;
    boost::shared_array<C3f> m_color;
};


//------------------------------------------------------------------------------
/// OpenGL-based viewer widget for point clouds (or more precisely, clouds of
/// disk-like surface elements).
class PointView : public QGLWidget
{
    Q_OBJECT

public:
    /// Point (surface element) visualization mode
    enum VisMode
    {
        Vis_Points,  ///< Draw surfels using GL_POINTS
        Vis_Disks    ///< Draw surfels as disks
    };

    PointView(QWidget *parent = NULL);

    /// Load a point cloud from a file
    void loadPointFiles(const QStringList& fileNames);
    void loadConfigureCloud(SurfelCloudT::Ptr& cloud);

    /// Set properties for rendering probe environment map
    void setProbeParams(int cubeFaceRes, float maxSolidAngle);
    void setProjectionMatrix(const QMatrix4x4& projectionMatrix);
    void setViewMatrix(const QMatrix4x4& viewMatrix);
    void setLightingPos(const QVector3D& lightingPos);

    /// Get the visualization mode
    VisMode visMode() const;

    /// Hint at an appropriate size
    QSize sizeHint() const;

    cv::Mat drawImage();

public slots:
    /// Set the backgroud color
    void setBackground(QColor col);
    void setVisMode(VisMode mode);
    //void setColorChannel(QString channel);
    void toggleDrawAxes();

signals:
    void colorChannelsChanged(QStringList channels);

protected:
    // Qt OpenGL callbacks
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();

    // Qt event callbacks
    //void mousePressEvent(QMouseEvent* event);
    //void mouseMoveEvent(QMouseEvent* event);
    //void wheelEvent(QWheelEvent* event);
    //void keyPressEvent(QKeyEvent* event);

private:
    static void drawAxes();
    //void drawCursor(const V3f& P) const;
    static void drawPoints(const PointArrayModel& points, VisMode visMode,
                           bool useLighting);

    /// Mouse-based camera positioning
    //InteractiveCamera m_camera;
    QMatrix4x4 m_projectionMatrix;
    QMatrix4x4 m_viewMatrix;
    QPoint m_lastPos;
    bool m_zooming;
    /// Position of 3D cursor
    //V3f m_cursorPos;
    /// Light probe resolution
    //int m_probeRes;
    //float m_probeMaxSolidAngle;
    /// Background color for drawing
    QColor m_backgroundColor;
    /// Type of visualization
    VisMode m_visMode;
    bool m_drawAxes;
    /// Flag for whether to use OpenGL lighting or not
    bool m_lighting;
    static GLfloat m_lightingPos[4];
    /// Point cloud data
    std::vector<boost::shared_ptr<PointArrayModel> > m_points;
    //boost::shared_ptr<const DiffusePointOctree> m_pointTree;
    V3f m_cloudCenter;
};

cv::Mat render_surfel_image(SurfelCloudT::Ptr& cloud, const Eigen::Matrix4f& T,
                            const Eigen::Matrix3f& K, size_t height, size_t width);

} // namespace benchmark_retrieval

#endif // SURFEL_RENDERER_H
