#ifndef __CAMERA_PARAMETERS__
#define __CAMERA_PARAMETERS__

#include <cassert>

class CameraParameters
{
    public:
        static const CameraParameters & get(double fx = 0.0, double fy = 0.0, double cx = 0.0, double cy = 0.0, unsigned int width = 0, unsigned int height = 0)
        {
            static const CameraParameters instance(fx, fy, cx, cy, width, height);
            return instance;
        }
        const double & fx() const
        {
            return fx_;
        }
        const double & fy() const
        {
            return fy_;
        }
        const double & cx() const
        {
            return cx_;
        }
        const double & cy() const
        {
            return cy_;
        }
        const unsigned int & width() const
        {
            return width_;
        }
        const unsigned int & height() const
        {
            return height_;
        }
    private:
        CameraParameters(double fx, double fy, double cx, double cy, unsigned int width, unsigned int height)
         : fx_(fx),
           fy_(fy),
           cx_(cx),
           cy_(cy),
           width_(width),
           height_(height)
        {
            assert(fx_ > 0.0 && fy_ > 0.0 && cx_ > 0.0 && cy_ > 0.0 && height_ > 0 && width_ > 0 && "Please set the camera parameters!");
        }
        const double fx_;
        const double fy_;
        const double cx_;
        const double cy_;
        const unsigned int width_;
        const unsigned int height_;
};

#endif
