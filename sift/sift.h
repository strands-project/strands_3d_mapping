#include <opencv2/features2d/features2d.hpp>

/*!
 SIFT implementation.
 
 The class implements SIFT algorithm by D. Lowe.
*/

namespace cv
{

class CV_EXPORTS SIFT
{
public:
    struct CV_EXPORTS CommonParams
    {
        static const int DEFAULT_NOCTAVES = 4;
        static const int DEFAULT_NOCTAVE_LAYERS = 3;
        static const int DEFAULT_FIRST_OCTAVE = -1;
        enum { FIRST_ANGLE = 0, AVERAGE_ANGLE = 1 };

        CommonParams();
        CommonParams( int _nOctaves, int _nOctaveLayers, int /*_firstOctave*/, int /*_angleMode*/ );
        CommonParams( int _nOctaves, int _nOctaveLayers );
        int nOctaves, nOctaveLayers;
        int firstOctave; // it is not used now (firstOctave == 0 always)
        int angleMode;   // it is not used now
    };

    struct CV_EXPORTS DetectorParams
    {
        static double GET_DEFAULT_THRESHOLD() { return 0.04; }
        static double GET_DEFAULT_EDGE_THRESHOLD() { return 10.0; }

        DetectorParams();
        DetectorParams( double _threshold, double _edgeThreshold );
        double threshold, edgeThreshold;
    };

    struct CV_EXPORTS DescriptorParams
    {
        static double GET_DEFAULT_MAGNIFICATION() { return 3.0; }
        static const bool DEFAULT_IS_NORMALIZE = true;
        static const int DESCRIPTOR_SIZE = 128;

        DescriptorParams();
        DescriptorParams( double _magnification, bool /*_isNormalize*/, bool _recalculateAngles );
        DescriptorParams( bool _recalculateAngles );
        double magnification;
        bool isNormalize; // it is not used now (true always)
        bool recalculateAngles;
    };

    SIFT();
    //! sift-detector constructor
    SIFT( double _threshold, double _edgeThreshold,
          int _nOctaves=CommonParams::DEFAULT_NOCTAVES,
          int _nOctaveLayers=CommonParams::DEFAULT_NOCTAVE_LAYERS,
          int _firstOctave=CommonParams::DEFAULT_FIRST_OCTAVE,
          int _angleMode=CommonParams::FIRST_ANGLE );
    //! sift-descriptor constructor
    SIFT( double _magnification, bool _isNormalize=true,
          bool _recalculateAngles = true,
          int _nOctaves=CommonParams::DEFAULT_NOCTAVES,
          int _nOctaveLayers=CommonParams::DEFAULT_NOCTAVE_LAYERS,
          int _firstOctave=CommonParams::DEFAULT_FIRST_OCTAVE,
          int _angleMode=CommonParams::FIRST_ANGLE );
    SIFT( const CommonParams& _commParams,
          const DetectorParams& _detectorParams = DetectorParams(),
          const DescriptorParams& _descriptorParams = DescriptorParams() );

    //! returns the descriptor size in floats (128)
    int descriptorSize() const;
    //! finds the keypoints using SIFT algorithm
    void operator()(const Mat& img, const Mat& mask,
                    vector<KeyPoint>& keypoints) const;
    //! finds the keypoints and computes descriptors for them using SIFT algorithm.
    //! Optionally it can compute descriptors for the user-provided keypoints
    void operator()(const Mat& img, const Mat& mask,
                    vector<KeyPoint>& keypoints,
                    Mat& descriptors,
                    bool useProvidedKeypoints=false) const;

    CommonParams getCommonParams () const;
    DetectorParams getDetectorParams () const;
    DescriptorParams getDescriptorParams () const;

protected:
    CommonParams commParams;
    DetectorParams detectorParams;
    DescriptorParams descriptorParams;
};

class CV_EXPORTS SiftFeatureDetector : public FeatureDetector
{
public:
    SiftFeatureDetector( const SIFT::DetectorParams& detectorParams=SIFT::DetectorParams(),
                         const SIFT::CommonParams& commonParams=SIFT::CommonParams() );
    SiftFeatureDetector( double threshold, double edgeThreshold,
                         int nOctaves=SIFT::CommonParams::DEFAULT_NOCTAVES,
                         int nOctaveLayers=SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS,
                         int firstOctave=SIFT::CommonParams::DEFAULT_FIRST_OCTAVE,
                         int angleMode=SIFT::CommonParams::FIRST_ANGLE );
    virtual void read( const FileNode& fn );
    virtual void write( FileStorage& fs ) const;

protected:
    virtual void detectImpl( const Mat& image, vector<KeyPoint>& keypoints, const Mat& mask=Mat() ) const;

    SIFT sift;
};

/*
 * SiftDescriptorExtractor
 */
class CV_EXPORTS SiftDescriptorExtractor : public DescriptorExtractor
{
public:
    SiftDescriptorExtractor( const SIFT::DescriptorParams& descriptorParams=SIFT::DescriptorParams(),
                             const SIFT::CommonParams& commonParams=SIFT::CommonParams() );
    SiftDescriptorExtractor( double magnification, bool isNormalize=true, bool recalculateAngles=true,
                             int nOctaves=SIFT::CommonParams::DEFAULT_NOCTAVES,
                             int nOctaveLayers=SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS,
                             int firstOctave=SIFT::CommonParams::DEFAULT_FIRST_OCTAVE,
                             int angleMode=SIFT::CommonParams::FIRST_ANGLE );

    virtual void read( const FileNode &fn );
    virtual void write( FileStorage &fs ) const;

    virtual int descriptorSize() const;
    virtual int descriptorType() const;

protected:
	virtual void computeImpl( const Mat& image, vector<KeyPoint>& keypoints, Mat& descriptors ) const;
    
    SIFT sift;
};

}
