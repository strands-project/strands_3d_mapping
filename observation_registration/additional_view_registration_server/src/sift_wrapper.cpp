#include "additional_view_registration_server/sift_wrapper.h"

SIFTWrapper::SIFTWrapper(){
    const char *argv_sift[] = {"-m", "-fo","-1", "-s", "-v", "0", "-pack", "-cuda", "-maxd", "3840"};
    int argc_sift = sizeof(argv_sift)/sizeof(char*);
    sift = new SiftGPU;
    matcher = new SiftMatchGPU(4096 * 4);
    sift->ParseParam(argc_sift, (char **)argv_sift);

    if(sift->CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED){
        throw std::runtime_error("SIFT cannot create GL context");
        initialized = false;
    }

    matcher->VerifyContextGL();
    initialized = true;
//    std::cout<<"Sift object created "<<std::endl;
}

SIFTWrapper::~SIFTWrapper(){
    delete matcher;
    delete sift;
}

void SIFTWrapper::extractSIFT(const cv::Mat& image, int& num_desc, std::vector<float>& descriptors, std::vector<SiftGPU::SiftKeypoint>& keypoints){
    using namespace std;
    using namespace cv;

    if (!initialized){
        num_desc = -1;
        cout<<"SIFT object not initialized. Aborting."<<endl;
        return;
    }

    Mat image_gray;
    cvtColor(image,image_gray,CV_RGB2GRAY);

    // GO SIFT!!
    if (sift->RunSIFT(image_gray.cols,image_gray.rows,image_gray.data, GL_LUMINANCE, GL_UNSIGNED_BYTE ))
    {
        //get feature count
        num_desc = sift->GetFeatureNum();

        //allocate memory
        keypoints.resize(num_desc);
        descriptors.resize(128*num_desc);

        sift->GetFeatureVector(&keypoints[0], &descriptors[0]);
    }

    return;
}


void SIFTWrapper::matchSIFT(const int& num_desc1, const int& num_desc2,
               const std::vector<float>& desc1, const std::vector<float>& desc2,
               const std::vector<SiftGPU::SiftKeypoint>& keypoints1, const std::vector<SiftGPU::SiftKeypoint>& keypoints2,
               std::vector<std::pair<SiftGPU::SiftKeypoint, SiftGPU::SiftKeypoint>>& matches)
{
    using namespace std;
    using namespace cv;

    matcher->SetDescriptors(0, num_desc1, &desc1[0]); //image 1
    matcher->SetDescriptors(1, num_desc2, &desc2[0]); //image 2

    //match and get result.
    int (*match_buf)[2] = new int[num_desc1][2];

//        int num_match = matcher->GetSiftMatch(num1, match_buf,0.75, 0.8, 1);
    int num_match = matcher->GetSiftMatch(num_desc1, match_buf,0.5, 0.95, 1);
//    std::cout << num_match << " sift matches were found;\n";


    for(int j = 0; j < num_match; ++j)
    {
        //How to get the feature matches:
        SiftGPU::SiftKeypoint  key1 = keypoints1[match_buf[j][0]];
        SiftGPU::SiftKeypoint  key2 = keypoints2[match_buf[j][1]];

        matches.push_back(pair<SiftGPU::SiftKeypoint, SiftGPU::SiftKeypoint>(key1, key2));
    }

    delete match_buf;
    return;
}
