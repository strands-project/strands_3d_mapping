#include "load_utilities.h"


std::vector<semantic_map_registration_features::RegistrationFeatures> semantic_map_registration_features::loadRegistrationFeaturesFromSingleSweep(std::string sweepXmlPath, bool verbose, std::string registrationFeaturesFilename)
{
    std::vector<semantic_map_registration_features::RegistrationFeatures> toRet;

    unsigned found = sweepXmlPath.find_last_of("/");
    std::string base_path = sweepXmlPath.substr(0,found+1);
    std::string regFile = base_path + registrationFeaturesFilename;

    cv::FileStorage fs2;
    if (!fs2.open(regFile,cv::FileStorage::READ))
    {
        std::cout<<"Could not open file "<<regFile<<" to read registration features."<<std::endl;
        return toRet;
    }

    int counter=0;
    while (true)
    {
        std::stringstream ss_k;ss_k<<"keypoints"<<counter;
        std::stringstream ss_d;ss_d<<"descriptors"<<counter;
        std::stringstream ss_v;ss_v<<"depths"<<counter;
        counter++;

        RegistrationFeatures reg;

        cv::FileNode kpFN = fs2[ss_k.str()];
        cv::read(kpFN, reg.keypoints);
        if (kpFN.empty() || kpFN.isNone())
        {
            break;
        }

        kpFN = fs2[ss_d.str()];
        cv::read(kpFN, reg.descriptors);
        kpFN = fs2[ss_v.str()];
        cv::read(kpFN, reg.depths);
        toRet.push_back(reg);
    }

    if (verbose)
    {
        std::cout<<"Read registration descriptors for "<<counter-1<<" intermediate images from sweep "<<sweepXmlPath<<std::endl;
    }

    return toRet;
}
