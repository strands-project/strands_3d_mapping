#include <semantic_map/reg_features.h>

RegistrationFeatures::RegistrationFeatures(bool verbose, std::string data_filename)
{
    m_verbose = verbose;
    m_dataFilename = data_filename;
}

RegistrationFeatures::~RegistrationFeatures()
{

}
