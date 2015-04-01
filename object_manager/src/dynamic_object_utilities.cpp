#include "dynamic_object_utilities.h"
#include <QDir>

using namespace std;

std::vector<DynamicObject::Ptr> dynamic_object_utilities::loadDynamicObjects(std::string folder, bool verbose)
{
    std::vector<DynamicObject::Ptr>  objects;

    folder+=std::string("/");
    QString qfolder = folder.c_str();
    if (!QDir(qfolder).exists())
    {
        if (verbose)
        {
            cout<<"Folder "<<folder<<" does not exists. Cannot load dynamic objects."<<endl;
        }
        return objects;
    }

    QStringList objectFiles = QDir(qfolder).entryList(QStringList("*object*.xml"));

    for (size_t i=0; i<objectFiles.size(); i++)
    {
        string object_file = folder+objectFiles[i].toStdString();
        if (verbose)
        {
            cout<<"Now parsing object "<<object_file<<endl;
        }
        DynamicObjectXMLParser parser;
        DynamicObject::Ptr parsed = parser.loadFromXML(object_file);
        objects.push_back(parsed);
    }

    if (verbose)
    {
        cout<<"Parsed "<<objects.size()<<" from folder "<<folder<<endl;
    }

    return objects;
}
