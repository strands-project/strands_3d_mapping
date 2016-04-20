#ifndef reglibMesh_H
#define reglibMesh_H

#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>


#include <iostream>
#include <fstream>

#include "../model/Model.h"
#include <pcl/visualization/pcl_visualizer.h>

namespace reglib
{

class Triangle;
class Vertex{
    public:
    float x;
    float y;
    float z;
    std::vector<Triangle*> triangles;

    Vertex(float x_, float y_, float z_);
    ~Vertex();
};

class Triangle{
    public:
    Vertex * a;
    Vertex * b;
    Vertex * c;
    Triangle(Vertex * a_, Vertex * b_, Vertex * c_);
    ~Triangle();
};



class Mesh{
        public:
        std::vector<Vertex*> vertexes;
        std::vector<Triangle*> triangles;

        Mesh();
        ~Mesh();

        void build(Model * model, int type);
        void show(pcl::visualization::PCLVisualizer * viewer);
        //void save(std::string path = "");
        //void print();
        //static Camera * load(std::string path);
    };

}

#endif // reglibMesh_H
