#ifndef TRACK_LOADER_H
#define TRACK_LOADER_H

#include <iostream>
#include <fstream>
#include <GL/freeglut.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
using namespace std;

const int NPTS = 568;
float ptx[NPTS], ptz[NPTS];

//Reads flight path data from TrackPath.txt
void trackloader ()
{
    ifstream ifile;
    ifile.open("../Kei_Carden_A1/I_Railway/track2.txt");

        for (int i = 0; i < NPTS; i++){
            ifile >> ptx[i] >> ptz[i];
        }
    ifile.close();
}
#endif // TRACK_LODER_H

