//  ========================================================================
//  COSC363: Computer Graphics (2022)
//
//  FILE NAME: RailWay.cpp
//  ========================================================================

#include <iostream>
#include <fstream>
#include <cmath>
#include <GL/freeglut.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "trackLoder.h"
#include "RailModels.h"
#include "loadBMP.h"
#include "loadTGA.h"

using namespace std;

#define PI 3.1415926
#define TRAIN_LENGTH 3
float angle = -40;		//Scene rotation angle
int option = 0;			//View modes:  1 = model view, 2 = room view

GLUquadric *q;
GLuint txId[5];
float signalArmAngle = 45;
int indx = 0;

typedef struct {
    float *x, *y, *z;					//vertex coordinates
    int *nv, *t1, *t2, *t3, *t4;		//number of vertices and vertex indices of each face
} model_t;

model_t models[2];
//--Globals ---------------------------------------------------------------
int nvert, nface;					//total number of vertices and faces
float angleX = 10.0,  angleY = -20;	//Rotation angles about x, y axes
float xmin, xmax, ymin, ymax;		//min, max values of  object coordinates
float eye_angle=0, look_x = 0, look_z=-100., eye_x = 0, eye_y = 15, eye_z = 0;  //Camera parameters
int ariveAtStaion = 0, signalTimer = 0;
int change = 0;
int newPointCount = 0;

float ptxNew[NPTS*4], ptzNew[NPTS*4];

//------Function to load a texture in bmp format  ------------------------
void loadTexture(int i, const char* filename, int imgType)
{
    glGenTextures(2, &txId[i]); 				// Create a Texture object
    glBindTexture(GL_TEXTURE_2D, txId[i]);		//Use this texture

    if (!imgType)
        loadBMP(filename);
    else
        loadTGA(filename);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);	//Set texture parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
}

//-- Computes the min, max values of coordinates  -----------------------
void computeMinMax(int modelNum)
{
    xmin = xmax = models[modelNum].x[0];
    ymin = ymax = models[modelNum].y[0];
    for(int i = 1; i < nvert; i++)
    {
        if(models[modelNum].x[i] < xmin) xmin = models[modelNum].x[i];
        else if(models[modelNum].x[i] > xmax) xmax = models[modelNum].x[i];
        if(models[modelNum].y[i] < ymin) ymin = models[modelNum].y[i];
        else if(models[modelNum].y[i] > ymax) ymax = models[modelNum].y[i];
    }
}

//-- Loads mesh data in OFF format    -------------------------------------
void loadMeshFile(const char* fname, int modelNum)
{
    ifstream fp_in;
    int ne;

    fp_in.open(fname, ios::in);
    if(!fp_in.is_open())
    {
        cout << "Error opening mesh file" << endl;
        exit(1);
    }

    fp_in.ignore(INT_MAX, '\n');				//ignore first line
    fp_in >> nvert >> nface >> ne;			    // read number of vertices, polygons, edges (not used)

    models[modelNum].x = new float[nvert];                        //create arrays
    models[modelNum].y = new float[nvert];
    models[modelNum].z = new float[nvert];

    models[modelNum].nv = new int[nface];
    models[modelNum].t1 = new int[nface];
    models[modelNum].t2 = new int[nface];
    models[modelNum].t3 = new int[nface];
    models[modelNum].t4 = new int[nface];

    for(int i=0; i < nvert; i++)                         //read vertex list
        fp_in >> models[modelNum].x[i] >> models[modelNum].y[i] >> models[modelNum].z[i];

    for(int i=0; i < nface; i++)                         //read polygon list
    {
        fp_in >> models[modelNum].nv[i] >> models[modelNum].t1[i] >> models[modelNum].t2[i] >> models[modelNum].t3[i];
        if (models[modelNum].nv[i] == 4)
            fp_in >> models[modelNum].t4[i];
        else
            models[modelNum].t4[i] = -1;
    }

    fp_in.close();
    cout << " File successfully read." << endl;

   computeMinMax(modelNum);						    //Compute min, max values of x, y coordinates for defining camera frustum
}

//--Function to compute the normal vector of a triangle with index indx ----------
void normal(int indx, int modelNum)
{
    float x1 = models[modelNum].x[models[modelNum].t1[indx]], x2 = models[modelNum].x[models[modelNum].t2[indx]], x3 = models[modelNum].x[models[modelNum].t3[indx]];
    float y1 = models[modelNum].y[models[modelNum].t1[indx]], y2 = models[modelNum].y[models[modelNum].t2[indx]], y3 = models[modelNum].y[models[modelNum].t3[indx]];
    float z1 = models[modelNum].z[models[modelNum].t1[indx]], z2 = models[modelNum].z[models[modelNum].t2[indx]], z3 = models[modelNum].z[models[modelNum].t3[indx]];
    float nx, ny, nz;
    nx = y1*(z2-z3) + y2*(z3-z1) + y3*(z1-z2);
    ny = z1*(x2-x3) + z2*(x3-x1) + z3*(x1-x2);
    nz = x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2);
    glNormal3f(nx, ny, nz);
}

void equidistantPoints()
{

    int i = 0, pointNum = 0;  //index of the original array
    glm::vec2 currentPoint(ptx[0], ptz[0]);
    glm::vec2 previousPoint(0, 0);
    float d, d1 = 0, d2, dreq = 1;

    for (int i = 0; i <= 100; i++) {
        glm::vec2 p0(ptx[i], ptz[i]);
        glm::vec2 p1(ptx[i + 1], ptz[i + 1]);

        dreq += glm::distance(p1, p0);

    }
    dreq = 2;

    while (i <= NPTS)
    {
            pointNum = i;

            if (pointNum == NPTS)
                pointNum = 0;

            glm::vec2 nextPoint(ptx[pointNum], ptz[pointNum]);
            d = glm::distance(currentPoint, nextPoint);
            if (d < dreq)
            {
                    d1 = d;
                    previousPoint = nextPoint;
                    i++;
            }
            else
            {
                    d2 = d;
                    currentPoint = previousPoint + (dreq - d1)*(nextPoint - previousPoint) / (d2 - d1);
                    ptxNew[newPointCount] = currentPoint[0];  //Store values
                    ptzNew[newPointCount] = currentPoint[1];
                    newPointCount++;
                    d1 = 0;
                    previousPoint = currentPoint;
            }
    }

    cout << newPointCount << endl;
}


void renderModel(int modelNum)
{

    glEnable(GL_TEXTURE_2D);
    glEnable(GL_NORMALIZE);

    if (modelNum == 1)
        glBindTexture(GL_TEXTURE_2D, txId[4]);
    else
        glBindTexture(GL_TEXTURE_2D, txId[0]);

    float texturePoint1;
    float texturePoint2;
    float texturePoint3;
    float texturePoint4;
     for(int indx = 0; indx < nface; indx++)		//draw each face
     {
         if (modelNum == 0) {
             texturePoint2 = 1;
             switch (indx) {
                 case 0:
                 case 1:
                 case 2:
                 case 3:
                 case 4:
                     texturePoint1 = 0;
                     break;
                 case 5:
                 case 6:
                 case 7:
                 case 8:
                     texturePoint1 = 0.2;
                     break;
                 case 12:
                 case 13:
                     texturePoint1 = 0.2;
                     texturePoint2 = 0.5;
                     break;
                 default:
                     texturePoint1 = 0.4;
                     break;
             }
          } else {
             glColor4f(1.0, 0.5, 1.0, 1.0);
             texturePoint2 = 1;
             switch (indx) {
                 case 0:
                 case 1:
                 case 2:
                 case 3:
                 case 4:
                     texturePoint1 = 0;
                     break;
                 case 5:
                 case 6:
                 case 7:
                 case 8:
                     texturePoint1 = 0.2;
                     break;
                 case 12:
                 case 13:
                     texturePoint1 = 0.2;
                     texturePoint2 = 0.5;
                     break;
                 default:
                     texturePoint1 = 0.4;
                     break;
             }
             glDisable(GL_TEXTURE_2D);
         }

         normal(indx, modelNum);
         if (models[modelNum].nv[indx] == 3)	glBegin(GL_TRIANGLES);
         else				glBegin(GL_QUADS);
             glTexCoord2f(0.0, 0.0 + texturePoint1); glVertex3d(models[modelNum].x[models[modelNum].t1[indx]], models[modelNum].y[models[modelNum].t1[indx]], models[modelNum].z[models[modelNum].t1[indx]]);
             glTexCoord2f(1.0, texturePoint1); glVertex3d(models[modelNum].x[models[modelNum].t2[indx]], models[modelNum].y[models[modelNum].t2[indx]], models[modelNum].z[models[modelNum].t2[indx]]);
             glTexCoord2f(1.0 * texturePoint2, 0.2 + texturePoint1); glVertex3d(models[modelNum].x[models[modelNum].t3[indx]], models[modelNum].y[models[modelNum].t3[indx]], models[modelNum].z[models[modelNum].t3[indx]]);
             if(models[modelNum].nv[indx]==4)
               glTexCoord2f(0.0, 0.2 + texturePoint1); glVertex3d(models[modelNum].x[models[modelNum].t4[indx]], models[modelNum].y[models[modelNum].t4[indx]], models[modelNum].z[models[modelNum].t4[indx]]);
         glEnd();
     }
     glDisable(GL_TEXTURE_2D);
}

void normal2( float x1, float y1, float z1,
    float x2, float y2, float z2,
    float x3, float y3, float z3 )
    {
        float nx, ny, nz;
        nx = y1*(z2-z3) + y2*(z3-z1) + y3*(z1-z2);
        ny = z1*(x2-x3) + z2*(x3-x1) + z3*(x1-x2);
        nz = x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2);
        glNormal3f(nx, ny, nz);
}

void floor()
{

    float white[4] = {1., 1., 1., 1.};
    float black[4] = {0};
    glColor4f(0.7, 0.7, 0.7, 1.0);  //The floor is gray in colour


    glNormal3f(0.0, 1.0, 0.0);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, txId[2]);
    //The floor is made up of several tiny squares on a 400 x 400 grid. Each square has a unit size.
    glMaterialfv(GL_FRONT, GL_SPECULAR, black);
    glBegin(GL_QUADS);
    for(int i = -200; i < 200; i++)
    {
        for(int j = -200;  j < 200; j++)
        {
            float txtPt1 = (((float)i + (float)200)/ (float)400);
            float txtPt2 = (((float)j + (float)200)/ (float)400);
            glTexCoord2f(txtPt1, txtPt2); glVertex3f(i, 0, j);
            glTexCoord2f(txtPt1 + 0.0025, txtPt2 ); glVertex3f(i, 0, j+1);
            glTexCoord2f(txtPt1 + 0.0025, txtPt2 + 0.0025); glVertex3f(i+1, 0, j+1);
            glTexCoord2f(txtPt1, txtPt2 + 0.0025); glVertex3f(i+1, 0, j);
        }
    }
    glEnd();
    glMaterialfv(GL_FRONT, GL_SPECULAR, white);
    glDisable(GL_TEXTURE_2D);

}

void tracks()
{
    int exceedPointsFix;
    for (int i = 0; i <= newPointCount-1; i++) {

        exceedPointsFix = i;

        glm::vec2 pts1(ptxNew[i], ptzNew[i]);
        if ((exceedPointsFix + 1) >= newPointCount-1)
            exceedPointsFix = 0;
        glm::vec2 pts2(ptxNew[exceedPointsFix + 1], ptzNew[exceedPointsFix + 1]);
        if ((exceedPointsFix + 2) >= newPointCount-1)
            exceedPointsFix = 0;
        glm::vec2 pts3(ptxNew[exceedPointsFix + 2], ptzNew[exceedPointsFix + 2]);

        glm::vec2 unitVector1 = glm::normalize(pts2 - pts1);
        glm::vec2 unitVector2 = glm::normalize(pts3 - pts2);

        glm::vec2 tangentToUnit1(unitVector1[1], -unitVector1[0]);
        glm::vec2 tangentToUnit2(unitVector2[1], -unitVector2[0]);

        float trackMin = 4.5;
        float trackMax = 5;

        glm::vec2 a1 = pts1 + tangentToUnit1*trackMin;
        glm::vec2 a2 = pts1 + tangentToUnit1*trackMax;

        glm::vec2 b1 = pts2 + tangentToUnit2*trackMin;
        glm::vec2 b2 = pts2 + tangentToUnit2*trackMax;

        glm::vec2 c1 = pts1 + -tangentToUnit1*trackMin;
        glm::vec2 c2 = pts1 + -tangentToUnit1*trackMax;

        glm::vec2 d1 = pts2 + -tangentToUnit2*trackMin;
        glm::vec2 d2 = pts2 + -tangentToUnit2*trackMax;

        float sleeperLength = 6;
        float sleeperWidth = 2;

        glm::vec2 p1 = pts1 + tangentToUnit1*sleeperLength;
        glm::vec2 p2 = pts1 + -tangentToUnit1*sleeperLength;
        glm::vec2 p3 = p1 + unitVector1 * sleeperWidth;
        glm::vec2 p4 = p2 + unitVector1 * sleeperWidth;


        if (i % 6 == 0) {
            glColor4f(0.33, 0.22, 0.09, 1.0);
            //glEnable(GL_TEXTURE_2D);
            //glBindTexture(GL_TEXTURE_2D, txId[3]);
            glBegin(GL_QUADS);
                glNormal3f(0., 1., 0.);       //Quad 1 facing up
                //glTexCoord2f(0, 0);
                glVertex3f(p1[0], 0.1, p1[1]);
                //glTexCoord2f(1, 0);
                glVertex3f(p3[0], 0.1, p3[1]);
                //glTexCoord2f(1, 1);
                glVertex3f(p4[0], 0.1, p4[1]);
                //glTexCoord2f(0, 1);
                glVertex3f(p2[0], 0.1, p2[1]);
            glEnd();
            //glDisable(GL_TEXTURE_2D);
        }

        //float angle1 = atan2(-unitVector1[1], -unitVector1[0]);
        float angle1 = (atan2(ptzNew[i + 1] - ptzNew[i], -(ptxNew[i + 1] - ptxNew[i])) * 180) / PI;
        float angle2 = atan2(-unitVector2[1], -unitVector2[0]);

        float ca1 = cos(angle1); float ca2 = cos(angle2);
        float sa1 = sin(angle1); float sa2 = sin(angle2);

        glColor4f(0.35, 0.36, 0.37, 1.0);
        glBegin(GL_QUADS);
//        for (int b = 0; b < 2; b++)   //Two parallel tracks (radius = inRad, outRad)
//        {

                glNormal3f(0., 1., 0.);       //Quad 1 facing up
                glVertex3f(a1[0], 1.0, a1[1]);
                glVertex3f(a2[0], 1.0, a2[1]);
                glVertex3f(b2[0], 1.0, b2[1]);
                glVertex3f(b1[0], 1.0, b1[1]);

                normal2(a1[0], 0, a1[1], a1[0], 1, a1[1], b1[0], 1, b1[1]);   //Quad 2 facing inward
                glVertex3f(a1[0], 0.0, a1[1]);
                glVertex3f(a1[0], 1.0, a1[1]);
                normal2(b1[0], 1, b1[1], b1[0], 0, b1[1], a1[0], 0, a1[1]);
                glVertex3f(b1[0], 1.0, b1[1]);
                glVertex3f(b1[0], 0.0, b1[1]);

                normal2(a2[0], 0, a2[1], a2[0], 1, a2[1], b2[0], 1, b2[1]);   //Quad 3 facing outward
                glVertex3f(a2[0], 1.0, a2[1]);
                glVertex3f(a2[0], 0.0, a2[1]);
                normal2(b2[0], 1, b2[1], b2[0], 0, b2[1], a2[0], 0, a2[1]);
                glVertex3f(b2[0], 0.0, b2[1]);
                glVertex3f(b2[0], 1.0, b2[1]);

                glNormal3f(0., 1., 0.);       //Quad 1 facing up
                glVertex3f(c1[0], 1.0, c1[1]);
                glVertex3f(c2[0], 1.0, c2[1]);
                glVertex3f(d2[0], 1.0, d2[1]);
                glVertex3f(d1[0], 1.0, d1[1]);

                normal2(c1[0], 0, c1[1], c1[0], 1, c1[1], d1[0], 1, d1[1]);   //Quad 2 facing inward
                glVertex3f(c1[0], 0.0, c1[1]);
                glVertex3f(c1[0], 1.0, c1[1]);
                normal2(d1[0], 1, d1[1], d1[0], 0, d1[1], c1[0], 0, c1[1]);
                glVertex3f(d1[0], 1.0, d1[1]);
                glVertex3f(d1[0], 0.0, d1[1]);

                normal2(c2[0], 0, c2[1], c2[0], 1, c2[1], d2[0], 1, d2[1]);   //Quad 3 facing outward
                glVertex3f(c2[0], 1.0, c2[1]);
                glVertex3f(c2[0], 0.0, c2[1]);
                normal2(d2[0], 1, d2[1], d2[0], 0, d2[1], c2[0], 0, c2[1]);
                glVertex3f(d2[0], 0.0, d2[1]);
                glVertex3f(d2[0], 1.0, d2[1]);

//        }
        glEnd();
    }

}

void cargo()
{
    glColor4f(1, 1.0, 1.0, 1.0);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, txId[0]);
    glBegin(GL_QUADS);

        // Sides
        glNormal3f(-1, 0.0, 0);
        glTexCoord2f(0.0, 0.8); glVertex3f(-9, -5, -5);
        glTexCoord2f(1, 0.8); glVertex3f(9, -5, -5);
        glTexCoord2f(1, 1); glVertex3f(9, 5, -5);
        glTexCoord2f(0.0, 1); glVertex3f(-9, 5, -5);

        glNormal3f(1, 0.0, 0);
        glTexCoord2f(0.0, 0.8); glVertex3f(-9, -5, 5);
        glTexCoord2f(1, 0.8); glVertex3f(9, -5, 5);
        glTexCoord2f(1, 1); glVertex3f(9, 5, 5);
        glTexCoord2f(0.0, 1); glVertex3f(-9, 5, 5);


        // SmallSides
        glNormal3f(0, 0.0, -1);
        glTexCoord2f(0.47, 0.6); glVertex3f(-9, -5, -5);
        glTexCoord2f(0.81, 0.6); glVertex3f(-9, -5, 5);
        glTexCoord2f(0.81, 0.8); glVertex3f(-9, 5, 5);
        glTexCoord2f(0.47, 0.8); glVertex3f(-9, 5, -5);

        glNormal3f(0, 0.0, 1);
        glTexCoord2f(0.47, 0.6); glVertex3f(9, -5, -5);
        glTexCoord2f(0.81, 0.6); glVertex3f(9, -5, 5);
        glTexCoord2f(0.81, 0.8); glVertex3f(9, 5, 5);
        glTexCoord2f(0.47, 0.8); glVertex3f(9, 5, -5);


        // Top
        glColor4f(0.0, 0.0, 0.0, 1.0);
        glNormal3f(0, 1, 0);
        glVertex3f(-9, 5, -5);
        glVertex3f(9, 5, -5);
        glVertex3f(9, 5, 5);
        glVertex3f(-9, 5, 5);
    glEnd();
    glDisable(GL_TEXTURE_2D);
}

void wagon()
{
    base();

    glPushMatrix();
        glTranslatef(0.0, 10.0, 0.0);
        cargo();
    glPopMatrix();
}

void signalArm()
{
    glColor4f(1, 1.0, 1.0, 1.0);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, txId[0]);
    glBegin(GL_QUADS);
        glNormal3f(0, 0, 1);
        glTexCoord2f(0.93, 0.6); glVertex3f(0, 0, 0);
        glTexCoord2f(1, 0.6); glVertex3f(0, 2, 0);
        glTexCoord2f(1, 0.8); glVertex3f(-20, 2, 0);
        glTexCoord2f(0.93, 0.8); glVertex3f(-20, 0, 0);
    glEnd();
    glDisable(GL_TEXTURE_2D);
}

void signal()
{

    if (signalTimer > 0 && signalArmAngle > 0)
        signalArmAngle -= 0.45;
    else if (signalTimer == 0 && signalArmAngle < 45)
        signalArmAngle += 0.45;


    glPushMatrix();
        glTranslatef(8, 5, 1);
        glRotatef(-signalArmAngle, 0, 0, 1);
        signalArm();
    glPopMatrix();


    if (signalTimer == 0)
        change = 0;
    else if (signalTimer % 10 == 0) {
        change += 1;
        if (change == 3)
            change = 0;
    }

    glEnable(GL_TEXTURE_2D);
    glEnable(GL_ALPHA_TEST);
    glAlphaFunc(GL_GREATER,0);
    glBindTexture(GL_TEXTURE_2D, txId[1]);

    glBegin(GL_QUADS);
        glNormal3f(0, 0, 1);
        glTexCoord2f(change * 0.333, 0); glVertex3f(0, 0, 0);
        glTexCoord2f(change * 0.333 + 0.333, 0); glVertex3f(10, 0, 0);
        glTexCoord2f(change * 0.333 + 0.333, 1); glVertex3f(10, 20, 0);
        glTexCoord2f(change * 0.333, 1); glVertex3f(0, 20, 0);
    glEnd();

    glDisable(GL_TEXTURE_2D);
    glDisable(GL_ALPHA_TEST);
}


void tunnel()
{
    glColor4f(1, 1.0, 1.0, 1.0);
    const int npoints = 12;
    //float vx[npoints] = {10, -20, -20, -10, -6, -4, -2, 0, 10};
    //float vy[npoints] = {30, 30, 25, 25, 22, 19, 13, 0, 0};
    float vx[npoints] = {30, 30, 28, 22, 18, 12, -12, -18, -22, -28, -30, -30};
    float vy[npoints] = {0, 15, 20, 25, 28, 30, 30, 28, 25, 20, 15, 0};

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, txId[0]);


//    for (int n = 0; n < 2; n++) {
//        glBegin(GL_TRIANGLE_FAN);
//            glVertex3f(vx[0], vy[0], n*100);
//            for (int i = 1; i < npoints; i++)
//            {
//                glVertex3f(vx[i], vy[i], n*100);
//            }
//        glEnd();
//     }


        glBegin(GL_QUADS);
        for (int i = 0; i < npoints-1; i++)
        {
            normal2(vx[i], vy[i], 0, vx[i + 1], vy[i + 1], 0, vx[i], vy[i], 100);
            glTexCoord2f(0.0, 0.0); glVertex3f(vx[i], vy[i], 0);
            glTexCoord2f(1, 0.0); glVertex3f(vx[i], vy[i], 100);
            glTexCoord2f(1, 0.2); glVertex3f(vx[i+ 1], vy[i + 1], 100);
            glTexCoord2f(0, 0.2); glVertex3f(vx[i + 1], vy[i + 1], 0);
        }
//            glNormal3f(1, 0.0, 0);
//            glTexCoord2f(0.0, 0.0); glVertex3f(vx[8], vy[8], 0);
//            glTexCoord2f(1, 0.0); glVertex3f(vx[8], vy[8], 100);
//            glTexCoord2f(0, 0.2); glVertex3f(vx[0], vy[0], 0);
//            glTexCoord2f(1, 0.2); glVertex3f(vx[0], vy[0], 100);
        glEnd();
        glDisable(GL_TEXTURE_2D);

}

void drawRoom()
{

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, txId[3]);
    glEnable(GL_LIGHTING);
    glBegin(GL_QUADS);			//4 walls
      glColor3f(0.75, 0.75, 0.75);
      glNormal3f(0, 0, 1);
      glTexCoord2f(0, 0); glVertex3f(-200, 0, -200);
      glTexCoord2f(0.25, 0); glVertex3f(200, 0, -200);
      glTexCoord2f(0.25, 1); glVertex3f(200, 140, -200);
      glTexCoord2f(0, 1); glVertex3f(-200, 140, -200);

      glNormal3f(0, 0, -1);
      glTexCoord2f(0.5, 0); glVertex3f(-200, 0, 200);
      glTexCoord2f(0.75, 0); glVertex3f(200, 0, 200);
      glTexCoord2f(0.75, 1); glVertex3f(200, 140, 200);
      glTexCoord2f(0.5, 1); glVertex3f(-200, 140, 200);

      glColor3f(1, 0.75, 0.75);
      glNormal3f(1, 0, 0);
      glTexCoord2f(0.75, 0); glVertex3f(-200, 0, -200);
      glTexCoord2f(0.75, 1); glVertex3f(-200, 140, -200);
      glTexCoord2f(1, 1); glVertex3f(-200, 140, 200);
      glTexCoord2f(1, 0); glVertex3f(-200, 0, 200);

      glNormal3f(-1, 0, 0);
      glTexCoord2f(0.25, 0); glVertex3f(200, 0, -200);
      glTexCoord2f(0.25, 1); glVertex3f(200, 140, -200);
      glTexCoord2f(0.75, 1); glVertex3f(200, 140, 200);
      glTexCoord2f(0.75, 0); glVertex3f(200, 0, 200);
    glEnd();
    glDisable(GL_TEXTURE_2D);

   glBegin(GL_QUADS);
    glColor3f(.28, .48, .64);
    glNormal3f(0, -1, 0);
    glTexCoord2f(0, 0); glVertex3f(-200, 140, 200);
    glTexCoord2f(1, 0); glVertex3f(200, 140, 200);
    glTexCoord2f(1, 1); glVertex3f(200, 140, -200);
    glTexCoord2f(0, 1); glVertex3f(-200, 140, -200);
  glEnd();



}

//---------------------------------------------
void initialise(void)
{
    float model_wid, model_hgt;
    loadMeshFile("../Kei_Carden_A1/I_Railway/trainStation.off", 0);			//Specify mesh file name here
    loadMeshFile("../Kei_Carden_A1/I_Railway/car.off", 1);

    q = gluNewQuadric();
    eye_y = 20;
    trackloader();
    loadTexture(0, "../Kei_Carden_A1/I_Railway/textures.bmp", 0);
    loadTexture(1,"../Kei_Carden_A1/I_Railway/crossing.tga", 1);
    loadTexture(2,"../Kei_Carden_A1/I_Railway/feild.bmp", 0);
    loadTexture(3,"../Kei_Carden_A1/I_Railway/skybox.bmp", 0);
    loadTexture(4,"../Kei_Carden_A1/I_Railway/car.bmp", 0);

    equidistantPoints();

    model_wid = xmax-xmin;						//Model width and height
    model_hgt = ymax-ymin;
    xmin -= 0.2*model_wid;						//Extend minmax window further by 20% of its size.
    xmax += 0.2*model_wid;
    ymin -= 0.2*model_hgt;
    ymax += 0.2*model_hgt;



    float grey[4] = {0.2, 0.2, 0.2, 1.0};
    float white[4]  = {1.0, 1.0, 1.0, 1.0};
    float black[4] = { 0, 0, 0, 1 };

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    //glEnable(GL_LIGHT1);

    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);

    glMaterialfv(GL_FRONT, GL_SPECULAR, white);
    glMaterialf(GL_FRONT, GL_SHININESS, 50);

//	Define light's ambient, diffuse, specular properties
    glLightfv(GL_LIGHT0, GL_AMBIENT, grey);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, white);
    glLightfv(GL_LIGHT0, GL_SPECULAR, white);

    glLightfv(GL_LIGHT1, GL_AMBIENT, grey);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, white);
    glLightfv(GL_LIGHT1, GL_SPECULAR, white);

    glLightf(GL_LIGHT1, GL_SPOT_CUTOFF, 30.0);
    glLightf(GL_LIGHT1, GL_SPOT_EXPONENT,0.01);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_AMBIENT, grey);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, white);
    glLightfv(GL_LIGHT0, GL_SPECULAR, white);

    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);

    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, black);
    glClearColor (0.0, 0.0, 0.0, 0.0);

    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    gluPerspective(80.0, 1.0, 10.0, 500.0);
}

void train()
{
    int modelLocation = indx;
    for (int modelIndex = 0; modelIndex <= TRAIN_LENGTH; modelIndex++) {

        if (modelIndex > 0) {
            modelLocation -= 12;
        }

        if ((modelLocation - 1) < 0)
            modelLocation = newPointCount + modelLocation-1;

        glm::vec2 pts1(ptxNew[modelLocation - 1], ptzNew[modelLocation - 1]);
        glm::vec2 icrr(ptxNew[modelLocation], ptzNew[modelLocation]);

        if ((modelLocation + 1) >= newPointCount-1)
            modelLocation = -1;
        glm::vec2 pts2(ptxNew[modelLocation + 1], ptzNew[modelLocation + 1]);
        glm::vec2 distVec = pts2 - pts1;

        glm::vec2 unitVector = glm::normalize(distVec);
        float modelAngle = (atan2(unitVector[1], -unitVector[0]) * 180)/PI;

        glPushMatrix();
            glTranslatef(icrr[0], 1, icrr[1]);
            glRotatef(modelAngle, 0, 1, 0);

            if(modelIndex == 0) {
                engine();
            } else {
                wagon();
            }


        glPopMatrix();
    }
}


//------------------------------------------
void display(void)
{
    float lgt_pos[] = {0.0f, 50.0f, 0.0f, 1.0f};  //light0 position (directly above the origin)

    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    if (option == 1)
       gluLookAt (30, 15, 125, 0, 15, 0, 0, 1.0, 0.0);
    //else if (option == 2)
        //gluLookAt (ptxNew[indx] + ptxNew[indx] * 0.5, 20, + ptzNew[indx] * 0.5, ptxNew[indx+1], 20, ptzNew[indx+1], 0.0, 1.0, 0.0); // Cinamatic
        //gluLookAt (ptxNew[indx], 20, ptzNew[indx], ptxNew[indx+1], 20, ptzNew[indx+1], 0.0, 1.0, 0.0);
    else {
        gluLookAt(eye_x, eye_y, eye_z,  look_x, 15, look_z,   0, 1, 0);
        glRotatef(eye_angle, 0, 1, 0);
    }


    glLightfv(GL_LIGHT0, GL_POSITION, lgt_pos);   //light position



    floor();
    drawRoom();
    tracks();

    static float carPos = 50;
    static int across = 0;
    static int rotate = 0;
    if ( carPos <= 145 && across == 2) {
        rotate = 0;
        carPos += 0.5;
    }
    else if (carPos > 50 && across == 4) {
        rotate = 180;
        carPos -= 0.5;
    }

    if (signalArmAngle > 20 && signalArmAngle < 21)
        across++;
    else if (across == 6)
        across = 0;

    glPushMatrix();
        glTranslatef(-15, 0, carPos);
        glRotatef(rotate, 0, 1, 0);
        renderModel(1);
    glPopMatrix();

    glPushMatrix();
        glTranslatef(ptxNew[40], 0, ptzNew[40]);
        glRotatef(-10, 0, 1, 0);
        tunnel();
    glPopMatrix();

    glPushMatrix();
        glTranslatef(ptxNew[650], 0, ptzNew[650]);
        float trainLineAngle = (atan2(ptzNew[650] - ptzNew[649], -(ptxNew[650] - ptxNew[649])) * 180) / PI;
        glRotatef(trainLineAngle, 0, 1, 0);
        glTranslatef(0, 0, 15);
        signal();
        glTranslatef(-20, 0, -50);
        glRotatef(180, 0, 1, 0);
        signal();
    glPopMatrix();

    glPushMatrix();
        glTranslatef(30, 0, 140);
        glRotatef(90, 0, 1, 0);
        renderModel(0);
    glPopMatrix();

    train();

    glutSwapBuffers();
}



//  Keyboard call-back function
//  Used for selecting a view mode
void keyboard(unsigned char key, int x, int y)
{
    static int viewSelection = 0;
    if (key == 'c')
        viewSelection++;

    switch(viewSelection) {
        case 1:
            option = 1;
            break;
        case 2:
            option = 2;
            viewSelection = 0;
            break;
        case 3:
            viewSelection = 0;
        default:
            option = 0;
            break;
    }
    glutPostRedisplay();
}


//  Special keyboard call-back function
//  Used for rotating the scene about y-axis
void special(int key, int x, int y)
{
    if(key == GLUT_KEY_LEFT) eye_angle -= 0.1;  //Change direction
    else if(key == GLUT_KEY_RIGHT) eye_angle += 0.1;
    else if(key == GLUT_KEY_DOWN)
    {  //Move backward
        eye_x -= 5*sin(eye_angle);
        eye_z += 5*cos(eye_angle);

    }
    else if(key == GLUT_KEY_UP)
    { //Move forward
        eye_x += 5*sin(eye_angle);
        eye_z -= 5*cos(eye_angle);

    } else if (key == GLUT_KEY_PAGE_UP) {
        eye_y += 5;//*tan(eye_angle);
    } else if (key == GLUT_KEY_PAGE_DOWN) {
        eye_y -= 5;//*tan(eye_angle);
    }

    look_x = eye_x + 100*sin(eye_angle);
    look_z = eye_z - 100*cos(eye_angle);
    glutPostRedisplay();
}

void myTimer (int value)
{
    if (indx == 600) {
        ariveAtStaion++;
        if(ariveAtStaion == 100) {
            ariveAtStaion = 0;
            indx++;
        }
    }
    else if(indx >= (newPointCount-1)) {
        indx = 0;
    } else if (ariveAtStaion == 0) {
        indx++;
    }

    if ((indx > 150) && (indx < 300) || (indx > 630) && (indx < 700))
        signalTimer++;
    else
        signalTimer = 0;

    glutPostRedisplay();
    glutTimerFunc(1, myTimer, value);
}

//----------------------------------------------
int main(int argc, char** argv)
{
   glutInit(&argc, argv);
   glutInitDisplayMode (GLUT_DOUBLE | GLUT_DEPTH);
   glutInitWindowSize (700, 700);
   glutInitWindowPosition (100, 100);
   glutCreateWindow ("Train");
   initialise ();
   glutDisplayFunc(display);
   glutSpecialFunc(special);
   glutTimerFunc(1, myTimer, 0);
   glutKeyboardFunc(keyboard);
   glutMainLoop();
   return 0;
}
