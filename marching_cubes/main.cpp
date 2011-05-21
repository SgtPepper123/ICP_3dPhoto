/**
 * Author: Erik Smistad
 * www.thebigblob.com
**/

#include <GL/glut.h>
#include "marchingcubes.hpp"

// Global variables
GLfloat angle = 0.0;
vector<vertex> vertices;

void renderScene(void) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glShadeModel(GL_SMOOTH);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
     
    // Create light components
    GLfloat ambientLight[] = { 0.2f, 0.2f, 0.2f, 1.0f };
    GLfloat diffuseLight[] = { 0.8f, 0.8f, 0.8, 1.0f };
    GLfloat specularLight[] = { 0.5f, 0.5f, 0.5f, 1.0f };
    GLfloat position[] = { -1.5f, 1.0f, -4.0f, 1.0f };
     
    // Assign created components to GL_LIGHT0
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight);
    glLightfv(GL_LIGHT0, GL_POSITION, position);
        
    glEnable(GL_NORMALIZE);
    glEnable(GL_COLOR_MATERIAL);

    // Set material properties which will be assigned by glColor
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
    float specReflection[] = { 0.8f, 0.8f, 0.8f, 1.0f };
    glMaterialfv(GL_FRONT, GL_SPECULAR, specReflection);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glRotatef(angle, 0.0f, 1.0f, 0.0f);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glPushMatrix();
    glScalef(0.003f, 0.003f, 0.003f);
    glColor3f(0.0f, 0.6f, 0.0f);
    glTranslatef(0.0f, -128.0f, 32.0f);
    glRotatef(80.0f, 0.0f, 0.0f, 1.0f);

    // Draw the triangles
    vector<vertex>::iterator it;
    glBegin(GL_TRIANGLES);
        for(it = vertices.begin(); it < vertices.end(); it++) {
            glNormal3d(it->normal_x, it->normal_y, it->normal_z);
            glVertex3d(it->x, it->y, it->z);
        }
    glEnd();
    glPopMatrix();

    glutSwapBuffers();
    angle += 0.5;
}

int main(int argc, char **argv) {
    vertices = runMarchingCubes(parseRawFile("aneurism.raw", 256, 256, 256), 
            256, 256, 256, 1, 1, 1, 37.0);

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowPosition(100,100);
    glutInitWindowSize(1024,1024);
    glutCreateWindow("Marching cubes");
    glutDisplayFunc(renderScene);
    glutIdleFunc(renderScene);
    glutMainLoop();

    return 0;
}
