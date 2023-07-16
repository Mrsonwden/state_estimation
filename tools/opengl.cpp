#include <GL/glut.h>
#include <cmath>

void display() {
    glClearColor(1.0, 1.0, 1.0, 1.0); // 设置背景颜色为白色
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // 绘制坐标轴
    glColor3f(0.0, 0.0, 0.0);
    glBegin(GL_LINES);
    glVertex2f(-1.0, 0.0);
    glVertex2f(1.0, 0.0);
    glVertex2f(0.0, -1.0);
    glVertex2f(0.0, 1.0);
    glEnd();

    glColor3f(1.0, 0.0, 0.0);
    glBegin(GL_LINE_STRIP);
    
    for (float x = -1.0; x <= 1.0; x += 0.005) {
        float y = sin(x * M_PI);
        glVertex2f(x, y);
    }
    
    glEnd();
    glutSwapBuffers();
}

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
    glutInitWindowSize(800, 600);
    glutCreateWindow("OpenGL Curve");
    glutDisplayFunc(display);
    glutMainLoop();
    return 0;
}