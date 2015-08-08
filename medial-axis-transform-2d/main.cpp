#ifdef __APPLE_CC__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include "BoundaryGenerator.h"
#include "MedialAxisTransform.h"

#define HLEN 25

BoundaryGenerator bg;
std::vector<BoundaryElement> boundaryElements;

MedialAxisTransform mat;
std::vector<Path> medialPaths;

int gridX = 600;
int gridY = 600;

int step = 0;

bool showBoundary = true;
bool showHalfline = true;
bool showKeyPoint = true;
bool showPath = true;
bool showRadius = true;

void printInstructions()
{
    std::cout << "Press q,w,e,r,t,y,u,i,o,p for different test shapes."
              << " Hit space to draw medial axis one path at a time."
              << " Yellow points are keypoints on the medial axis."
              << " Orange points represent transitions."
              << " Blue boundary elements are the path governors.\n\n"
              << std::endl;
}

void init2D()
{
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glMatrixMode(GL_PROJECTION);
    gluOrtho2D(0.0, gridX, gridY, 0.0);
}

void drawCircle(const double cx, const double cy, const double r)
{
    glBegin(GL_LINE_LOOP);
    int segments = 50;
    for(int i = 0; i < segments; i++) {
        double theta = 2.0f * 3.1415926f * double(i) / double(segments);
        
        double x = r * cosf(theta);
        double y = r * sinf(theta);
        
        glVertex2f(x + cx, y + cy);
    }
    glEnd();
}

void drawParabola(const Parabola& p, const Vector2d& start, const Vector2d& end)
{
    double startX = start.x();
    double endX = end.x();
    if (endX < startX) {
        startX = end.x();
        endX = start.x();
    }
    
    double dx = 0.1;
    double diff = endX - startX;
    while (diff < 1) {
        diff *= 10;
        dx /= 10;
    }
    
    // TODO: Which parabola to draw???
    // FIX: GL_LINE_STRIP does not render for small "diff"
    glBegin(GL_LINE_STRIP);
    for (double x = startX; x < endX; x += dx) {
        if (p.set == 1) {
            glVertex2f(x, p.getY(x).first);
        }
    }
    glEnd();
    
    glBegin(GL_LINE_STRIP);
    for (double x = startX; x < endX; x += dx) {
        if (p.set == 2) {
            glVertex2f(x, p.getY(x).second);
        }
    }
    glEnd();
}

void display()
{
    glClear(GL_COLOR_BUFFER_BIT);
    glPointSize(4.0);
    
    if (showHalfline) {
        // draw halflines
        glBegin(GL_LINES);
        glColor3f(0.0f, 1.0f, 0.0f);
        for (size_t i = 1; i < boundaryElements.size(); i++) {
            if (boundaryElements[i].type == "Vertex") {
                glVertex2f(boundaryElements[i].x(), boundaryElements[i].y());
                glVertex2f(boundaryElements[i].x() + boundaryElements[i].halfLine1.x() * HLEN,
                           boundaryElements[i].y() + boundaryElements[i].halfLine1.y() * HLEN);
                
                glVertex2f(boundaryElements[i].x(), boundaryElements[i].y());
                glVertex2f(boundaryElements[i].x() + boundaryElements[i].halfLine2.x() * HLEN,
                           boundaryElements[i].y() + boundaryElements[i].halfLine2.y() * HLEN);
                
            } else {
                glVertex2f(boundaryElements[i].vertex1.x(), boundaryElements[i].vertex1.y());
                glVertex2f(boundaryElements[i].vertex1.x() + boundaryElements[i].halfLine1.x() * HLEN,
                           boundaryElements[i].vertex1.y() + boundaryElements[i].halfLine1.y() * HLEN);
                
                glVertex2f(boundaryElements[i].vertex2.x(), boundaryElements[i].vertex2.y());
                glVertex2f(boundaryElements[i].vertex2.x() + boundaryElements[i].halfLine2.x() * HLEN,
                           boundaryElements[i].vertex2.y() + boundaryElements[i].halfLine2.y() * HLEN);
            }
        }
        glEnd();
    }
    
    if (showKeyPoint) {
        // draw keypoints
        glBegin(GL_POINTS);
        for (int i = 0; i < step; i++) {
            glColor3f(1.0f, 1.0f, 0.0f);
            Path path = medialPaths[i];
            if (i == 0) {
                glVertex2f(path.keyPoint1.x(), path.keyPoint1.y());
            }
            
            if (path.keyPoint2.isTransition) {
                glColor3f(1.0f, 0.65f, 0.0f);
            } 
            
            glVertex2f(path.keyPoint2.x(), path.keyPoint2.y());
        }
        glEnd();
    }
    
    if (showPath) {
        // draw path
        glColor3f(0.0f, 1.0f, 1.0f);
        for (int i = 0; i < step; i++) {
            Path path = medialPaths[i];
            if (path.parabola.set > 0) {
                drawParabola(path.parabola, path.keyPoint1, path.keyPoint2);
                
            } else {
                glBegin(GL_LINES);
                glVertex2f(path.keyPoint1.x(), path.keyPoint1.y());
                glVertex2f(path.keyPoint2.x(), path.keyPoint2.y());
                glEnd();
            }
                        
            if (i == step-1) {
                boundaryElements[path.gov1.index].isGov = true;
                boundaryElements[path.gov2.index].isGov = true;
            }
        }
    }
    
    if (showBoundary) {
        // draw boundary
        for (size_t i = 0; i < boundaryElements.size(); i++) {
            if (boundaryElements[i].isGov) {
                glColor3f(0.5f, 0.5f, 1.0f);
                boundaryElements[i].isGov = false;
            } else {
                glColor3f(1.0f, 0.0f, 1.0f);
            }
            
            if (boundaryElements[i].type == "Edge") {
                glBegin(GL_LINES);
                glVertex2f(boundaryElements[i].vertex1.x(), boundaryElements[i].vertex1.y());
                glVertex2f(boundaryElements[i].vertex2.x(), boundaryElements[i].vertex2.y());
                glEnd();
            } else {
                glBegin(GL_POINTS);
                glVertex2f(boundaryElements[i].x(), boundaryElements[i].y());
                glEnd();
            }
        }
    }
    
    if (showRadius) {
        // draw radius function
        glColor3f(1.0f, 1.0f, 1.0f);
        for (int i = 0; i < step; i++) {
            Path path = medialPaths[i];
            if (i == 0) {
                drawCircle(path.keyPoint1.x(), path.keyPoint1.y(), path.keyPoint1.radius);
            }
            
            drawCircle(path.keyPoint2.x(), path.keyPoint2.y(), path.keyPoint2.radius);
        }
        glEnd();
    }
    
    glutSwapBuffers();
    glutPostRedisplay();
}

void keyboard(unsigned char key, int x, int y)
{
    switch(key) {
        case 27 :
            exit(0);
        case ' ':
            step++;
            if (step > medialPaths.size()) {
                step = 0;
            }
            break;
        case 'a':
            showBoundary = !showBoundary;
            break;
       	case 's':
            showHalfline = !showHalfline;
            break;
       	case 'd':
            showKeyPoint = !showKeyPoint;
            break;
       	case 'f':
            showPath = !showPath;
            break;
       	case 'g':
            showRadius = !showRadius;
            break;
        default:
            boundaryElements = bg.getBoundaryElements(key);
            mat.setBoundaryElements(boundaryElements);
            medialPaths = mat.run();
            step = 0;
    }
    
    glutPostRedisplay();	
}

int main(int argc, char** argv) 
{
    printInstructions();
    boundaryElements = bg.getBoundaryElements('q');
    mat.setBoundaryElements(boundaryElements);
    medialPaths = mat.run();
    
    glutInit(&argc,argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(gridX, gridY);
    glutCreateWindow("Medial Axis Transform");
    init2D();
    glutDisplayFunc(display);
    glutKeyboardFunc(keyboard);				
    glutMainLoop();
    
    return 0;
}

