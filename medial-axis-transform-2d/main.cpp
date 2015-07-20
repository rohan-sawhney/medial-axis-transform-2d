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
    std::cout << "Press q,w,e,r,t,y,u,i for different test shapes."
              << " Hit enter to draw medial axis one path at a time."
              << " Yellow points are keypoints on the medial axis."
              << " Orange points represent transitions.\n\n"
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
    
    // FIX: GL_LINE_STRIP does not render for small "diff"
    glBegin(GL_LINE_STRIP);
    for (double x = startX; x < endX; x += dx) {
        glVertex2f(x, p.getY(x));
    }
    glEnd();
}

void display()
{
    glClear(GL_COLOR_BUFFER_BIT);
    
    if (showBoundary) {
        // draw boundary
        glColor3f(1.0f, 0.0f, 1.0f);
        for (size_t i = 0; i < boundaryElements.size(); i++) {
            if (boundaryElements[i].type == "Edge") {
                glBegin(GL_LINES);
                glVertex2f(boundaryElements[i].vertex1.x(), boundaryElements[i].vertex1.y());
                glVertex2f(boundaryElements[i].vertex2.x(), boundaryElements[i].vertex2.y());
                glEnd();
            }
        }
    }
    
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
        glPointSize(4.0);
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
            if (path.parabola.set) {
                drawParabola(path.parabola, path.keyPoint1, path.keyPoint2);
                
            } else {
                glBegin(GL_LINES);
                glVertex2f(path.keyPoint1.x(), path.keyPoint1.y());
                glVertex2f(path.keyPoint2.x(), path.keyPoint2.y());
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
    boundaryElements = bg.getBoundaryElements('q');
    mat.setBoundaryElements(boundaryElements);
    medialPaths = mat.run();
    
    printInstructions();
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

