#include <iostream>
#include <fstream>
#include <vector>
#include <array>
#include <cmath>
//edit 
#include <cstdlib> // for using rand function
#include <ctime>
//end
#include <GL/glut.h>
#include "FrameXform.h"
#include "wavefront_obj.h"

//edit
//color
#define cowHex 0xFF54A7
#define betHex 0x9BEF05
#define stmHex 0x00B5EC
#define floorHex 0xB68B68
//end

using double2 = std::array<double, 2>;
using double3 = std::array<double, 3>;

//type-invariant PI value
template<typename T>
T const PI = std::acos( -T( 1 ) );

// 'cameras' stores infomation of 5 cameras.
std::vector<std::array<double, 9>> cameras{
    {28, 18, 28, 0, 2, 0, 0, 1, 0},
    {28, 18, -28, 0, 2, 0, 0, 1, 0},
    {-28, 18, 28, 0, 2, 0, 0, 1, 0},
    {-12, 12, 0, 0, 2, 0, 0, 1, 0},
    {0, 100, 0,  0, 0, 0, 1, 0, 0}
};

int cameraIndex, camID;
std::vector<FrameXform> wld2cam, cam2wld;
wavefront_obj_t *cam;

// Variables for 'cow' object.
FrameXform cow2wld;
wavefront_obj_t *cow;
int cowID;

// Variables for 'beethovan' object.
FrameXform bet2wld;
wavefront_obj_t *bet;
int betID;

//edit
// Variables for 'stormtrooper' object.
FrameXform stm2wld;
wavefront_obj_t *stm;
int stmID;
//end

unsigned floorTexID;
int frame = 0;
int width, height;
int selectMode = 0, oldX, oldY;

//edit
float oldAngle = 45; // zoom����� �����ϴ� �� ī�޶��� fov ���� �����ϴµ� ���� ����
int pdot = 0; // 0: nothing, 1: pan, 2: dolly, etc..
int selectOn = 0; // disable/enable selection mode
int selObjInx = 1; // ���õ� ������Ʈ�� �ε���. �̰ɷ� ����. 1: cow, 2: bet, 3: stm, 4:  current cam
int onX = 1; int onY = 0; int onZ = 0; // KŰ�� �������� onK�� 1, �ƴ϶�� 0
int tmpOnX = onX; int tmpOnY = onY; int tmpOnZ = onZ; // r Ű�� �������� �� onK�� ���� tmpOnK�� �ӽ������ϰ� onK�� 0����. �׷��� ȸ������ �巡�׸� �ص� �Ҵ� �������� ����. �ٽ� rŰ�� �������� �����ߴ� ���� onK��.
GLdouble angleX = 0; GLdouble angleY = 0; GLdouble angleZ = 0; // glRotated�� x,y,z �ڸ��� �� �۷ι� ����
int isRotate = 0; // ȸ����忡�� 1, ȸ����尡 �ƴ� �� 0
int modelOrView = 0; // ��ȯ�� ���ϴ� ������ modelSpace�� 0, viewSpace�� 1
//end
void drawFrame( float len );
void setRotationAxis(void);
void renderRotation();

double3 munge( int x ) {
	double r, g, b;
    r = ( x & 255 ) / double( 255 );
    g = ( ( x >> 8 ) & 255 ) / double( 255 );
    b = ( ( x >> 16 ) & 255 ) / double( 255 );
	return double3{ r, g, b };
}

int unmunge( double3 color ) {
	double r = color[0], g = color[1], b = color[2];
    return ( int( r ) + ( int( g ) << 8 ) + ( int( b ) << 16 ) );
}

void setCamera() {
    int i;
    if ( frame == 0 ) {
        // intialize camera model.
        cam = new wavefront_obj_t( "camera.obj" );	// Read information of camera from camera.obj.
        camID = glGenLists( 1 );					// Create display list of the camera.
        glNewList( camID, GL_COMPILE );			// Begin compiling the display list using camID.
        cam->draw();							// Draw the camera. you can do this job again through camID..
        glEndList();							// Terminate compiling the display list.

        // initialize camera frame transforms.
        for ( i = 0; i < cameras.size(); i++ ) {
            auto &camera = cameras[i];											// 'c' points the coordinate of i-th camera.
            wld2cam.push_back( FrameXform() );								// Insert {0} matrix to wld2cam vector.
            glPushMatrix();													// Push the current matrix of GL into stack.
            glLoadIdentity();												// Set the GL matrix Identity matrix.
            gluLookAt( camera[0], camera[1], camera[2], camera[3], camera[4], camera[5], camera[6], camera[7], camera[8] );		// Setting the coordinate of camera.
            glGetDoublev( GL_MODELVIEW_MATRIX, wld2cam[i].matrix() );		// Read the world-to-camera matrix computed by gluLookAt.
            glPopMatrix();													// Transfer the matrix that was pushed the stack to GL.
            cam2wld.push_back( wld2cam[i].inverse() );						// Get the camera-to-world matrix.
        }
        cameraIndex = 0;
    }

    // set viewing transformation.
    glLoadMatrixd( wld2cam[cameraIndex].matrix() );

    // draw other cameras.
    for ( i = 0; i < ( int )wld2cam.size(); i++ ) {
        if ( i != cameraIndex ) {
            glPushMatrix();												// Push the current matrix on GL to stack. The matrix is wld2cam[cameraIndex].matrix().
            glMultMatrixd( cam2wld[i].matrix() );							// Multiply the matrix to draw i-th camera.
            if ( selectMode == 0 ) {									// selectMode == 1 means backbuffer mode.
                drawFrame( 5 );											// Draw x, y, and z axis.
                float frontColor[] = {0.2f, 0.2f, 0.2f, 1.0f};
                glEnable( GL_LIGHTING );
                glMaterialfv( GL_FRONT, GL_AMBIENT, frontColor );			// Set ambient property frontColor.
                glMaterialfv( GL_FRONT, GL_DIFFUSE, frontColor );			// Set diffuse property frontColor.
            } else {
                double3 color;
                glDisable( GL_LIGHTING );									// Disable lighting in backbuffer mode.
                color = munge( i + 1 );										// Match the corresponding (i+1)th color to r, g, b. You can change the color of camera on backbuffer.
                glColor3dv( color.data() );										// Set r, g, b the color of camera.
            }
            glScaled( 0.5, 0.5, 0.5 );										// Reduce camera size by 1/2.
            glTranslated( 1.1, 1.1, 0.0 );									// Translate it (1.1, 1.1, 0.0).
            glCallList( camID );											// Re-draw using display list from camID.
            glPopMatrix();												// Call the matrix on stack. wld2cam[cameraIndex].matrix() in here.
        }
    }
}

/*********************************************************************************
* Draw x, y, z axis of current frame on screen.
* x, y, and z are corresponded Red, Green, and Blue, resp.
**********************************************************************************/
void drawFrame( float len ) {
    glDisable( GL_LIGHTING );		// Lighting is not needed for drawing axis.
    glBegin( GL_LINES );			// Start drawing lines.
    glColor3d( 1, 0, 0 );			// color of x-axis is red.
    glVertex3d( 0, 0, 0 );
    glVertex3d( len, 0, 0 );		// Draw line(x-axis) from (0,0,0) to (len, 0, 0).
    glColor3d( 0, 1, 0 );			// color of y-axis is green.
    glVertex3d( 0, 0, 0 );
    glVertex3d( 0, len, 0 );		// Draw line(y-axis) from (0,0,0) to (0, len, 0).
    glColor3d( 0, 0, 1 );			// color of z-axis is  blue.
    glVertex3d( 0, 0, 0 );
    glVertex3d( 0, 0, len );		// Draw line(z-axis) from (0,0,0) - (0, 0, len).
    glEnd();						// End drawing lines.
}

//edit
void drawRotFrame( int n ) { // drawFrame()�� ����ϴ�. ȸ���ϴ� ����(angleX, ...)�� �����ϴ� ȸ������ ������� �׸���. ���̴� 10�̰� �߽��� ���� �𵨸� ���� (0,0,0)�� �ִ�.
	if (isRotate != 0) { // ��, �Ұ� ���� �ʰ� �����ִٸ� ȸ������ �׸� �ʿ䰡 ����.
		GLdouble norm = sqrt(angleX*angleX + angleY*angleY + angleZ*angleZ); //�븧
		glDisable(GL_LIGHTING);
		glBegin(GL_LINES);
		glColor3d(1, 1, 1);
		if (norm != 0.0) {
			glVertex3d(-n * angleX / norm, -n * angleY / norm, -n * angleZ / norm); // ���͸� ����ȭ�� �� �˸��� ���̸� �����ش�.
			glVertex3d(n * angleX / norm, n * angleY / norm, n * angleZ / norm);
		}
		glEnd();
	}
}
//end
/*********************************************************************************
* Draw 'cow' object.
**********************************************************************************/
void drawCow() {
    if ( frame == 0 ) {
        // Initialization part.

        // Read information from cow.obj.
        cow = new wavefront_obj_t( "cow.obj" );

        // Make display list. After this, you can draw cow using 'cowID'.
        cowID = glGenLists( 1 );				// Create display lists
        glNewList( cowID, GL_COMPILE );		// Begin compiling the display list using cowID
        cow->draw();						// Draw the cow on display list.
        glEndList();						// Terminate compiling the display list. Now, you can draw cow using 'cowID'.
        glPushMatrix();						// Push the current matrix of GL into stack.
        glLoadIdentity();					// Set the GL matrix Identity matrix.
        glTranslated( 0, -cow->aabb.first[1], -8 );	// Set the location of cow.
        glRotated( -90, 0, 1, 0 );			// Set the direction of cow. These information are stored in the matrix of GL.
        glGetDoublev( GL_MODELVIEW_MATRIX, cow2wld.matrix() );	// Read the modelview matrix about location and direction set above, and store it in cow2wld matrix.
        glPopMatrix();						// Pop the matrix on stack to GL.
    }

    glPushMatrix();		// Push the current matrix of GL into stack. This is because the matrix of GL will be change while drawing cow.

    // The information about location of cow to be drawn is stored in cow2wld matrix.
    // (Project2 hint) If you change the value of the cow2wld matrix or the current matrix, cow would rotate or move.
    glMultMatrixd( cow2wld.matrix() );

    if ( selectMode == 0 ) {								// selectMode == 1 means backbuffer mode.
        drawFrame( 5 );	
		//edit
		drawRotFrame( selObjInx == 1 ? 5 : 0);
		//end
        float frontColor[] = {0.8f, 0.2f, 0.9f, 1.0f};
        glEnable( GL_LIGHTING );
        glMaterialfv( GL_FRONT, GL_AMBIENT, frontColor );		// Set ambient property frontColor.
        glMaterialfv( GL_FRONT, GL_DIFFUSE, frontColor );		// Set diffuse property frontColor.
    } else {
        glDisable( GL_LIGHTING );								// Disable lighting in backbuffer mode
		drawRotFrame(selObjInx == 1 ? 5 : 0);
		double3 color = munge(cowHex);									// Match the corresponding constant color to r, g, b. You can change the color of camera on backbuffer
        glColor3dv( color.data() );
    }
    glCallList( cowID );		// Draw cow.
    glPopMatrix();			// Pop the matrix in stack to GL. Change it the matrix before drawing cow.
}

/*********************************************************************************
* Draw 'beethovan' object.
**********************************************************************************/
void drawBet() {
    if ( frame == 0 ) {
        // Initialization part.

        // Read information from beethovan.obj.
        bet = new wavefront_obj_t( "beethovan.obj" );

        // Make display list. After this, you can draw beethovan using 'betID'.
        betID = glGenLists( 1 );
        glNewList( betID, GL_COMPILE );
        bet->draw();
        glEndList();
        glPushMatrix();
        glLoadIdentity();
        glTranslated( 0, -bet->aabb.first[1], 8 );
        glRotated( 180, 0, 1, 0 );
        glGetDoublev( GL_MODELVIEW_MATRIX, bet2wld.matrix() );
        glPopMatrix();
    }

    glPushMatrix();
    glMultMatrixd( bet2wld.matrix() );
    if ( selectMode == 0 ) {
        drawFrame( 8 );
		//edit
		drawRotFrame(selObjInx == 2 ? 8 : 0);
		//end
        float frontColor[] = {0.8, 0.3, 0.1, 1.0};
        glEnable( GL_LIGHTING );
        glMaterialfv( GL_FRONT, GL_AMBIENT, frontColor );
        glMaterialfv( GL_FRONT, GL_DIFFUSE, frontColor );
    } else {
        glDisable( GL_LIGHTING );
		drawRotFrame(selObjInx == 2 ? 8 : 0);
        double3 color = munge(betHex);
        glColor3dv(color.data() );
    }
    glCallList( betID );
    glPopMatrix();
}

//edit
void drawStm() {
	if (frame == 0) {
		
		stm = new wavefront_obj_t("stormtrooper.obj");

		stmID = glGenLists(1);
		glNewList(stmID, GL_COMPILE);
		stm->draw();
		glEndList();
		glPushMatrix();
		glLoadIdentity();
		glTranslated(0, -stm->aabb.first[1], 0); 
		glRotated(180, 0, 1, 0);
		glGetDoublev(GL_MODELVIEW_MATRIX, stm2wld.matrix());
		glPopMatrix();
	}

	glPushMatrix();
	glMultMatrixd(stm2wld.matrix());
	if (selectMode == 0) {
		drawFrame(8);
		//edit
		drawRotFrame(selObjInx == 3 ? 5 : 0);
		//end
		float frontColor[] = { 0.8, 0.5, 0.5, 1.0 };
		glEnable(GL_LIGHTING);
		glMaterialfv(GL_FRONT, GL_AMBIENT, frontColor);
		glMaterialfv(GL_FRONT, GL_DIFFUSE, frontColor);
	}
	else {
		glDisable(GL_LIGHTING);
		drawRotFrame(selObjInx == 3 ? 5 : 0);
		double3 color = munge(stmHex);
		glColor3dv(color.data());
	}
	glCallList(stmID);
	glPopMatrix();
}
//end

/*********************************************************************************
* Draw floor on 3D plane.
**********************************************************************************/
void drawFloor() {
    if ( frame == 0 ) {
        // Initialization part.
        // After making checker-patterned texture, use this repetitively.

        // Insert color into checker[] according to checker pattern.
        const int size = 8;
        unsigned char checker[size * size * 3];
        for( int i = 0; i < size * size; i++ ) {
            if ( ( ( i / size ) ^ i ) & 1 ) {
                checker[3 * i + 0] = 200;
                checker[3 * i + 1] = 32;
                checker[3 * i + 2] = 32;
            } else {
                checker[3 * i + 0] = 200;
                checker[3 * i + 1] = 200;
                checker[3 * i + 2] = 32;
            }
        }

        // Make texture which is accessible through floorTexID.
        glGenTextures( 1, &floorTexID );
        glBindTexture( GL_TEXTURE_2D, floorTexID );
        glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP );
        glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP );
        glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST );
        glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST );
        glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE );
        glTexImage2D( GL_TEXTURE_2D, 0, 3, size, size, 0, GL_RGB, GL_UNSIGNED_BYTE, checker );
    }

    glDisable( GL_LIGHTING );

    // Set background color.
    if ( selectMode == 0 )
        glColor3d( 0.35, .2, 0.1 );
    else {
        // In backbuffer mode.
		double3 color = munge(30);
		glColor3dv(color.data());
    }

    // Draw background rectangle.
    glBegin( GL_POLYGON );
    glVertex3f( 2000, -0.2, 2000 );
    glVertex3f( 2000, -0.2, -2000 );
    glVertex3f( -2000, -0.2, -2000 );
    glVertex3f( -2000, -0.2, 2000 );
    glEnd();


    // Set color of the floor.
    if ( selectMode == 0 ) {
        // Assign checker-patterned texture.
        glEnable( GL_TEXTURE_2D );
        glBindTexture( GL_TEXTURE_2D, floorTexID );
    } else {
        // Assign color on backbuffer mode.
		double3 color = munge(floorHex);
		glColor3dv(color.data());
    }

    // Draw the floor. Match the texture's coordinates and the floor's coordinates resp.
    glBegin( GL_POLYGON );
    glTexCoord2d( 0, 0 );
    glVertex3d( -12, -0.1, -12 );		// Texture's (0,0) is bound to (-12,-0.1,-12).
    glTexCoord2d( 1, 0 );
    glVertex3d( 12, -0.1, -12 );		// Texture's (1,0) is bound to (12,-0.1,-12).
    glTexCoord2d( 1, 1 );
    glVertex3d( 12, -0.1, 12 );		// Texture's (1,1) is bound to (12,-0.1,12).
    glTexCoord2d( 0, 1 );
    glVertex3d( -12, -0.1, 12 );		// Texture's (0,1) is bound to (-12,-0.1,12).
    glEnd();

    if ( selectMode == 0 ) {
        glDisable( GL_TEXTURE_2D );
        drawFrame( 5 );				// Draw x, y, and z axis.
    }
}

/*********************************************************************************
* Call this part whenever display events are needed.
* Display events are called in case of re-rendering by OS. ex) screen movement, screen maximization, etc.
* Or, user can occur the events by using glutPostRedisplay() function directly.
* this part is called in main() function by registering on glutDisplayFunc(display).
**********************************************************************************/
void display() {
    // selectMode == 1 means backbuffer mode.
	if (selectMode == 0) {
		glClearColor(0, 0.6, 0.8, 1);
	}// Clear color setting
	else {
		glClearColor(0, 0, 0, 1);
	}
		// When the backbuffer mode, clear color is set to black
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );				// Clear the screen
    setCamera();													// Locate the camera's position, and draw all of them.

    drawFloor();													// Draw floor.
    drawCow();														// Draw cow.
    //edit
	drawBet();
	drawStm();
	//end
	glEnable(GL_LIGHTING);
    glFlush();

    // If it is not backbuffer mode, swap the screen. In backbuffer mode, this is not necessary because it is not presented on screen.
    if ( selectMode == 0 )
        glutSwapBuffers();
    frame += 1;
}

/*********************************************************************************
* Call this part whenever size of the window is changed.
* This part is called in main() function by registering on glutReshapeFunc(reshape).
**********************************************************************************/
void reshape( int w, int h ) {
    width = w;
    height = h;
    glViewport( 0, 0, width, height );
    glMatrixMode( GL_PROJECTION );          // Select The Projection Matrix
    glLoadIdentity();                       // Reset The Projection Matrix
    // Define perspective projection frustum
    double aspect = width / double( height );
    gluPerspective( oldAngle, aspect, 1, 1024 );
    glMatrixMode( GL_MODELVIEW );           // Select The Modelview Matrix
    glLoadIdentity();                       // Reset The Projection Matrix
}

void initialize() {
    // Set up OpenGL state
    glShadeModel( GL_SMOOTH );       // Set Smooth Shading
    glEnable( GL_DEPTH_TEST );       // Enables Depth Testing
    glDepthFunc( GL_LEQUAL );        // The Type Of Depth Test To Do
    // Use perspective correct interpolation if available
    glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );
    // Initialize the matrix stacks
    reshape( width, height );
    // Define lighting for the scene
    float lightDirection[]   = {1.0, 1.0, 1.0, 0};
    float ambientIntensity[] = {0.1, 0.1, 0.1, 1.0};
    float lightIntensity[]   = {0.9, 0.9, 0.9, 1.0};
    glLightfv( GL_LIGHT0, GL_AMBIENT, ambientIntensity );
    glLightfv( GL_LIGHT0, GL_DIFFUSE, lightIntensity );
    glLightfv( GL_LIGHT0, GL_POSITION, lightDirection );
    glEnable( GL_LIGHT0 );
}

/*********************************************************************************
* Call this part whenever mouse button is clicked.
* This part is called in main() function by registering on glutMouseFunc(onMouseButton).
**********************************************************************************/
void onMouseButton( int button, int state, int x, int y ) {
    y = height - y - 1;
    if ( button == GLUT_LEFT_BUTTON ) {
        if ( state == GLUT_DOWN ) {
            printf( "Left mouse click at (%d, %d)\n", x, y );

            // (Project 4) After drawing object on backbuffer, you can recognize which object is selected by reading pixel of (x, y).
            // Change the value of selectMode to 1, then draw the object on backbuffer when display() function is called.
			int tmp = selectMode; //edit
            selectMode = 1;
            display();
            glReadBuffer( GL_BACK );
            unsigned char pixel[3];
            glReadPixels( x, y, 1, 1, GL_RGB, GL_UNSIGNED_BYTE, pixel );
			double3 pixel_d3 = { double(pixel[0]), double(pixel[1]), double(pixel[2]) };
			printf("pixel = %d\n", unmunge(pixel_d3));
            selectMode = tmp; //edit

            // (Project 4) TODO : Perform the proper task about selected object.
            // hint : you can recognize which object is selected by pixel value.
			//edit	
			switch(unmunge(pixel_d3)*(selectOn)) { // selection�� disable�Ǹ� ���� �ֱٿ� ���õ� ������Ʈ��, enable�Ǹ� ���콺�� Ŭ���� ������Ʈ��
			case 0:
				break;
			case cowHex:
				selObjInx = 1;
				break;
			case betHex:
				selObjInx = 2;
				break;
			case stmHex:
				selObjInx = 3;
				break;
			default:
				selObjInx = 4;
				break;
			}

			if (isRotate == 1) {
				if (selectOn) {
					if (unmunge(pixel_d3) == cowHex || unmunge(pixel_d3) == betHex || unmunge(pixel_d3) == stmHex) {
						setRotationAxis();
						if (modelOrView == 0)		glutIdleFunc(renderRotation); //modeling �������� ȸ���� ���ϸ� IdleFunc����
					}
					else {
						if (modelOrView == 0)		glutIdleFunc(NULL); // stop rotating
					}
				}
				else {
					//nothing
				}
			}
			//end
            // Save current clicked location of mouse here, and then use this on onMouseDrag function.
            oldX = x;
            oldY = y;
        }
    } else if ( button == GLUT_RIGHT_BUTTON ) {
        printf( "Right mouse click at (%d, %d)\n", x, y );
    }

	if (selectMode) {
		display();
		glutSwapBuffers();
	}

    glutPostRedisplay();
}

//edit
FrameXform* selObj2Wld() { // ���õ� ��ü�� ������Ʈ ���� to ���� ���� ��Ʈ������ �ּҸ� ��ȯ
	switch (selObjInx) {
	case 1:
		return &cow2wld;
		break;
	case 2:
		return &bet2wld;
		break;
	case 3:
		return &stm2wld;
		break;
	case 4:
		return &cam2wld[cameraIndex];
		break;
	default: //�̷� ���� �Ͼ�� �ʴ´�.
		printf("error\n");
	}
}

void setNormalized(double* a, double* b, double* c) {
	double norm = sqrt((*a)*(*a) + (*b)*(*b) + (*c)*(*c));
	*a /= norm; *b /= norm; *c /= norm;
}

void crossProduct(double * A, double * B, double * C) {
	C[0] = A[1] * B[2] - A[2] * B[1];
	C[1] = A[2] * B[0] - A[0] * B[2];
	C[2] = A[0] * B[1] - A[1] * B[0]; 
	setNormalized(&(C[0]), &(C[1]), &(C[2]));
}

double getDotProduct(double *A, double *B) {
	return A[0] * B[0] + A[1] * B[1] + A[2] * B[2];
}

double z(double x, double y) {
	double x2 = x*x;
	double y2 = y*y;
	double r2 = 1000000;
	if (x2 + y2 <= r2 * 0.5) {
		return sqrt(r2 - (x2 + y2));
	}
	else {
		return r2 * 0.5 / sqrt(x2 + y2);
	}
}

void trackballProject(FrameXform* obj2wldPtr, double x, double y, double* vec) {
	double center[2] = { 0,0 };
	FrameXform tmp;
	glPushMatrix();
	glLoadIdentity();
	glMultMatrixd(wld2cam[cameraIndex].matrix());
	glMultMatrixd((*obj2wldPtr).matrix());
	glGetDoublev(GL_MODELVIEW_MATRIX, tmp.matrix());
	glPopMatrix();

	center[0] = tmp.matrix()[12] * 1;
	center[1] = tmp.matrix()[13] * 1;

	x = x - width * 0.5 - center[0];
	y = y - height * 0.5 - center[1];
	vec[0] = x;	vec[1] = y;	vec[2] = z(x, y);
	setNormalized(&(vec[0]), &(vec[1]), &(vec[2]));
}

void trackballRotate(FrameXform* obj2wldPtr, double x1, double y1, double x2, double y2, double* arr) {
	double v1[3], v2[3], normal[3];
	trackballProject(obj2wldPtr, x1, y1, v1);
	trackballProject(obj2wldPtr, x2, y2, v2);
	crossProduct(v1, v2, normal);
	double theta = acos(getDotProduct(v1, v2)); // dot product of v1 and v2, v1,v2�� �̹� ũ�Ⱑ 1�̴�.
	arr[0] = theta; arr[1] = normal[0]; arr[2] = normal[1]; arr[3] = normal[2];
}

void pan(int x, int y) {
	auto &camera = cameras[cameraIndex];
	glPushMatrix();													// Push the current matrix of GL into stack.
	glLoadIdentity();
	// Set the GL matrix Identity matrix.
	double tmp0 = (cam2wld[cameraIndex].matrix()[0] * (x - oldX) * 0.05 + cam2wld[cameraIndex].matrix()[4] * (y - oldY) * 0.05);
	double tmp1 = (cam2wld[cameraIndex].matrix()[1] * (x - oldX) * 0.05 + cam2wld[cameraIndex].matrix()[5] * (y - oldY) * 0.05);
	double tmp2 = (cam2wld[cameraIndex].matrix()[2] * (x - oldX) * 0.05 + cam2wld[cameraIndex].matrix()[6] * (y - oldY) * 0.05); //  ((x - oldX) * 0.05, (y - oldY) * 0.05), 0) ��ŭ translate

	cameras[cameraIndex][0] = camera[0] - tmp0; cameras[cameraIndex][1] = camera[1] - tmp1; cameras[cameraIndex][2] = camera[2] - tmp2;
	cameras[cameraIndex][3] = camera[3] - tmp0; cameras[cameraIndex][4] = camera[4] - tmp1; cameras[cameraIndex][5] = camera[5] - tmp2;

	gluLookAt(camera[0], camera[1], camera[2], camera[3], camera[4], camera[5], camera[6], camera[7], camera[8]);		// Setting the coordinate of camera.
	glGetDoublev(GL_MODELVIEW_MATRIX, wld2cam[cameraIndex].matrix());		// Read the world-to-camera matrix computed by gluLookAt.
	glPopMatrix();													// Transfer the matrix that was pushed the stack to GL.
	cam2wld[cameraIndex] = wld2cam[cameraIndex].inverse();
	oldX = x; oldY = y;
	glutPostRedisplay();
}

void dolly(int x, int y) {
	auto &camera = cameras[cameraIndex];
	glPushMatrix();													// Push the current matrix of GL into stack.
	glLoadIdentity();
	// Set the GL matrix Identity matrix.
	double tmp0 = (cam2wld[cameraIndex].matrix()[8] * (y - oldY) * 0.05);
	double tmp1 = (cam2wld[cameraIndex].matrix()[9] * (y - oldY) * 0.05);
	double tmp2 = (cam2wld[cameraIndex].matrix()[10] * (y - oldY) * 0.05); //  (0, 0, (y - oldY) * 0.05)) ��ŭ translate

	cameras[cameraIndex][0] = camera[0] - tmp0; cameras[cameraIndex][1] = camera[1] - tmp1; cameras[cameraIndex][2] = camera[2] - tmp2;
	cameras[cameraIndex][3] = camera[3] - tmp0; cameras[cameraIndex][4] = camera[4] - tmp1; cameras[cameraIndex][5] = camera[5] - tmp2;

	gluLookAt(camera[0], camera[1], camera[2], camera[3], camera[4], camera[5], camera[6], camera[7], camera[8]);		// Setting the coordinate of camera.
	glGetDoublev(GL_MODELVIEW_MATRIX, wld2cam[cameraIndex].matrix());		// Read the world-to-camera matrix computed by gluLookAt.
	glPopMatrix();													// Transfer the matrix that was pushed the stack to GL.
	cam2wld[cameraIndex] = wld2cam[cameraIndex].inverse();
	oldX = x; oldY = y;
	glutPostRedisplay();
}

void zoom(int x, int y) {
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();           
	double aspect = width / double(height);
	gluPerspective(oldAngle - (y - oldY)*0.05, aspect, 1, 1024);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	oldAngle = oldAngle - (y - oldY)*0.05;
	oldX = x; oldY = y;
	glutPostRedisplay();
}

void trackball(int x, int y) {
	double oldR, newR, t;
	auto &camera = cameras[cameraIndex];
	glPushMatrix();													// Push the current matrix of GL into stack.
	glLoadIdentity();
	// Set the GL matrix Identity matrix.
	oldR = sqrt(pow(camera[0]-camera[3], 2) + pow(camera[1] - camera[4], 2) + pow(camera[2] - camera[5], 2));

	double tmp0 = (cam2wld[cameraIndex].matrix()[0] * (x - oldX) * 0.1 + cam2wld[cameraIndex].matrix()[4] * (y - oldY) * 0.1);
	double tmp1 = (cam2wld[cameraIndex].matrix()[1] * (x - oldX) * 0.1 + cam2wld[cameraIndex].matrix()[5] * (y - oldY) * 0.1);
	double tmp2 = (cam2wld[cameraIndex].matrix()[2] * (x - oldX) * 0.1 + cam2wld[cameraIndex].matrix()[6] * (y - oldY) * 0.1);
	cameras[cameraIndex][0] = camera[0] - tmp0; cameras[cameraIndex][1] = camera[1] - tmp1; cameras[cameraIndex][2] = camera[2] - tmp2; // e_x, e_y, e_z

	newR = sqrt(pow(camera[0] - camera[3], 2) + pow(camera[1] - camera[4], 2) + pow(camera[2] - camera[5], 2));

	t = oldR / newR;

	cameras[cameraIndex][0] = t * camera[0] + (1 - t) * camera[3]; cameras[cameraIndex][1] = t * camera[1] + (1 - t) * camera[4]; cameras[cameraIndex][2] = t * camera[2] + (1 - t) * camera[5];
	
	double tmp3[3] = { camera[0] - camera[3], camera[1] - camera[4], camera[2] - camera[5] };
	double tmp4[3] = { camera[6], camera[7], camera[8] };
	double tmp5[3] = {0,0,0};
	crossProduct(tmp4, tmp3, tmp5);
	crossProduct(tmp3, tmp5, tmp4);

	cameras[cameraIndex][6] = tmp4[0]; cameras[cameraIndex][7] = tmp4[1]; cameras[cameraIndex][8] = tmp4[2];

	gluLookAt(camera[0], camera[1], camera[2], camera[3], camera[4], camera[5], camera[6], camera[7], camera[8]);		// Setting the coordinate of camera.
	glGetDoublev(GL_MODELVIEW_MATRIX, wld2cam[cameraIndex].matrix());		// Read the world-to-camera matrix computed by gluLookAt.
	glPopMatrix();													// Transfer the matrix that was pushed the stack to GL.
	cam2wld[cameraIndex] = wld2cam[cameraIndex].inverse();
	oldX = x; oldY = y;
	glutPostRedisplay();
}


void rotAxisCam2Obj(FrameXform* obj2wldPtr, double x, double y, double z) {
	FrameXform tmp;
	glPushMatrix();
	tmp = (*obj2wldPtr).inverse(); //wld2obj
	glLoadMatrixd(tmp.matrix());
	glMultMatrixd(cam2wld[cameraIndex].matrix()); // wld2obj * cam2wld = cam2obj
	glGetDoublev(GL_MODELVIEW_MATRIX, tmp.matrix()); // tmp�� �� ��� ����
	glPopMatrix();
	//���� tmp ����� cam ��ǥ�� ���� �� (Ȥ�� ����)�� obj ��ǥ�� ���� ������ ���ɺ�ȯ�̴�.

	angleX = tmp.matrix()[0] * x + tmp.matrix()[4] * y + tmp.matrix()[8] * z;
	angleY = tmp.matrix()[1] * x + tmp.matrix()[5] * y + tmp.matrix()[9] * z;
	angleZ = tmp.matrix()[2] * x + tmp.matrix()[6] * y + tmp.matrix()[10] * z;
	// that is, tmp * {1,0,0}^t 
	// -> �� cam space �� x�� ���� �������͸� ��ȯ�Ѵ�.
}

void renderRotation(void) { // IdleAnimation���� �Ҹ� ȸ����Ű�� �Լ�
	FrameXform* obj2wldPtr = selObj2Wld();

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPushMatrix();
	glLoadMatrixd((*obj2wldPtr).matrix());
	glRotated(1, angleX, angleY, angleZ); // ������ ȸ���� (angleX, ...) �� �������� 1��ŭ ȸ��
	glGetDoublev(GL_MODELVIEW_MATRIX, (*obj2wldPtr).matrix());
	glPopMatrix();

	if (selectMode) {
		display();
		glutSwapBuffers();
	}

	glutPostRedisplay();
}

void setRotationAxis(void) { // ��쿡 ���� �����ϰ� ���� �����ϰų�, ȭ���� x �࿡ �ش��ϴ� ���� �����Ѵ�. �̴� modelOrView�� ���� �����Ѵ�.
	FrameXform* obj2wldPtr = selObj2Wld();

	if (cameraIndex >= (int)wld2cam.size())
		cameraIndex = 0;

	if (modelOrView == 0) { //modeling space. PA2 ������ onKeyPress()�� �ִ��� ������ ���ƴ�.
		srand((unsigned int)time(NULL));
		angleX = (GLdouble)rand(); angleY = (GLdouble)rand(); angleZ = (GLdouble)rand();
	}
	else if (modelOrView == 1) { //viewing space
		rotAxisCam2Obj(obj2wldPtr, 1, 0, 0);
	}
}
//end

/*********************************************************************************
* Call this part whenever user drags mouse.
* Input parameters x, y are coordinate of mouse on dragging.
* Value of global variables oldX, oldY is stored on onMouseButton,
* Then, those are used to verify value of x - oldX,  y - oldY to know its movement.
**********************************************************************************/
void onMouseDrag( int x, int y ) {
    y = height - y - 1;
    printf( "in drag (%d, %d)\n", x - oldX,  y - oldY );
	
    // (Project 2,3,4) TODO : Implement here to perform properly when drag the mouse on each case, respectively.
	//edited

	FrameXform* obj2wldPtr = selObj2Wld();

	glPushMatrix();

	if (pdot == 4) {
		if (selObjInx == 4 || selectOn == 0) { //������ Ŭ���ߴ�.
			trackball(x, y);
		}
		else { //��ü�� Ŭ���ߴ�.
			glLoadMatrixd((*obj2wldPtr).matrix());
			double arr[4];
			trackballRotate(obj2wldPtr, oldX, oldY, x, y, arr); // cam ���� �󿡼� ȸ����� ���� ���Ѵ�.
			rotAxisCam2Obj(obj2wldPtr, arr[1], arr[2], arr[3]); // cam ���� �� ȸ����(����)�� obj������ ���ͷ� ��ȯ�Ѵ�.
			glRotated(200 * arr[0], angleX, angleY, angleZ); // 100�� �ӷ� ����ġ

			glGetDoublev(GL_MODELVIEW_MATRIX, (*obj2wldPtr).matrix());
			oldX = x; // ��ġ�� �����Ѵ�
			oldY = y;
		}
	}
	else if (pdot != 0 && (selObjInx == 4 || selectOn == 0)) { // p,d,o �� �ϳ��� Ŭ���ߴ�.
		glPopMatrix();
		switch (pdot) {
		case 1:
			pan(x, y);
			break;
		case 2:
			dolly(x, y);
			break;
		case 3:
			zoom(x, y);
			break;
		}
		if (selectMode) {
			display();
			glutSwapBuffers();
		}
		return;
	}
	else {
		if (modelOrView == 0 && isRotate == 0) { //���� modeling space���� �̵��� ���Ѵٸ�,
			
			glLoadMatrixd((*obj2wldPtr).matrix()); // ���� ��ȯ ����� �ε��Ѵ�
			glTranslated((x - oldX)*onX*0.05, (x - oldX)*onY*0.05, (x - oldX)*onZ*0.05);// x - oldX�� �巡�� ����, onK�� � �������� �������� �ϴ���. 0.1�� ������ �ӷ� ���
																						//obj2wld ����� �ε������Ƿ�, �� arguments �� ���� �� ��ǥ�� �����̴�. �� translation�� world�� �ݿ��ȴ�.
			glGetDoublev(GL_MODELVIEW_MATRIX, (*obj2wldPtr).matrix());
		}
		else if (modelOrView == 1 && isRotate == 0) { // viewing space���� �̵��� ���Ѵٸ�,
			glLoadMatrixd(cam2wld[cameraIndex].matrix());
			glTranslated((x - oldX)*(onX || onY)*0.05, (y - oldY)*(onX || onY)*0.05, (x - oldX)*onZ*0.05); // 'x'�Ǵ� 'y'�� �����ٸ� x-y plane �󿡼� �̵�. 'z'�� �����ٸ� (ķ ��ǥ��) z�� �������� �̵�.
																										   //���� ����������, cam2wld[cameraIndex] ����� �ε������Ƿ�, �� arguments �� ���� ķ ��ǥ�� �����̴�.
			glMultMatrixd(wld2cam[cameraIndex].matrix());
			glMultMatrixd((*obj2wldPtr).matrix());
			//�� �� ����� ���� �ᱹ obj2cam �� ��ġ
			//�� �� ����� current matrix�� ���������ν� ķ ��ǥ�� ������ ��ȯ�� �� ��ǥ�� �������� ȯ���Ѵ�. translation�� world�� �ݿ��Ѵ�.
			glGetDoublev(GL_MODELVIEW_MATRIX, (*obj2wldPtr).matrix());
		}
		if (isRotate == 1 && modelOrView == 1) { // �� �� �ִ� �����̰�, viewing space���� �̵��� ���Ѵٸ�,
			tmpOnX = onX; tmpOnY = onY; tmpOnZ = onZ; // tmpOnK <-swap-> onK
			onX = 0; onY = 0; onZ = 0; // 0���� ����� �����̵��� �۵����� �ʴ´�.
			setRotationAxis();
			glLoadMatrixd((*obj2wldPtr).matrix());
			glRotated((x - oldX), angleX, angleY, angleZ);
			//renderRotaion() ������ ��� �⺻���� modeling space�� ȸ��. ���� v���¿��� rŰ�� ���� �� �����ȴ�.
			glGetDoublev(GL_MODELVIEW_MATRIX, (*obj2wldPtr).matrix());
		}
		oldX = x; // ��ġ�� �����Ѵ�
		oldY = y;
	}

	glPopMatrix();

	if (selectMode) {
		display();
		glutSwapBuffers();
	}
	//end
    glutPostRedisplay();

}


/*********************************************************************************
* Call this part whenever user types keyboard.
* This part is called in main() function by registering on glutKeyboardFunc(onKeyPress).
**********************************************************************************/
void onKeyPress( unsigned char key, int x, int y ) {
    // If 'c' or space bar are pressed, alter the camera.
    // If a number is pressed, alter the camera corresponding the number.
    if ( ( key == ' ' ) || ( key == 'c' ) ) {
        printf( "Toggle camera %d\n", cameraIndex );
        cameraIndex += 1;
		//edit
		if ( isRotate == 1 && modelOrView == 1 ) setRotationAxis(); // viewing space���� ȸ���ϴ� ��, ī�޶� ������ �� ȸ���൵ ���� �ٲ��ش�.
		//end
	}
	else if ((key >= '0') && (key <= '9')) {
		cameraIndex = key - '0';
		//edit
		if (isRotate == 1 && modelOrView == 1) setRotationAxis(); // viewing space���� ȸ���ϴ� ��, ī�޶� ������ �� ȸ���൵ ���� �ٲ��ش�.
		//end
	}

	if (cameraIndex >= (int)wld2cam.size())
		cameraIndex = 0;

    // (Project 2,3,4) TODO : Implement here to handle keyboard input.
	//edited
	if (key == 'p' || key == 'd' || key == 'o' || key == 't') {
		pdot = 1 * (key == 'p') + 2 * (key == 'd') + 3 * (key == 'o') + 4 * (key == 't');
		if (isRotate && modelOrView == 0) {
			glutIdleFunc(NULL);
			isRotate = 0;
		}
	}

	if (key == 'h') {
		printf("========================================\n");
		printf("Key Map\n");
		printf("========================================\n");
		printf("s: selection mode toggle\n"
		    	"x, y, z: translate along each axis\n"
				"r: rotate\n"
				"v: viewing space\n"
				"m: modeling space\n"
				"p: pan\n"
				"d: dolly\n"
				"o: zoom\n"
				"t: trackball\n"
				"b: toggle show back buffer\n");
		printf("========================================\n");
	}

	if (key == 's') selectOn = 1 - selectOn; // object selection enable/disable

	if (key == 'b') {
		selectMode = 1 - selectMode; // back buffer mode on/off
	}
	
	if (key == 'v') {
		if (isRotate == 1 && modelOrView == 0) { // modeling space���� ȸ�� �� �̸�,
			glutIdleFunc(NULL); // stop rotating
			modelOrView = 1;
			setRotationAxis();
			//�ϴ� ȸ���� ���߰� viewing space���� ���� �����ϰ� ��ٸ���.
		}
		modelOrView = 1;
	}

	if (key == 'm') {
		if (isRotate == 1 && modelOrView == 1) { // viewing space���� ȸ�� �� �̸�,
			modelOrView = 0;
			setRotationAxis();
			glutIdleFunc(renderRotation);
			//modeling space���� ���� ������ �� ��� ȸ���Ѵ�.
		}
		modelOrView = 0;
	}

	if (key == 'x' || key == 'y' || key == 'z'){
		pdot = 0;
		if (isRotate == 1) { //ȸ�� ���̸� ȸ���� �����.
			if (modelOrView == 0)		glutIdleFunc(NULL); // stop rotating
			isRotate = 0;
		}
		onX = (key == 'x');	onY = (key == 'y'); onZ = (key == 'z'); // �� Ű�� ��������, �ش��ϴ� onK �� 1��, �ƴϸ� 0����
	}
	
	if (key == 'r') {
		pdot = 0;
		if (isRotate == 1) { //ȸ�� ���̸� ȸ���� �����. ���� translation ������ �����϶�
			if (modelOrView == 0)		glutIdleFunc(NULL); // stop rotating
			isRotate = 0;
			onX = tmpOnX; onY = tmpOnY; onZ = tmpOnZ; // re-assignment
		}
		else {
			tmpOnX = onX; tmpOnY = onY; tmpOnZ = onZ; // tmpOnK <-swap-> onK
			onX = 0; onY = 0; onZ = 0; // 0���� ����� �����̵��� �۵����� �ʴ´�.
			isRotate = 1;
			setRotationAxis();
			if (modelOrView == 0)		glutIdleFunc(renderRotation); //modeling �������� ȸ���� ���ϸ� IdleFunc����
		}
	}
	//end

	if (selectMode) {
		display();
		glutSwapBuffers();
	}

    glutPostRedisplay();
}

void main( int argc, char *argv[] ) {
    width = 800;
    height = 600;
    frame = 0;
    glutInit( &argc, argv );						// Initialize openGL.
    glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGB );	// Initialize display mode. This project will use double buffer and RGB color.
    glutInitWindowSize( width, height );				// Initialize window size.
    glutInitWindowPosition( 100, 100 );				// Initialize window coordinate.
    glutCreateWindow( "Simple Scene" );				// Make window whose name is "Simple Scene".
    glutDisplayFunc( display );						// Register display function to call that when drawing screen event is needed.
    glutReshapeFunc( reshape );						// Register reshape function to call that when size of the window is changed.
    glutKeyboardFunc( onKeyPress );					// Register onKeyPress function to call that when user presses the keyboard.
    glutMouseFunc( onMouseButton );					// Register onMouseButton function to call that when user moves mouse.
    glutMotionFunc( onMouseDrag );					// Register onMouseDrag function to call that when user drags mouse.
    int rv, gv, bv;
    glGetIntegerv( GL_RED_BITS, &rv );					// Get the depth of red bits from GL.
    glGetIntegerv( GL_GREEN_BITS, &gv );				// Get the depth of green bits from GL.
    glGetIntegerv( GL_BLUE_BITS, &bv );				// Get the depth of blue bits from GL.
    printf( "Pixel depth = %d : %d : %d\n", rv, gv, bv );
    initialize();									// Initialize the other thing.
    glutMainLoop();									// Execute the loop which handles events.
}