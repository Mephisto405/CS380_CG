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

unsigned floorTexID;
int frame = 0;
int width, height;
int selectMode, oldX, oldY;
//edit
int onX = 1; int onY = 0; int onZ = 0; // K키가 눌러지면 onK는 1, 아니라면 0
int tmpOnX = onX; int tmpOnY = onY; int tmpOnZ = onZ; // r 키가 눌러졌을 때 onK의 값을 tmpOnK에 임시저장하고 onK는 0으로. 그러면 회전도중 드래그를 해도 소는 움직이지 않음. 다시 r키가 눌러지면 저장했던 값을 onK에.
GLdouble angleX = 0; GLdouble angleY = 0; GLdouble angleZ = 0; // glRotated의 x,y,z 자리에 들어갈 글로벌 변수
int rCount = 0; // 소가 돌고있으면 1, 멈춰있으면 0
//end
void drawFrame( float len );

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
void drawRotFrame() { // drawFrame()과 흡사하다. 회전하는 방향(angleX, ...)에 대응하는 회전축을 흰색으로 그린다. 길이는 10이고 중심은 소의 모델링 공간 (0,0,0)에 있다.
	if (rCount != 0) { // 즉, 소가 돌지 않고 멈춰있다면 회전축을 그릴 필요가 없다.
		GLdouble norm = sqrt(angleX*angleX + angleY*angleY + angleZ*angleZ);
		glDisable(GL_LIGHTING);
		glBegin(GL_LINES);
		glColor3d(1, 1, 1);
		if (norm != 0.0) {
			glVertex3d(-5 * angleX / norm, -5 * angleY / norm, -5 * angleZ / norm);
			glVertex3d(5 * angleX / norm, 5 * angleY / norm, 5 * angleZ / norm);
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
		drawRotFrame();
		//end
        float frontColor[] = {0.8f, 0.2f, 0.9f, 1.0f};
        glEnable( GL_LIGHTING );
        glMaterialfv( GL_FRONT, GL_AMBIENT, frontColor );		// Set ambient property frontColor.
        glMaterialfv( GL_FRONT, GL_DIFFUSE, frontColor );		// Set diffuse property frontColor.
    } else {
        glDisable( GL_LIGHTING );								// Disable lighting in backbuffer mode.
		double3 color = munge( 32 );									// Match the corresponding constant color to r, g, b. You can change the color of camera on backbuffer
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
        float frontColor[] = {0.8, 0.3, 0.1, 1.0};
        glEnable( GL_LIGHTING );
        glMaterialfv( GL_FRONT, GL_AMBIENT, frontColor );
        glMaterialfv( GL_FRONT, GL_DIFFUSE, frontColor );
    } else {
        glDisable( GL_LIGHTING );
        double3 color = munge( 33 );
        glColor3dv(color.data() );
    }
    glCallList( betID );
    glPopMatrix();
}


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
		double3 color = munge(34);
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
		double3 color = munge(35);
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
    if ( selectMode == 0 )
        glClearColor( 0, 0.6, 0.8, 1 );								// Clear color setting
    else
        glClearColor( 0, 0, 0, 1 );									// When the backbuffer mode, clear color is set to black
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );				// Clear the screen
    setCamera();													// Locate the camera's position, and draw all of them.

    drawFloor();													// Draw floor.
    drawCow();														// Draw cow.
    //drawBet();


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
    gluPerspective( 45, aspect, 1, 1024 );
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
            selectMode = 1;
            display();
            glReadBuffer( GL_BACK );
            unsigned char pixel[3];
            glReadPixels( x, y, 1, 1, GL_RGB, GL_UNSIGNED_BYTE, pixel );
			double3 pixel_d3 = { double(pixel[0]), double(pixel[1]), double(pixel[2]) };
			printf("pixel = %d\n", unmunge(pixel_d3));
            selectMode = 0;

            // (Project 4) TODO : Perform the proper task about selected object.
            // hint : you can recognize which object is selected by pixel value.

            // Save current clicked location of mouse here, and then use this on onMouseDrag function.
            oldX = x;
            oldY = y;
        }
    } else if ( button == GLUT_RIGHT_BUTTON ) {
        printf( "Right mouse click at (%d, %d)\n", x, y );
    }
    glutPostRedisplay();
}


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
	glPushMatrix();
	glLoadMatrixd(cow2wld.matrix()); // 소의 변환 행렬을 로드한다
	glTranslated((x - oldX)*onX*0.1, (x - oldX)*onY*0.1, (x - oldX)*onZ*0.1);// x - oldX는 드래그 보폭, onK는 어떤 방향으로 움직여야 하는지. 0.1은 임의의 속력 상수
	glGetDoublev(GL_MODELVIEW_MATRIX, cow2wld.matrix());
	oldX = x; // 위치를 저장한다
	oldY = y;
	glPopMatrix();
	//end
    glutPostRedisplay();
}

//edited
void renderRotation(void) { // IdleAnimation으로 소를 회전시키는 함수
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPushMatrix();
	glLoadMatrixd(cow2wld.matrix());
	glRotated(1, angleX, angleY, angleZ); // 정해진 회전축 (angleX, ...) 를 기준으로 1만큼 회전
	glGetDoublev(GL_MODELVIEW_MATRIX, cow2wld.matrix());
	glPopMatrix();
	glutPostRedisplay();
}
//end

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
    } else if ( ( key >= '0' ) && ( key <= '9' ) )
        cameraIndex = key - '0';

    if ( cameraIndex >= ( int )wld2cam.size() )
        cameraIndex = 0;

    // (Project 2,3,4) TODO : Implement here to handle keyboard input.
	//edited
	if (key == 'x'){
		if (rCount == 1) {
			glutIdleFunc(NULL);
			rCount = 0;
		}
		onX = 1;
		onY = 0; onZ = 0;
	}

	if (key == 'y') {
		if (rCount == 1) {
			glutIdleFunc(NULL);
			rCount = 0;
		}
		onY = 1;
		onX = 0; onZ = 0;
	}

	if (key == 'z') {
		if (rCount == 1) {
			glutIdleFunc(NULL);
			rCount = 0;
		}
		onZ = 1;
		onY = 0; onX = 0;
	}

	if (key == 'r') {
		if (rCount == 1) {
			glutIdleFunc(NULL);
			rCount = 0;
			onX = tmpOnX; onY = tmpOnY; onZ = tmpOnZ;
		}
		else {
			tmpOnX = onX; tmpOnY = onY; tmpOnZ = onZ;
			onX = 0; onY = 0; onZ = 0;
			srand((unsigned int)time(NULL));
			angleX = (GLdouble)rand(); angleY = (GLdouble)rand(); angleZ = (GLdouble)rand();
			glutIdleFunc(renderRotation);
			rCount = 1;
		}
	}
	//end
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