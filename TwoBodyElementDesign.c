// gcc TwoBodyElementForce.c -o temp -lglut -lm -lGLU -lGL
//To stop hit "control c" in the window you launched it from.
#include <GL/glut.h>
#include <GL/glu.h>
#include <GL/gl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define XWindowSize 1000
#define YWindowSize 1000

#define PI 3.14159265359

#define STOP_TIME 10000.0
#define DT        0.0001

#define GRAVITY 0.1

#define MASS 10.0  
#define DIAMETER 1.0
#define STRING_STRENGTH 50.0

#define DAMP 0.0000001

#define DRAW 100

#define LENGTH_OF_BOX 3.0

const float XMax = LENGTH_OF_BOX;
const float YMax = LENGTH_OF_BOX;
const float ZMax = LENGTH_OF_BOX;
const float XMin = -LENGTH_OF_BOX;
const float YMin = -LENGTH_OF_BOX;
const float ZMin = -LENGTH_OF_BOX;

// Globals
double Px1, Py1, Pz1, Vx1, Vy1, Vz1, Fx1, Fy1, Fz1, Mass1, Radius1, Temperature1;
double Px2, Py2, Pz2, Vx2, vy2, Vz2, Fx2, Fy2, Fz2, Mass2, Radius2, Temperature2;

void set_initail_conditions()
{
	Px1 = 2.0;
	Py1 = 0.0;
	Pz1 = 0.0;
	Px2 = -2.0;
	Py2 = 0.0;
	Pz2 = 0.0;
	Vx1 = -1.0;
	Vy1 = 0.0;
	Vz1 = 0.0;
	Vx2 = 1.0;
	vy2 = 0.0;
	Vz2 = 0.0;
	Mass1 = 1.0;
	Mass2 = 1.0;
	Temperature1 = 1.0;
	Temperature2 = 1.0;
	Radius1 = DIAMETER/2.0;
	Radius2 = DIAMETER/2.0;
}

void Drawwirebox()
{
	glColor3f (5.0,1.0,1.0);
	glBegin(GL_LINE_STRIP);
	glVertex3f(XMax,YMax,ZMax);
	glVertex3f(XMax,YMax,ZMin);
	glVertex3f(XMax,YMin,ZMin);
	glVertex3f(XMax,YMin,ZMax);
	glVertex3f(XMax,YMax,ZMax);
	glVertex3f(XMin,YMax,ZMax);
	glVertex3f(XMin,YMax,ZMax);
	glVertex3f(XMin,YMax,ZMin);
	glVertex3f(XMin,YMin,ZMin);
	glVertex3f(XMin,YMin,ZMax);
	glVertex3f(XMin,YMax,ZMax);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(XMin,YMin,ZMax);
	glVertex3f(XMax,YMin,ZMax);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(XMin,YMin,ZMin);
	glVertex3f(XMax,YMin,ZMin);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(XMin,YMax,ZMin);
	glVertex3f(XMax,YMax,ZMin);
	glEnd();
}

void draw_picture()
{
	glClear(GL_COLOR_BUFFER_BIT);
	glClear(GL_DEPTH_BUFFER_BIT);
	Drawwirebox();
	glColor3d(1.0,0.5,1.0);
	glPushMatrix();
	glTranslatef(Px1, Py1, Pz1);
	glutSolidSphere(Radius1,20,20);
	glPopMatrix();
	glColor3d(0.0,0.5,0.0);
	glPushMatrix();
	glTranslatef(Px2, Py2, Pz2);
	glutSolidSphere(Radius2,20,20);
	glPopMatrix();
	glutSwapBuffers();
}

void keep_in_box()
{
	if(Px1 > LENGTH_OF_BOX)
	{
		Px1 = 2.0*LENGTH_OF_BOX - Px1;
		Vx1 = - Vx1;
	}
	else if(Px1 < -LENGTH_OF_BOX)
	{
		Px1 = -2.0*LENGTH_OF_BOX - Px1;
		Vx1 = - Vx1;
	}
	if(Py1 > LENGTH_OF_BOX)
	{
		Py1 = 2.0*LENGTH_OF_BOX - Py1;
		Vy1 = - Vy1;
	}
	else if(Py1 < -LENGTH_OF_BOX)
	{
		Py1 = -2.0*LENGTH_OF_BOX - Py1;
		Vy1 = - Vy1;
	}
	if(Pz1 > LENGTH_OF_BOX)
	{
		Pz1 = 2.0*LENGTH_OF_BOX - Pz1;
		Vz1 = - Vz1;
	}
	else if(Pz1 < -LENGTH_OF_BOX)
	{
		Pz1 = -2.0*LENGTH_OF_BOX - Pz1;
		Vz1 = - Vz1;
	}
	if(Px2 > LENGTH_OF_BOX)
	{
		Px2 = 2.0*LENGTH_OF_BOX - Px2;
		Vx2 = - Vx2;
	}
	else if(Px2 < -LENGTH_OF_BOX)
	{
		Px2 = -2.0*LENGTH_OF_BOX - Px2;
		Vx2 = - Vx2;
	}
	if(Py2 > LENGTH_OF_BOX)
	{
		Py2 = 2.0*LENGTH_OF_BOX - Py2;
		vy2 = - vy2;
	}
	else if(Py2 < -LENGTH_OF_BOX)
	{
		Py2 = -2.0*LENGTH_OF_BOX - Py2;
		vy2 = - vy2;
	}
	if(Pz2 > LENGTH_OF_BOX)
	{
		Pz2 = 2.0*LENGTH_OF_BOX - Pz2;
		Vz2 = - Vz2;
	}
	else if(Pz2 < -LENGTH_OF_BOX)
	{
		Pz2 = -2.0*LENGTH_OF_BOX - Pz2;
		Vz2 = - Vz2;
	}
}

void get_forces()
{
	double dpx,dpy,dpz,d,d2,d3,forceMag;
	double dvx,dvy,dvz,v,v2;
	double inOut;
	double PushBack, PushBackReduction, dVolume, volume1, volume2;
	double stuff; 
	double radius1Squared, radius2Squared, radius1Cubed, radius2Cubed;
	double shell;
	
	dpx = Px2 - Px1;
	dpy = Py2 - Py1;
	dpz = Pz2 - Pz1;
	
	d2 = dpx*dpx + dpy*dpy + dpz*dpz;
	d = sqrt(d2);
	d3 = d2*d;

	double gravityUp = 100.0;
	double nR = 1000.0;
	double volumeReductionDividerIn = 1000.0;
	double volumeReductionDividerOut = 100.0;
	double presure1;
	double presure2;
	double averagePresure;
	double dTemp = 300.9;
	double EnergyLoose;
	PushBackReduction = 0.6;
	shell = 0.0001;
	
	if(Radius1+Radius2 <= d)
	{
		forceMag =  gravityUp*Mass1*Mass2*GRAVITY/d2;
	}
	else if( Radius1+Radius2 - 2.0*shell <= d)
	{
		forceMag =  gravityUp*Mass1*Mass2*GRAVITY/d2;
		
		radius1Squared = Radius1*Radius1;
		radius2Squared = Radius2*Radius2;
		radius1Cubed   = Radius1*Radius1*Radius1;
		radius2Cubed   = Radius2*Radius2*Radius2;
		
		volume1 = (4.0*PI*radius1Cubed)/3.0;
		volume2 = (4.0*PI*radius2Cubed)/3.0;
		presure1 = nR*Temperature1/volume1;
		presure2 = nR*Temperature2/volume2;
		
		averagePresure = (presure1 + presure2)/2.0;
		
		dVolume = 4.0*radius1Cubed/3.0 - radius1Squared*d + d2/12.0;
		
		PushBack = averagePresure*dVolume;
		forceMag  += -PushBack;
	}
	else
	{
		//forceMag =  gravityUp*Mass1*Mass2*GRAVITY/(Radius1+Radius2); // Locked gravity at the touching amount
		forceMag =  gravityUp*Mass1*Mass2*GRAVITY/d2;
		
		radius1Squared = Radius1*Radius1;
		radius2Squared = Radius2*Radius2;
		radius1Cubed   = Radius1*Radius1*Radius1;
		radius2Cubed   = Radius2*Radius2*Radius2;
		
		volume1 = (4.0*PI*radius1Cubed)/3.0;
		volume2 = (4.0*PI*radius2Cubed)/3.0;
		presure1 = nR*Temperature1/volume1;
		presure2 = nR*Temperature2/volume2;
		
		averagePresure = (presure1 + presure2)/2.0;
		
		dVolume = 4.0*radius1Cubed/3.0 - radius1Squared*d + d2/12.0;
		
		dvx = Vx1 - Vx2;
		dvy = Vy1 - vy2;
		dvz = Vz1 - Vz2;
		
		inOut = dpx*dvx + dpy*dvy + dpz*dvz;
		if(0.0 <= inOut) // Moving in
		{
			PushBack = averagePresure*dVolume;
			forceMag  += -PushBack;
			
			// Reducing radius
			stuff = (3.0*(volume1 - dVolume/volumeReductionDividerIn))/(4.0*PI);
			Radius1 = pow(stuff, 1.0/3.0);
			stuff = (3.0*(volume2 - dVolume/volumeReductionDividerIn))/(4.0*PI);
			Radius2 = pow(stuff, 1.0/3.0);
		}
		else  // Moving out
		{
			PushBack = averagePresure*dVolume;
			forceMag  += -PushBackReduction*PushBack;
		
			// Increasing radius
			//stuff = (3.0*(volume1 + dVolume/volumeReductionDividerOut))/(4.0*PI);
			//Radius1 = pow(stuff, 1.0/3.0);
			//stuff = (3.0*(volume2 + dVolume/volumeReductionDividerOut))/(4.0*PI);
			//Radius2 = pow(stuff, 1.0/3.0);
			
			v2 = dvx*dvx + dvy*dvy + dvz*dvz;
			v = sqrt(v2);
			
			// Finding the linear energy that was lost because we reduced the pushBack force
			// so we can put it into heat. We do this by finding the difference between the work
			// required to move the particles into each other and the work given back when the particles 
			// move apart (in this time step). We lost potientail because the pushBack was reduced.
			// Force is the pushBack and distance is v*DT
			EnergyLoose = PushBack*(1.0 - PushBackReduction)*v*DT;
			
			printf("lll=%f\n", EnergyLoose/DT);
			
			// Putting that lost energy into heat.
			Temperature1 += 1.0*EnergyLoose/2.0;
			Temperature2 += 1.0*EnergyLoose/2.0;
			
			//Temperature1 += dTemp*DT;
			//Temperature2 += dTemp*DT;
		}
	}

	Fx1 =  forceMag*dpx/d;
	Fy1 =  forceMag*dpy/d;
	Fz1 =  forceMag*dpz/d;
	Fx2 = -forceMag*dpx/d;
	Fy2 = -forceMag*dpy/d;
	Fz2 = -forceMag*dpz/d;
}

void move_bodies()
{
	Vx1 += DT*Fx1/Mass1 - DAMP*Vx1/Mass1;
	Vy1 += DT*Fy1/Mass1 - DAMP*Vy1/Mass1;
	Vz1 += DT*Fz1/Mass1 - DAMP*Vz1/Mass1;
	Vx2 += DT*Fx2/Mass2 - DAMP*Vx2/Mass2;
	vy2 += DT*Fy2/Mass2 - DAMP*vy2/Mass2;
	Vz2 += DT*Fz2/Mass2 - DAMP*Vz2/Mass2;

	Px1 += DT*Vx1;
	Py1 += DT*Vy1;
	Pz1 += DT*Vz1;
	Px2 += DT*Vx2;
	Py2 += DT*vy2;
	Pz2 += DT*Vz2;
	keep_in_box();
}

void nbody()
{
	int    tdraw = 0;
	double  time = 0.0;

	set_initail_conditions();
	draw_picture();
	while(time < STOP_TIME)
	{
		get_forces();
		move_bodies();
		tdraw++;
	if(tdraw == DRAW)
	{
		draw_picture();
		tdraw = 0;
	}
		time += DT;
	}
	printf("\n DONE \n");
	while(1);
}

void Display(void)
{
	gluLookAt(0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
	glClear(GL_COLOR_BUFFER_BIT);
	glClear(GL_DEPTH_BUFFER_BIT);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glutSwapBuffers();
	glFlush();
	nbody();
}

void reshape(int w, int h)
{
	glViewport(0, 0, (GLsizei) w, (GLsizei) h);

	glMatrixMode(GL_PROJECTION);

	glLoadIdentity();

	glFrustum(-0.2, 0.2, -0.2, 0.2, 0.2, 50.0);

	glMatrixMode(GL_MODELVIEW);
}

int main(int argc, char** argv)
{
	glutInit(&argc,argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGB);
	glutInitWindowSize(XWindowSize,YWindowSize);
	glutInitWindowPosition(0,0);
	glutCreateWindow("2 Body 3D");
	GLfloat light_position[] = {1.0, 1.0, 1.0, 0.0};
	GLfloat light_ambient[]  = {0.0, 0.0, 0.0, 1.0};
	GLfloat light_diffuse[]  = {1.0, 1.0, 1.0, 1.0};
	GLfloat light_specular[] = {1.0, 1.0, 1.0, 1.0};
	GLfloat lmodel_ambient[] = {0.2, 0.2, 0.2, 1.0};
	GLfloat mat_specular[]   = {1.0, 1.0, 1.0, 1.0};
	GLfloat mat_shininess[]  = {10.0};
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glShadeModel(GL_SMOOTH);
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lmodel_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_DEPTH_TEST);
	glutDisplayFunc(Display);
	glutReshapeFunc(reshape);
	glutMainLoop();
	return 0;
}
