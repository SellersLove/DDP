#include "raytracer.h"
#include "bmp_io.h"
#include <cmath>
#include <iostream>
#include <fstream>
#include <cstdlib>
using namespace std;

Raytracer::Raytracer() : _lightSource(NULL), _paraLightSource(NULL) {
	_root = new SceneDagNode();
}

Raytracer::~Raytracer() {
	delete _root;
}

SceneDagNode* Raytracer::addObject( SceneDagNode* parent, 
		SceneObject* obj, Material* mat ) {
	SceneDagNode* node = new SceneDagNode( obj, mat );
	node->parent = parent;
	node->next = NULL;
	node->child = NULL;
	
	// Add the object to the parent's child list, this means
	// whatever transformation applied to the parent will also
	// be applied to the child.

	if (parent->child == NULL) {
		parent->child = node;
	}
	else {
		parent = parent->child;
		while (parent->next != NULL) {
			parent = parent->next;
		}
		parent->next = node;
	}
	
	return node;;
}

LightListNode* Raytracer::addLightSource( LightSource* light ) {
	LightListNode* tmp = _lightSource;
	_lightSource = new LightListNode( light, tmp );
	return _lightSource;
}

ParaLightListNode* Raytracer::addParaLightSource( ParaLightSource* paraLight ) {
	ParaLightListNode* tmp = _paraLightSource;
	_paraLightSource = new ParaLightListNode( paraLight, tmp );
	return _paraLightSource;
}

void Raytracer::rotate( SceneDagNode* node, char axis, double angle ) {
	Matrix4x4 rotation;
	double toRadian = 2*M_PI/360.0;
	int i;
	
	for (i = 0; i < 2; i++) {
		switch(axis) {
			case 'x':
				rotation[0][0] = 1;
				rotation[1][1] = cos(angle*toRadian);
				rotation[1][2] = -sin(angle*toRadian);
				rotation[2][1] = sin(angle*toRadian);
				rotation[2][2] = cos(angle*toRadian);
				rotation[3][3] = 1;
			break;
			case 'y':
				rotation[0][0] = cos(angle*toRadian);
				rotation[0][2] = sin(angle*toRadian);
				rotation[1][1] = 1;
				rotation[2][0] = -sin(angle*toRadian);
				rotation[2][2] = cos(angle*toRadian);
				rotation[3][3] = 1;
			break;
			case 'z':
				rotation[0][0] = cos(angle*toRadian);
				rotation[0][1] = -sin(angle*toRadian);
				rotation[1][0] = sin(angle*toRadian);
				rotation[1][1] = cos(angle*toRadian);
				rotation[2][2] = 1;
				rotation[3][3] = 1;
			break;
		}
		if (i == 0) {
		    node->trans = node->trans*rotation; 	
			angle = -angle;
		} 
		else {
			node->invtrans = rotation*node->invtrans; 
		}	
	}
}

void Raytracer::translate( SceneDagNode* node, Vector3D trans ) {
	Matrix4x4 translation;
	
	translation[0][3] = trans[0];
	translation[1][3] = trans[1];
	translation[2][3] = trans[2];
	node->trans = node->trans*translation; 	
	translation[0][3] = -trans[0];
	translation[1][3] = -trans[1];
	translation[2][3] = -trans[2];
	node->invtrans = translation*node->invtrans; 
}

void Raytracer::scale( SceneDagNode* node, Point3D origin, double factor[3] ) {
	Matrix4x4 scale;
	
	scale[0][0] = factor[0];
	scale[0][3] = origin[0] - factor[0] * origin[0];
	scale[1][1] = factor[1];
	scale[1][3] = origin[1] - factor[1] * origin[1];
	scale[2][2] = factor[2];
	scale[2][3] = origin[2] - factor[2] * origin[2];
	node->trans = node->trans*scale; 	
	scale[0][0] = 1/factor[0];
	scale[0][3] = origin[0] - 1/factor[0] * origin[0];
	scale[1][1] = 1/factor[1];
	scale[1][3] = origin[1] - 1/factor[1] * origin[1];
	scale[2][2] = 1/factor[2];
	scale[2][3] = origin[2] - 1/factor[2] * origin[2];
	node->invtrans = scale*node->invtrans; 
}

Matrix4x4 Raytracer::initInvViewMatrix( Point3D eye, Vector3D view, 
		Vector3D up ) {
	
	Matrix4x4 mat; 
	Vector3D w;
	view.normalize();
	up = up - up.dot(view)*view;
	up.normalize();
	w = view.cross(up);

	mat[0][0] = w[0];
	mat[1][0] = w[1];
	mat[2][0] = w[2];
	mat[0][1] = up[0];
	mat[1][1] = up[1];
	mat[2][1] = up[2];
	mat[0][2] = -view[0];
	mat[1][2] = -view[1];
	mat[2][2] = -view[2];
	mat[0][3] = eye[0];
	mat[1][3] = eye[1];
	mat[2][3] = eye[2];

	return mat; 

}

void Raytracer::traverseScene( SceneDagNode* node, Ray3D& ray ) {
	SceneDagNode *childPtr;

	// Applies transformation of the current node to the global
	// transformation matrices.
	_modelToWorld = _modelToWorld*node->trans;
	_worldToModel = node->invtrans*_worldToModel; 
	if (node->obj) {
		// Perform intersection.
		if (node->obj->intersect(ray, _worldToModel, _modelToWorld)) {
			ray.intersection.mat = node->mat;
		}
	}
	// Traverse the children.
	childPtr = node->child;
	while (childPtr != NULL) {
		traverseScene(childPtr, ray);
		childPtr = childPtr->next;
	}

	// Removes transformation of the current node from the global
	// transformation matrices.
	_worldToModel = node->trans*_worldToModel;
	_modelToWorld = _modelToWorld*node->invtrans;
}


void Raytracer::projectionShading( Ray3D& ray ){
	LightListNode* curLight = _lightSource;
	
	for (;;) {
		if (curLight == NULL) break;
		
		curLight->light->projectContinuousTest(ray, 5, 1.0/24.0);
		//curLight->light->projectDiscrete(ray);
		
		curLight->light->shade(ray);
		
		curLight = curLight->next;
	}	
}

void Raytracer::projectionParaShading( Ray3D& ray ){
	ParaLightListNode* curLight = _paraLightSource;
	
	for (;;) {
		if (curLight == NULL) break;
		
		curLight->paraLight->projectContinuous(ray, 1, 0.5);
		
		curLight->paraLight->shade(ray);
		
		curLight = curLight->next;
	}	
}

void Raytracer::initPixelBuffer() {
	int numbytes = _scrWidth * _scrHeight * sizeof(unsigned char);
	_rbuffer = new unsigned char[numbytes];
	_gbuffer = new unsigned char[numbytes];
	_bbuffer = new unsigned char[numbytes];
	for (int i = 0; i < _scrHeight; i++) {
		for (int j = 0; j < _scrWidth; j++) {
			_rbuffer[i*_scrWidth+j] = 0;
			_gbuffer[i*_scrWidth+j] = 0;
			_bbuffer[i*_scrWidth+j] = 0;
		}
	}
}

void Raytracer::flushPixelBuffer( char *file_name ) {
	bmp_write( file_name, _scrWidth, _scrHeight, _rbuffer, _gbuffer, _bbuffer );
	delete _rbuffer;
	delete _gbuffer;
	delete _bbuffer;
}

Colour Raytracer::shadeRay( Ray3D& ray ) {
	Colour col(0.0, 0.0, 0.0); 
	traverseScene(_root, ray); 
	
	
	// Don't bother shading if the ray didn't hit 
	// anything.
	if (!ray.intersection.none) {
		//projectionShading(ray);
		projectionParaShading(ray); 
		col = ray.col;  
	}  

	return col; 
}	


void Raytracer::render( int width, int height, Point3D eye, Vector3D view, 
		Vector3D up, double fov, char* fileName ) {
	Matrix4x4 viewToWorld;
	_scrWidth = width;
	_scrHeight = height;
	double factor = (double(height)/2)/tan(fov*M_PI/360.0);

	initPixelBuffer();
	viewToWorld = initInvViewMatrix(eye, view, up);

	// Construct a ray for each pixel.
	for (int i = 0; i < _scrHeight; i++) {
		for (int j = 0; j < _scrWidth; j++) {
			// Sets up ray origin and direction in view space, 
			// image plane is at z = -1.
			Point3D origin(0, 0, 0);
			Point3D imagePlane;
			imagePlane[0] = (-double(width)/2 + 0.5 + j)/factor;
			imagePlane[1] = (-double(height)/2 + 0.5 + i)/factor;
			imagePlane[2] = -1;
				
			Ray3D ray;
			ray.origin = viewToWorld * origin;

			// Calculate and set direction
			ray.dir = viewToWorld * imagePlane - ray.origin;
			ray.dir.normalize();
			
			Colour col = shadeRay(ray); 
			_rbuffer[i*width+j] = int(col[0]*255);
			_gbuffer[i*width+j] = int(col[1]*255);
			_bbuffer[i*width+j] = int(col[2]*255);

		}
	
	}

	flushPixelBuffer(fileName);
}


void Raytracer::renderParallel( double width, double height, int resWidth, int resHeight, 
	Vector3D view, char* fileName ) {
	
	_scrWidth = resWidth;
	_scrHeight = resHeight;

	double cameraHeight = 15.0;
	double factor_x = width / resWidth;
	double factor_y	= height / resHeight;

	int numRay = 3;
	int num = 2 * numRay - 1;
	double frac_x = factor_x / ( numRay * 2);
	double frac_y = factor_y / ( numRay * 2);
	
	ofstream myfile ("data.txt");
	
	if (myfile.is_open()){
		myfile <<resWidth << " "<<resHeight << "\n";
  	}
	initPixelBuffer();
	// Construct a ray for each pixel.
	for (int i = 0; i < resHeight; i++) {
		for (int j = 0; j < resWidth; j++) {
			//set up ray in world space;
		
			Point3D imagePlane;
			Colour col; 
			Ray3D ray;
			ray.dir = view;
			ray.dir.normalize();
			double brightness = 0.0;
			imagePlane[2] = cameraHeight;
			ray.origin = imagePlane;
			//col = shadeRay(ray);			
			for (int u = -(numRay-1); u < numRay; u++){
				for (int v = -(numRay-1); v < numRay; v++){
					imagePlane[0] = (-double(width) / 2) + factor_x * j 
					 				+ 0.125 + frac_x * u;
					//imagePlane[0] = factor_x * j + 0.125 + frac_x * u;
					// imagePlane[1] = (-double(height) / 2) + factor_y * i 
					//  				+ 0.125 + frac_x * v;
					imagePlane[1] = factor_y * i + 0.125 + frac_x * v;
					// Calculate and set direction
					
					ray.origin = imagePlane;
					col = col + shadeRay(ray);
					shadeRay(ray);
					brightness = brightness + ray.brightness;
				}
			}
			
			Point3D intersection =  ray.intersection.point;
			col = (1./(num * num)) * col;
			brightness = brightness / (num * num);
			_rbuffer[i*_scrWidth+j] = int(col[0]*255);
			_gbuffer[i*_scrWidth+j] = int(col[1]*255);
			_bbuffer[i*_scrWidth+j] = int(col[2]*255);
			// _rbuffer[i*_scrWidth+j] = 156.303*brightness;
			// _gbuffer[i*_scrWidth+j] = 156.303*brightness;
			// _bbuffer[i*_scrWidth+j] = 156.303*brightness;
			
			if (myfile.is_open()){
				myfile.precision(24);
    			myfile << col[0]*255<< " "<< col[1]*255<< " "<< col[2]*255 << "\n";
    			//myfile << brightness<< " "<< brightness<< " "<< brightness << "\n";
  			}

  			// if(intersection[0] <= 0.1 && intersection[0] >= -0.1 && intersection[1] <= 0.1
  			// && intersection[1] >= -0.1 ){
  			// 	std::cout << col[0]*255 << " " << col[1]*255<< " "<<col[2]*255<<"\n";
  			// }
		}
		if (i%20 == 0){
			std::cout << "Row "<< i << " shaded.\n";
		}
	}
	myfile.close();
	flushPixelBuffer(fileName);
}

int main(int argc, char* argv[])
{	

	Raytracer raytracer;
	
	// define Width
	double width = 25; 
	double height = 30; 

	int resWidth = 100;
	int resHeight = 120;

	if (argc == 3) {
		width = atoi(argv[1]);
		height = atoi(argv[2]);
	}

	// Camera parameters.
	Point3D eye(0, 0, 30);
	Vector3D view(0, 0, -1);
	Vector3D up(0, 1, 0);
	
	double fov = 60;

	Material silver( Colour(0.19225, 0.19225, 0.19225), Colour(0.50754, 0.50754, 0.50754), 
			Colour(0.508273, 0.508273, 0.508273), 51.2 );

	double lightFactor[3] = { 6.0, 5.0, 1.0 };
	double planeFactor[3] = { 30.0, 25.0, 25.0 };
	
	// Defines a point light source.
	
	// Projector* light = new Projector ( 1.0 , Colour(0.9, 0.9, 0.9) );
	// light->scale(Point3D(0, 0, 0), lightFactor);
	// light->translate(Vector3D(0, 0, 3.0));
	// light->rotate('y', -45); 
	// Point3D pos = light->get_position();
	// Defines a para light source.
	
	// ParaLight 
	ProjectorParaLight* paraLight = new ProjectorParaLight ( Vector3D(0, 1, -1) , 
		Colour(0.9, 0.9, 0.9) );
	
	raytracer.addParaLightSource(paraLight);
	
	//SceneDagNode* plane = raytracer.addObject( new UnitDentSquare(), &silver );
	SceneDagNode* plane = raytracer.addObject( new CarSurface(), &silver );
	
	raytracer.renderParallel(width, height, resWidth, resHeight, view, (char*)"view.bmp");

	//SceneDagNode* plane = raytracer.addObject( new UnitSquare(), &silver );
	
	//raytracer.render(resWidth, resHeight, eye, view, up, fov, "view2.bmp");
	
	//raytracer.renderParallel(width, height, resWidth, resHeight, view, (char*)"view2.bmp"); 	
	return 0;
}

