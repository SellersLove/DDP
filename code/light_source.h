#include "util.h"

// Base class for a light source.  You could define different types
// of lights here, but point light is sufficient for most scenes you
// might want to render.  Different light sources shade the ray 
// differently.


class LightSource {
public:
	
	virtual void shade( Ray3D& ) = 0;
	
	virtual Point3D get_position() const = 0; 
	virtual Matrix4x4 get_modelToWorld() const = 0; 
	virtual Matrix4x4 get_worldToModel() const = 0; 

	virtual void projectDiscrete( Ray3D& ) = 0;
	virtual void projectContinuous(Ray3D& ray, int M, double tau) = 0;
	virtual void projectContinuousTest(Ray3D& ray, int M, double tau) = 0;

};

class ParaLightSource {

public:

	virtual void shade( Ray3D& ) = 0;

	virtual Vector3D get_direction() const = 0; 

	virtual void projectContinuous(Ray3D& ray, int M, double tau) = 0;
};

class Projector : public LightSource {

public:
		
	Projector( float distance, Colour col) :_distance(distance), _pos(0, 0, distance), 
	_col_ambient(col), _col_diffuse(col), _col_specular(col) {
	    
	    _width = 0.5;
	    
	    _height = 0.5;
	    
	    _resolution = 10;
		
	}

	Projector(float distance, Colour ambient, Colour diffuse, Colour specular ) 
	: _distance(distance), _pos(0, 0, distance), _col_ambient(ambient), _col_diffuse(diffuse), 
	_col_specular(specular) {
	    
	    _width = 0.5;
	    
	    _height = 0.5;
	    
	    _resolution = 10;

	}
	
	void shade( Ray3D& ray );
	
	// Apply rotation about axis 'x', 'y', 'z' angle degrees to node.
	
	void rotate(char axis, double angle );

	// Apply translation in the direction of trans to node.
	
	void translate(Vector3D trans);

	// Apply scaling about a fixed point origin.
	
	void scale( Point3D origin, double factor[3] );

	Point3D get_position() const { return _pos; }

	Matrix4x4 get_modelToWorld() const {return _trans; }; 
	
	Matrix4x4 get_worldToModel() const { return _invtrans; }; 

	void projectDiscrete(Ray3D& ray);
	
	void projectContinuous(Ray3D& ray, int M, double tau);
	
	void projectContinuousTest(Ray3D& ray, int M, double tau);

private:
	
	Point3D _pos;
	
	Colour _col_ambient;
	
	Colour _col_diffuse; 
	
	Colour _col_specular; 
	
	float _distance;
    
    float _width;
    
    float _height;
    
    int _resolution;
	
	Matrix4x4 _trans;
	
	Matrix4x4 _invtrans;
	
};


class ProjectorParaLight : public ParaLightSource {

public:
		
	ProjectorParaLight(Vector3D dir, Colour col) :_direction(dir), 
	_col_ambient(col), _col_diffuse(col), _col_specular(col){}

	Vector3D get_direction() const { return _direction; }

	void shade( Ray3D& ray );
	
	void projectContinuous(Ray3D& ray, int M, double tau);
	


private:
	
	Vector3D _direction;

	Colour _col_ambient;
	
	Colour _col_diffuse; 
	
	Colour _col_specular; 

};