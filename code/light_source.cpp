#include <cmath>
#include "light_source.h"

double max(double a, double b) {
    return a < b ? b : a;
}

void Projector::shade( Ray3D& ray ) {
    // normal vector n
    Vector3D normal = ray.intersection.normal; 
    normal.normalize();

    
    // light direction l
    Vector3D lightSource = _pos - ray.intersection.point; 
    //Vector3D lightSource(0,-1,1); 
    lightSource.normalize();
    
    //light Reflection r
    Vector3D lightReflect = ( 2 * lightSource.dot(normal) ) * normal - lightSource; 
    lightReflect.normalize();

    // view direction v
    Vector3D view_vector = -ray.dir;
    view_vector.normalize();


    Colour ambLight = (ray.intersection.mat->ambient) * _col_ambient;

    Colour diffLight = max(0, lightSource.dot(normal)) * (ray.intersection.mat->diffuse) * _col_diffuse;

    Colour specLight = max(0, pow((view_vector.dot(lightReflect)), (ray.intersection.mat->specular_exp))) 
                        * (ray.intersection.mat->specular) * _col_specular;
    
    Colour colour = ambLight + diffLight + specLight;
 
    if(ray.inshadow){
        ray.col = ray.col + ambLight;
    }
    else{
        ray.col = ray.col + colour;
    }
    Colour col = ray.col;
    ray.col  = (ray.brightness * ray.col);
    //std::cout << col[0]*255 << " " << col[1]*255<< " "<<col[2]*255<<"\n";
    ray.col.clamp();

}

void Projector::rotate(char axis, double angle ) {
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

            _pos   = rotation * _pos;
            _trans = _trans * rotation;     
            angle = -angle;
        } 
        else {
            _invtrans = rotation*_invtrans; 
        }   
    }
}


void Projector::translate( Vector3D trans ) {
    
    Matrix4x4 translation;

    translation[0][3] = trans[0];
    translation[1][3] = trans[1];
    translation[2][3] = trans[2];
    
    _pos   = translation * _pos;
    _trans = _trans*translation;  
    

    translation[0][3] = -trans[0];
    translation[1][3] = -trans[1];
    translation[2][3] = -trans[2];
    
    _invtrans = translation*_invtrans; 

}

void Projector::scale(Point3D origin, double factor[3] ) {
    
    Matrix4x4 scale;
    
    scale[0][0] = factor[0];
    scale[0][3] = origin[0] - factor[0] * origin[0];
    scale[1][1] = factor[1];
    scale[1][3] = origin[1] - factor[1] * origin[1];
    scale[2][2] = factor[2];
    scale[2][3] = origin[2] - factor[2] * origin[2];
    
    _pos   = scale * _pos;
    _trans = _trans*scale;    
    
    scale[0][0] = 1/factor[0];
    scale[0][3] = origin[0] - 1/factor[0] * origin[0];
    scale[1][1] = 1/factor[1];
    scale[1][3] = origin[1] - 1/factor[1] * origin[1];
    scale[2][2] = 1/factor[2];
    scale[2][3] = origin[2] - 1/factor[2] * origin[2];
    
    _invtrans = scale*_invtrans; 

}

void Projector::projectDiscrete(Ray3D& ray){
    
    Vector3D dir = _pos - ray.intersection.point;
    Point3D origin = ray.intersection.point + 0.00001 * dir;
    
    float space = (2 * _height) / _resolution;
    
    Ray3D projectRay = Ray3D(_invtrans * origin, 
        _invtrans * dir);
    
    double t_value = (0.0 - projectRay.origin[2]) / projectRay.dir[2];
    Point3D x_check = projectRay.point_at(t_value);
    
    if(t_value > 0 && x_check[0] >= -_width && x_check[0] <= _width && 
    x_check[1] >= -_height && x_check[1] <= _height){ 

        Point3D v = x_check;

        for(int i = 0; i < _resolution; i = i + 2){
            if( -_width + (i * space)<=x_check[1] && x_check[1] <= -_width + ( (i + 1) * space))
            {       
                ray.inshadow = true;
            }
        }          
    }    
}

void Projector::projectContinuous(Ray3D& ray, int M, double tau){
    
    Vector3D dir = _pos - ray.intersection.point;
    Point3D origin = ray.intersection.point + 0.00001 * dir;
    
    float space = (2 * _height) / _resolution;
    
    Ray3D projectRay = Ray3D(_invtrans * origin, 
        _invtrans * dir);
    
    double t_value = (0.0 - projectRay.origin[2]) / projectRay.dir[2];
    Point3D x_check = projectRay.point_at(t_value);
    
    if(t_value > 0 && x_check[0] >= -_width && x_check[0] <= _width && 
    x_check[1] >= -_height && x_check[1] <= _height){ 

        float omage = M_PI / (M * tau);
        ray.brightness = (sin ( omage * x_check[1]) + 1.0) / 2.0;
         
    }    
}

void Projector::projectContinuousTest(Ray3D& ray, int M, double tau){
    
    double d = 12.5;
    Vector3D dir(0, -1, 1);
    Point3D origin = ray.intersection.point;
    
    
    Ray3D projectRay = Ray3D(origin, dir);
    
    double t_value = (origin[1] + d - origin[2])/2.0;
    Point3D x_check = projectRay.point_at(t_value);

    float omage = M_PI / (M * tau);
    ray.brightness = (sin ( omage * x_check[1]) + 1.0) / 2.0;
         
     
}

void ProjectorParaLight::shade(Ray3D& ray){
    // normal vector n
    Vector3D normal = ray.intersection.normal; 
    normal.normalize();

    // light direction l
    Vector3D lightSource = - _direction;
    lightSource.normalize();
    
    //light Reflection r
    Vector3D lightReflect = ( 2 * lightSource.dot(normal) ) * normal - lightSource; 
    lightReflect.normalize();

    // view direction v
    Vector3D view_vector = -ray.dir;
    view_vector.normalize();


    Colour ambLight = (ray.intersection.mat->ambient) * _col_ambient;

    Colour diffLight = max(0, lightSource.dot(normal)) * (ray.intersection.mat->diffuse) * _col_diffuse;

    Colour specLight = max(0, pow((view_vector.dot(lightReflect)), (ray.intersection.mat->specular_exp))) 
                        * (ray.intersection.mat->specular) * _col_specular;
    
    Colour colour = ambLight + diffLight + specLight;
 
    if(ray.inshadow){
        ray.col = ray.col + ambLight;
    }
    else{
        ray.col = ray.col + colour;
    }
    //ray.col  = ray.brightness * ray.col;
    ray.col.clamp();

}

// calculate the brightness of intersection point, using sine model
//  T = M * tau
void ProjectorParaLight::projectContinuous(Ray3D& ray, int M, double tau){

    double d = 12.5;
    // opposite direction of light direction
    Vector3D dir = - _direction;
    Point3D origin = ray.intersection.point;
    
    
    Ray3D projectRay = Ray3D(origin, dir);
    
    double t_value = (origin[1] + d - origin[2])/2.0;
    Point3D x_check = projectRay.point_at(t_value);

    float omage = 2 * M_PI / (M * tau);
    ray.brightness = 1;
    //ray.brightness = (cos( omage * x_check[2]) + 1.0) / 2.0;
}