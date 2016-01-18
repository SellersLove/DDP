#include <cmath>
#include <iostream>
#include "scene_object.h"

// bool UnitSquare::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
//     const Matrix4x4& modelToWorld ) {

	
//   Ray3D rayModelSpace = Ray3D(worldToModel * ray.origin, worldToModel * ray.dir);

//   double t_value = - (rayModelSpace.origin[2] / rayModelSpace.dir[2]);

//   Point3D x_check = rayModelSpace.point_at(t_value);

//   if(t_value <= 0){
//     return false;
//   }

//   if(x_check[0] >= -0.5 && x_check[0] <= 0.5 && 
//     x_check[1] >= -0.5 && x_check[1] <= 0.5){
//     if(ray.intersection.none || t_value < ray.intersection.t_value){
//       Intersection intersection;
//       intersection.t_value = t_value;
//       intersection.point = modelToWorld * x_check;
//       intersection.none = false;
//       intersection.normal = worldToModel.transpose() * Vector3D(0, 0, 1);
//       ray.intersection = intersection;
//       ray.intersection.normal.normalize();
//       return true;  
    
//     }
  
//   }

//   return false;

// }

bool UnitSquare::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
    const Matrix4x4& modelToWorld ) {

  
  Ray3D rayModelSpace = Ray3D(worldToModel * ray.origin, worldToModel * ray.dir);

  double t_value = - (rayModelSpace.origin[2] / rayModelSpace.dir[2]);

  Point3D x_check = rayModelSpace.point_at(t_value);

  if(t_value <= 0){
    return false;
  }

  if(x_check[0] >= -12.5 && x_check[0] <= 12.5 && 
    x_check[1] >= -15.0 && x_check[1] <= 15.0){
    if(ray.intersection.none || t_value < ray.intersection.t_value){
      Intersection intersection;
      intersection.t_value = t_value;
      intersection.point = modelToWorld * x_check;
      intersection.none = false;
      intersection.normal = worldToModel.transpose() * Vector3D(0, 0, 1);
      ray.intersection = intersection;
      ray.intersection.normal.normalize();
      return true;  
    
    }
  
  }

  return false;

}
bool SphereIntersect( Ray3D& ray, const Matrix4x4& worldToModel,
    const Matrix4x4& modelToWorld, double dentX, double dentY, double radius) {

  double t_value;
  Ray3D rayModelSpace = Ray3D(worldToModel * ray.origin, worldToModel * ray.dir);
  Vector3D rayOriginModelVector = rayModelSpace.origin - Point3D(dentX, dentY, 0.0); 
  // convert point to vector for dot product 

  double a = rayModelSpace.dir.dot(rayModelSpace.dir);
  double b = 2 * rayModelSpace.dir.dot(rayOriginModelVector);
  double c = rayOriginModelVector.dot(rayOriginModelVector) - radius * radius;
  
  double discriminant = b * b - 4 * a * c;
  if (discriminant >= 0 && (ray.intersection.none || t_value < ray.intersection.t_value)){
    
    double root1 = ( -b + sqrt(discriminant) ) / (2 * a);
    double root2 = ( -b - sqrt(discriminant) ) / (2 * a);

    double point1 = rayModelSpace.point_at(root1)[2];
    double point2 = rayModelSpace.point_at(root2)[2];
    
    if( (root1 > 0 && point1 >= 0.0 ) || ( root2 > 0 && point2 >= 0.0) ){     
      if((root1 > 0 && point1 >= 0.0 ) && ( root2 > 0 && point2 >= 0.0)){       
        if(point1 >= 0){
          t_value = root1;
        } 
        else {
          t_value = root2;
        }
      }
      else if( root1 > 0 && point1 >= 0){
        t_value = root1;
      }
      else{
        t_value = root2;
      } 
      Intersection intersection;
      Point3D intersectionPointModelSpace = rayModelSpace.point_at(t_value);
      intersection.point = modelToWorld * intersectionPointModelSpace;
      intersection.normal = worldToModel.transpose() * Vector3D(intersectionPointModelSpace[0], 
      intersectionPointModelSpace[1], intersectionPointModelSpace[2]);
      intersection.t_value = t_value;
      intersection.none = false;
      ray.intersection = intersection;
      ray.intersection.normal.normalize();
      return true;
    }
  }

  return false;
} 


// intersection with lower half ellipsoid, center of origin.
// ellipsoid: i^2/x^2 + j^2/y^2 + k^2/z^2 = 1, where k <=0.

bool ellipsoidIntersect(Ray3D& ray, const Matrix4x4& worldToModel,
    const Matrix4x4& modelToWorld, double x, double y, double z) {

  double t_value;
  // convert point to vector for dot product 
  Ray3D rayModelSpace = Ray3D(worldToModel * ray.origin, worldToModel * ray.dir);
  
  double a = ray.dir[0] * ray.dir[0] / (x * x) + ray.dir[1] * ray.dir[1] / (y * y)  
              + ray.dir[2] * ray.dir[2] / (z * z); 
  double b = 2 * (ray.origin[0] * ray.dir[0] / (x * x) + ray.origin[1] * ray.dir[1] / (y * y)  
              + ray.origin[2] * ray.dir[2] / (z * z) );
  double c = ray.origin[0] * ray.origin[0] / (x * x) + ray.origin[1] * ray.origin[1] / (y * y)  
              + ray.origin[2] * ray.origin[2] / (z * z) - 1.0; 
  
  double discriminant = b * b - 4 * a * c;
  if (discriminant >= 0 && (ray.intersection.none || t_value < ray.intersection.t_value)){
    
    double root1 = ( -b + sqrt(discriminant) ) / (2 * a);
    double root2 = ( -b - sqrt(discriminant) ) / (2 * a);

    double point1 = ray.point_at(root1)[2];
    double point2 = ray.point_at(root2)[2];
    
    if( (root1 > 0 && point1 <= 0.0 ) || ( root2 > 0 && point2 <= 0.0) ){     
      if((root1 > 0 && point1 <= 0.0 ) && ( root2 > 0 && point2 <= 0.0)){       
        if(root1 <= root2){
          t_value = root1;
        } 
        else {
          t_value = root2;
        }
      }
      else if( root1 > 0 && point1 <= 0){
        t_value = root1;
      }
      else{
        t_value = root2;
      } 
      Intersection intersection;
      Point3D intersectionPointModelSpace = rayModelSpace.point_at(t_value);
      intersection.point = modelToWorld * intersectionPointModelSpace;
      intersection.normal = worldToModel.transpose() * Vector3D(intersectionPointModelSpace[0], 
      intersectionPointModelSpace[1], intersectionPointModelSpace[2]);
      intersection.t_value = t_value;
      intersection.none = false;
      ray.intersection = intersection;
      ray.intersection.normal.normalize();
      return true;
    }
  }
  
  return false;
} 

// bool UnitDentSquare::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
//     const Matrix4x4& modelToWorld ) {
  
//   if(SphereIntersect(ray, worldToModel, modelToWorld, _dentX, _dentY, _radius)) {
//       return true;
//   }
//   else{

//     Ray3D rayModelSpace = Ray3D(worldToModel * ray.origin, worldToModel * ray.dir);

//     double t_value = - (rayModelSpace.origin[2] / rayModelSpace.dir[2]);

//     Point3D x_check = rayModelSpace.point_at(t_value);

//     if(t_value <= 0){
//         return false;
//     }

//     if(x_check[0] >= -0.5 && x_check[0] <= 0.5 && 
//        x_check[1] >= -0.5 && x_check[1] <= 0.5){
//         if(ray.intersection.none || t_value < ray.intersection.t_value){
//           Intersection intersection;
//           intersection.t_value = t_value;
//           intersection.point = modelToWorld * x_check;
//           intersection.none = false;
//           intersection.normal = worldToModel.transpose() * Vector3D(0, 0, 1);
//           ray.intersection = intersection;
//           ray.intersection.normal.normalize();
//           return true;  
//         }
//       }
//   }
  
//   return false;

// }

bool UnitDentSquare::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
    const Matrix4x4& modelToWorld ) {
  
  Ray3D rayModelSpace = Ray3D(worldToModel * ray.origin, worldToModel * ray.dir);

  double t_value = - (rayModelSpace.origin[2] / rayModelSpace.dir[2]);

  Point3D x_check = rayModelSpace.point_at(t_value);

  if(t_value <= 0){
      return false;
  }
   if(x_check[0] >= -12.5 && x_check[0] <= 12.5 && 
    x_check[1] >= -15.0 && x_check[1] <= 15.0){
      
    if(x_check[0] >= -_a && x_check[0] <= _a && x_check[1] >= -_b
      && x_check[1] <= _b ){
      
      if(ellipsoidIntersect(ray, worldToModel, modelToWorld, _a, _b, _c)){
        return true;
      }
      
      else{
        return false;
      }
    }
      
    if(ray.intersection.none || t_value < ray.intersection.t_value){
      Intersection intersection;
      intersection.t_value = t_value;
      intersection.point = modelToWorld * x_check;
      intersection.none = false;
      intersection.normal = worldToModel.transpose() * Vector3D(0, 0, 1);
      ray.intersection = intersection;
      ray.intersection.normal.normalize();
      return true;  
    }  
  }  
  
  return false;

}


// // x 30mm y 25mm z 4mm
// bool CarSurface::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
//     const Matrix4x4& modelToWorld ){
//   double t_value;
//   //Ray3D ray = Ray3D(worldToModel * ray.origin, worldToModel * ray.dir);

//   double a = - ray.dir[0] * ray.dir[0] / 156.25; 
//   double b = - (2.0 * ray.dir[0] * ray.origin[0]) / 156.25 - ray.dir[2];
//   double c = (625.0 - ray.origin[0] * ray.origin[0]) / 156.25 - ray.origin[2];

//   std::cout << ray.origin[0] << "  "<<ray.origin[1] << " " << ray.origin[2]<<"\n";
//   double discriminant = b * b - 4 * a * c;
  
//   if (discriminant >= 0 && ray.intersection.none){
    
//     double root1 = ( -b + sqrt(discriminant) ) / (2 * a);
//     double root2 = ( -b - sqrt(discriminant) ) / (2 * a);
//     if(root1 > 0 || root2 > 0){     
//       if(root1 > 0 && root2 > 0){       
//         if(root1 <= root2){
//           t_value = root1;
//         } 
//         else {
//           t_value = root2;
//         }
//       }
//       else if( root1 > 0){
//         t_value = root1;
//       }
//       else{
//         t_value = root2;
//       } 
//       //t_value = (25 - ray.origin[0]) * (ray.origin[0] + 25) / 156.25 - ray.origin[2];
//       Point3D x_check = ray.point_at(t_value);
      
//       if(x_check[0] >= 0.0 && x_check[0] <= 25.0 && 
//         x_check[1] >= -15.0 && x_check[1] <= 15.0 && x_check[2] <= 4.0 
//         && x_check[2] >= 0.0 ){
//           Intersection intersection;
//           intersection.point = modelToWorld * x_check;
//           intersection.normal = worldToModel.transpose() * Vector3D(
//           x_check[0], x_check[1], x_check[2]);
//           intersection.t_value = t_value;
//           intersection.none = false;
//           ray.intersection = intersection;
//           ray.intersection.normal.normalize();
//           return true;
//       }
//     }
//   }
//   return false;
// }

bool CarSurface::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
    const Matrix4x4& modelToWorld ){
  double t_value;
  double width = 30;
  double  t = width * width /4;
  if (ray.intersection.none){
      t_value = ((width - ray.origin[1]) * (ray.origin[1] + width) / 
        t - ray.origin[2])/ ray.dir[2];
      Point3D x_check = ray.point_at(t_value);
      
      if(x_check[0] >=-12.5 && x_check[0] <= 12.5 && 
        x_check[1] >= 0.0 && x_check[1] <= 30.0 && x_check[2] <= 4.0 
        && x_check[2] >= 0.0 ){
          Intersection intersection;
          intersection.point = modelToWorld * x_check;
          intersection.normal = worldToModel.transpose() * Vector3D(
          x_check[0], x_check[1], x_check[2]);
          intersection.t_value = t_value;
          intersection.none = false;
          ray.intersection = intersection;
          ray.intersection.normal.normalize();
          return true;
      }
    }
  return false;
}

