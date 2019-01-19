#include <iostream>
#include "parser.h"
#include "ppm.h"
#include <cmath>
#include <float.h>
#include <vector>
#include <map>
#include <limits.h>

typedef unsigned char RGB[3];

struct ray {
    parser::Vec3f o;
    parser::Vec3f d;
};

float epsilon = 1e-7;
parser::Vec3f false_indicator(FLT_MIN,FLT_MIN,FLT_MIN);

parser::Vec3i round(parser::Vec3f vec)
{
    parser::Vec3i result;
    
    result.x = round(vec.x);
    result.y = round(vec.y);
    result.z = round(vec.z);
    
    if(result.x<0)
    {
        result.x = 0;
    } 
    if(result.y<0)
    {
        result.y = 0;
    }
    if(result.z<0)
    {
        result.z = 0;
    } 
    if(result.x>255)
    {
        result.x = 255;
    }
    
    if(result.y>255)
    {
        result.y = 255;
    }
    if(result.z>255)
    {
        result.z = 255;
    }
    return result;
}
parser::Vec3i rounds(parser::Vec3i vec)
{
    parser::Vec3i result;
    
    if(vec.x<0)
    {
        result.x = 0;
    } 
    if(vec.y<0)
    {
        result.y = 0;
    }
    if(vec.z<0)
    {
        result.z = 0;
    } 
    if(vec.x>255)
    {
        result.x = 255;
    }
    
    if(vec.y>255)
    {
        result.y = 255;
    }
    if(vec.z>255)
    {
        result.z = 255;
    }
    return result;
}
float find_matrix_determinant(parser::Vec3f v1, parser::Vec3f v2, parser::Vec3f v3)
{
    float determinant = v1.x*(v2.y*v3.z-v2.z*v3.y)
                       -v2.x*(v1.y*v3.z-v1.z*v3.y)
                       +v3.x*(v1.y*v2.z-v1.z*v2.y);
    return determinant;
}

parser::Vec3f sphere_intersection(parser::Sphere sphere, ray ray,float tmin,float &t)
{
    parser::Vec3f center_vertex = sphere.center_vertex;
    parser::Vec3f result;
    
    float first_t;
    float second_t;
    float min_t;
    parser::Vec3f ray_cv = ray.o-center_vertex;
    float determinant =pow(ray.d.dotprod(ray_cv),2) - (ray.d.dotprod(ray.d))*((ray_cv).dotprod(ray_cv) - sphere.radius*sphere.radius);
    
    if(determinant<0)
    {
        return false_indicator;
    }
    
    else if(determinant == 0)
    {
      t =(-ray.d.dotprod(ray.o-center_vertex))/ray.d.dotprod(ray.d);
      if(t<tmin)
      {
          return false_indicator;
      }
      result = ray.o + ray.d*t;
    }
    
    else
    {  
      first_t = (-ray.d.dotprod(ray.o-center_vertex) + sqrt(determinant))/ray.d.dotprod(ray.d);        
      second_t = (-ray.d.dotprod(ray.o-center_vertex) - sqrt(determinant))/ray.d.dotprod(ray.d); 
      min_t = fmin(first_t,second_t);
      
      
      if(min_t<tmin)
      {
          return false_indicator;
      }
      else
      {   
          t = min_t;
        result = ray.o + ray.d*min_t;          
      }
    }
    
    return result;
}

parser::Vec3f triangle_intersection(parser::Triangle triangle, ray ray,float tmin,float &t)
{
    parser::Vec3f a = triangle.indices.v0;
    parser::Vec3f b = triangle.indices.v1;
    parser::Vec3f c = triangle.indices.v2;
    
    parser::Vec3f Aa(a.x-b.x,a.y-b.y,a.z-b.z);
    parser::Vec3f Ab(a.x-c.x,a.y-c.y,a.z-c.z);
    parser::Vec3f Ac(ray.d.x,ray.d.y,ray.d.z);
    
    parser::Vec3f changing(a.x-ray.o.x,a.y-ray.o.y,a.z-ray.o.z);
    
    float detA = find_matrix_determinant(Aa,Ab,Ac);
    
    float beta,gamma;
    
    beta = find_matrix_determinant(changing,Ab,Ac)/detA;
    gamma = find_matrix_determinant(Aa,changing,Ac)/detA;
    t = find_matrix_determinant(Aa,Ab,changing)/detA;
    //std::cout<<beta<<" "<<gamma<<" "<<t<<" "<<tmin<<std::endl;    
    if(t>tmin&&beta+gamma<=1&&beta>=0&&gamma>=0)
    {
        return ray.o + ray.d*t;
    }
    else
    {
        return false_indicator;
    }
}
parser::Vec3f triangle_intersection_mesh(parser::Face triangle, ray ray,float tmin,float &t)
{
    parser::Vec3f a = triangle.v0;
    parser::Vec3f b = triangle.v1;
    parser::Vec3f c = triangle.v2;
    
    /*parser::Vec3f v0v1 = b - a;
    parser::Vec3f v0v2 = c - a;
    parser::Vec3f vecc = ray.d.crossprod(v0v2);
            
    float det = v0v1.dotprod(vecc);
    
    if(det<epsilon)
        return false_indicator;
    
    if(fabs(det) < epsilon)
        return false_indicator;
    
    float invDet = 1/det;
    
    parser::Vec3f tvec = ray.o - a;
    float u = tvec.dotprod(vecc);
    if( u<0 || u>1)
        return false_indicator;
    
    parser::Vec3f qvec = tvec.crossprod(v0v1);
    float v = ray.d.dotprod(qvec)*invDet;
    
    if( v<0 || u+v>1)
        return false;
    
    t = v0v2.dotprod(qvec)*invDet;
    
    return ray.o + ray.d*t; */
    parser::Vec3f Aa(a.x-b.x,a.y-b.y,a.z-b.z);
    parser::Vec3f Ab(a.x-c.x,a.y-c.y,a.z-c.z);
    parser::Vec3f Ac(ray.d.x,ray.d.y,ray.d.z);
    
    parser::Vec3f changing(a.x-ray.o.x,a.y-ray.o.y,a.z-ray.o.z);
    
    float detA = find_matrix_determinant(Aa,Ab,Ac);
    
    float beta,gamma;
    
    beta = find_matrix_determinant(changing,Ab,Ac)/detA;
    gamma = find_matrix_determinant(Aa,changing,Ac)/detA;
    t = find_matrix_determinant(Aa,Ab,changing)/detA;
    
    
    if(t>tmin&&beta+gamma<=1+epsilon&&beta>=0-epsilon&&gamma>=0-epsilon)
    {
        return ray.o + ray.d*t;
    }
    else
    {
        return false_indicator;
    }
}

ray compute_viewing_ray(int width,int height,int i,int j,float near_distance,parser::Vec4f near_plane,parser::Vec3f e,parser::Vec3f up,parser::Vec3f gaze){
    
    parser::Vec3f gaze_up; 
    gaze_up = up.crossprod(-gaze);
    parser::Vec3f d;
    
    
    /*compute d = -w.d + l.u + t.v + s_u.u - s_v.v 
    d.x = gaze.x*near_distance + gaze_up.x*near_plane.w + up.x*near_plane.z + (((near_plane.x - near_plane.w)*(i + 0.5))/width)*gaze_up.x - (((near_plane.z - near_plane.y)*(j + 0.5))/height)*up.x;
    d.y = gaze.y*near_distance + gaze_up.y*near_plane.w + up.y*near_plane.z + (((near_plane.x - near_plane.w)*(i + 0.5))/width)*gaze_up.y - (((near_plane.z - near_plane.y)*(j + 0.5))/height)*up.y;
    d.z = gaze.z*near_distance + gaze_up.z*near_plane.w + up.z*near_plane.z + (((near_plane.x - near_plane.w)*(i + 0.5))/width)*gaze_up.z - (((near_plane.z - near_plane.y)*(j + 0.5))/height)*up.z; */
    
    d = (gaze*near_distance) + (gaze_up*near_plane.x) + up*near_plane.w + gaze_up*(((near_plane.y - near_plane.x)*(i + 0.5))/width) - up*(((near_plane.w - near_plane.z)*(j + 0.5))/height);
    //d.normalize();
    ray result;
    result.o = e;
    result.d = d;
    
    return result;
}

float mint = FLT_MAX;
float tinner;
parser::Vec3f intersection;
parser::Vec3f nv;
parser::Vec3f final_intersection;
parser::Material bos;
parser::Vec3f ambient;
parser::Vec3f diffuse;
parser::Vec3f mirror;
parser::Vec3f specular;
parser::Vec3f wr;
ray yansiyan_ray;

float phong_c;

parser::Vec3i calculated_colour;
parser::Vec3i ambient_colour;
parser::Vec3i diffuse_colour;
parser::Vec3i mirror_colour;
parser::Vec3i specular_colour;
parser::Vec3i background_color;

int flag = 0;
int max_recursion_depth;
float cosinus_uzulur;
float cosinus_uzulur_specular;
float light_distance;
float light_selami;
ray light_s;    
parser::Vec3f intersection_s;
parser::Vec3f recursive_path_tracking(ray ray,int count,std::vector<parser::Triangle> triangles,std::vector<parser::Mesh> meshess,std::vector<parser::Sphere> spheres,std::vector<parser::Vec3f> triangle_normals,std::vector<std::vector<parser::Vec3f>> mftriangle_normals,std::vector<parser::PointLight> point_lights,parser::Vec3f ambient_light,float shadow_ray_epsilon,parser::Vec3i background_color)
{
    parser::Vec3f calculated_colour_r;    
    parser::Vec3f ambient_colour_r;
    parser::Vec3f diffuse_colour_r;
    parser::Vec3f mirror_colour_r;
    parser::Vec3f specular_colour_r;
    parser::Vec3f background_color_r; 
    
    
    parser::Vec3f intersection;
    parser::Vec3f nv;
    parser::Vec3f final_intersection;
    parser::Material bos;
    parser::Vec3f ambient;
    parser::Vec3f diffuse;
    parser::Vec3f mirror;
    parser::Vec3f specular;
    parser::Vec3f wr; 

    bos.phong_exponent = FLT_MIN;
    parser::Material intersection_material = bos;    
    if(count == 0)
    {
        parser::Vec3f result(0,0,0);
        return result;
    }
    
    mint = FLT_MAX;
    for(int l=0;l<spheres.size();l++)
    {                       
        intersection = sphere_intersection(spheres[l],ray,0,tinner);  
        if(intersection.x != FLT_MIN)
        {                      
            if(tinner<mint)
            {
                mint = tinner;                          
                intersection_material = spheres[l].material;
                nv = intersection - spheres[l].center_vertex;
                nv.normalize();
                final_intersection = intersection;                       
            }
        }
    }
    for(int u=0;u<triangles.size();u++)
    {
        intersection = triangle_intersection(triangles[u],ray,0,tinner);
        if(intersection.x != FLT_MIN)
        {
            if(tinner<mint)
            {            
                mint = tinner;
                intersection_material = triangles[u].material;
                nv = triangle_normals[u];
                final_intersection = intersection;
            }
        }   
    }
    for(int b=0;b<meshess.size();b++)
    {
        for(int x=0;x<meshess[b].faces.size();x++)
        {                      
            intersection = triangle_intersection_mesh(meshess[b].faces[x],ray,0,tinner);                      
            if(intersection.x != FLT_MIN)
            {                                             
                if(tinner<mint)
                {          
                    //std::cout<<ray.o.x<<","<<ray.o.y<<","<<ray.o.z<<" t<"<<ray.d.x<<","<<ray.d.y<<","<<ray.d.z<<">"<<std::endl;
                    mint = tinner;
                    intersection_material = meshess[b].material;
                    nv = mftriangle_normals[b][x];
                    final_intersection = intersection;
                }
            }                     
        }
    }
    
    if(intersection_material.phong_exponent != FLT_MIN)
    {
        ambient = intersection_material.ambient;
        diffuse = intersection_material.diffuse;
        mirror = intersection_material.mirror;
        specular = intersection_material.specular;
        phong_c = intersection_material.phong_exponent;           
        ambient_colour = round(ambient*ambient_light);
        calculated_colour_r.x = ambient_colour.x;
        calculated_colour_r.y = ambient_colour.y;
        calculated_colour_r.z = ambient_colour.z;        

        for(int pl=0;pl<point_lights.size();pl++)
        {
            light_s.o = final_intersection;
            light_s.d = point_lights[pl].position - final_intersection;
            light_s.d.normalize();
            light_s.o =light_s.o + light_s.d*shadow_ray_epsilon;

            light_selami = (point_lights[pl].position.x - light_s.o.x)/light_s.d.x;
            for(int l=0;l<spheres.size();l++)
            {
                intersection_s = sphere_intersection(spheres[l],light_s,0,tinner);
                if(intersection_s.x != FLT_MIN)
                {
                    if(tinner<light_selami)
                    {
                        flag = 1;
                        break;                                    
                    }
                }

            } 
            for(int u=0;u<triangles.size()&&flag==0;u++)
            {
                intersection_s = triangle_intersection(triangles[u],light_s,0,tinner);
                if(intersection_s.x != FLT_MIN)
                {
                    if(tinner<light_selami)
                    {
                        flag = 1;
                        break;                                    
                    }
                }

            }
            for(int b=0;b<meshess.size()&&flag==0;b++)
            {
                for(int x=0;x<meshess[b].faces.size();x++)
                {
                    intersection_s = triangle_intersection_mesh(meshess[b].faces[x],light_s,0,tinner);
                    if(intersection_s.x != FLT_MIN)
                    {
                        if(tinner<light_selami)
                        {
                            flag = 1;
                            break;                                    
                        }
                    }

                }
            }                    
            if(flag == 1)
            {
                flag = 0;                           
                continue;
            }
            else
            {
                float zero = 0;                            
                parser::Vec3f dv = (point_lights[pl].position - final_intersection);
                parser::Vec3f dv2 = dv;
                dv2.normalize();
                cosinus_uzulur = std::max(dv2.dotprod(nv),zero); 
                light_distance = dv.magnitude()*dv.magnitude();
                parser::Vec3f aab = (diffuse*cosinus_uzulur*point_lights[pl].intensity);
                parser::Vec3f aaa = aab/light_distance;
                diffuse_colour_r = (diffuse*cosinus_uzulur)*point_lights[pl].intensity/light_distance; 
                //calculated_colour = calculated_colour + diffuse_colour;                          
                parser::Vec3f half = light_s.d - ray.d;
                half.normalize();

                cosinus_uzulur_specular = std::max(nv.dotprod(half),zero);
                specular_colour_r = (specular*point_lights[pl].intensity)*std::pow(cosinus_uzulur_specular,phong_c)/light_distance;

                calculated_colour_r = calculated_colour_r + diffuse_colour_r + specular_colour_r;
                if(calculated_colour_r.x<0)
                {
                    calculated_colour_r.x = 0;
                }
                if(calculated_colour_r.y<0)
                {
                    calculated_colour_r.y = 0;                               
                }
                if(calculated_colour_r.z<0)
                {
                    calculated_colour_r.z = 0;                                
                }

                if(calculated_colour_r.x>255)
                {
                    calculated_colour_r.x = 255;                                
                }
                if(calculated_colour_r.y>255)
                {
                    calculated_colour_r.y = 255;                                
                }
                if(calculated_colour_r.z>255)
                {
                    calculated_colour_r.z = 255;                                
                }

            }  
        }
        if(mirror.x ==0&&mirror.y==0&&mirror.z==0)
        {
            return calculated_colour_r;                                 
        }
        else
        {
            yansiyan_ray.o = final_intersection;
            //viewing_ray.d.normalize();
            nv.normalize();
            yansiyan_ray.d = ray.d - nv*(nv.dotprod(ray.d))*2;
            yansiyan_ray.d.normalize();
            yansiyan_ray.o = yansiyan_ray.o + yansiyan_ray.d*shadow_ray_epsilon;
            
            return calculated_colour_r + recursive_path_tracking(yansiyan_ray,count-1,triangles,meshess,spheres,triangle_normals,mftriangle_normals,point_lights,ambient_light,shadow_ray_epsilon,background_color)*mirror;                                  
        }        
    }
    else
    {        
        calculated_colour_r.x = background_color.x;
        calculated_colour_r.y = background_color.y;
        calculated_colour_r.z = background_color.z;        
        return calculated_colour_r;                     
    }
    intersection_material = bos;    
    
}
int main(int argc, char* argv[])
{
    // Sample usage for reading an XML scene file
    parser::Scene scene;

    int gostergec = 0;
    
    scene.loadFromXml(argv[1]);
    max_recursion_depth = scene.max_recursion_depth;
    int height = 0;
    int width = 0;
    float near_distance = 0;
    float tmin = 0;
    parser::Vec3f ambient_light = scene.ambient_light;
    float shadow_ray_epsilon = scene.shadow_ray_epsilon;
    
    background_color = scene.background_color;
    
    std::vector<parser::Vec3f> triangle_normals;
    
    std::map<int,std::vector<std::vector<parser::Vec3f>>> mesh_face_normals; 
    
    std::vector<std::vector<parser::Vec3f>> mftriangle_normals(scene.meshes.size());
            
    std::vector<parser::Material> meshmaterials;
    
    std::vector<parser::Material> trianglematerials;
    
    std::vector<parser::Material> spherematerials;
    
    std::vector<parser::Material> all_materials = scene.materials;
    
    std::vector<parser::Triangle> triangles = scene.triangles;
    
    std::vector<parser::Mesh> meshess = scene.meshes;
    
    std::vector<parser::Sphere> spheres = scene.spheres;
    
    std::vector<parser::Vec3f> vertex_data = scene.vertex_data;
    
    std::vector<parser::PointLight> point_lights = scene.point_lights;
    for(int i=0;i<spheres.size();i++)
    {
        spheres[i].center_vertex = vertex_data[spheres[i].center_vertex_id-1];
        spheres[i].material = all_materials[spheres[i].material_id-1];
    }    
    
    
    /*This is for precomputing triangles in the file*/
    for(int i=0;i<triangles.size();i++)
    {
        parser::Vec3f a = vertex_data[triangles[i].indices.v0_id-1];
        parser::Vec3f b = vertex_data[triangles[i].indices.v1_id-1];
        parser::Vec3f c = vertex_data[triangles[i].indices.v2_id-1];
        triangles[i].indices.v0 = a;
        triangles[i].indices.v1 = b;
        triangles[i].indices.v2 = c;
        triangles[i].material = all_materials[triangles[i].material_id-1];        
        
        parser::Vec3f v1 = a-b;
        parser::Vec3f v2 = c-b;
        parser::Vec3f normal = v2.crossprod(v1);
        normal.normalize();
        triangle_normals.push_back(normal);
    }
    
    /* This for precomputing the triangles' normals in meshes*/
    for(int i=0;i<meshess.size();i++)
    {
        meshess[i].material = all_materials[meshess[i].material_id-1];
        for(int j=0;j<meshess[i].faces.size();j++)
        {
            parser::Vec3f mfa = vertex_data[meshess[i].faces[j].v0_id-1];
            parser::Vec3f mfb = vertex_data[meshess[i].faces[j].v1_id-1];
            parser::Vec3f mfc = vertex_data[meshess[i].faces[j].v2_id-1]; 
            meshess[i].faces[j].v0 = mfa;
            meshess[i].faces[j].v1 = mfb;
            meshess[i].faces[j].v2 = mfc;
            
                    
            parser::Vec3f mfv1 = mfa-mfb;
            parser::Vec3f mfv2 = mfc-mfb;
            parser::Vec3f mfnormal = mfv2.crossprod(mfv1);
            mfnormal.normalize();
            mftriangle_normals[i].push_back(mfnormal);
        }
    }
    
    
    for(int i=0;i<scene.cameras.size();i++){
        
        /*Image Height and Width in pixels*/
        height = scene.cameras[i].image_height;
        width = scene.cameras[i].image_width;
        /*Gaze and Up vectors*/
        parser::Vec3f gaze = scene.cameras[i].gaze;
        parser::Vec3f up = scene.cameras[i].up;

        /*Position and near_plane*/
        parser::Vec4f near_plane = scene.cameras[i].near_plane;
        parser::Vec3f position = scene.cameras[i].position;

        /*Boundaries of image plane*/
        near_distance = scene.cameras[i].near_distance;
        
        parser::Vec3f m = position + gaze*near_distance;

        
        parser::Vec3f final_intersection;
        parser::Vec3i rgb;
        
        bos.phong_exponent = FLT_MIN;
        parser::Material intersection_material = bos;      
        parser::Vec3i color;

        
        unsigned char* image_my = new unsigned char [width * height * 3]; 
        
        for(int j=0;j<height;j++){
            for(int k=0;k<width;k++){
                int yer = (width*3)*j + k*3;
                ray viewing_ray = compute_viewing_ray(width,height,k,j,near_distance,near_plane,position,up,gaze);                   
                //viewing_ray.d.normalize();
                parser::Vec3f nom = m-position;
                tmin = nom.dotprod(gaze)/(viewing_ray.d).dotprod(gaze);
                //std::cout<<tmin<<std::endl;
                //std::cout<<"<"<<viewing_ray.o.x<<","<<viewing_ray.o.y<<","<<viewing_ray.o.z<<">"<<" + t<"<<viewing_ray.d.x<<","<<viewing_ray.d.y<<","<<viewing_ray.d.z<<">"<<" "<<k<<" "<<j<<std::endl;         
                mint = FLT_MAX;
                for(int l=0;l<spheres.size();l++)
                {                       
                    intersection = sphere_intersection(spheres[l],viewing_ray,0,tinner);  
                    if(intersection.x != FLT_MIN)
                    {                      
                        if(tinner<mint)
                        {
                            mint = tinner;                          
                            intersection_material = spheres[l].material;
                            nv = intersection - spheres[l].center_vertex;
                            nv.normalize();
                            final_intersection = intersection;                       
                        }
                    }
                }
                for(int u=0;u<triangles.size();u++)
                {
                    intersection = triangle_intersection(triangles[u],viewing_ray,0,tinner);
                    if(intersection.x != FLT_MIN)
                    {
                        if(tinner<mint)
                        {                                               
                            mint = tinner;
                            intersection_material = triangles[u].material;
                            nv = triangle_normals[u];
                            final_intersection = intersection;
                        }
                    }   
                }
                for(int b=0;b<meshess.size();b++)
                {
                    for(int x=0;x<meshess[b].faces.size();x++)
                    {                      
                        intersection = triangle_intersection_mesh(meshess[b].faces[x],viewing_ray,0,tinner);                      
                        if(intersection.x != FLT_MIN)
                        {                                             
                            if(tinner<mint)
                            {              
                                mint = tinner;
                                intersection_material = meshess[b].material;
                                nv = mftriangle_normals[b][x];
                                final_intersection = intersection;
                            }
                        }                     
                    }
                } 
                /*TO control whether ray intersected any object*/
                if(intersection_material.phong_exponent != FLT_MIN)
                {
                    ambient = intersection_material.ambient;
                    diffuse = intersection_material.diffuse;
                    mirror = intersection_material.mirror;
                    specular = intersection_material.specular;
                    phong_c = intersection_material.phong_exponent;           
                    ambient_colour = round(ambient*ambient_light);
                    calculated_colour = ambient_colour;
                    
                    for(int pl=0;pl<point_lights.size();pl++)
                    {
                        ray light_s;
                        light_s.o = final_intersection;
                        light_s.d = point_lights[pl].position - final_intersection;
                        light_s.d.normalize();
                        light_s.o =light_s.o + light_s.d*shadow_ray_epsilon;
                        
                        light_selami = (point_lights[pl].position.x - light_s.o.x)/light_s.d.x;
                        for(int l=0;l<spheres.size();l++)
                        {
                            intersection_s = sphere_intersection(spheres[l],light_s,0,tinner);
                            if(intersection_s.x != FLT_MIN)
                            {
                                if(tinner<light_selami)
                                {
                                    flag = 1;
                                    break;                                    
                                }
                            }
                       
                        } 
                        for(int u=0;u<triangles.size()&&flag==0;u++)
                        {
                            intersection_s = triangle_intersection(triangles[u],light_s,0,tinner);
                            if(intersection_s.x != FLT_MIN)
                            {
                                if(tinner<light_selami)
                                {
                                    flag = 1;
                                    break;                                    
                                }
                            }

                        }
                        for(int b=0;b<meshess.size()&&flag==0;b++)
                        {
                            for(int x=0;x<meshess[b].faces.size();x++)
                            {
                                intersection_s = triangle_intersection_mesh(meshess[b].faces[x],light_s,0,tinner);
                                if(intersection_s.x != FLT_MIN)
                                {
                                    if(tinner<light_selami)
                                    {
                                        flag = 1;
                                        break;                                    
                                    }
                                }

                            }
                        }                    
                        if(flag == 1)
                        {
                            flag = 0;
                            image_my[yer] = round(ambient_colour.x);
                            image_my[yer+1] = round(ambient_colour.y);
                            image_my[yer+2] = round(ambient_colour.z);                            
                            continue;
                        }
                        else
                        {
                            float zero = 0;                            
                            parser::Vec3f dv = (point_lights[pl].position - final_intersection);
                            parser::Vec3f dv2 = dv;
                            dv2.normalize();
                            cosinus_uzulur = std::max(dv2.dotprod(nv),zero); 
                            light_distance = dv.magnitude()*dv.magnitude();
                            parser::Vec3f aab = (diffuse*cosinus_uzulur*point_lights[pl].intensity);
                            parser::Vec3f aaa = aab/light_distance;
                            diffuse_colour = round((diffuse*cosinus_uzulur)*point_lights[pl].intensity/light_distance); 
                            //calculated_colour = calculated_colour + diffuse_colour;                          
                            parser::Vec3f half = light_s.d - viewing_ray.d;
                            half.normalize();
                            
                            cosinus_uzulur_specular = std::max(nv.dotprod(half),zero);
                            specular_colour = round((specular*point_lights[pl].intensity)*std::pow(cosinus_uzulur_specular,phong_c)/light_distance);
                            
                            calculated_colour = calculated_colour + diffuse_colour + specular_colour;
                            if(calculated_colour.x<0)
                            {
                                calculated_colour.x = 0;
                            }
                            if(calculated_colour.y<0)
                            {
                                calculated_colour.y = 0;                               
                            }
                            if(calculated_colour.z<0)
                            {
                                calculated_colour.z = 0;                                
                            }

                            if(calculated_colour.x>255)
                            {
                                calculated_colour.x = 255;                                
                            }
                            if(calculated_colour.y>255)
                            {
                                calculated_colour.y = 255;                                
                            }
                            if(calculated_colour.z>255)
                            {
                                calculated_colour.z = 255;                                
                            }
                                                     

                        }  
                    }
                    if(mirror.x ==0&&mirror.y==0&&mirror.z==0)
                    {
                        image_my[yer] = calculated_colour.x;
                        image_my[yer+1] = calculated_colour.y;
                        image_my[yer+2] = calculated_colour.z;                                 
                    }
                    else
                    {
                        yansiyan_ray.o = final_intersection;
                        //viewing_ray.d.normalize();
                        nv.normalize();
                        yansiyan_ray.d = viewing_ray.d - nv*(nv.dotprod(viewing_ray.d))*2;
                        yansiyan_ray.d.normalize();
                        yansiyan_ray.o = yansiyan_ray.o + yansiyan_ray.d*shadow_ray_epsilon;
                        parser::Vec3f recs = recursive_path_tracking(yansiyan_ray,max_recursion_depth,triangles,meshess,spheres,triangle_normals,mftriangle_normals,point_lights,ambient_light,shadow_ray_epsilon,background_color)*mirror;
                        
                        calculated_colour = calculated_colour + round(recs);                    
                        if(calculated_colour.x<0)
                        {
                            calculated_colour.x = 0;
                        }
                        if(calculated_colour.y<0)
                        {
                            calculated_colour.y = 0;                               
                        }
                        if(calculated_colour.z<0)
                        {
                            calculated_colour.z = 0;                                
                        }

                        if(calculated_colour.x>255)
                        {
                            calculated_colour.x = 255;                                
                        }
                        if(calculated_colour.y>255)
                        {
                            calculated_colour.y = 255;                                
                        }
                        if(calculated_colour.z>255)
                        {
                            calculated_colour.z = 255;                                
                        }                                
                        image_my[yer] = calculated_colour.x;
                        image_my[yer+1] = calculated_colour.y;
                        image_my[yer+2] = calculated_colour.z;                                  
                    }                    
                }
                else
                {
                    calculated_colour = scene.background_color;
                    image_my[yer] = calculated_colour.x;
                    image_my[yer+1] = calculated_colour.y;
                    image_my[yer+2] = calculated_colour.z;                     
                }
                intersection_material = bos;                  
            }
        }          
            write_ppm(scene.cameras[i].image_name.c_str(), image_my, width, height);    
    }
}
