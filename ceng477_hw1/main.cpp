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

/*parser::Vec3f mirrorfunc(int count)
{
    if(count>0)
    {
        ray viewing_ray = compute_viewing_ray(width,height,k,j,near_distance,near_plane,position,-up,gaze);
        //viewing_ray.d.normalize();
        parser::Vec3f nom = m-position;
        tmin = nom.dotprod(gaze)/(viewing_ray.d).dotprod(gaze);
        //std::cout<<tmin<<std::endl;
        //std::cout<<"<"<<viewing_ray.o.x<<","<<viewing_ray.o.y<<","<<viewing_ray.o.z<<">"<<" + t<"<<viewing_ray.d.x<<","<<viewing_ray.d.y<<","<<viewing_ray.d.z<<">"<<" "<<k<<" "<<j<<std::endl;         
        mint = FLT_MAX;
        for(int l=0;l<scene.spheres.size();l++)
        {                       
            intersection = sphere_intersection(scene,scene.spheres[l],viewing_ray,0.9,tinner);  
            if(intersection.x != FLT_MIN)
            {                      
                if(tinner<mint)
                {
                    mint = tinner;                          
                    intersection_material_id = scene.spheres[l].material_id;
                    nv = intersection - scene.vertex_data[scene.spheres[l].center_vertex_id-1];
                    nv.normalize();
                    final_intersection = intersection;                       
                }
            }
        }
        for(int u=0;u<scene.triangles.size();u++)
        {
            intersection = triangle_intersection(scene,scene.triangles[u],viewing_ray,tmin,tinner);
            if(intersection.x != FLT_MIN)
            {
                //std::cout<<intersection.x<<" "<<intersection.y<<" "<<intersection.z<<std::endl;
                if(tinner<mint)
                {
                    if(j==10&&k==10)
                    {
                        std::cout<<"ucgen var saniyor"<<std::endl;
                    }                            
                    //std::cout<<intersection.x<<" "<<intersection.y<<" "<<intersection.z<<std::endl;                           
                    mint = tinner;
                    intersection_material_id = scene.triangles[u].material_id;
                    nv = triangle_normals[u];
                    final_intersection = intersection;
                }
            }

        }
        for(int b=0;b<scene.meshes.size();b++)
        {
            for(int x=0;x<scene.meshes[b].faces.size();x++)
            {
                intersection = triangle_intersection_mesh(scene,scene.meshes[b].faces[x],viewing_ray,tmin,tinner);
                if(intersection.x != FLT_MIN)
                {
                    if(tinner<mint)
                    {                             
                        //std::cout<<intersection.x<<" "<<intersection.y<<" "<<intersection.z<<std::endl;
                        mint = tinner;
                        intersection_material_id = scene.meshes[b].material_id;
                        nv = mftriangle_normals[b][x];
                        final_intersection = intersection;
                    }
                }

            }
        } 

        //TO control whether ray intersected any object
        if(intersection_material_id != INT_MIN)
        {
            ambient = scene.materials[intersection_material_id-1].ambient;
            diffuse = scene.materials[intersection_material_id-1].diffuse;
            mirror = scene.materials[intersection_material_id-1].mirror;
            specular = scene.materials[intersection_material_id-1].specular;
            phong_c = scene.materials[intersection_material_id-1].phong_exponent;

            ambient_colour = round(ambient*scene.ambient_light);
            calculated_colour = ambient_colour;

            for(int pl=0;pl<scene.point_lights.size();pl++)
            {
                ray light_s;
                light_s.o = final_intersection;
                light_s.d = scene.point_lights[pl].position - final_intersection;
                light_s.d.normalize();
                light_s.o =light_s.o + light_s.d*scene.shadow_ray_epsilon;
                float light_selami = (scene.point_lights[pl].position.x -final_intersection.x)*light_s.d.x; 
                for(int l=0;l<scene.spheres.size();l++)
                {
                    intersection_s = sphere_intersection(scene,scene.spheres[l],light_s,0,tinner);
                    if(intersection_s.x != FLT_MIN)
                    {
                        if(tinner<light_selami)
                        {
                            flag = 1;
                            break;                                    
                        }
                    }

                } 
                for(int u=0;u<scene.triangles.size()&&flag==0;u++)
                {
                    intersection_s = triangle_intersection(scene,scene.triangles[u],light_s,0,tinner);
                    if(intersection_s.x != FLT_MIN)
                    {
                        if(tinner<light_selami)
                        {
                            flag = 1;
                            break;                                    
                        }
                    }

                }
                for(int b=0;b<scene.meshes.size()&&flag==0;b++)
                {
                    for(int x=0;x<scene.meshes[b].faces.size();x++)
                    {
                        intersection_s = triangle_intersection_mesh(scene,scene.meshes[b].faces[x],light_s,0,tinner);
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
                    image_my[yer] = round(scene.ambient_light.x);
                    image_my[yer+1] = round(scene.ambient_light.y);
                    image_my[yer+2] = round(scene.ambient_light.z);                            
                    continue;
                }
                else
                {
                    float zero = 0;

                    //std::cout<<nv.x<<" "<<nv.y<<" "<<nv.z<<std::endl;

                    parser::Vec3f dv = (scene.point_lights[pl].position - final_intersection);
                    parser::Vec3f dv2 = dv;
                    dv2.normalize();
                    cosinus_uzulur = std::max(dv2.dotprod(nv),zero); 
                    light_distance = dv.magnitude()*dv.magnitude();
                    parser::Vec3f aab = (diffuse*cosinus_uzulur*scene.point_lights[pl].intensity);
                    parser::Vec3f aaa = aab/light_distance;
                    diffuse_colour = round((diffuse*cosinus_uzulur)*scene.point_lights[pl].intensity/light_distance); 
                    //calculated_colour = calculated_colour + diffuse_colour;                          
                    parser::Vec3f half = light_s.d - viewing_ray.d;
                    half.normalize();

                    cosinus_uzulur_specular = std::max(nv.dotprod(half),zero);
                    specular_colour = round((specular*scene.point_lights[pl].intensity)*std::pow(cosinus_uzulur_specular,phong_c)/light_distance);

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

                    if(k==799&&j==799)
                    {
                        std::cout<<"renk:"<<calculated_colour.x<<std::endl;                                
                    }                              
                        //std::cout<<mirror.x<<" "<<mirror.y<<" "<<mirror.z<<std::endl;                          
                    if(mirror.x ==0&&mirror.y==0&&mirror.z==0)
                    {
                        image_my[yer] = calculated_colour.x;
                        image_my[yer+1] = calculated_colour.y;
                        image_my[yer+2] = calculated_colour.z;                                 
                    }
                    else
                    {
                        image_my[yer] = calculated_colour.x;
                        image_my[yer+1] = calculated_colour.y;
                        image_my[yer+2] = calculated_colour.z;                                  
                    }

                }


            }
        
    }



    }
    else
    {
        calculated_colour = scene.background_color;
        //std::cout<<j<<" "<<k<<std::endl;
        image_my[yer] = calculated_colour.x;
        image_my[yer+1] = calculated_colour.y;
        image_my[yer+2] = calculated_colour.z;                     
    }
    
    
} */


float epsilon = 1e-7;

//ray compute_viewing_ray(parser::vec3f u,parser::vec3f v,)

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

parser::Vec3f sphere_intersection(parser::Scene scene,parser::Sphere sphere, ray ray,float tmin,float &t)
{
    parser::Vec3f center_vertex = scene.vertex_data[sphere.center_vertex_id-1];
    parser::Vec3f result;
    
    float first_t;
    float second_t;
    float min_t;
    parser::Vec3f ray_cv = ray.o-center_vertex;
    float determinant =pow(ray.d.dotprod(ray_cv),2) - (ray.d.dotprod(ray.d))*((ray_cv).dotprod(ray_cv) - sphere.radius*sphere.radius);
    
    //std::cout<<"<"<<ray.o.x<<","<<ray.o.y<<","<<ray.o.z<<">"<<" + t<"<<ray.d.x<<","<<ray.d.y<<","<<ray.d.z<<">"<<std::endl;
    //std::cout<<scene.vertex_data[sphere.center_vertex_id-1].x<<std::endl;
    //std::cout<<determinant<<std::endl;
    if(determinant<0)
    {
        //std::cout<<determinant<<std::endl;
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

parser::Vec3f triangle_intersection(parser::Scene scene,parser::Triangle triangle, ray ray,float tmin,float &t)
{
    parser::Vec3f a = scene.vertex_data[triangle.indices.v0_id-1];
    parser::Vec3f b = scene.vertex_data[triangle.indices.v1_id-1];
    parser::Vec3f c = scene.vertex_data[triangle.indices.v2_id-1];
    
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

parser::Vec3f triangle_intersection_s(parser::Scene scene,parser::Triangle triangle, ray ray,float &t)
{
    parser::Vec3f a = scene.vertex_data[triangle.indices.v0_id-1];
    parser::Vec3f b = scene.vertex_data[triangle.indices.v1_id-1];
    parser::Vec3f c = scene.vertex_data[triangle.indices.v2_id-1];
    
    parser::Vec3f Aa(a.x-b.x,a.y-b.y,a.z-b.z);
    parser::Vec3f Ab(a.x-c.x,a.y-c.y,a.z-c.z);
    parser::Vec3f Ac(ray.d.x,ray.d.y,ray.d.z);
    
    parser::Vec3f changing(a.x-ray.o.x,a.y-ray.o.y,a.z-ray.o.z);
    
    float detA = find_matrix_determinant(Aa,Ab,Ac);
    
    float beta,gamma;
    
    beta = find_matrix_determinant(changing,Ab,Ac)/detA;
    gamma = find_matrix_determinant(Aa,changing,Ac)/detA;
    t = find_matrix_determinant(Aa,Ab,changing)/detA;
    
    
    if(beta+gamma<=1&&beta>=0&&gamma>=0)
    {
        return ray.o + ray.d*t;
    }
    else
    {
        return false_indicator;
    }
}

bool sphere_intersection_s(parser::Scene scene,parser::Sphere sphere, ray ray,float light_selami,float &t)
{
    parser::Vec3f center_vertex = scene.vertex_data[sphere.center_vertex_id-1];
    parser::Vec3f result;
    
    float first_t;
    float second_t;
    float min_t;
    parser::Vec3f ray_cv = ray.o-center_vertex;
    float determinant =pow(ray.d.dotprod(ray_cv),2) - (ray.d.dotprod(ray.d))*((ray_cv).dotprod(ray_cv) - sphere.radius*sphere.radius);
    
    if(determinant<0)
    {
        return false;
    }
    
    else if(determinant == 0)
    {
      t =(-ray.d.dotprod(ray.o-center_vertex))/ray.d.dotprod(ray.d);
      if(t>light_selami)
      {
          return false;
      }
      result = ray.o + ray.d*t;
      return true;
    }
    
    else
    {
      first_t = (-ray.d.dotprod(ray.o-center_vertex) + sqrt(determinant))/ray.d.dotprod(ray.d);        
      second_t = (-ray.d.dotprod(ray.o-center_vertex) - sqrt(determinant))/ray.d.dotprod(ray.d); 
      min_t = fmin(first_t,second_t);
      t = min_t;
      if(min_t>light_selami)
      {
          return false;
      }
      result = ray.o + ray.d*min_t;
    }
    
    return true;
}

parser::Vec3f triangle_intersection_mesh(parser::Scene scene,parser::Face triangle, ray ray,float tmin,float &t)
{
    parser::Vec3f a = scene.vertex_data[triangle.v0_id-1];
    parser::Vec3f b = scene.vertex_data[triangle.v1_id-1];
    parser::Vec3f c = scene.vertex_data[triangle.v2_id-1];
    
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

parser::Vec3f triangle_intersection_mesh_s(parser::Scene scene,parser::Face triangle, ray ray,float &t)
{
    parser::Vec3f a = scene.vertex_data[triangle.v0_id-1];
    parser::Vec3f b = scene.vertex_data[triangle.v1_id-1];
    parser::Vec3f c = scene.vertex_data[triangle.v2_id-1];
    
    parser::Vec3f Aa(a.x-b.x,a.y-b.y,a.z-b.z);
    parser::Vec3f Ab(a.x-c.x,a.y-c.y,a.z-c.z);
    parser::Vec3f Ac(ray.d.x,ray.d.y,ray.d.z);
    
    parser::Vec3f changing(a.x-ray.o.x,a.y-ray.o.y,a.z-ray.o.z);
    
    float detA = find_matrix_determinant(Aa,Ab,Ac);
    
    float beta,gamma;
    
    beta = find_matrix_determinant(changing,Ab,Ac)/detA;
    gamma = find_matrix_determinant(Aa,changing,Ac)/detA;
    t = find_matrix_determinant(Aa,Ab,changing)/detA;
    
    if(beta+gamma<=1&&beta>=0&&gamma>=0)
    {
        return ray.o + ray.d*t;
    }
    else
    {
        return false_indicator;
    }
}
ray compute_viewing_ray(int width,int height,int i,int j,float near_distance,parser::Vec4f near_plane,parser::Vec3f e,parser::Vec3f up,parser::Vec3f gaze){
    
    /*compute vec_u*/
    parser::Vec3f vec_u;
    vec_u.x = 0;
    vec_u.y = 0;
    vec_u.z = 0;
    parser::Vec3f gaze_up; 
    gaze_up = up.crossprod(-gaze);
    parser::Vec3f d;
    
    
    /*compute d = -w.d + l.u + t.v + s_u.u - s_v.v 
    d.x = gaze.x*near_distance + gaze_up.x*near_plane.w + up.x*near_plane.z + (((near_plane.x - near_plane.w)*(i + 0.5))/width)*gaze_up.x - (((near_plane.z - near_plane.y)*(j + 0.5))/height)*up.x;
    d.y = gaze.y*near_distance + gaze_up.y*near_plane.w + up.y*near_plane.z + (((near_plane.x - near_plane.w)*(i + 0.5))/width)*gaze_up.y - (((near_plane.z - near_plane.y)*(j + 0.5))/height)*up.y;
    d.z = gaze.z*near_distance + gaze_up.z*near_plane.w + up.z*near_plane.z + (((near_plane.x - near_plane.w)*(i + 0.5))/width)*gaze_up.z - (((near_plane.z - near_plane.y)*(j + 0.5))/height)*up.z; */
    
    d = (gaze*near_distance) + (gaze_up*near_plane.w) + up*near_plane.z + gaze_up*(((near_plane.x - near_plane.w)*(i + 0.5))/width) - up*(((near_plane.z - near_plane.y)*(j + 0.5))/height);
    //d.normalize();
    ray result;
    result.o = e;
    result.d = d;
    
    result.d.normalize();
    //std::cout<<"<"<<result.o.x<<","<<result.o.y<<","<<result.o.z<<">"<<" + t<"<<result.d.x<<","<<result.d.y<<","<<result.d.z<<">"<<" "<<i<<" "<<j<<std::endl;
    
    return result;
      
}

int main(int argc, char* argv[])
{
    // Sample usage for reading an XML scene file
    parser::Scene scene;

    int gostergec = 0;
    
    scene.loadFromXml(argv[1]);
    int height = 0;
    int width = 0;
    float near_distance = 0;
    float tmin = 0;
 
    float cosinus_uzulur;
    float cosinus_uzulur_specular;
    float light_distance;
    
    std::vector<parser::Vec3f> triangle_normals;
    
    std::map<int,std::vector<std::vector<parser::Vec3f>>> mesh_face_normals; 
    
    std::vector<std::vector<parser::Vec3f>> mftriangle_normals(scene.meshes.size());
            

    /*This is for precomputing triangles in the file*/
    for(int i=0;i<scene.triangles.size();i++)
    {
        parser::Vec3f a = scene.vertex_data[scene.triangles[i].indices.v0_id-1];
        parser::Vec3f b = scene.vertex_data[scene.triangles[i].indices.v1_id-1];
        parser::Vec3f c = scene.vertex_data[scene.triangles[i].indices.v2_id-1];
        
        parser::Vec3f v1 = a-b;
        parser::Vec3f v2 = c-b;
        parser::Vec3f normal = v2.crossprod(v1);
        normal.normalize();
        triangle_normals.push_back(normal);
    }
    
    /* This for precomputing the triangles' normals in meshes*/
    for(int i=0;i<scene.meshes.size();i++)
    {
        for(int j=0;j<scene.meshes[i].faces.size();j++)
        {
            parser::Vec3f mfa = scene.vertex_data[scene.meshes[i].faces[j].v0_id-1];
            parser::Vec3f mfb = scene.vertex_data[scene.meshes[i].faces[j].v1_id-1];
            parser::Vec3f mfc = scene.vertex_data[scene.meshes[i].faces[j].v2_id-1]; 

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

        parser::Vec3f intersection;
        parser::Vec3f intersection_s;
        
        parser::Vec3f final_intersection;
        parser::Vec3i rgb;
        
        float mint = FLT_MAX;
        float tinner;
        
        int intersection_material_id = INT_MIN;
        
        parser::Vec3f nv;
        
        parser::Vec3i color;

        parser::Vec3f ambient;
        parser::Vec3f diffuse;
        parser::Vec3f mirror;
        parser::Vec3f specular;
        
        float phong_c;
     
        parser::Vec3i calculated_colour;
        parser::Vec3i ambient_colour;
        parser::Vec3i diffuse_colour;
        parser::Vec3i mirror_colour;
        parser::Vec3i specular_colour;
        
        int flag = 0;
        
        unsigned char* image_my = new unsigned char [width * height * 3]; 
        
        for(int j=0;j<height;j++){
            for(int k=0;k<width;k++){
                int yer = width*j*3 + k*3;
                ray viewing_ray = compute_viewing_ray(width,height,k,j,near_distance,near_plane,position,-up,gaze);
                //viewing_ray.d.normalize();
                parser::Vec3f nom = m-position;
                tmin = nom.dotprod(gaze)/(viewing_ray.d).dotprod(gaze);
                //std::cout<<tmin<<std::endl;
                //std::cout<<"<"<<viewing_ray.o.x<<","<<viewing_ray.o.y<<","<<viewing_ray.o.z<<">"<<" + t<"<<viewing_ray.d.x<<","<<viewing_ray.d.y<<","<<viewing_ray.d.z<<">"<<" "<<k<<" "<<j<<std::endl;         
                mint = FLT_MAX;
                for(int l=0;l<scene.spheres.size();l++)
                {                       
                    intersection = sphere_intersection(scene,scene.spheres[l],viewing_ray,0.9,tinner);  
                    if(intersection.x != FLT_MIN)
                    {                      
                        if(tinner<mint)
                        {
                            mint = tinner;                          
                            intersection_material_id = scene.spheres[l].material_id;
                            nv = intersection - scene.vertex_data[scene.spheres[l].center_vertex_id-1];
                            nv.normalize();
                            final_intersection = intersection;                       
                        }
                    }
                }
                for(int u=0;u<scene.triangles.size();u++)
                {
                    intersection = triangle_intersection(scene,scene.triangles[u],viewing_ray,tmin,tinner);
                    if(intersection.x != FLT_MIN)
                    {
                        //std::cout<<intersection.x<<" "<<intersection.y<<" "<<intersection.z<<std::endl;
                        if(tinner<mint)
                        {
                            if(j==10&&k==10)
                            {
                                std::cout<<"ucgen var saniyor"<<std::endl;
                            }                            
                            //std::cout<<intersection.x<<" "<<intersection.y<<" "<<intersection.z<<std::endl;                           
                            mint = tinner;
                            intersection_material_id = scene.triangles[u].material_id;
                            nv = triangle_normals[u];
                            final_intersection = intersection;
                        }
                    }
                    
                }
                for(int b=0;b<scene.meshes.size();b++)
                {
                    for(int x=0;x<scene.meshes[b].faces.size();x++)
                    {
                        intersection = triangle_intersection_mesh(scene,scene.meshes[b].faces[x],viewing_ray,tmin,tinner);
                        if(intersection.x != FLT_MIN)
                        {
                            if(tinner<mint)
                            {                             
                                //std::cout<<intersection.x<<" "<<intersection.y<<" "<<intersection.z<<std::endl;
                                mint = tinner;
                                intersection_material_id = scene.meshes[b].material_id;
                                nv = mftriangle_normals[b][x];
                                final_intersection = intersection;
                            }
                        }
                        
                    }
                } 
                
                /*TO control whether ray intersected any object*/
                if(intersection_material_id != INT_MIN)
                {
                    ambient = scene.materials[intersection_material_id-1].ambient;
                    diffuse = scene.materials[intersection_material_id-1].diffuse;
                    mirror = scene.materials[intersection_material_id-1].mirror;
                    specular = scene.materials[intersection_material_id-1].specular;
                    phong_c = scene.materials[intersection_material_id-1].phong_exponent;
                    
                    ambient_colour = round(ambient*scene.ambient_light);
                    calculated_colour = ambient_colour;
                    
                    for(int pl=0;pl<scene.point_lights.size();pl++)
                    {
                        ray light_s;
                        light_s.o = final_intersection;
                        light_s.d = scene.point_lights[pl].position - final_intersection;
                        light_s.d.normalize();
                        light_s.o =light_s.o + light_s.d*scene.shadow_ray_epsilon;
                        float light_selami = (scene.point_lights[pl].position.x -final_intersection.x)*light_s.d.x; 
                        for(int l=0;l<scene.spheres.size();l++)
                        {
                            intersection_s = sphere_intersection(scene,scene.spheres[l],light_s,0,tinner);
                            if(intersection_s.x != FLT_MIN)
                            {
                                if(tinner<light_selami)
                                {
                                    flag = 1;
                                    break;                                    
                                }
                            }
                       
                        } 
                        for(int u=0;u<scene.triangles.size()&&flag==0;u++)
                        {
                            intersection_s = triangle_intersection(scene,scene.triangles[u],light_s,0,tinner);
                            if(intersection_s.x != FLT_MIN)
                            {
                                if(tinner<light_selami)
                                {
                                    flag = 1;
                                    break;                                    
                                }
                            }

                        }
                        for(int b=0;b<scene.meshes.size()&&flag==0;b++)
                        {
                            for(int x=0;x<scene.meshes[b].faces.size();x++)
                            {
                                intersection_s = triangle_intersection_mesh(scene,scene.meshes[b].faces[x],light_s,0,tinner);
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
                            image_my[yer] = round(scene.ambient_light.x);
                            image_my[yer+1] = round(scene.ambient_light.y);
                            image_my[yer+2] = round(scene.ambient_light.z);                            
                            continue;
                        }
                        else
                        {
                            float zero = 0;
                            
                            //std::cout<<nv.x<<" "<<nv.y<<" "<<nv.z<<std::endl;
                            
                            parser::Vec3f dv = (scene.point_lights[pl].position - final_intersection);
                            parser::Vec3f dv2 = dv;
                            dv2.normalize();
                            cosinus_uzulur = std::max(dv2.dotprod(nv),zero); 
                            light_distance = dv.magnitude()*dv.magnitude();
                            parser::Vec3f aab = (diffuse*cosinus_uzulur*scene.point_lights[pl].intensity);
                            parser::Vec3f aaa = aab/light_distance;
                            diffuse_colour = round((diffuse*cosinus_uzulur)*scene.point_lights[pl].intensity/light_distance); 
                            //calculated_colour = calculated_colour + diffuse_colour;                          
                            parser::Vec3f half = light_s.d - viewing_ray.d;
                            half.normalize();
                            
                            cosinus_uzulur_specular = std::max(nv.dotprod(half),zero);
                            specular_colour = round((specular*scene.point_lights[pl].intensity)*std::pow(cosinus_uzulur_specular,phong_c)/light_distance);
                            
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
                            
                            if(k==799&&j==799)
                            {
                                std::cout<<"renk:"<<calculated_colour.x<<std::endl;                                
                            }                              
                                //std::cout<<mirror.x<<" "<<mirror.y<<" "<<mirror.z<<std::endl;                          
                            if(mirror.x ==0&&mirror.y==0&&mirror.z==0)
                            {
                                image_my[yer] = calculated_colour.x;
                                image_my[yer+1] = calculated_colour.y;
                                image_my[yer+2] = calculated_colour.z;                                 
                            }
                            else
                            {
                                image_my[yer] = calculated_colour.x;
                                image_my[yer+1] = calculated_colour.y;
                                image_my[yer+2] = calculated_colour.z;                                  
                            }
                            
                        }
                        
                        
                    }

                    
                            
                }
                else
                {
                    calculated_colour = scene.background_color;
                    //std::cout<<j<<" "<<k<<std::endl;
                    image_my[yer] = calculated_colour.x;
                    image_my[yer+1] = calculated_colour.y;
                    image_my[yer+2] = calculated_colour.z;                     
                }
                
                
                
                intersection_material_id = INT_MIN;                  
            }
         
        }
        char *eren;
        

        
            write_ppm(scene.cameras[i].image_name.c_str(), image_my, height, width);
        
        
        
        
        
        
    }

    // The code below creates a test pattern and writes
    // it to a PPM file to demonstrate the usage of the
    // ppm_write function.

    const RGB BAR_COLOR[8] =
    {
        { 255, 255, 255 },  // 100% White
        { 255, 255,   0 },  // Yellow
        {   0, 255, 255 },  // Cyan
        {   0, 255,   0 },  // Green
        { 255,   0, 255 },  // Magenta
        { 255,   0,   0 },  // Red
        {   0,   0, 255 },  // Blue
        {   0,   0,   0 },  // Black
    };

    width = 640, height = 480;
    int columnWidth = width / 8;

    unsigned char* image = new unsigned char [width * height * 3];

    int i = 0;
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            int colIdx = x / columnWidth;
            image[i++] = BAR_COLOR[colIdx][0];
            image[i++] = BAR_COLOR[colIdx][1];
            image[i++] = BAR_COLOR[colIdx][2];
        }
    }

    write_ppm("dsa", image, width, height);

}
