#ifndef __HW1__PARSER__
#define __HW1__PARSER__

#include <string>
#include <vector>
#include <math.h>

namespace parser
{
    //Notice that all the structures are as simple as possible
    //so that you are not enforced to adopt any style or design.
    struct Vec3f
    {
        float x, y, z;
        
        Vec3f(float x=0,float y=0,float z=0): x(x),y(y),z(z)
        {            
        }
        
        Vec3f& operator=(const Vec3f& vec)
        {
            x = vec.x;
            y = vec.y;
            z = vec.z;
            return *this;
        }
        
        float dotprod(const Vec3f& vec) const
        {
            return x*vec.x + y*vec.y + z*vec.z; 
        }
        
        Vec3f operator-(const Vec3f& vec) 
        {
            return Vec3f(x-vec.x,y-vec.y,z-vec.z);
        }
        
        Vec3f operator-() 
        {
            return Vec3f(-x,-y,-z);
        }
        
        Vec3f operator+(const Vec3f& vec) 
        {
            return Vec3f(vec.x+x,vec.y+y,vec.z+z);
        }      
        
        Vec3f& normalize()
        {
            float magnitude = sqrt(x*x+y*y+z*z);
            x = x/magnitude;
            y = y/magnitude;
            z = z/magnitude;
            return *this;
        }
        
        /*int operator*(const Vec3f vec) const
        {
            return x*vec.x + y*vec.y + z*vec.z;
        } */
        
        float magnitude() const
        {
            return sqrt(x*x+y*y+z*z);
        }
        
        Vec3f crossprod(const Vec3f& vec) const
        {
            /* | i  j  k |
               | x  y  z |
               | a  b  c |
       i*(yc-bz) -j*(xc-az) + k(xb-ay) 
                                        */
            
            return Vec3f(y*vec.z - vec.y*z,-(x*vec.z-vec.x*z),x*vec.y-vec.x*y);
        }
        
        Vec3f operator*(int a) 
        {
            return Vec3f(x*a,y*a,z*a);
        }
        Vec3f operator*(float a) 
        {
            return Vec3f(x*a,y*a,z*a);
        }
        Vec3f operator*(double a) 
        {
            return Vec3f(x*a,y*a,z*a);
        }
        Vec3f operator*(const Vec3f& vec)
        {
            return Vec3f(x*vec.x,y*vec.y,z*vec.z);
        } 
        Vec3f operator/(const Vec3f& vec) 
        {
            return Vec3f(x/vec.x,y/vec.y,z/vec.z);
        }
        Vec3f operator/(float vec) 
        {
            return Vec3f(x/vec,y/vec,z/vec);
        }           
        
    };

    struct Vec3i
    {
        int x, y, z;
        Vec3i(int x=0,int y=0,int z=0): x(x),y(y),z(z)
        {            
        }
        Vec3i operator-(const Vec3i& vec) 
        {
            return Vec3i(x-vec.x,y-vec.y,z-vec.z);
        }
        
        Vec3i operator+(const Vec3i& vec) 
        {
            return Vec3i(vec.x+x,vec.y+y,vec.z+z);
        }          
        Vec3i operator*(const Vec3i& vec) 
        {
            return Vec3i(x*vec.x,y*vec.y,z*vec.z);
        }    
        Vec3i operator*(const Vec3f& vec)
        {
            return Vec3i(round(x*vec.x),round(y*vec.y),round(z*vec.z));
        }         
    };

    struct Vec4f
    {
        float x, y, z, w;
    };

    struct Camera
    {
        Vec3f position;
        Vec3f gaze;
        Vec3f up;
        Vec4f near_plane;
        float near_distance;
        int image_width, image_height;
        std::string image_name;
    };

    struct PointLight
    {
        Vec3f position;
        Vec3f intensity;
    };

    struct Material
    {
        Vec3f ambient;
        Vec3f diffuse;
        Vec3f specular;
        Vec3f mirror;
        float phong_exponent;
    };

    struct Face
    {
        int v0_id;
        int v1_id;
        int v2_id;
        
        Vec3f v0;
        Vec3f v1;
        Vec3f v2;
    };

    struct Mesh
    {
        int material_id;
        std::vector<Face> faces;
        Material material;
    };

    struct Triangle
    {
        int material_id;
        Face indices;
        Material material;
        Vec3f normal;
    };

    struct Sphere
    {
        int material_id;
        int center_vertex_id;
        float radius;
        Vec3f center_vertex;
        Material material;
    };

    struct Scene
    {
        //Data
        Vec3i background_color;
        float shadow_ray_epsilon;
        int max_recursion_depth;
        std::vector<Camera> cameras;
        Vec3f ambient_light;
        std::vector<PointLight> point_lights;
        std::vector<Material> materials;
        std::vector<Vec3f> vertex_data;
        std::vector<Mesh> meshes;
        std::vector<Triangle> triangles;
        std::vector<Sphere> spheres;

        //Functions
        void loadFromXml(const std::string& filepath);
    };
}

#endif
