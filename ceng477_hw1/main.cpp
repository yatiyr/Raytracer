#include <iostream>
#include "parser.h"
#include "ppm.h"
#include <cmath>


typedef unsigned char RGB[3];


struct ray {
    parser::Vec3f o;
    parser::Vec3f d;
};

//ray compute_viewing_ray(parser::vec3f u,parser::vec3f v,)


ray compute_viewing_ray(int width,int height,int i,int j,float near_distance,parser::Vec4f near_plane,parser::Vec3f e,parser::Vec3f up,parser::Vec3f gaze){
    
    /*compute vec_u*/
    parser::Vec3f vec_u;
    vec_u.x = 0;
    vec_u.y = 0;
    vec_u.z = 0;
    parser::Vec3f gaze_up;
    if(gaze.x | up.x == 0){
        vec_u.x = 1;
    }
    else if(gaze.y | up.y == 0){
        vec_u.y = 1;
    }
    else if(gaze.z | up.z == 0){
        vec_u.z = 1;
    }
    
    parser::Vec3f d;
    
    
    /*compute d = -w.d + l.u + t.v + s_u.u - s_v.v */
    d.x = gaze.x*near_distance + vec_u.x*near_plane.w + up.x*near_plane.z + (((near_plane.x - near_plane.w)*(i + 0.5))/width)*vec_u.x - (((near_plane.z - near_plane.y)*(j + 0.5))/height)*up.x;
    d.y = gaze.y*near_distance + vec_u.y*near_plane.w + up.y*near_plane.z + (((near_plane.x - near_plane.w)*(i + 0.5))/width)*vec_u.y - (((near_plane.z - near_plane.y)*(j + 0.5))/height)*up.y;
    d.z = gaze.z*near_distance + vec_u.z*near_plane.w + up.z*near_plane.z + (((near_plane.x - near_plane.w)*(i + 0.5))/width)*vec_u.z - (((near_plane.z - near_plane.y)*(j + 0.5))/height)*up.z;
    
    ray result;
    result.o = e;
    result.d = d;
    
    return result;
      
}

int main(int argc, char* argv[])
{
    // Sample usage for reading an XML scene file
    parser::Scene scene;

    scene.loadFromXml(argv[1]);
    
    int height = 0;
    int width = 0;
    float near_distance = 0;
    
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

        
        for(int j=0;j<height;j++){
            for(int k=0;k<width;k++){
                
                
            }
        }
        
        
        
        
        
        
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

    int width = 640, height = 480;
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

    write_ppm(argv[2], image, width, height);

}
