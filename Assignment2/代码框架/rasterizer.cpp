// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <cmath>
#include <vector>
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

constexpr bool SSAA = true;


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)//point at the first element of array<vector3f,3>
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    const auto& A = _v[0];
    const auto& B = _v[1];
    const auto& C = _v[2];
    Eigen::Vector3f P;
    P << (float)x,(float)y,0.0f;//any z is acceptable, 'casue we don't even use it
    
    Eigen::Vector3f AB = A-B;
    Eigen::Vector3f BC = B-C;
    Eigen::Vector3f CA = C-A;
    Eigen::Vector3f AP = A-P;
    Eigen::Vector3f BP = B-P;
    Eigen::Vector3f CP = C-P;

    float z1 = AB.cross(AP).z();
    float z2 = BC.cross(BP).z();
    float z3 = CA.cross(CP).z();
    float res = z1*z2*z3;

    return res >= 0 ?  true : false;//or return res
    //if this point is on the edge,assume it inside the triangle
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    int max_x = static_cast<int>(std::ceil(std::max({v[0].x(),v[1].x(),v[2].x()})));
    int min_x = static_cast<int>(std::floor(std::min({v[0].x(),v[1].x(),v[2].x()})));

    int max_y = static_cast<int>(std::ceil(std::max({v[0].y(),v[1].y(),v[2].y()})));
    int min_y = static_cast<int>(std::floor(std::min({v[0].y(),v[1].y(),v[2].y()})));


    for(int x = min_x;x<=max_x;x++)
    {
        for(int y = min_y;y<=max_y;y++)
        {
            if(SSAA)
            {
                int cnt =0;
                for(float i=0.25;i<1;i+=0.5)
                {
                    for(float j=0.25;j<1;j+=0.5)
                    {
                        if(!insideTriangle(x+i, y+j, t.v))
                        {
                            continue;
                        }
                        cnt++;
                    }
                }
                if(cnt == 0)
                {
                    continue;
                }
                auto[alpha,beta,gamma] = computeBarycentric2D(x,y,t.v);//the ptr to array's first element itself just represent the whole array 
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());//矫正系数
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;//same process as color interpolate

                Eigen::Vector3f interpolate_p;
                interpolate_p << x,y,z_interpolated;
                auto idx = get_index(x,y);
                //if z_interpolate > z_buffer,不渲染，小于等于则渲染后更新z_buffer，这样可以顺带覆盖之前的渲染结果
                if(z_interpolated <= depth_buf[idx] || depth_buf[idx] == 0.0f)
                {
                    set_pixel(interpolate_p,cnt/4.0f * t.getColor());//SSAA for actually ,in this exmaple same as MSAA(becuase triangle is in same color)
                    depth_buf[idx] = z_interpolated;
                }
            }
            else {
                if(!insideTriangle(x,y,t.v))
                {
                    continue;
                }
                auto[alpha,beta,gamma] = computeBarycentric2D(x,y,t.v);//the ptr to array's first element itself just represent the whole array 
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());//矫正系数
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;//same process as color interpolate

                Eigen::Vector3f interpolate_p;
                interpolate_p << x,y,z_interpolated;
                auto idx = get_index(x,y);
                //if z_interpolate > z_buffer,不渲染，小于等于则渲染后更新z_buffer，这样可以顺带覆盖之前的渲染结果
                if(z_interpolated <= depth_buf[idx] || depth_buf[idx] == 0.0f)
                {
                    set_pixel(interpolate_p,t.getColor());
                    depth_buf[idx] = z_interpolated;
                }
                /* or you can 
                *  std::fill(depth_buf.begin(),depth_buf.end(),std::numeric_limits<float>::infinity());
                *  then you can skip "depth_buf[idx] ==0.0f"
                */
            }
        }
    }
    //interpolate z value

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on