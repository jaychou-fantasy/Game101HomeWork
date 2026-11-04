#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 
                 0, 1, 0, -eye_pos[1],
                 0, 0, 1, -eye_pos[2],
                 0, 0, 0, 1;

    view = translate * view;//only translate no view rotation.

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    float rotate_rad = rotation_angle / 180.0f * MY_PI;
    Eigen::Matrix4f r_matrix;
    r_matrix << std::cos(rotate_rad), -std::sin(rotate_rad), 0 ,0,
                       std::sin(rotate_rad), std::cos(rotate_rad),0,0,
                       0,0,1,0,
                       0,0,0,1;
    //for matrix don't use *= 'cause martix multipy has order : modle *= roatation_matrix;
    model = r_matrix * model;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    float fov_to_rad = eye_fov / 180.0f * MY_PI;

    //as we suppose our z-axis has already aligned with 0----only consider about -(n+f)/2 translate 
    float t = zNear * std::tan(fov_to_rad/2);
    float b = -t;
    float r = t * aspect_ratio;
    float l = -r;
    float near = -zNear;
    float far = -zFar;
    Eigen::Matrix4f Mortho_translate;
    Mortho_translate << 1,0,0,-(r+l)/2,
                        0,1,0,-(t+b)/2, //these are actually 0
                        0,0,1,-(near+far)/2,
                        0,0,0,1;

    Eigen::Matrix4f Mortho_scale;
    Mortho_scale << 2/(r-l),0,0,0,
                    0,2/(t-b),0,0,
                    0,0,2/(near-far),0,//as is from -Zaxis direction,zNear is larger than zFar
                    0,0,0,1;

    Eigen::Matrix4f MorthoGraphic;
    MorthoGraphic = Mortho_scale * Mortho_translate;


    Eigen::Matrix4f Mprojection_to_ortho;
    Mprojection_to_ortho << near,0,0,0,
                            0,near,0,0,
                            0,0,(near+far),-near*far,
                            0,0,1,0;
    // Eigen::Matrix4f Mt(4,4);
    // Mt << 1, 0, 0, 0,
    //     0, 1, 0, 0,
    //     0, 0, -1, 0,
    //     0, 0, 0, 1; 
    // Mpersp2ortho = Mpersp2ortho *Mt;

    projection =  MorthoGraphic * Mprojection_to_ortho;

    return projection;
}

//option work : rotate around any n-axis about any angles
//while didn;t implement in main function
Eigen::Matrix4f get_rotation(Vector3f axis,float angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    //use Godrigues' Rotation Formula
    Eigen::Matrix3f RotateMatrix;
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    
    float rot_rad = angle / 180.0f * MY_PI;

    
    float nx = axis[0];
    float ny = axis[1];
    float nz = axis[2];
    float norm = sqrt(nx*nx + ny*ny+nz*nz);
    nx /= norm;
    ny /= norm;
    nz /= norm;
    axis /= norm;

    //initalize RotateMatrix with Dual Matrix
    RotateMatrix << 0,-nz,ny,
                    nz,0,-nx,
                    -ny,nx,0;
    RotateMatrix *= sin(rot_rad);
    RotateMatrix += (cos(rot_rad) * I);
    //t stands for transpose
    RotateMatrix += (axis * axis.transpose()*(1-cos(rot_rad)));

    //从0，0位置截止3*3的矩阵，可直接赋值，其他内容不变
    model.block(0,0,3,3) = RotateMatrix;

    return model;
}



int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    //rotate axis
    Vector3f rot_axis = {1,1,0};

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);


        //r.set_model(get_model_matrix(angle));//rotate around z-axis
        r.set_model(get_rotation(rot_axis,angle));

        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {//key27---esc
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        //r.set_model(get_model_matrix(angle));//rotate around z-axis
        r.set_model(get_rotation(rot_axis,angle));

        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
