#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>

constexpr double MY_PI = 3.1415926;
constexpr double D2R(float degree) { return degree * MY_PI / 180.0; };

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    double rad_angle = D2R(rotation_angle);

    double c = cos(rad_angle);
    double s = sin(rad_angle);

    model(0, 0) = c;
    model(0, 1) = -s;
    model(1, 0) = s;
    model(1, 1) = c;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    double eye_fov_rad, t, b, l, r;
    eye_fov_rad = D2R(eye_fov);
    t = abs(zNear * tan(eye_fov_rad / 2.0));
    r = t * aspect_ratio;
    l = -r;
    b = -t;
    Eigen::Matrix4f o, o1, o2, p2o;
    o1 << 2.0 / (r - l), 0, 0, 0, 
          1, 2.0 / (t - b), 0, 0, 
          0, 0, 2.0 / (zNear - zFar), 0, 
          0, 0, 0, 1;
    o2 << 1, 0, 0, -(r + l) / 2.0, 
          0, 1, 0, -(t + b) / 2.0, 
          0, 0, 1, -(zNear + zFar) / 2.0,
          0, 0, 0, 1;
    o = o1 * o2;
    p2o << zNear, 0, 0, 0, 
           0, zNear, 0, 0, 
           0, 0, zNear + zFar, -zNear * zFar, 
           0, 0, 1, 0;
    projection = o * p2o * projection;

    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    Eigen::Matrix4f rotation_T = Eigen::Matrix4f::Identity();

    double angle_rad = D2R(angle);
    Eigen::Matrix3f m, rotation_R = Eigen::Matrix3f::Identity();
    m << 0, -axis.z(), axis.y(), 
        axis.z(), 0, -axis.x(), 
        -axis.y(), axis.x(), 0;
    rotation_R = cos(angle_rad) * rotation_R + (1.0 - cos(angle_rad)) * axis * axis.transpose() + sin(angle_rad) * m;
    rotation_T.block<3, 3>(0, 0) = rotation_R;

    return rotation_T;
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
    }

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

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
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
