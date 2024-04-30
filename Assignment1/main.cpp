#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos, Eigen::Vector3f look_at)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    Eigen::Vector3f up = look_at.cross(Eigen::Vector3f(0, 0, 1)).cross(look_at).normalized();

    std::cout << "up:\n" << up << "\n";

    Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
    rotation.col(0).head(3) = look_at.cross(up);
    rotation.col(1).head(3) = up;
    rotation.col(2).head(3) = -look_at;

    view = rotation.transpose() * translate;

    return view;
}


float get_radian(float angle)
{
    return angle * MY_PI / 180;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    auto cos_v = cos(get_radian(rotation_angle));
    auto sin_v = sin(get_radian(rotation_angle));
    model << cos_v, -sin_v, 0, 0,
             sin_v, cos_v, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;

    return model;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle, Eigen::Vector3f axis)
{
    axis.normalize();
    auto cos_v = cos(get_radian(rotation_angle));
    auto sin_v = sin(get_radian(rotation_angle));
    Eigen::Matrix3f model = cos_v * Eigen::Matrix3f::Identity() +
                 (1 - cos_v) * axis * axis.transpose() + 
                sin_v * Eigen::Matrix3f{{0, -axis.z(), axis.y()},
                                         {axis.z(), 0, -axis.x()},
                                         {-axis.y(), axis.x(), 0}};
    Eigen::Matrix4f model4 = Eigen::Matrix4f::Identity();
    model4.block<3, 3>(0, 0) = model;
    return model4;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // projection << 
    //     -1 / (tan(get_radian(eye_fov / 2)) * aspect_ratio), 0, 0, 0,
    //     0, -1 / tan(get_radian(eye_fov / 2)), 0, 0,
    //     0, 0, (zNear + zFar) / (zNear - zFar), -2 * zNear * zFar / (zNear - zFar),
    //     0, 0, 1, 0;

    Eigen::Matrix4f m_scale = Eigen::Matrix4f();
    Eigen::Matrix4f m_translation = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f m_press = Eigen::Matrix4f();
    // t - b
    float t_b = abs(2 * (zNear * tan(get_radian(eye_fov / 2))));
    float t = t_b / 2;
    float b = -t;
    // r - l
    float r_l = t_b * aspect_ratio;;
    float r = r_l / 2;
    float l = -r;

    m_scale << 2 / r_l, 0, 0, 0,
               0, 2 / t_b, 0, 0,
               0, 0, 2 / (zNear - zFar), 0,
               0, 0, 0, 1;

    m_translation.col(3).head(3) << 0, 0, -(zNear + zFar) / 2;

    m_press << zNear, 0, 0, 0,
               0, zNear, 0, 0,
               0, 0, zNear + zFar, -zNear * zFar,
               0, 0, 1, 0;
    
    projection = m_scale * m_translation * m_press;
    return projection;
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

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {5, 5, 5};
    Eigen::Vector3f look_at = -eye_pos.normalized();

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos, look_at));
        r.set_projection(get_projection_matrix(45, 1, -0.1, -50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // r.set_model(get_model_matrix(angle));
        auto axis = Eigen::Vector3f(0.7, 0.95, 2);
        
        r.set_model(get_model_matrix(angle, axis));
        r.set_view(get_view_matrix(eye_pos, look_at));
        r.set_projection(get_projection_matrix(45, 1, -0.1, -50));

        r.vp_and_draw_line(-axis, axis);
        r.vp_and_draw_line(Eigen::Vector3f::Zero(), Eigen::Vector3f(1, 0, 0));
        r.vp_and_draw_line(Eigen::Vector3f::Zero(), Eigen::Vector3f(0, 2, 0));
        r.vp_and_draw_line(Eigen::Vector3f::Zero(), Eigen::Vector3f(0, 0, 3));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        for(int i=0; i<350; ++i) r.set_pixel(Eigen::Vector3f(15, i, 1), Eigen::Vector3f(255, 255, 255));
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
