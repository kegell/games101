#include <iostream>
#include <opencv2/opencv.hpp>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"

float get_radian(float angle)
{
    return angle * MY_PI / 180;
}

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float angle)
{
    Eigen::Matrix4f rotation;
    angle = angle * MY_PI / 180.f;
    rotation << cos(angle), 0, sin(angle), 0,
                0, 1, 0, 0,
                -sin(angle), 0, cos(angle), 0,
                0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale << 2.5, 0, 0, 0,
              0, 2.5, 0, 0,
              0, 0, 2.5, 0,
              0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    return translate * rotation * scale;
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

Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
    return payload.position;
}

Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis)
{
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}

struct light
{
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f texture_color = {0, 0, 0};
    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        texture_color = payload.texture->getColor(payload.tex_coords[0], payload.tex_coords[1]);
    }

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    // payload.view_pos 是对经过MV变换后的三个顶点插值的结果，此时相机位置在原点。
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        
        // light坐标也应该进行V变换。
        Eigen::Vector3f view_space_light_pos = (get_view_matrix(eye_pos) * Eigen::Vector4f(light.position.x(), light.position.y(), light.position.z(), 1.0)).head<3>();
        
        Eigen::Vector3f view_pos_to_light = view_space_light_pos - point;
        Eigen::Vector3f view_pos_to_eye = -point;
        Eigen::Vector3f norm_half_vector = (view_pos_to_light.normalized() + view_pos_to_eye.normalized()).normalized();
        
        Eigen::Vector3f diffuse = kd.cwiseProduct(light.intensity) / view_pos_to_light.squaredNorm() * MAX(0, normal.normalized().dot(view_pos_to_light.normalized()));
        Eigen::Vector3f specular = ks.cwiseProduct(light.intensity) / view_pos_to_light.squaredNorm() * pow(MAX(0, normal.normalized().dot(norm_half_vector)), p);
        Eigen::Vector3f ambient = ka.cwiseProduct(amb_light_intensity);

        result_color += diffuse + specular + ambient;
    }
    return result_color * 255.f;
}

Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};
    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        // light坐标也应该进行V变换。
        Eigen::Vector3f view_space_light_pos = (get_view_matrix(eye_pos) * Eigen::Vector4f(light.position.x(), light.position.y(), light.position.z(), 1.0)).head<3>();
        
        Eigen::Vector3f view_pos_to_light = view_space_light_pos - point;
        Eigen::Vector3f view_pos_to_eye = -point;
        Eigen::Vector3f norm_half_vector = (view_pos_to_light.normalized() + view_pos_to_eye.normalized()).normalized();
        
        Eigen::Vector3f diffuse = kd.cwiseProduct(light.intensity) / view_pos_to_light.squaredNorm() * MAX(0, normal.normalized().dot(view_pos_to_light.normalized()));
        Eigen::Vector3f specular = ks.cwiseProduct(light.intensity) / view_pos_to_light.squaredNorm() * pow(MAX(0, normal.normalized().dot(norm_half_vector)), p);
        Eigen::Vector3f ambient = ka.cwiseProduct(amb_light_intensity);

        result_color += diffuse + specular + ambient;
    }

    return result_color * 255.f;
}



Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;
    
    // TODO: Implement displacement mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Position p = p + kn * n * h(u,v)
    // Normal n = normalize(TBN * ln)

    Eigen::Vector3f n = normal;
    Eigen::Vector3f t = payload.tagent;
    // 正交化
    t = (t - t.dot(n) * n).normalized();
    Eigen::Vector3f b = n.cross(t);

    Eigen::Matrix3f tbn;
    tbn << t, b, n;

    auto uv = payload.tex_coords;
    auto texture = payload.texture;
    auto h_p = (texture->getColor(uv.x(), uv.y())).norm();
    auto du = kh * kn * ((texture->getColor(uv.x() + 1.0 / texture->width, uv.y())).norm() - h_p);
    auto dv = kh * kn * ((texture->getColor(uv.x(), uv.y() + 1.0 / texture->height)).norm() - h_p);

    Eigen::Vector3f ln(-du, -dv, 1.0f);
    // 经过法线贴图后的法线
    Eigen::Vector3f result_normal = (tbn * ln).normalized();

    // 把texture的模当作高度贴图处理
    point += kn * n * h_p;
    normal = result_normal;


    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.

        // light坐标也应该进行V变换。
        Eigen::Vector3f view_space_light_pos = (get_view_matrix(eye_pos) * Eigen::Vector4f(light.position.x(), light.position.y(), light.position.z(), 1.0)).head<3>();
        
        Eigen::Vector3f view_pos_to_light = view_space_light_pos - point;
        Eigen::Vector3f view_pos_to_eye = -point;
        Eigen::Vector3f norm_half_vector = (view_pos_to_light.normalized() + view_pos_to_eye.normalized()).normalized();
        
        Eigen::Vector3f diffuse = kd.cwiseProduct(light.intensity) / view_pos_to_light.squaredNorm() * MAX(0, normal.normalized().dot(view_pos_to_light.normalized()));
        Eigen::Vector3f specular = ks.cwiseProduct(light.intensity) / view_pos_to_light.squaredNorm() * pow(MAX(0, normal.normalized().dot(norm_half_vector)), p);
        Eigen::Vector3f ambient = ka.cwiseProduct(amb_light_intensity);

        result_color += diffuse + specular + ambient;

    }

    return result_color * 255.f;
}


Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;


    float kh = 0.2, kn = 0.1;

    // TODO: Implement bump mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Normal n = normalize(TBN * ln)

    Eigen::Vector3f n = normal;
    Eigen::Vector3f t = payload.tagent;
    // 正交化
    t = (t - t.dot(n) * n).normalized();
    Eigen::Vector3f b = n.cross(t);

    Eigen::Matrix3f tbn;
    tbn << t, b, n;

    auto uv = payload.tex_coords;
    auto texture = payload.texture;
    auto h_p = (texture->getColor(uv.x(), uv.y())).norm();
    auto du = kh * kn * ((texture->getColor(uv.x() + 1.0 / texture->width, uv.y())).norm() - h_p);
    auto dv = kh * kn * ((texture->getColor(uv.x(), uv.y() + 1.0 / texture->height)).norm() - h_p);

    Eigen::Vector3f ln(-du, -dv, 1.0f);
    // 经过法线贴图后的法线
    Eigen::Vector3f result_normal = (tbn * ln).normalized();
    Eigen::Vector3f result_color(result_normal);

    return result_color * 255.f;
}

int main(int argc, const char** argv)
{
    std::vector<Triangle*> TriangleList;

    float angle = 140.0;
    bool command_line = false;

    std::string filename = "output.png";
    objl::Loader Loader;
    std::string obj_path = "../models/spot/";

    // Load .obj File
    bool loadout = Loader.LoadFile("../models/spot/spot_triangulated_good.obj");
    for(auto mesh:Loader.LoadedMeshes)
    {
        for(int i=0;i<mesh.Vertices.size();i+=3)
        {
            Triangle* t = new Triangle();
            for(int j=0;j<3;j++)
            {
                t->setVertex(j,Vector4f(mesh.Vertices[i+j].Position.X,mesh.Vertices[i+j].Position.Y,mesh.Vertices[i+j].Position.Z,1.0));
                t->setNormal(j,Vector3f(mesh.Vertices[i+j].Normal.X,mesh.Vertices[i+j].Normal.Y,mesh.Vertices[i+j].Normal.Z));
                t->setTexCoord(j,Vector2f(mesh.Vertices[i+j].TextureCoordinate.X, mesh.Vertices[i+j].TextureCoordinate.Y));
            }
            // 计算三角形面的切线向量
            t->calcTagent();
            TriangleList.push_back(t);
        }
    }

    rst::rasterizer r(700, 700);

    auto texture_path = "hmap.jpg";
    r.set_texture(Texture(obj_path + texture_path));

    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = texture_fragment_shader;

    if (argc >= 2)
    {
        command_line = true;
        filename = std::string(argv[1]);

        if (argc == 3 && std::string(argv[2]) == "texture")
        {
            std::cout << "Rasterizing using the texture shader\n";
            active_shader = texture_fragment_shader;
            texture_path = "spot_texture.png";
            r.set_texture(Texture(obj_path + texture_path));
        }
        else if (argc == 3 && std::string(argv[2]) == "normal")
        {
            std::cout << "Rasterizing using the normal shader\n";
            active_shader = normal_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "phong")
        {
            std::cout << "Rasterizing using the phong shader\n";
            active_shader = phong_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "bump")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = bump_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "displacement")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = displacement_fragment_shader;
        }
    }

    Eigen::Vector3f eye_pos = {0,0,10};

    r.set_vertex_shader(vertex_shader);
    r.set_fragment_shader(active_shader);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, -0.1, -50));

        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, -0.1, -50));

        //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);
        cv::imwrite(filename, image);
        key = cv::waitKey(10);

        if (key == 'a' )
        {
            angle -= 0.1;
        }
        else if (key == 'd')
        {
            angle += 0.1;
        }

    }
    return 0;
}
