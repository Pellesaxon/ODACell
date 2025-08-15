
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "world_creator_cpp",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    RCLCPP_INFO(node->get_logger(), "Starting collision object adder node...");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]()
                { executor.spin(); })
        .detach();

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "base_link";
    collision_object.id = "sensor_assembly_collision_object";
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("mg400_ros2_bringup");
    std::string mesh_file_path = package_share_directory + "/urdf/meshes/sensor_assembly.stl";

    shapes::Mesh *m = shapes::createMeshFromResource("file://" + mesh_file_path, Eigen::Vector3d(0.001, 0.001, 0.001));
    if (!m)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to load mesh file: %s", mesh_file_path.c_str());
        rclcpp::shutdown();
        return -1;
    }
    RCLCPP_INFO(node->get_logger(), "Successfully loaded mesh file: %s", mesh_file_path.c_str());

    // Find some way to shut the material error up

    shape_msgs::msg::Mesh sensor_assembly_mesh;
    shapes::ShapeMsg sensor_assembly_mesh_msg;
    shapes::constructMsgFromShape(m, sensor_assembly_mesh_msg);
    sensor_assembly_mesh = boost::get<shape_msgs::msg::Mesh>(sensor_assembly_mesh_msg);
    collision_object.meshes.push_back(sensor_assembly_mesh);

    // Set pose (position and orientation) of the collision object
    geometry_msgs::msg::Pose sensor_pose;

    // Offset for sensor_assembly.stl assembly
    // Object position in respect to robot (which is at the origin)
    sensor_pose.position.x = 0.005;
    sensor_pose.position.y = 0.0;
    sensor_pose.position.z = -0.0405;
   
    sensor_pose.orientation.w = 0.0; 
    sensor_pose.orientation.x = 0.0;
    sensor_pose.orientation.y = 0.0;
    sensor_pose.orientation.z = 0.0;


    collision_object.mesh_poses.push_back(sensor_pose);

    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    // Light gray colour for the collision object
    std_msgs::msg::ColorRGBA object_color;
    object_color.r = 0.8f; // Red
    object_color.g = 0.8f; // Green
    object_color.b = 0.8f; // Blue
    object_color.a = 1.0f; // Alpha (opacity)
    // Add the collision object to the scene
    //planning_scene_interface.applyCollisionObject(collision_object);
    planning_scene_interface.applyCollisionObject(collision_object, object_color);

    RCLCPP_INFO(node->get_logger(), "Successfully added collision object '%s' to the planning scene.", collision_object.id.c_str());


    // PRINTER 
    // --- Add printer as a large rectangle ---
    moveit_msgs::msg::CollisionObject printer_object;
    printer_object.header.frame_id = "base_link";
    printer_object.id = "AnserSPH";

    // Define the primitive and its dimensions (a box)
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    double printer_x_dim = 0.1;
    double printer_y_dim = 0.1;
    double printer_z_dim = 0.25;
    primitive.dimensions[primitive.BOX_X] = printer_x_dim;
    primitive.dimensions[primitive.BOX_Y] = printer_y_dim;
    primitive.dimensions[primitive.BOX_Z] = printer_z_dim;

    // Define the pose of the printer. The pose is for the center of the object.
    // We calculate the center based on the "starting" XZ position.
    geometry_msgs::msg::Pose printer_pose;
    printer_pose.position.x = 0.391 + (printer_x_dim / 2.0); // Starts at 391mm and extends in +x
    printer_pose.position.y = 0.0;                           // Centered on the y-axis
    printer_pose.position.z = 0.2085 + (printer_z_dim / 2.0); // bottom surface is at 208mm
    printer_pose.orientation.w = 1.0;

    printer_object.primitives.push_back(primitive);
    printer_object.primitive_poses.push_back(printer_pose);
    printer_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    // Add the printer to the scene
    planning_scene_interface.applyCollisionObject(printer_object);
    RCLCPP_INFO(node->get_logger(), "Successfully added collision object '%s' to the planning scene.", printer_object.id.c_str());


    rclcpp::shutdown();
    return 0;
}