// Nodo que ejecuta una secuencia de trayectorias en el UR5 con gripper
// usando la interfaz MoveGroupInterface de MoveIt 2.
// Requiere que move_group esté corriendo (demo.launch.py de ur5_robotiq_moveit).

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>

using moveit::planning_interface::MoveGroupInterface;

// Planifica y ejecuta el objetivo que ya tenga configurado el grupo.
static bool planAndExecute(MoveGroupInterface & group, const rclcpp::Logger & logger,
                           const std::string & descripcion)
{
  RCLCPP_INFO(logger, "[%s] Planificando: %s", group.getName().c_str(), descripcion.c_str());

  MoveGroupInterface::Plan plan;
  if (!static_cast<bool>(group.plan(plan))) {
    RCLCPP_ERROR(logger, "[%s] No se pudo planificar: %s", group.getName().c_str(),
                 descripcion.c_str());
    return false;
  }
  RCLCPP_INFO(logger, "[%s] Plan encontrado en %.3f s. Ejecutando...", group.getName().c_str(),
              plan.planning_time);

  if (!static_cast<bool>(group.execute(plan))) {
    RCLCPP_ERROR(logger, "[%s] Falló la ejecución: %s", group.getName().c_str(),
                 descripcion.c_str());
    return false;
  }
  RCLCPP_INFO(logger, "[%s] Listo: %s", group.getName().c_str(), descripcion.c_str());
  return true;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "ur5_move_trajectory",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto logger = node->get_logger();

  // MoveGroupInterface necesita que el nodo esté girando en otro hilo
  // para recibir el estado actual del robot.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  // Grupos definidos en el Setup Assistant
  MoveGroupInterface arm(node, "ur_arm");
  MoveGroupInterface gripper(node, "gripper");

  arm.setEndEffectorLink("wrist_3_link");
  arm.setPlanningTime(5.0);
  arm.setNumPlanningAttempts(10);
  arm.setMaxVelocityScalingFactor(0.5);
  arm.setMaxAccelerationScalingFactor(0.5);
  gripper.setMaxVelocityScalingFactor(1.0);
  gripper.setMaxAccelerationScalingFactor(1.0);

  RCLCPP_INFO(logger, "Marco de planificación: %s", arm.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "Link de referencia del brazo: %s", arm.getEndEffectorLink().c_str());

  bool ok = true;

  // 1. Brazo a la pose con nombre "ready" (herramienta hacia abajo)
  arm.setNamedTarget("ready");
  ok = ok && planAndExecute(arm, logger, "brazo a la pose 'ready'");

  // 2. Abrir el gripper
  gripper.setNamedTarget("open");
  ok = ok && planAndExecute(gripper, logger, "abrir gripper");

  // 3. Brazo a un punto deseado: 15 cm a un lado y 10 cm más abajo,
  //    manteniendo la orientación actual de la herramienta
  geometry_msgs::msg::Pose objetivo;
  if (ok) {
    objetivo = arm.getCurrentPose().pose;
    RCLCPP_INFO(logger, "Pose actual: x=%.3f y=%.3f z=%.3f", objetivo.position.x,
                objetivo.position.y, objetivo.position.z);
    objetivo.position.y += 0.15;
    objetivo.position.z -= 0.10;
    RCLCPP_INFO(logger, "Punto objetivo: x=%.3f y=%.3f z=%.3f", objetivo.position.x,
                objetivo.position.y, objetivo.position.z);
    arm.setPoseTarget(objetivo);
    ok = planAndExecute(arm, logger, "brazo al punto objetivo");
  }

  // 4. Cerrar el gripper (simula tomar un objeto)
  if (ok) {
    gripper.setNamedTarget("close");
    ok = planAndExecute(gripper, logger, "cerrar gripper");
  }

  // 5. Subir 10 cm en línea recta (trayectoria cartesiana)
  if (ok) {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose arriba = objetivo;
    arriba.position.z += 0.10;
    waypoints.push_back(arriba);

    moveit_msgs::msg::RobotTrajectory trayectoria;
    const double fraccion = arm.computeCartesianPath(waypoints, 0.01, trayectoria);
    RCLCPP_INFO(logger, "[ur_arm] Trayectoria cartesiana calculada al %.1f %%", fraccion * 100.0);

    if (fraccion > 0.9) {
      ok = static_cast<bool>(arm.execute(trayectoria));
      RCLCPP_INFO(logger, "[ur_arm] Subida en línea recta %s", ok ? "ejecutada" : "falló");
    } else {
      RCLCPP_WARN(logger, "[ur_arm] No se completó la trayectoria cartesiana, se omite");
    }
  }

  // 6. Abrir el gripper (simula soltar el objeto)
  if (ok) {
    gripper.setNamedTarget("open");
    ok = planAndExecute(gripper, logger, "abrir gripper");
  }

  // 7. Volver a "home"
  if (ok) {
    arm.setNamedTarget("home");
    ok = planAndExecute(arm, logger, "brazo a la pose 'home'");
  }

  if (ok) {
    RCLCPP_INFO(logger, "Secuencia completada con éxito");
  } else {
    RCLCPP_ERROR(logger, "La secuencia se detuvo por un error");
  }

  rclcpp::shutdown();
  spinner.join();
  return ok ? 0 : 1;
}
