# Dobot CR5 - Tecnológico de Monterrey Campus Estado de México

## Objetivo
Este repositorio tiene como objetivo hacer uso de ROS para manipular un brazo Robótico Dobot CR5.
Este movimiento se ve implementa visión computacional para la detección de arucos y hacer que el brazo se mueva a la ubicación detectada.

## Build del proyecto

```
cd catwkin_ws
git clone https://github.com/Dobot-CR-Tec-CEM/TecCEM_DobotCR5
catkin_make
```

## Definición de variable de entorno
Es importante que se defina desde el bash el tipo de modelo de brazo que se va a utilizar. En el repositorio se pueden lanzar en simulación sin gripper, los modelos CR3, CR5 y CR10.

Ejecutar una única vez. El tipo se puede cambiar con los otros modelos disponibles: cr3, cr5, cr10. Para los fines de este proyecto y por la automatización del repositorio original, se definió a cr5_gripper.
```
echo "export DOBOT_TYPE=cr5_gripper" >> ~/.bashrc
source ~/.bashrc
```
Se hace un source del workspace
```
source devel/setup.bash
```

## Ejecución de Simulación
Para lanzar la simulación de Gazebo con los controladores de Moviet y Rviz es el siguiente:

Mundo de gazebo diseñado. Es importante señalar que este mundo **Lanza un tópico de una cámara Kinect v1 que se encuentra montada en la parte superior del área de trabajo.** *El gripper no se mueve*
```
roslaunch arm_manipulator gazebo_world.launch
```
Para levantar moveit con los controladores definidos. 
```
roslaunch cr5_gripper_moveit cr5_gripper_moveit_planning_execution.launch sim:=True
```
Para abrir Rviz en la configuración previamente definido.
```
roslaunch cr5_gripper_moveit moveit_rviz.launch config:=true
```
Para lanzar el nodo que realiza el movimiento del brazo
```
rosrun arm_manipulator interaction.py 
```

## Ejecución en Físico

Para ejecutar los nodos que cuentan 

Nodo que se conecta via TCP al controlador del Brazo CR5. 
```
roslaunch dobot_bringup bringup.launch
```
Para lanzar moveit con la configuración del brazo fsico y lanzar RVIZ con la configuración previamente diseñada.
```
roslaunch dobot_moveit moveit.launch config:=true
```
Es importante asegurarse que la computadora esté conectada al robot y pueda enviar instrucciones por lo que es necesario que en Rviz se agrege un panel para habilitar y deshabilitar el brazo.
1. En la barra de herramientas de Rviz, Panel --> "Add New Panel"
2. Seleccionar DobotControl -> "OK"
3. Dar click en "DisableRobot" y luego "EnableRobot" para habilitarlo

**En caso de no escuchar un *click* del brazo, volver a ejecutar las primeras dos líneas.**

Para lanzar el nodo que lee los frames de la cámara montada en la parte superior del área de tareas del robot.
```
roslaunch usb_cam usb_cam-test.launch
```
Para lanzar el nodo que identifica los arucos y manda sus coordenadas en en /pose_aruco
```
rosrun arm_manipulator aruco_coords.py
```
Nodo principal que realiza la trayectoria del robot y que ejecuta varios servicios para abrir y cerrar el gripper, así como para definir la carga mínima en el gripper.
```
rosrun arm_manipulator interaction.py
```

# Documentación de ayuda
### Información del robot y del gripper
- [Documentacion de Dobot](https://docs.trossenrobotics.com/dobot_cr_cobots_docs/index.html)

- [Página Oficial de Dobot](https://www.dobot-robots.com/products/cr-series/dobot-cr-series.html)

- [Documentación de Dobot adicional](https://www.trossenrobotics.com/Shared/DOBOT/CR_App_Guide.pdf)

- [Repositorio de Github de Dobot para manipulación con ROS](https://github.com/Dobot-Arm/CR_ROS)

- [Repositorio de Gripper DH instalado](https://github.com/DH-Robotics/dh_gripper_ros/tree/master/dh_gripper_driver)

- [Documentacin del Gripper DH instalado](https://www.trossenrobotics.com/Shared/DH/ag-95-short-manual_v2.1-modbus-rtu.pdf)

- [Formato de SDF](http://sdformat.org/)


### Problemas del pasado
- [Path dinámico para modelos](https://answers.gazebosim.org//question/3402/import-custom-dae-file-in-gazebo-from-the-world-file-error-of-path-does-not-exist/)

- [Path dinámico pt.2](https://answers.gazebosim.org//question/16159/how-to-include-uri-relatively/)

- [Errores de Fake_controller_manager](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/fake_controller_manager/fake_controller_manager_tutorial.html)
- [Error Type-effort-controllers-jointsposition-controller-does-not-exist](https://answers.ros.org/question/144556/controller-type-effort_controllersjointpositioncontroller-does-not-exist/)

Cambios de controladores en gripper para simulación en gazebo. La versión para melodic es Gazebo 9, el gripper originalmente está hecho para Kinetic, gazebo 7.
- [Issue resuelto en headers](https://github.com/filesmuggler/robotiq/commit/70fd72e982674c4231bd6aed414db63bf3ccb55b)

# Futuras mejoras ideales.
- Mejorar el algoritmo de visión que obtenga X, Y y Theta del aruco y realizar la transformación hacia el efector final.
- Realizar un nodo que tenga el estado actual de cada joint físico.
