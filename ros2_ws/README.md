# azemauto ROS 2 Humble Workspace

Workspace ROS 2 Humble structuré pour un robot réel avec ArduPilot bas niveau, Raspberry Pi 4 embarqué et PC Ubuntu pour supervision, téléopération, mapping puis préparation à la navigation autonome.

## Vision du projet

Plan d'évolution retenu :

1. affichage RViz et validation des capteurs
2. téléopération manuelle depuis le PC
3. mapping pendant la conduite manuelle puis sauvegarde de carte
4. localisation sur carte existante
5. navigation autonome avec `Nav2`
6. amélioration future via `robot_localization`

ArduPilot reste le contrôleur bas niveau à chaque étape.

## Choix technique pour l'étape mapping

Solution retenue pour cette étape :

- `rplidar_ros` sur le Raspberry Pi pour fournir le topic `/scan` officiel du robot
- `slam_toolbox` pour construire une carte 2D exploitable plus tard avec la pile navigation
- `depthimage_to_laserscan` seulement en secours si `use_lidar:=false`

Pourquoi ce choix :

- il fournit un vrai scan 2D plus stable pour le mapping, AMCL puis Nav2
- il exploite directement la TF et l'odom déjà disponibles
- il prépare une carte 2D standard adaptée à la suite `robot_localization` + `Nav2`
- la D435 reste disponible pour la perception sans être surchargée par le rôle de scanner principal

Alternative non retenue pour cette étape :

- `rtabmap_ros` reste très intéressant avec une D435 si tu veux plus tard du SLAM RGB-D/3D plus riche
- en revanche, pour une progression courte et propre vers une carte 2D de navigation, `slam_toolbox` est plus direc

## Choix technique pour l'étape localization

Solution retenue pour cette étape :

- `nav2_map_server` pour charger le fichier de carte `.yaml`
- `nav2_amcl` pour localiser le robot sur cette carte à partir de `/scan` et de la TF `odom -> base_link`

Pourquoi je ne retiens pas `slam_toolbox` comme solution principale ici :

- `slam_toolbox` en mode localization attend un pose-graph sérialisé, pas simplement une carte `.yaml`
- ton besoin principal est de lancer directement `map_file:=.../map.yaml`
- `AMCL` est la solution la plus naturelle pour une carte d'occupation 2D et prépare mieux la transition vers `Nav2`

Bonus fourni dans le workspace :

- un template `slam_toolbox` de localisation sur pose-graph si tu veux plus tard exploiter un fichier `.posegraph`

## Choix technique pour l'étape navigation autonome

Solution retenue pour cette étape :

- `SmacPlannerHybrid` comme planner global
- `RegulatedPurePursuitController` comme contrôleur local
- `nav2_velocity_smoother` pour lisser les consignes avant `/cmd_vel`
- `AMCL` + carte existante pour la localisation
- un arbre de comportement Nav2 dédié sans `Spin` pour respecter la cinématique Ackermann

Pourquoi ce choix :

- `SmacPlannerHybrid` produit des trajectoires non-holonomiques plus crédibles pour un rover Ackermann
- `Regulated Pure Pursuit` est une option simple et robuste pour un robot réel avec `/scan` stable
- la suppression des rotations sur place évite les comportements incompatibles avec le châssis
- `velocity_smoother` aide à envoyer des consignes plus propres au bridge MAVROS

## Hypothèses retenues

- ROS 2 Humble est installé sur le Raspberry Pi 4 et sur le PC Ubuntu.
- Le workspace est présent sur les deux machines sous `~/azemauto/ros2_ws`.
- La carte ArduPilot est déjà opérationnelle et accessible depuis le Pi.
- Le lien Pi -> contrôleur ArduPilot passe par `/dev/ttyACM0` en `115200` bauds par défaut.
- Le Pi et le PC sont sur le même réseau local et peuvent échanger du multicast DDS.
- La RealSense D435 est branchée au Raspberry Pi 4 et reconnue par le système.
- Un RPLIDAR A1M8 est branché au Raspberry Pi 4 sur `/dev/ttyUSB0`.
- L'odom ArduPilot fournit bien la TF `odom -> base_link` via `my_robot_bridge`.
- Pour la téléop ROS 2 vers ArduPilot, le rover sera placé en mode `GUIDED` avant envoi des commandes.

Adapte surtout `fcu_url` si ta carte apparaît ailleurs, par exemple `serial:///dev/serial0:921600`.

## Architecture

### Ce qui tourne sur le Raspberry Pi

- `mavros` pour l'interface ArduPilot
- `rplidar_ros` pour publier le LiDAR USB sur `/scan`
- `realsense2_camera_node` pour la D435
- `robot_state_publisher` pour les frames du robot
- `my_robot_bridge/mavros_bridge` pour republier les données MAVROS proprement
- `my_robot_bridge/cmd_vel_to_mavros` pour convertir `/cmd_vel` en commandes MAVROS pendant la téléop et le mapping
- `my_robot_bridge/auto_arm_disarm` pour automatiser `GUIDED`, l'armement et le désarmement sécurisés via MAVROS

### Ce qui tourne sur le PC

- `rviz2` pour la visualisation
- `depthimage_to_laserscan` seulement en repli si `use_lidar:=false`
- `slam_toolbox` pour le mapping 2D
- `nav2_map_server` pour charger une carte sauvegardée
- `nav2_amcl` pour la localisation sur carte existante
- `nav2_planner` + `nav2_controller` + `nav2_bt_navigator` + `nav2_velocity_smoother` pour la navigation autonome
- `nav2_lifecycle_manager` pour piloter `map_server` et `amcl`
- `joy_node` + `teleop_twist_joy` si tu téléopères à la manette
- `teleop_twist_keyboard` si tu téléopères au clavier

## Arborescence

```text
ros2_ws/
├── README.md
├── pc_env.sh
├── pi_env.sh
├── scripts/
│   └── setup_geographiclib.sh
└── src/
    ├── my_robot_bridge/
    │   ├── my_robot_bridge/
    │   │   ├── __init__.py
    │   │   ├── auto_arm_disarm_node.py
    │   │   ├── cmd_vel_to_manual_control_node.py
    │   │   ├── cmd_vel_to_mavros_node.py
    │   │   ├── cmd_vel_to_rc_override_node.py
    │   │   ├── mavros_bridge_node.py
    │   │   └── scan_timestamp_bridge_node.py
    │   ├── package.xml
    │   ├── resource/
    │   │   └── my_robot_bridge
    │   ├── setup.cfg
    │   └── setup.py
    ├── my_robot_bringup/
    │   ├── config/
    │   │   ├── future/
    │   │   │   ├── nav2/
    │   │   │   │   └── nav2_params_template.yaml
    │   │   │   └── robot_localization/
    │   │   │       ├── ekf_template.yaml
    │   │   │       └── navsat_transform_template.yaml
    │   │   ├── localization/
    │   │   │   ├── amcl_localization.yaml
    │   │   │   └── slam_toolbox_localization_posegraph_template.yaml
    │   │   ├── nav2/
    │   │   │   └── nav2_ackermann.yaml
    │   │   ├── lidar/
    │   │   │   └── rplidar_a1m8.yaml
    │   │   ├── mapping/
    │   │   │   ├── depth_to_scan.yaml
    │   │   │   └── slam_toolbox_mapping.yaml
    │   │   ├── mavros/
    │   │   │   └── mavros_params.yaml
    │   │   ├── network/
    │   │   │   └── cyclonedds.xml
    │   │   ├── realsense/
    │   │   │   └── realsense_d435.yaml
    │   │   ├── safety/
    │   │   │   └── auto_arm_disarm.yaml
    │   │   └── teleop/
    │   │       ├── cmd_vel_to_manual_control.yaml
    │   │       ├── cmd_vel_to_mavros.yaml
    │   │       ├── cmd_vel_to_rc_override.yaml
    │   │       └── xbox_teleop.yaml
    │   ├── launch/
    │   │   ├── pc_nav2.launch.py
    │   │   ├── future_localization.launch.py
    │   │   ├── pc_localization.launch.py
    │   │   ├── pc_mapping.launch.py
    │   │   ├── pc_teleop_joy.launch.py
    │   │   ├── pc_visualization.launch.py
    │   │   ├── pi_bringup.launch.py
    │   │   └── pi_mapping.launch.py
    │   ├── behavior_trees/
    │   │   ├── navigate_through_poses_ackermann.xml
    │   │   └── navigate_to_pose_ackermann.xml
    │   ├── package.xml
    │   ├── setup.cfg
    │   └── setup.py
    ├── my_robot_description/
    │   ├── launch/
    │   │   └── display_description.launch.py
    │   ├── rviz/
    │   │   └── display_description.rviz
    │   └── urdf/
    │       └── azemauto.urdf.xacro
    └── my_robot_rviz/
        └── rviz/
            ├── azemauto.rviz
            ├── azemauto_localization.rviz
            ├── azemauto_mapping.rviz
            └── azemauto_nav2.rviz
```

## Référence technique du code actuel

### Packages ROS 2 du workspace

| Package | Type | Rôle principal |
| --- | --- | --- |
| `my_robot_bridge` | `ament_python` | Nœuds de bridge entre MAVROS, topics propres ROS 2 et supervision sécurité |
| `my_robot_bringup` | `ament_python` | Launch files, paramètres capteurs/navigation/sécurité, BT Nav2 Ackermann |
| `my_robot_description` | `ament_cmake` | Modèle `xacro` du rover, frames, launch RViz de description |
| `my_robot_rviz` | `ament_cmake` | Profils RViz dédiés (visualisation, mapping, localisation, nav2) |

### Nœuds personnalisés (`my_robot_bridge`)

| Nœud | Entrées | Sorties | Usage |
| --- | --- | --- | --- |
| `mavros_bridge` | `/mavros/imu/data`, `/mavros/global_position/global`, `/mavros/mavros/odom` | `/sensors/imu/data`, `/sensors/gps/fix`, `/odom/raw`, marqueurs GPS/IMU, path GPS | Nettoyer les topics MAVROS et publier TF `odom -> base_link` |
| `cmd_vel_to_mavros` | `/cmd_vel` | `/teleop/cmd_vel_limited`, `/mavros/setpoint_velocity/cmd_vel`, `/mavros/setpoint_velocity/cmd_vel_unstamped` | Bridge vitesse standard vers MAVROS |
| `cmd_vel_to_rc_override` | `/cmd_vel` | `/mavros/mavros/override` (`OverrideRCIn`) | Pilotage par PWM RC override |
| `cmd_vel_to_manual_control` | `/cmd_vel` | `/mavros/mavros/send` (`ManualControl`) | Pilotage via message MAVLink `MANUAL_CONTROL` |
| `auto_arm_disarm` | état MAVROS + `/scan` + `/odom/raw` + `/amcl_pose` + `/cmd_vel` + `/cmd_vel_nav` | appels services `/mavros/set_mode` et `/mavros/cmd/arming` | Arm/disarm automatique sous contraintes capteurs/sécurité |
| `scan_timestamp_bridge` | `/scan/raw` | `/scan` | Re-stamp du LaserScan (utilisé en fallback depth-to-scan pendant le mapping PC) |

### Modes `control_mode` disponibles sur le Raspberry Pi

Le launch `pi_mapping.launch.py` peut sélectionner 3 bridges de commande :

| `control_mode` | Nœud activé | Topic MAVROS de sortie | Fichier de config |
| --- | --- | --- | --- |
| `mavros_cmd_vel` (défaut) | `cmd_vel_to_mavros` | `/mavros/setpoint_velocity/cmd_vel` (+ `cmd_vel_unstamped`) | `config/teleop/cmd_vel_to_mavros.yaml` |
| `rc_override` | `cmd_vel_to_rc_override` | `/mavros/mavros/override` | `config/teleop/cmd_vel_to_rc_override.yaml` |
| `manual_control` | `cmd_vel_to_manual_control` | `/mavros/mavros/send` | `config/teleop/cmd_vel_to_manual_control.yaml` |

Exemples :

```bash
# Bridge RC override
ros2 launch my_robot_bringup pi_mapping.launch.py control_mode:=rc_override

# Bridge MANUAL_CONTROL
ros2 launch my_robot_bringup pi_mapping.launch.py control_mode:=manual_control
```

### Launch files et arguments clés

| Launch | Machine | Arguments utiles (default) |
| --- | --- | --- |
| `pi_bringup.launch.py` | Pi | `fcu_url:=serial:///dev/serial0:921600`, `use_lidar:=true`, `use_camera:=true`, `auto_arm:=false` |
| `pi_mapping.launch.py` | Pi | `fcu_url:=serial:///dev/serial0:921600`, `control_mode:=mavros_cmd_vel`, `command_timeout:=0.35`, `publish_rate:=15.0` |
| `pc_mapping.launch.py` | PC | `use_lidar:=true`, `use_camera:=true`, `use_joy:=false`, `use_static_odom_tf:=false` |
| `pc_localization.launch.py` | PC | `map_file:=$HOME/azemauto_maps/azemauto_site_01.yaml`, `autostart:=true` |
| `pc_nav2.launch.py` | PC | `map_file:=$HOME/azemauto_maps/azemauto_site_01.yaml`, `params_file:=config/nav2/nav2_ackermann.yaml`, `use_rviz:=true` |
| `pc_visualization.launch.py` | PC | `start_description:=false` |
| `pc_teleop_joy.launch.py` | PC | `joy_dev:=0`, `start_description:=false` |
| `future_localization.launch.py` | PC | Lance `ekf_node` + `navsat_transform_node` (templates `robot_localization`) |

### Profils RViz disponibles

| Fichier RViz | Cible principale | Fixed frame par défaut |
| --- | --- | --- |
| `my_robot_rviz/rviz/azemauto.rviz` | Visualisation générale capteurs + TF | `odom` |
| `my_robot_rviz/rviz/azemauto_mapping.rviz` | Mapping `slam_toolbox` | `map` |
| `my_robot_rviz/rviz/azemauto_localization.rviz` | AMCL + carte + `2D Pose Estimate` | `map` |
| `my_robot_rviz/rviz/azemauto_nav2.rviz` | Navigation Nav2 (plans + costmaps + Goal tool) | `map` |
| `my_robot_description/rviz/display_description.rviz` | Inspection du modèle URDF seul | `base_link` |

### Dimensions et capteurs (URDF `azemauto.urdf.xacro`)

- Gabarit robot : `1.28 m` (longueur), `1.05 m` (largeur), `0.46 m` (hauteur).
- Roues : rayon `0.155 m`, demi-empattement `0.475 m`, demi-voie `0.535 m`.
- Position capteurs dans `base_link` :
  `imu_link` `(0.00, 0.00, 0.05)`,
  `gps_link` `(-0.20, 0.00, 0.28)`,
  `laser_link` `(0.22, 0.00, 0.34)`,
  `camera_link` `(0.40, 0.00, 0.30)` (montée depuis `laser_link`).
- Footprint Nav2 utilisé dans les costmaps :
  `[[0.64, 0.525], [0.64, -0.525], [-0.64, -0.525], [-0.64, 0.525]]`.

### Scripts utilitaires inclus

| Script | Fonction |
| --- | --- |
| `pi_env.sh` | Charge ROS 2 + workspace + variables réseau (`ROS_DOMAIN_ID`, CycloneDDS) côté Pi |
| `pc_env.sh` | Même logique côté PC |
| `scripts/setup_geographiclib.sh` | Installe/vérifie le dataset `EGM96` requis par MAVROS |

## Dépendances système

### Raspberry Pi

```bash
sudo apt update
sudo apt install -y \
  ros-humble-mavros \
  ros-humble-mavros-extras \
  ros-humble-mavros-msgs \
  ros-humble-realsense2-camera \
  ros-humble-rplidar-ros \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2 \
  ros-humble-tf2-tools \
  ros-humble-xacro

sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
sudo usermod -a -G dialout $USER
```

### PC Ubuntu

```bash
sudo apt update
sudo apt install -y \
  ros-humble-depthimage-to-laserscan \
  ros-humble-joy \
  ros-humble-nav2-amcl \
  ros-humble-nav2-bringup \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-nav2-map-server \
  ros-humble-navigation2 \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2 \
  ros-humble-slam-toolbox \
  ros-humble-teleop-twist-joy \
  ros-humble-teleop-twist-keyboard \
  ros-humble-tf2-tools \
  ros-humble-xacro
```

## Build

### Raspberry Pi

```bash
cd ~/azemauto/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### PC Ubuntu

```bash
cd ~/azemauto/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Réseau ROS 2

Les deux machines doivent partager :

- le même `ROS_DOMAIN_ID`
- le même `RMW_IMPLEMENTATION`
- `ROS_LOCALHOST_ONLY=0`
- une configuration Cyclone DDS qui autorise la découverte réseau

Variables exactes :

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI=file://$HOME/azemauto/ros2_ws/src/my_robot_bringup/config/network/cyclonedds.xml
```

Tu peux aussi simplement sourcer les scripts fournis :

```bash
source ~/azemauto/ros2_ws/pi_env.sh
```

ou

```bash
source ~/azemauto/ros2_ws/pc_env.sh
```

## Topics importants

### Acquisition et état robot

- `/mavros/imu/data` : IMU brute issue d'ArduPilot
- `/mavros/global_position/global` : GPS brut MAVROS
- `/mavros/mavros/odom` (ou `/mavros/local_position/odom` selon la config MAVROS) : odométrie MAVROS
- `/sensors/imu/data` : IMU propre republiée par `my_robot_bridge`
- `/sensors/gps/fix` : GPS propre republié par `my_robot_bridge`
- `/odom/raw` : odométrie ROS 2 propre pour TF et futurs modules
- `/tf` et `/tf_static` : frames robot + RealSense + mapping

### Caméra RealSense

- `/sensors/camera/color/image_raw` : image couleur
- `/sensors/camera/depth/image_rect_raw` : image profondeur
- `/sensors/camera/depth/color/points` : nuage de points

### LiDAR

- `/scan` : scan 2D principal publié par le RPLIDAR
- `depthimage_to_laserscan` n'est lancé que si `use_lidar:=false` et `use_camera:=true`

### Téléopération

- `/cmd_vel` : commande de vitesse publiée depuis le PC
- `/teleop/cmd_vel_limited` : commande limitée et republiée par le bridge sur le Pi
- `/mavros/setpoint_velocity/cmd_vel` : consigne `TwistStamped` vers MAVROS (`control_mode:=mavros_cmd_vel`)
- `/mavros/setpoint_velocity/cmd_vel_unstamped` : consigne `Twist` vers MAVROS (`control_mode:=mavros_cmd_vel`)
- `/mavros/mavros/override` : sortie PWM `OverrideRCIn` (`control_mode:=rc_override`)
- `/mavros/mavros/send` : sortie `ManualControl` (`control_mode:=manual_control`)

### Mapping

- `/map` : carte d'occupation 2D en construction
- `/slam_toolbox/graph_visualization` : visualisation interne du graphe SLAM

### Localization

- `/map` : carte chargée depuis le fichier `.yaml`
- `/amcl_pose` : pose estimée du robot dans la carte
- `/initialpose` : pose initiale envoyée depuis RViz

### Navigation Nav2

- `/navigate_to_pose` : action Nav2 déclenchée depuis RViz ou CLI
- `/cmd_vel_nav` : commande brute générée par Nav2 avant lissage
- `/cmd_vel` : commande lissée envoyée au bridge MAVROS
- `/plan` : plan global calculé par le planner
- `/local_plan` : trajectoire locale suivie par le contrôleur
- `/global_costmap/costmap` : costmap globale Nav2
- `/local_costmap/costmap` : costmap locale Nav2

## Frames recommandées

- `map` : frame globale de la carte pendant le mapping
- `odom` : frame locale issue de l'odom robot
- `base_link` : frame principale du robot
- `imu_link` : IMU physique reliée à ArduPilot
- `gps_link` : antenne GPS
- `laser_link` : frame du RPLIDAR
- `camera_link` : base de la RealSense, montée en aval du support LiDAR
- `camera_depth_frame` : frame profondeur publiée par `realsense2_camera`

La chaîne attendue pour la localisation est :

- `map -> odom -> base_link -> laser_link -> camera_link`

## Étape 1 : affichage RViz

### Sur le Raspberry Pi

```bash
cd ~/azemauto/ros2_ws
source pi_env.sh
ros2 launch my_robot_bringup pi_bringup.launch.py \
  fcu_url:=serial:///dev/serial0:921600 \
  use_lidar:=true \
  lidar_serial_port:=/dev/ttyUSB0 \
  use_camera:=true
```

### Sur le PC Ubuntu

```bash
cd ~/azemauto/ros2_ws
source pc_env.sh
ros2 launch my_robot_bringup pc_visualization.launch.py
```

### Vérifications minimales

```bash
ros2 topic echo /sensors/imu/data --once
ros2 topic echo /sensors/gps/fix --once
ros2 topic echo /odom/raw --once
ros2 topic echo /scan --once
ros2 topic hz /scan
ros2 topic hz /sensors/camera/color/image_raw
ros2 topic hz /sensors/camera/depth/color/points
ros2 run tf2_ros tf2_echo base_link laser_link
ros2 run tf2_tools view_frames
```

## Étape 2 : téléopération

### Sur le Raspberry Pi

Lance la version Pi qui inclut le bridge de commandes vers MAVROS :

```bash
cd ~/azemauto/ros2_ws
source pi_env.sh
ros2 launch my_robot_bringup pi_mapping.launch.py \
  fcu_url:=serial:///dev/serial0:921600 \
  use_lidar:=true \
  lidar_serial_port:=/dev/ttyUSB0 \
  use_camera:=true
```

### Préparer ArduPilot pour accepter les commandes

Option recommandée :

- passer le rover en mode `GUIDED` depuis ton GCS habituel
- armer ensuite le rover avec les procédures de sécurité habituelles

Option ROS 2 depuis le Pi si tu veux tout faire par MAVROS :

```bash
cd ~/azemauto/ros2_ws
source pi_env.sh
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: GUIDED}"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```

### Téléop clavier depuis le PC

```bash
cd ~/azemauto/ros2_ws
source pc_env.sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Téléop joystick depuis le PC

```bash
cd ~/azemauto/ros2_ws
source pc_env.sh
ros2 launch my_robot_bringup pc_teleop_joy.launch.py joy_dev:=0
```

### Contrôles et vérifications

Sur le Pi ou le PC :

```bash
ros2 topic echo /teleop/cmd_vel_limited
```

Tu dois voir les commandes limitées apparaître quand tu conduis le robot.

## Étape 3 : mapping par téléopération

### Sur le Raspberry Pi

```bash
cd ~/azemauto/ros2_ws
source pi_env.sh
ros2 launch my_robot_bringup pi_mapping.launch.py \
  fcu_url:=serial:///dev/serial0:921600 \
  use_lidar:=true \
  lidar_serial_port:=/dev/ttyUSB0 \
  use_camera:=true
```

### Sur le PC Ubuntu

```bash
cd ~/azemauto/ros2_ws
source pc_env.sh
ros2 launch my_robot_bringup pc_mapping.launch.py use_lidar:=true use_camera:=true
```

### Conduite manuelle pendant le mapping

Clavier :

```bash
cd ~/azemauto/ros2_ws
source pc_env.sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Joystick :

```bash
cd ~/azemauto/ros2_ws
source pc_env.sh
ros2 launch my_robot_bringup pc_mapping.launch.py use_joy:=true joy_dev:=0
```

### Ce que tu dois voir dans RViz

- la carte `/map` qui se construit
- le robot et ses TF
- le scan `/scan`
- l'odom `/odom/raw`
- les images caméra et le nuage de points

### Vérifications utiles pendant le mapping

```bash
ros2 topic hz /scan
ros2 topic echo /map --once
ros2 topic echo /teleop/cmd_vel_limited
```

## Étape 3B : sauvegarde de la carte

Depuis le PC, pendant que `slam_toolbox` tourne :

```bash
mkdir -p ~/azemauto_maps
cd ~/azemauto/ros2_ws
source pc_env.sh
ros2 run nav2_map_server map_saver_cli -t /map -f ~/azemauto_maps/azemauto_site_01
```

Cela produit en général :

- `~/azemauto_maps/azemauto_site_01.pgm`
- `~/azemauto_maps/azemauto_site_01.yaml`

Option utile pour reprendre une session SLAM plus tard :

```bash
cd ~/azemauto/ros2_ws
source pc_env.sh
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: $HOME/azemauto_maps/azemauto_site_01.posegraph}"
```

## Étape 4 : localisation sur carte existante

### Choix retenu

La localisation est implémentée avec :

- `nav2_map_server` pour charger `map_file:=.../map.yaml`
- `nav2_amcl` pour estimer la pose du robot
- le LiDAR comme source principale de `/scan`
- `depthimage_to_laserscan` seulement en secours si `use_lidar:=false`

### Sur le Raspberry Pi

Le Pi reste sur la chaîne acquisition classique :

```bash
cd ~/azemauto/ros2_ws
source pi_env.sh
ros2 launch my_robot_bringup pi_bringup.launch.py \
  fcu_url:=serial:///dev/serial0:921600 \
  use_lidar:=true \
  lidar_serial_port:=/dev/ttyUSB0 \
  use_camera:=true
```

### Sur le PC Ubuntu

Utilise de préférence un chemin absolu ou `$HOME/...` pour la carte :

```bash
cd ~/azemauto/ros2_ws
source pc_env.sh
ros2 launch my_robot_bringup pc_localization.launch.py \
  map_file:=$HOME/azemauto_maps/map1.yaml \
  use_lidar:=true \
  use_camera:=true
```

Tu peux aussi démarrer localement la description robot sur le PC :

```bash
ros2 launch my_robot_bringup pc_localization.launch.py \
  map_file:=$HOME/azemauto_maps/map1.yaml \
  use_lidar:=true \
  use_camera:=true \
  start_description:=true
```

### Initialiser la pose dans RViz

Dans RViz :

1. vérifie que le fixed frame est `map`
2. clique sur l'outil `2D Pose Estimate`
3. clique à l'endroit où se trouve réellement le robot sur la carte
4. oriente la flèche dans le sens réel du robot

Cela publie sur :

- `/initialpose`

et permet à `AMCL` de converger rapidement.

### Vérifications utiles

Vérifier la carte :

```bash
ros2 topic echo /map --once
```

Vérifier la pose estimée :

```bash
ros2 topic echo /amcl_pose --once
```

Vérifier la chaîne TF :

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map base_link
```

### Résultat attendu

- le robot apparaît correctement dans la carte
- `map -> odom -> base_link` est bien publié
- quand le robot bouge, sa pose dans RViz reste cohérente sans refaire de mapping

### Bonus fourni

Si tu veux plus tard tester `slam_toolbox` en mode localization avec un pose-graph sérialisé :

- `src/my_robot_bringup/config/localization/slam_toolbox_localization_posegraph_template.yaml`

Important :

- ce mode attend un fichier `.posegraph`
- il ne remplace pas directement la carte `.yaml` chargée par `map_server`

## Étape 5 : navigation autonome avec Nav2

### Choix retenu

La navigation autonome est implémentée avec :

- `AMCL` + `map_server` pour la localisation sur carte existante
- `SmacPlannerHybrid` pour produire des chemins adaptés à un robot non holonomique
- `RegulatedPurePursuitController` réglé pour ralentir près des obstacles et dans les virages serrés
- une `local_costmap` fusionnée : LiDAR sur `/scan` + point cloud RealSense sur `/sensors/camera/depth/color/points`
- une `global_costmap` volontairement plus stable, basée sur la carte et le LiDAR
- `nav2_velocity_smoother` puis `nav2_collision_monitor` pour sortir un `/cmd_vel` plus sûr vers le bridge MAVROS
- un BT Nav2 Ackermann sans action `Spin`

### Sur le Raspberry Pi

Le Pi doit exposer les capteurs et surtout le bridge `/cmd_vel -> MAVROS` :

```bash
cd ~/azemauto/ros2_ws
source pi_env.sh
ros2 launch my_robot_bringup pi_mapping.launch.py \
  fcu_url:=serial:///dev/serial0:921600 \
  use_lidar:=true \
  lidar_serial_port:=/dev/ttyUSB0 \
  use_camera:=true \
  auto_arm:=true \
  max_linear_speed:=0.60 \
  max_angular_speed:=0.60
```

Avant d'envoyer un goal :

- mettre ArduPilot en mode `GUIDED`
- armer le rover selon tes procédures de sécurité habituelles
- vérifier que `/cmd_vel` arrive bien sur le Pi

### Sur le PC Ubuntu

```bash
cd ~/azemauto/ros2_ws
source pc_env.sh
ros2 launch my_robot_bringup pc_nav2.launch.py \
  map_file:=$HOME/azemauto_maps/map1.yaml \
  use_lidar:=true \
  use_camera:=true
```

Option utile si tu veux aussi publier localement la description robot :

```bash
ros2 launch my_robot_bringup pc_nav2.launch.py \
  map_file:=$HOME/azemauto_maps/map1.yaml \
  use_lidar:=true \
  use_camera:=true \
  start_description:=true
```

### Envoi d'un goal depuis RViz

Dans RViz :

1. vérifie que le fixed frame est `map`
2. initialise la pose avec `2D Pose Estimate` si nécessaire
3. clique sur l'outil `Nav2 Goal`
4. clique sur la position cible puis oriente la flèche d'arrivée

Le robot doit alors :

- calculer un plan global
- produire une trajectoire locale
- publier d'abord `/cmd_vel_nav`, puis `/cmd_vel_smoothed`, puis `/cmd_vel`
- se déplacer automatiquement vers le goal

### Obstacle avoidance avancé

Le profil Nav2 fourni pour cette étape ajoute trois niveaux de protection :

- `obstacle_layer` LiDAR pour des obstacles 2D robustes et stables
- `voxel_layer` RealSense pour détecter les volumes et obstacles hors plan LiDAR
- `collision_monitor` avec zones `slow` et `stop` autour du rover avant l'envoi final à ArduPilot

Choix important :

- la caméra est utilisée dans la `local_costmap` et dans la surveillance proche du robot
- la `global_costmap` reste volontairement basée sur `map + /scan` pour éviter les faux obstacles persistants dus à un nuage de points court-portée

### Vérifications utiles

Vérifier les actions Nav2 :

```bash
ros2 action list | sort
```

Vérifier la localisation :

```bash
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map base_link
```

Vérifier la commande :

```bash
ros2 topic echo /cmd_vel_nav --once
ros2 topic echo /cmd_vel_smoothed --once
ros2 topic echo /cmd_vel --once
```

Vérifier les plans et costmaps :

```bash
ros2 topic echo /plan --once
ros2 topic echo /local_plan --once
ros2 topic echo /global_costmap/costmap --once
ros2 topic echo /local_costmap/costmap --once
```

Vérifier la chaîne de sécurité :

```bash
ros2 lifecycle get /collision_monitor
ros2 topic echo /collision_monitor/polygon_stop --once
ros2 topic echo /collision_monitor/polygon_slow --once
```

Vérifier les capteurs utilisés pour l'évitement avancé :

```bash
ros2 topic hz /scan
ros2 topic hz /sensors/camera/depth/color/points
```

### Test par ligne de commande

Tu peux aussi envoyer un goal sans RViz :

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

### Résultat attendu

- le robot reçoit un goal depuis RViz
- `SmacPlannerHybrid` génère un plan compatible non holonomique
- `Regulated Pure Pursuit` suit la trajectoire sans rotation sur place
- le LiDAR alimente le coeur 2D de l'évitement d'obstacles
- la RealSense enrichit la `local_costmap` pour les obstacles en volume ou plus hauts
- `collision_monitor` ralentit ou stoppe la commande finale si un obstacle est trop proche
- `/cmd_vel` sécurisé est ensuite transmis à ArduPilot via MAVROS

### Procédure de validation obstacle avoidance

1. Lance `pc_nav2.launch.py` avec `use_lidar:=true` et `use_camera:=true`.
2. Vérifie que `local_costmap` et `global_costmap` se mettent à jour dans RViz.
3. Place un obstacle bas devant le robot : il doit apparaître au LiDAR et dans la `local_costmap`.
4. Place ensuite un obstacle partiellement hors plan LiDAR mais visible par la D435 : il doit apparaître dans la `local_costmap` via le point cloud.
5. Envoie un goal derrière cet obstacle.
6. Observe `cmd_vel_nav`, `cmd_vel_smoothed` puis `/cmd_vel` :
   le robot doit ralentir à l'approche, s'arrêter si la zone `stop` est pénétrée, puis replanifier si un passage alternatif existe.

## Supervision auto arm/disarm via ROS 2

Le workspace fournit désormais un superviseur `my_robot_bridge/auto_arm_disarm` pour éviter de passer par le GCS pendant la navigation autonome.

### Principe

- si le robot n'est pas en `GUIDED`, le nœud demande d'abord le mode `GUIDED`
- il n'arme que si `AMCL`, `scan`, `odom`, `cmd_vel` et `cmd_vel_nav` sont valides
- il désarme si la navigation devient inactive, si un capteur critique disparaît ou si un stop de sécurité est inféré

Important :

- le superviseur est volontairement désactivé par défaut
- il s'active en lançant le Pi avec `auto_arm:=true`
- la logique de stop `collision_monitor` est inférée à partir de `/cmd_vel_nav` non nul alors que `/cmd_vel` est forcé à zéro

### Fichier de configuration

- `src/my_robot_bringup/config/safety/auto_arm_disarm.yaml`

### Commande Raspberry Pi

```bash
cd ~/azemauto/ros2_ws
source pi_env.sh
ros2 launch my_robot_bringup pi_mapping.launch.py \
  fcu_url:=serial:///dev/serial0:921600 \
  use_lidar:=true \
  lidar_serial_port:=/dev/ttyUSB0 \
  use_camera:=true \
  auto_arm:=true \
  max_linear_speed:=0.60 \
  max_angular_speed:=0.60
```

### Topics et services surveillés

- `/mavros/state`
- `/cmd_vel`
- `/cmd_vel_nav`
- `/amcl_pose`
- `/scan`
- `/odom/raw`
- `/collision_monitor/polygon_stop`
- `/mavros/set_mode`
- `/mavros/cmd/arming`

### Procédure de test

1. Lance `pi_mapping.launch.py` sur le Pi avec `auto_arm:=true`.
2. Lance `pc_nav2.launch.py` sur le PC.
3. Initialise la pose si nécessaire dans RViz puis envoie un goal.
4. Vérifie sur le Pi que les logs montrent d'abord la demande `GUIDED`, puis la demande `ARM`.
5. Vérifie l'état :

```bash
ros2 topic echo /mavros/state --once
```

6. Arrête l'envoi de goals ou coupe Nav2 : le rover doit se désarmer après `disarm_timeout`.
7. Coupe temporairement `/scan` ou fais tomber `odom` : le rover doit se désarmer immédiatement.

### Résultat attendu

- pas besoin de GCS pour passer en `GUIDED`
- armement automatique seulement quand la navigation est réellement prête
- désarmement automatique en cas d'inactivité, de perte capteur ou de stop de sécurité
- logs explicites sur chaque transition

## Étape 6 : amélioration future via `robot_localization`

Les bases sont déjà présentes dans le workspace :

- `src/my_robot_bringup/config/future/robot_localization/ekf_template.yaml`
- `src/my_robot_bringup/config/future/robot_localization/navsat_transform_template.yaml`
- `src/my_robot_bringup/config/future/nav2/nav2_params_template.yaml`
- `src/my_robot_bringup/launch/future_localization.launch.py`

Suite logique recommandée :

1. stabiliser Nav2 avec `AMCL + /odom/raw + /scan`
2. ajouter `robot_localization` pour fusionner IMU + odom + GPS
3. produire un `/odometry/filtered`
4. réinjecter ensuite cette odométrie filtrée dans la pile navigation

## Procédure complète résumée

### Plan d'utilisation

1. Étape 1 : affichage RViz
   Lance `pi_bringup.launch.py` sur le Pi puis `pc_visualization.launch.py` sur le PC.
2. Étape 2 : téléop
   Lance `pi_mapping.launch.py` sur le Pi puis téléopère depuis le PC avec clavier ou joystick.
3. Étape 3 : mapping
   Lance `pc_mapping.launch.py` sur le PC pendant la téléop, puis sauvegarde la carte.
4. Étape 4 : localization
   Lance `pc_localization.launch.py`, charge la carte et initialise la pose dans RViz.
5. Étape 5 : navigation Nav2
   Lance `pi_mapping.launch.py` avec `auto_arm:=true`, puis `pc_nav2.launch.py` et envoie un goal avec `Nav2 Goal` dans RViz.
6. Étape 6 : suite
   Ajoute plus tard `robot_localization` pour améliorer encore la qualité de l'odom.

## Remarques importantes

- ArduPilot reste le contrôleur bas niveau.
- Ce workspace ne modifie pas l'architecture interne d'ArduPilot.
- La téléop ROS 2 s'appuie sur l'interface `/cmd_vel` puis sur un bridge vers MAVROS.
- Pour cette étape, `angular.z` est traité comme une consigne de rotation haute-niveau compatible avec une conduite manuelle simple.
- Le LiDAR est désormais la source officielle de `/scan` pour le mapping, la localisation et la future navigation.
- `depthimage_to_laserscan` reste disponible comme solution de repli si le LiDAR n'est pas branché.
- La RealSense reste dédiée aux images, à la profondeur et au nuage de points pour la perception future.
- `AMCL` utilise ici la TF `odom -> base_link` dérivée de `/odom/raw` via le bridge déjà en place.
- Le modèle de mouvement `AMCL` retenu est `DifferentialMotionModel`, par approximation réaliste pour ce rover Ackermann tant qu'on n'a pas encore une chaîne de fusion/commande plus avancée.
- Nav2 publie d'abord `/cmd_vel_nav`, puis `velocity_smoother` publie `/cmd_vel_smoothed`, puis `collision_monitor` arbitre la sécurité finale avant `/cmd_vel`.
- `auto_arm:=true` active un superviseur ROS 2 qui demande `GUIDED`, arme automatiquement quand la navigation est prête et désarme sur inactivité ou défaut critique.
- Le planner retenu est `SmacPlannerHybrid` avec rayon de braquage minimal estimé à `1.30 m`.
- Le BT Nav2 fourni supprime l'action `Spin` pour éviter les recoveries incompatibles avec un robot Ackermann.
- La fusion LiDAR + caméra est volontairement locale : cela améliore l'évitement proche sans dégrader la stabilité de la carte globale.
- Le topic point cloud utilisé dans cette configuration reste `/sensors/camera/depth/color/points`, pour rester compatible avec le workspace actuel.
