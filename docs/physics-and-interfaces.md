# Física, cenários e interfaces

## Geometria inicial

| Parâmetro | Valor inicial | Fonte |
|---|---:|---|
| Chassi | 0,32 × 0,19 × 0,072 m | `models/vacuum/model.sdf` |
| Largura total aproximada | 0,355 m | chassi e trilhas laterais |
| Altura máxima aproximada | 0,14 m | chassi e LiDAR |
| Massa do chassi | 3,2 kg | inércia SDF |
| Separação das esteiras | 0,29 m | `TrackedVehicle` |
| Rampa frontal | 0,14 × 0,18 × 0,020 m; inclinação de 0,25 rad | `models/vacuum/model.sdf` |
| Velocidade linear limitada | ±0,20 m/s | controlador nativo do Gazebo |
| Velocidade angular limitada | ±1,2 rad/s | controlador nativo do Gazebo |
| Passagem sob a cama | 0,20 m de vão | mundo `casa.sdf` |
| Tapete | 1,7 × 1,15 m; 6 mm de espessura; atrito `mu=0,55`, `mu2=0,75` | mundo `casa.sdf` |
| Soleira de estresse | 12 mm de altura; centro em `(-0,65; -1,15) m`, junto à borda inferior da porta | mundo `casa.sdf` |
| Passo da física | 2 ms | mundo `casa.sdf` |

Esses são valores de partida para ensaio. A soleira de 12 mm não é garantia de aprovação: o relatório deve registrar se a geometria realmente cruza o obstáculo nas condições instaladas.

Na última suíte executada, Nav2 alcançou o destino do cenário de soleira, mas o teste envia uma meta após a soleira e permite desvio pela rota livre; isso não demonstra uma subida frontal direta. O teste também ainda não mede a altura máxima vencida nem a passagem sob a cama. São pendências de validação física, não parâmetros assumidos como aprovados.

## Interfaces ROS

| Tópico | Tipo | Origem |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 ou comandos de teste → Gazebo |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR simulado |
| `/imu/data` | `sensor_msgs/Imu` | IMU simulada |
| `/wheel/odometry` | `nav_msgs/Odometry` | estimativa do sistema TrackedVehicle |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF de `robot_localization` |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM Toolbox ou map server |
| `/coverage/grid` | `nav_msgs/OccupancyGrid` | máscara separada de células visitadas |
| `/coverage/percent` | `std_msgs/Float32` | percentual de células livres conhecidas visitadas |
| `/battery_state` | `sensor_msgs/BatteryState` | modelo de bateria do supervisor |
| `/robot/state` | `std_msgs/String` | máquina de estados |
| `/robot/event` | `std_msgs/String` | eventos do supervisor |

O valor de `BatteryState.percentage` segue a convenção ROS de fração entre 0 e 1. A telemetria converte esse valor para percentual. `collision_proxy_lidar_clearance` é um alerta de aproximação do LiDAR, não um contato físico confirmado.
