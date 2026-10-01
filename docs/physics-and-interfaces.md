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
| Face inferior do colchão | 0,23 m acima do piso (`0,34 − 0,22/2`) | colisão da cama em `casa.sdf` e `bed_test.sdf` |
| Altura conservadora do robô | 0,14 m (LiDAR em 0,133 m; margem de 7 mm) | `model.sdf` e ensaio de cama |
| Folga nominal sob a cama | 0,09 m (`0,23 − 0,14`) | cálculo geométrico; não é medição dinâmica |
| Tapete | 1,7 × 1,15 m; 6 mm de espessura; atrito `mu=0,55`, `mu2=0,75` | mundo `casa.sdf` |
| Soleira de estresse | `0,12 × 0,68 m`, 12 mm de altura; centro em `(0; -0,40) m`, alinhada com a abertura da porta | mundo `casa.sdf` e fixture `threshold_test.sdf` |
| Passo da física | 2 ms | mundo `casa.sdf` |

Esses são valores de partida para ensaio. A soleira de 12 mm não é garantia de aprovação: o relatório deve registrar se a geometria realmente cruza o obstáculo nas condições instaladas.

O ensaio direto dedicado de 30/09/2026 aprovou a soleira de 12 mm. O robô percorreu 0,9756 m, manteve desvio lateral abaixo de 0,001 mm e terminou com o centro em `x=0,3256 m`, além do plano de saída `x=0,263 m`. O critério exige também assentamento por 0,5 s. A soleira no mundo `casa.sdf` agora usa as mesmas dimensões e orientação da fixture, no centro da abertura; o cenário de mobilidade cruza o obstáculo no eixo `x`. Este resultado comprova apenas a altura de 12 mm testada; não mede o limite máximo do robô.

## Ensaios físicos dedicados

Execute cada cenário em uma instância nova do simulador: `ros2 launch robot_tests physics_tests.launch.py scenario:=threshold` e `ros2 launch robot_tests physics_tests.launch.py scenario:=bed`. O script `scripts/run_physics_tests.sh` executa a sequência completa e guarda os relatórios em `results/`. Os dois fixtures usam o nome de mundo SDF `casa` para compartilhar a ponte de `/clock` do lançamento principal. A pose física vem do Gazebo em `/world/casa/pose/info`, isolada em ROS como `/physics/pose_info`; ela não é injetada no TF usado pelo SLAM.

| Ensaio | Condição de aprovação | Relatório |
|---|---|---|
| Soleira | Robô inicia em `(-0,65; 0)` alinhado, recebe avanço direto a 0,12 m/s, cruza o plano de saída `x=0,263 m` com toda a estrutura e permanece assentado por 0,5 s. O obstáculo atravessa toda a pista, impedindo desvio lateral. | `threshold_report.json` |
| Cama | Robô inicia em `(1,35; 1,75)`, recebe metas Nav2 no centro `(2,65; 1,75)` e após a cama `(4,05; 1,75)`. Caminho planejado e pose real precisam atravessar a faixa central sob o colchão, passar pelos planos de entrada/centro/saída e evitar as quatro pernas. | `bed_clearance_report.json` |

Ambos os relatórios incluem tentativa, motivo de aprovação ou falha, distância percorrida, duração, pose inicial/final e amostras de pose real com altura. O da soleira registra a altura testada e, se aprovada, a **maior altura aprovada entre as alturas ensaiadas**. O fixture atual contém somente 12 mm, portanto esse campo não estabelece o limite absoluto do robô. O da cama lê o vão do SDF instalado, registra a folga nominal e a menor folga calculada ao longo da travessia, incluindo inclinação do corpo; o travamento é detectado por falta de progresso da pose física. O ensaio passou com folga nominal e mínima de 0,09 m. Não há sensor de contato, então o relatório não afirma ausência absoluta de colisões.

O LiDAR opera a 0,133 m, abaixo do colchão. O mapa de fixture marca as quatro pernas e a área sob o colchão permanece navegável com folga vertical suficiente. O runner verifica a ocupação das pernas no mapa, a trajetória planejada e um teste geométrico de altura; um planejador 2D isolado não representa o volume vertical do colchão. A detecção de pernas online não foi validada: essas ocupações são pré-configuradas no mapa de teste. A face inferior de 0,23 m corrige o comentário anterior de 0,20 m: cada perna termina em 0,20 m, mas o colchão começa em 0,23 m.

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
