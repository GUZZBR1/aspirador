# Aspirador robótico simulado — MVP 1

Workspace modular ROS 2 Jazzy + Gazebo Harmonic para Ubuntu 24.04 no WSL2. O MVP testa esteiras, SLAM, navegação Nav2, cobertura, bateria e retorno à base; não simula sucção.

## Instalar, compilar e executar

```bash
./scripts/install_dependencies.sh
./scripts/build_ws.sh
./scripts/run_mvp.sh
```

O comando padrão abre Gazebo e RViz, inicia a casa, o aspirador na base, SLAM Toolbox, Nav2, EKF, supervisor e telemetria. Para rodar sem janela gráfica:

```bash
./scripts/run_mvp.sh headless:=true
```

O primeiro ciclo começa sem mapa. Para salvar a casa mapeada, em outro terminal execute:

```bash
source /opt/ros/jazzy/setup.bash
source robot_ws/install/setup.bash
ros2 run nav2_map_server map_saver_cli -f robot_ws/src/robot_navigation/maps/casa
```

Depois reinicie em localização com:

```bash
./scripts/run_mvp.sh slam:=false map:=robot_ws/src/robot_navigation/maps/casa.yaml
```

Use `2D Pose Estimate` no RViz para inicializar o AMCL quando necessário. Mapas e telemetria de execução não são versionados pelo workspace.

## Testes

```bash
./scripts/run_mobility_tests.sh
./scripts/run_battery_cycle_test.sh
```

O teste de mobilidade inicia SLAM e Nav2, deixa a exploração autônoma mapear primeiro, pausa o supervisor e então testa avanço, ré, giro e objetivos nos cenários da casa. O teste de bateria usa limiares altos apenas para encurtar o ciclo e verifica saída da base, retorno por pontos já percorridos, carga e retomada. O limiar de operação normal continua configurado em 20%.

Os relatórios ficam em `results/mobility_report.json` e `results/battery_report.json`; cada execução também grava amostras JSONL em `results/run_*.jsonl`. O log de colisões é um proxy de proximidade LiDAR, não um sensor de contato.

## Pacotes e interfaces

- `robot_description`: SDF físico, URDF/Xacro de frames e casa de teste.
- `robot_simulation`: Gazebo, bridge, EKF e lançamento do MVP.
- `robot_navigation`: SLAM Toolbox, AMCL, Nav2 e configurações do `robot_localization`.
- `robot_autonomy`: exploração, cobertura, estados, bateria e docking simplificado.
- `robot_metrics`: telemetria JSONL.
- `robot_tests`: cenários automatizados e relatórios JSON.

Interfaces principais: `/cmd_vel`, `/scan`, `/imu/data`, `/odometry/filtered`, `/battery_state`, `/coverage/grid`, `/coverage/percent`, `/robot/state` e `/robot/event`. Os detalhes de geometria e mensagens estão em [docs/physics-and-interfaces.md](docs/physics-and-interfaces.md).

## Validação atual

O workspace compilou no WSL. O Gazebo Harmonic desta instalação publica o relógio em `/world/casa/clock`; o bridge encaminha esse tópico para `/clock`, usado pelo EKF, SLAM e Nav2.

O ciclo headless de bateria passou em 116,8 s: `EXPLORING → RETURNING_TO_DOCK → DOCKING → CHARGING → EXPLORING`, com retorno de 2,44 m. O mapa foi salvo em PGM/YAML (161 × 120 células, 5 cm/célula) e carregado depois por `map_server` com AMCL em `slam:=false`.

A suíte mais recente de mobilidade registrou 7 sucessos e 4 falhas em `results/mobility_report.json`. Passaram avanço (0,219 m), ré (-0,306 m), giro (2,412 rad), meta após a soleira, corredor, cadeira e rejeição do objetivo fora do mapa. Expiraram os objetivos de piso normal, tapete, canto e obstáculo frontal. Os cenários usam metas Nav2 em uma execução compartilhada; estes timeouts indicam que a navegação e o isolamento dos testes precisam de correção e não comprovam falha mecânica isolada. A telemetria registrou 30,97 m de odometria, 9,90% da área livre conhecida visitada e 62,8% de bateria após 357 s simulados. A meta do teste de soleira permite desvio e ainda não comprova subida frontal direta; a travessia sob a cama também não foi validada por cenário dedicado.

O lançamento gráfico padrão (`./scripts/run_mvp.sh`) iniciou Gazebo GUI e RViz no WSL, além de SLAM, Nav2 e exploração autônoma. Os testes automatizados continuam usando `headless:=true`.
