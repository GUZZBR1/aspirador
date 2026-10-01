# Aspirador robótico simulado — MVP 1

Workspace modular ROS 2 Jazzy + Gazebo Harmonic para Ubuntu 24.04 no WSL2. O MVP testa esteiras, SLAM, navegação Nav2, cobertura, bateria e retorno à base; não simula sucção.

## Instalar, compilar e executar

```bash
./scripts/install_dependencies.sh
./scripts/build_ws.sh
./scripts/run_mvp.sh
```

O comando padrão abre Gazebo e RViz, inicia a casa, o aspirador na base, SLAM Toolbox, Nav2, EKF, supervisor e telemetria. No WSL2, use WSLg (com `WAYLAND_DISPLAY` ou `DISPLAY` configurado) ou um X server no Windows. Não é necessária uma GPU NVIDIA; para forçar renderização por software Mesa, rode `LIBGL_ALWAYS_SOFTWARE=1 ./scripts/run_mvp.sh`. Sem display, o script informa o requisito antes de iniciar; para rodar sem janela gráfica:

```bash
./scripts/run_mvp.sh headless:=true
```

É possível selecionar um mundo SDF alternativo com `world:=/caminho/absoluto/arquivo.sdf`.

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
./scripts/run_physics_tests.sh
```

O teste de mobilidade inicia SLAM e Nav2, deixa a exploração autônoma mapear, pausa o supervisor e move Nav2 para uma pose inicial definida antes de cada cenário. Use `scenario_name:=piso_normal` (ou outro nome em `mobility.json`) para testar um caso isolado; cada execução do launch inicia simulador novo. A meta da soleira exige comprovação de travessia frontal em pose Gazebo, não apenas sucesso do Nav2. O teste de bateria usa limiares altos para encurtar o ciclo, registra proximidade/velocidade do docking virtual, aumento de carga e retomada. O limiar normal continua em 20%. O teste físico roda em dois mundos isolados: soleira direta e passagem sob a cama.

Os relatórios ficam em `results/mobility_report.json`, `results/battery_report.json`, `results/threshold_report.json` e `results/bed_clearance_report.json`. Cada gravação de telemetria cria `run_*.jsonl`, `run_*_summary.json` e, quando `/map` e `/coverage/grid` coincidem, `run_*_coverage.json` com metadados e os dados que permitem refazer as contagens. Valores ausentes permanecem `null` com disponibilidade declarada. Proximidade LiDAR é um proxy de colisão; não há sensor de contato físico. O docking do supervisor também é uma zona virtual na pose inicial do robô, sem alinhamento ou contato físico com a base SDF.

### Checklist visual no WSL2

1. Rode `./scripts/run_mvp.sh` e confirme que o mundo `casa` aparece no Gazebo com o robô sobre a base verde.
2. No RViz, confirme os frames `map`, `odom` e `base_link`, o mapa e `/scan` atualizando, e a grade `Coverage` em `/coverage/grid`.
3. Em outro terminal, confirme telemetria e estado com `ros2 topic echo /battery_state --once` e `ros2 topic echo /robot/state --once`; confira também `/coverage/percent`.
4. Confirme que a pose do robô acompanha o movimento no RViz e que não há erros de TF, plugin Gazebo ou display nos terminais.

Se o display não iniciar, confira WSLg (`echo "$WAYLAND_DISPLAY $DISPLAY"`) ou o endereço/firewall do X server. Para diagnosticar os tópicos sem janelas use `./scripts/run_mvp.sh headless:=true`. A verificação headless não substitui a inspeção visual; não há sensor de contato físico e a visualização da bateria/estado é feita pelos tópicos ROS.

## Pacotes e interfaces

- `robot_description`: SDF físico, URDF/Xacro de frames e casa de teste.
- `robot_simulation`: Gazebo, bridge, EKF e lançamento do MVP.
- `robot_navigation`: SLAM Toolbox, AMCL, Nav2 e configurações do `robot_localization`.
- `robot_autonomy`: exploração, cobertura, estados, bateria e docking simplificado.
- `robot_metrics`: telemetria JSONL.
- `robot_tests`: cenários automatizados e relatórios JSON.

Interfaces principais: `/cmd_vel`, `/scan`, `/imu/data`, `/odometry/filtered`, `/battery_state`, `/coverage/grid`, `/coverage/percent`, `/robot/state` e `/robot/event`. Os detalhes de geometria e mensagens estão em [docs/physics-and-interfaces.md](docs/physics-and-interfaces.md).

## Validação atual

O workspace compilou com ROS 2 Jazzy no WSL e os seis pacotes do workspace. Nesta instalação, Gazebo Harmonic publica o relógio em `/world/casa/clock`; a ponte o encaminha para `/clock`, usado por EKF, SLAM e Nav2.

Em 30/09/2026, os ensaios físicos dedicados passaram: a soleira de 12 mm foi atravessada em linha reta e o robô parou assentado depois dela; o percurso sob a cama passou com folga mínima calculada de 9 cm. O teste de ciclo da bateria passou por `EXPLORING → RETURNING_TO_DOCK → DOCKING → CHARGING → EXPLORING`, confirmou aumento de carga e retomada. O docking continua sendo uma zona virtual; estes resultados não validam contato com uma base física.

O executor de mobilidade inicia um simulador limpo por cenário, permite selecionar um cenário isolado e grava poses, meta, rota planejada e resultado, distinguindo meta inválida, timeout e falha Nav2. Os ensaios isolados de `piso_normal`, `tapete`, `soleira_12mm`, `porta_corredor`, `cadeira` e `canto` passaram; os controles de ré, giro e avanço também passaram com warmup de 20 s. `obstaculo_frontal` foi validado em três execuções isoladas consecutivas com `warmup_seconds:=0.0`: Nav2 chegou à meta sem recuperação, acionou as zonas SlowZone esperadas e a trajetória medida manteve pelo menos 31 cm entre o centro do robô e a caixa do obstáculo (aprox. 7 cm além do raio Nav2 configurado de 24 cm). Os relatórios estão em `results/issue1/obstaculo_frontal_spawn_start_pass_20261001.json`, `results/issue1/obstaculo_frontal_spawn_start_pass_before_repeat_20261001.json` e `results/issue1/obstaculo_frontal_spawn_start_pass_final_20261001.json`. O cenário inclui um waypoint de preparação, e o runner só aceita o setup quando Nav2 informa sucesso e a pose/rotação respeitam a tolerância. O warmup autônomo padrão de 120 s não foi validado: sob concorrência com outro Gazebo, uma tentativa travou no percurso e outra excedeu o prazo ao aguardar o servidor `compute_path_to_pose`. Essas execuções são preservadas como falhas, não ocultadas pelos três passes isolados. O relatório conserva poses, distância restante, estado e código Nav2 quando disponíveis; timeout não é classificado como falha mecânica e a simulação não confirma contato físico. O teste da soleira nesta suíte não substitui o ensaio direto dedicado.

A exploração autônoma percorreu as três regiões separadas pelas paredes da casa: os registros de pose incluíram amostras na sala oeste e nas duas salas a leste. A execução isolada mais recente registrou 1.483 de 17.477 células livres visitadas (8,49%) ao atingir o limite de 360 s simulado; a exploração permaneceu em `COVERING` e não completou a limpeza integral. `results/issue5/run_20260930T235330_790796Z_summary.json` e o snapshot associado preservam contagens, grade e eventos. Contato físico e contagens de recuperação Nav2 são `null` quando não observáveis. O teste de cama planeja sobre um mapa estático de fixture com pernas predefinidas; ele valida rota e folga física, não percepção online das pernas.

O lançamento gráfico foi iniciado no WSLg com Gazebo GUI e RViz; o checklist visual acima ainda requer conferência manual dos painéis e frames. Um contexto GLX no display atual foi aberto com `LIBGL_ALWAYS_SOFTWARE=1` e reportou `llvmpipe` do Mesa, comprovando o caminho de renderização por software sem usar a GPU NVIDIA. O teste não substitui a inspeção visual do lançamento completo do Gazebo/RViz, que ainda precisa ser conferida na janela.
