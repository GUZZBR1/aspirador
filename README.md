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

Os relatórios ficam em `results/mobility_report.json`, `results/battery_report.json`, `results/threshold_report.json` e `results/bed_clearance_report.json`. Cada gravação de telemetria cria `run_*.jsonl`, `run_*_summary.json` e, quando `/map` e `/coverage/grid` coincidem, `run_*_coverage.json` com metadados e os dados que permitem refazer as contagens. Valores ausentes permanecem `null` com disponibilidade declarada. Proximidade LiDAR é um proxy de colisão; não há sensor de contato físico. A zona lógica de docking usa a pose inicial, agora alinhada ao centro da base SDF; a simulação ainda não mede contato elétrico ou mecânico de carga.

### Checklist visual no WSL2

1. Rode `./scripts/run_mvp.sh` e confirme que o mundo `casa` aparece no Gazebo com o robô centralizado sobre a base verde.
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


O executor de mobilidade inicia Gazebo, SLAM e Nav2 novos por cenário e grava poses map/world/odom, meta, rota, feedback e classificação de falha. Ele distingue `invalid_goal`, `nav2_failure`, `timeout` e falha no setup; sucesso Nav2 fora da tolerância estrita de 25 cm/0,35 rad pode repetir o setup até duas vezes, registrando cada tentativa sem aprovar pose fora do limite. O script executa os controles sem aquecimento (para manter o robô próximo ao spawn), usa 80 s para mapear o percurso da soleira e 40 s nos outros cenários.

Na matriz headless completa de 01/10/2026, 10 de 11 verificações passaram: os três controles e sete dos oito cenários de mobilidade. `canto` atingiu o timeout de inicialização de 120 s sem mapa/Nav2 pronto. Uma execução isolada anterior passou, mas tanto a matriz quanto a repetição isolada mais recente expiraram; o problema de inicialização intermitente permanece aberto. `tentativa_bloqueada_recovery` foi aprovada pelo timeout esperado contra célula ocupada (`100`), com 186 poses planejadas, 4,63 m restantes e zero recoveries; o ensaio não conclui falha mecânica. `obstaculo_frontal` passou, com rota medida mantendo pelo menos 31 cm entre o centro do robô e a caixa do obstáculo (aprox. 7 cm além do raio Nav2 de 24 cm). A matriz preserva resultados individuais em `results/mobility_cases/`; arquivos de `results/` são artefatos locais ignorados pelo Git.

Os ensaios físicos dedicados passaram: a soleira de 12 mm foi atravessada em linha reta, com elevação máxima de 13,5 mm, e o robô parou assentado após 0,998 m. O percurso sob a cama passou com folga nominal calculada de 9 cm sobre fixture com mapa conhecido das pernas. O ciclo de bateria passou em 58,7 s e aumentou a carga de 94,83% para 97,08%, retomando exploração. A zona lógica de docking representa proximidade virtual e não confirma contato elétrico ou mecânico. Relatórios preservam medições indisponíveis como `null`; proximidade LiDAR não é tratada como contato.

A execução isolada de 01/10/2026 percorreu as três regiões SDF e emitiu `coverage_incomplete` ao encerrar a exploração: visitou 1.421 de 17.533 células livres (8,10%), com amostras de pose e células visitadas nas salas oeste, leste-norte e leste-sul; o grid completo não foi coberto. Não houve eventos de robô preso, recoveries ou falhas de navegação nessa execução. O runner `ros2 run robot_tests coverage_runner --artifact-dir results/issue5 --wall-timeout-seconds 3600` inicia SLAM sem mapa estático, aguarda um evento terminal e salva metadados, snapshot, JSONL e grade para reproduzir as contagens. O validador exige mapa dinâmico em branco, consistência do grid e evidência nomeada das três regiões; uma execução com `coverage_incomplete` é reportada como cobertura parcial, nunca como limpeza completa. Contato físico e contagens de recovery Nav2 são `null` quando não observáveis. O teste de cama planeja sobre um mapa estático de fixture com pernas predefinidas; ele valida rota e folga física, não percepção online das pernas.

O lançamento gráfico foi iniciado no WSLg com Gazebo GUI e RViz; o checklist visual acima ainda requer conferência manual dos painéis e frames. Um contexto GLX no display atual foi aberto com `LIBGL_ALWAYS_SOFTWARE=1` e reportou `llvmpipe` do Mesa, comprovando o caminho de renderização por software sem usar a GPU NVIDIA. O teste não substitui a inspeção visual do lançamento completo do Gazebo/RViz, que ainda precisa ser conferida na janela.
