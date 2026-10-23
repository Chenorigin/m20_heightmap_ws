# M20Pro CPU 高程观测工作区

GitHub 仓库包含源码、配置、模型与测试；`evidence/` 历史实测数据、`.scratch/`、构建产物及运行日志仅保留在本地。下文指向 `evidence/` 的链接需在本地查看。

> **2026-10-17 R5 已部署并完成一轮移动核验。** 按用户澄清，地形原始输入按已标定前向扇形裁剪，保留扇形内全部原始点记录，独立定位仍接完整扫描。本轮已记录的 729 个前进策略输入全部含有效地形，0 个整图未知，其中 625 个含约一阶的高差提示；原始采集交集内的 631 个前进输入有独立位姿移动证据。该次结果不等于每周期都有阶高，也没有验收提前抬腿或楼梯成功率。现场仍站立，按用户要求保持 SDK 运行。见[本轮记录](evidence/terrain_dynamic_fix_20261017/report.md)及[参数台账](docs/DEPLOYMENT_PARAMETERS.md)。下方为此前实测结果。

> **2026-10-17 移动实测：地形输入仍不稳定。** 现场短距前进、LIO估计水平位移约24cm；133个前进推理周期中122个全未知，其余11个也未呈现15cm级阶高提示。118个未知周期在上游地形包中已未知，4个为单格正确超龄；SDK拼接/加龄逐值核对通过。同期原始点云离线可恢复部分有效地形，但在线更新时效不足，不能以静态可见或离线可恢复当作移动验收。用户趴下后本轮SDK及感知均已停止，原始数据保留，模型及生产门槛未改。见[动态核验报告](evidence/terrain_dynamic_validation_20261017/report.md)。下方静态结果与旧楼梯试验均为各自场景的历史证据。

> **2026-10-17 高程未知修复：** V6 感知修复已装到新狗，修正 IMU 历史积分、扫描结束边界、去畸变端点及回调发布顺序；实际加载的节点与库已核验。趴姿纯感知有效格中位7个；现场站稳、距首阶约30cm后，20秒采样收到972帧地形、无全未知帧，有效格20–34个、中位30，原始点可辨约15cm踏面高差。该姿态对照不能当软件增益；该次静态采样未验证动态地图稳定性及提前抬腿。SDK及本轮感知均已停止，模型、MuJoCo对齐限制和地图门槛未改；旧成品保留。见[修复与验收记录](evidence/terrain_unknown_fix_20261017/report.md)。下方楼梯试验结论是修复前的历史状态。

> **2026-10-17：MuJoCo限制与student-v3 10500已部署，楼梯试验结束，SDK及本轮感知已停止。** 现场反馈15cm高/30cm深木楼梯五六次成功一次，失败表现为撞阶后抬腿。封存日志确认前进周期约78%高程全未知、位姿异常反复清图；同学生的干净图单次MuJoCo对照也未证明稳定提前抬腿。当前不能将全部失败归为单一原因，未在排查中改动生产参数。见[楼梯复盘](evidence/student_v3_20261017/stairs_diagnosis/diagnosis.md)与[本轮记录](evidence/student_v3_20261017/summary.json)。原ONNX数值失败、原门槛和用户启动例外均保留。日志有显式丢弃，不能称整场无损。下方10月16日状态为历史记录。

> **2026-10-16轮速调整已上机试验，SDK已停止：** 新狗两板入口为`20261016_trial_d136c514135c`。关闭地形HOLD后的实机试验没有触发HOLD，推摇杆后仍倒地；日志确认姿态超限先于进入阻尼，不能归因为地形HOLD。用户批准仅提高轮目标速度上限，已写入本地及两板当前配置并重绑凭据，已完成短时零速运行，控制周期HOLD提前结束本次试验，轮速行走效果未验证。初始化状态凭证更新已在两板构建通过，最终状态见[迁移与启动记录](evidence/migration_20261016/summary.json)。

本工作区实现 **原厂融合点云 → GOS 本机原生 FastDDS 读取 → CPU 高程估计 → DDS 地形小消息到 AOS → 学生 SDK 内拼接观测并推理**。输出187点高度、有效性、置信度；地形核心使用NumPy，独立Super-LIO提供位姿，AOS使用CPU ONNX。本轮使用student-v3 10500权重，序列化及部署profile沿用v1的789维ABI；原v1模型已备份。用户已确认新狗稳定支撑、实体急停值守、SDK模式及原厂安装并授权启动；运动按键和摇杆由现场用户操作。

[完整操作步骤](docs/REAL_ROBOT_STEPS.md)、[部署参数唯一台账](docs/DEPLOYMENT_PARAMETERS.md)及[算法说明](docs/ALGORITHM.md)。当前通过本机网卡直连新狗两板，不使用旧狗的 NVIDIA 跳板；实际连接与版本以[迁移记录](evidence/migration_20261016/summary.json)为准。10月15日的[故障复查](docs/FALL_REAUDIT_20261015.md)、旧狗10月16日的[修复记录](docs/TERRAIN_OFFLINE_FIX_20261016.md)和[真实可视化](evidence/native_dds_20261015/aos_trial_500ms.html)仅作历史依据，不代表新狗已通过策略控制。

本轮以两个独立开关关闭覆盖率HOLD和地形时效HOLD；超龄或接收超时的地形使用全未知输入，单格仍按真实年龄清零，不把缺测伪装成有效地面。未来/非有限时间、契约、本体状态、所有权及其他故障保护保留。开关、轮速调整依据和预算只见[参数台账](docs/DEPLOYMENT_PARAMETERS.md#v1-准入与发布预算10-月-16-日在线修订)。默认strict入口保留；显式supported-init在关闭地形时效HOLD时，须由新鲜GOS状态及唯一同participant端点证明来源，才能把地形统计不足列为告警，不能据此宣称策略质量通过。

项目文档现保留六份：操作步骤、算法说明、部署参数、原生 DDS、故障复查与实机复验、最新离线修复。旧交付/候选/分步报告已清理，所需结论合并到对应现行文档。`evidence/` 保留实机原始数据和关键修复证据；历史回执中的临时路径可能已随本次清理删除。

旧狗10月16日三级木楼梯真实数据：独立参考层高约14–15cm，首级踏面生产重放与原始点支持比较的高度偏差p95约5.8mm；存在单帧覆盖波动，没有上下楼动态样本。见[数据汇总](evidence/stairs_reference_20261016/summary.json)、[对照图](evidence/stairs_reference_20261016/stairs_reference.png)及[分析边界](docs/TERRAIN_OFFLINE_FIX_20261016.md)。该次为只读监听，采集器已退出；其结果不能代替新狗验证。

## 实时点云与高程调试（只读）

桌面入口为 `scripts/view_lidar_debug.py`，需要本机 Python 3、NumPy 和 PyQt5。它通过 SSH 在 **GOS** 临时执行 `scripts/stream_lidar_debug.py`，读取正在运行的感知话题；不写板端脚本、不启动感知或 SDK。左右分别显示完整/前向点云与高程观测，可切换高度、有效性、置信度、年龄、方差，悬停查看数值，保存 JSON 和截图。原厂融合云虽标 `lidar_link`，其点已经转换到机体系；高程保持 `base_yaw` 的重力对齐轴。两栏没有做时空配准叠加。

显示工具栏增加“原点：机身中心／前方雷达”。前雷达位置来自当前 runtime 引用的 `site.json`，没有另存一套安装数值。切换会同步平移点云、高程格坐标、原点标记与悬停读数；高程图按查询时刻的位姿把安装杆臂转换到重力对齐坐标，保留俯仰/侧倾影响。高程格不重采样，生产消息、模型 h 和策略输入保持原定义。“相对原点地面高度 Z”由未裁剪的 h 换算，未知/裁剪格不显示成真实高度。缺少安装参数或对应时刻位姿时明确提示，不猜测前雷达位置；旧 JSONL 缺少这类元数据时仍可用机身原点回放。启动可加 `--origin front` 预选前雷达，截图 JSON 会记录当前显示原点。

2026-10-23 已通过 1515 的 AOS 跳板连上 GOS，核对两板序列号后启动有时限的既有纯感知链，桌面界面已接收真实完整点云、前向点云和 schema 2 高程。当前运行版本仍取板端 `current`，没有覆盖感知源码、参数或启动 SDK。验收窗口和实际资源读数见[本轮实测](../rl_training/logs/evaluation/m20_lidar_debug_20261023/live_capture_summary.json)与[验证记录](../rl_training/logs/evaluation/m20_lidar_debug_20261023/verification.json)。初始 SSH 密钥失败和金库拒绝不代表板卡不可用；本轮由用户直接提供会话凭据后成功。请用已核验的 GOS SSH 别名与跳板，不能将网关直接当作 GOS；采集器会核验板卡角色，拒绝在 AOS/NOS 上执行。

```bash
cd /home/ubuntu/cyq_shixi_projects/m20_heightmap_ws
# GOS_ALIAS 与 GATEWAY_ALIAS 替换为本次核对过的 SSH 入口。
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /usr/bin/python3 scripts/view_lidar_debug.py \
  --ssh-target GOS_ALIAS --jump GATEWAY_ALIAS \
  --output ../rl_training/logs/evaluation/m20_lidar_debug/snapshots
```

支持直接 SSH 时省略 `--jump`。SSH 保持主机密钥核验，首次连接需先用正常 SSH 核验并信任主机。密码连接需要本机 Paramiko；每一跳独立认证并核验已知主机密钥，避免嵌套 sshpass 争用密码提示。若两跳共用金库中的板端凭据，可先执行 `eval "$(getsecret M20_NEW_BOARD_PW)"`，再给上述命令加 `--password-env M20_NEW_BOARD_PW`；变量为空会拒绝，不将密码写入参数或文件。只读工具不会绕过金库的发放限制。用户直接授权的会话凭据也可通过 `read -rs M20_SESSION_PW; export M20_SESSION_PW` 输入，再指定 `--password-env M20_SESSION_PW`，不要将密码写入脚本或命令参数。

板端按运行配置推算缓冲和 Python/ROS 内存预算，读取整机可用内存及 load 后再订阅；使用单执行器、NumPy 单线程、低调度优先级和有上限的会话，显示抽样不改变生产点云。显示频率、点数、时长均可用 `--hz`、`--max-points`、`--seconds` 在脚本限制内设置。当前输出只供人工调试，不是 SDK 的实时性验收工具。默认显示每次采样时的高程快照，并明确标注“采样时有效”；断流后清空过期显示。勾选“严格实时效期”会进一步计入显示等待，低显示采样率下可用格可能在下次刷新前归零；这不是生产 mapper 停止更新的证据。“源数据有效”始终保留来源记录中的统计，旁路结果不冒称板端发布。

灰色问号代表未知或过期；未知格的 `age=-1` 是合法发送端哨兵。工具按生产 schema 2 解码，使用 GOS 的 ROS 时间计算源帧年龄，再以本机 monotonic 时间累计显示等待，不比较两机 wall clock。界面区分采样快照与严格实时效期，断流不会继续把旧地形标成当前有效；完整点云和前向点云仅为显示抽样，保留各自 frame、原始点数和 header 时间。配置/ROS 缺失、话题无数据、协议错误都会显示出来，不自动恢复或更改原感知服务。

历史 JSONL 可通过 `--replay FILE --cloud front` 查看，界面始终标为回放，并按记录采集时的年龄显示；不能用回放证明当前真机已接通。默认不连续写盘；“保存当前数据和截图”记录的是显示抽样与一帧高程，不能称为无损原始录制。关闭窗口会停止本次 SSH 采集。既有纯感知链是独立、有上限的调试会话，关闭查看器不会启动或停止机器人控制服务；2026-10-23 本次在线对照的临时感知进程已主动停止。

来源选择支持“当前数据源／原版旁路／候选旁路（未接入模型）／板端实际发布”；实验记录可明确提供各来源名称，例如 Patchwork++ 分割候选。仅显示记录中明确携带的来源，缺失时不会回退到其它来源。截图 JSON 同时记录来源、原点和图层；格子与悬停共用断流判据。2026-10-23 的内部边界余量实验没有在近静态窗口增加有效格，且影响了去畸变分桶的代表时间，候选未部署；完整结果和适用边界见[实验报告](../rl_training/logs/evaluation/m20_heightmap_margin_online_20261023/report.md)。回放本轮数据：

```bash
python3 scripts/view_lidar_debug.py \
  --replay ../rl_training/logs/evaluation/m20_heightmap_margin_online_20261023/live_baseline.jsonl \
  --terrain-source baseline --cloud front --origin front
```

2026-10-23 后续离线研究同时比较了点云分割与原 187 个位置上的高程提取。Patchwork++ 的有效格中位数为 4→7，但有原有效格损失且缺少高度真值；带实测支撑约束的局部平面查询为 4→1，因此候选均未接入模型。结果、反例、论文来源与同帧截图见[研究报告](../rl_training/logs/evaluation/m20_surface_research_20261023/report.md)。实验代码仅保存在 `experiment/surface-query-20261023` 分支。回放候选时可在来源栏切换原版：

```bash
python3 scripts/view_lidar_debug.py \
  --replay ../rl_training/logs/evaluation/m20_surface_research_20261023/segmentation_export/segmentation_display.jsonl \
  --terrain-source candidate --cloud front --origin front
```

## 1. 最容易接错的模型契约

| 模型 | 实际输入顺序 | 维数 |
|---|---|---:|
| v0 | 当前本体 57 → 高度 187 → 有效性 187 → 置信度 187 | 618 |
| v1，4 帧本体 | 当前本体 57 → 高度 187 → 有效性 187 → 置信度 187 → 过去本体 3×57 | 789 |

**不能把 57×4 先连在一起再接地形。** v1 的三个历史帧位于 `[618:789]`，依次是 `t−3, t−2, t−1`。重置后零填充，历史只包含过去的本体观测，地形没有在输入中堆叠历史。

v1 源码的默认 `history_steps` 其实是 1；用户描述的版本是训练时显式设置为 4 的版本。本工作区 v1 模板按 4 配置，AOS 启动必须读取**最终权重附带的** `observation_contract.json` 校验，不能仅凭文件名猜版本。

| 字段 | 定义 |
|---|---|
| 网格 | 11 行 Y × 17 列 X；间距 0.1 m；X ∈ [−0.8, 0.8]，Y ∈ [−0.5, 0.5] |
| 排列 | `i = iy * 17 + ix`，X 在内层递增，Y 在外层递增 |
| 坐标 | 重力对齐 Z，机器人 yaw 对齐 X/Y，原点在训练使用的 base_link |
| 模型高度 | `h = clip(base_z − terrain_z − 0.5, −1, 1)`；不是绝对海拔，也不是直接 terrain_z |
| 未知 | `h = validity = confidence = 0`，不能填成可信平地 |
| 有效性 | 有真实几何支持、不过期、没有被当前歧义观测推翻的格为 1，否则 0 |
| 置信度 | `m * exp(−age/tau) / (1 + variance/reference_std²)`；是质量分数，不是已标定概率 |

例如 base 离平地 0.6 m 时高度通道为 **+0.1**，脚下地面升高 0.15 m 时应变为 **−0.05**。可用这一正负号检查防止实机接反。

权威依据和源码哈希见 [训练对齐记录](evidence/training_audit.json)；噪声差异及数学模型见 [算法说明](docs/ALGORITHM.md)。通用训练基线参数见 `src/m20_heightmap/config/*.json`，实机试运行覆盖值以 `deploy/heightmap_v*.json` 与板端 `runtime/heightmap_v*.local.json` 为准，以上网格与切片是不可随意修改的模型 ABI。

## 2. 目录与数据流

```text
原厂雷达融合 -- GOS本机 /LIDAR/POINTS --> 原生DDS读取端
原生DDS读取端 -- 私有本机socket --> gos_native_inputs
gos_native_inputs -- GOS本机PointCloud2 --> Super-LIO / gos_mapper
Super-LIO -- base_pose_adapter --> gos_mapper
gos_mapper -- TerrainObservation小消息 --> AOS学生SDK runner
AOS SDK本体快照 + 地形 + 本体历史 --> 789维输入 --> CPU ONNX
```

新增原始大点云不跨板；AOS另接收IMU、位姿、状态及点云小回执。`aos_adapter`提供可选的`Proprioception → PolicyObservation`外接接口，不在当前学生SDK的推理链中。

- `src/m20_heightmap/m20_heightmap/`：估计器、坐标变换、输入校验、ROS 节点、观测拼接。
- `src/m20_heightmap_msgs/msg/`：两板必须安装的三个消息定义，支持 Python/C++ 消费。
- `src/m20_native_inputs/`、`src/m20_student_sdk/`：原厂ABI原生读取端、学生SDK runner及原有状态机。
- `src/m20_heightmap/config/v0.json`、`v1.json`：配置模板。
- `scripts/`：构建、启动、测试、离线回放、CPU 基准和权重检查。
- `docs/`、`evidence/`：数学假设、验收方法及本轮实测证据。

## 3. 构建（AOS、GOS 分别执行）

要求Python ≥3.8、匹配的ROS 2与NumPy；当前新狗已分别完成ARM/Foxy构建，AOS四包、GOS三包成功，本地离线环境为Humble。外部SDK、ARM ONNX Runtime、原生DDS和LIO依赖也须与板端匹配。**不要把本机x86的install/build拷到ARM板上使用**。

先查看 `free -m`、`uptime`，确定板端有余量再构建/启动。一个估计 worker、NumPy 单线程；单帧上限与队列容量由配置限制。板端 CPU 余量不足时，应先按实际点数运行基准，不能由桌面结果推断达标。

```bash
cd /home/user/m20_heightmap/current
bash scripts/build_board.sh  # 仅在需要重建对应版本时执行
source install/setup.bash
```

不自动安装系统包、不改系统服务、不复制旧项目的大量二进制。更新已有部署时先核对现场修改；建议每次放到一个新的版本目录，保留旧版本供回滚。

## 4. 填配置：必须确定的现场量

当前实机使用两板各自的`runtime/heightmap_v1.local.json`；部署模板来自`deploy/heightmap_v1.json`，两板使用相同的地形质量参数、profile、history_steps和消息话题。模板修改不会覆盖已有runtime，通用`src/m20_heightmap/config/`示例也不代表现场参数。新现场配置准备与确认流程见[操作步骤](docs/REAL_ROBOT_STEPS.md)。

| 配置项 | 应填写的内容 |
|---|---|
| `topics.cloud` | GOS本机输入节点发布的`/m20/gos/points`；源为原厂融合`/LIDAR/POINTS`的原生读取，不是AOS大云中继 |
| `topics.odom` / `topics.imu` | 现场位姿/姿态话题，默认名称仅占位 |
| `topics.terrain/proprio/policy/status` | 你决定的观测接口话题，两板保持一致 |
| `frames.world/base/imu/output` | 实际 frame 名；`base` 必须是训练中定义的机体原点；world 的 Z 必须沿重力反方向 |
| `lidar_extrinsics` | 键必须等于 PointCloud2 的 `header.frame_id`；值为 LiDAR→base 的平移和 xyzw 四元数 |
| `imu_to_base_xyzw` | IMU→base 的旋转；只在 IMU 模式下使用 |
| `self_filter_boxes` | 机体坐标中的自体排除盒 `[[min_x,min_y,min_z],[max_x,max_y,max_z]]`；来自实测/模型，不能挡掉地面 |
| `runtime.pose_std` | 位姿未知协方差时采用的正值下限，顺序 x,y,z,roll,pitch,yaw，单位 m/rad |
| `runtime.point_time_field` | 逐点时间字段名；没有就空字符串。不能猜 timestamp 的含义 |
| `runtime.point_time_mode` / `runtime.point_time_scale_s` | 声明绝对/相对逐点时间及单位比例；当前原生链保留绝对`timestamp`，生产管线按配置转换，不能改写点的采集时间 |
| `runtime.scan_stamp_reference` | 无逐点时间时，header 是扫描开始 `start` 还是结束 `end`；默认保守按 `end`，年龄从 header−scan_duration 算 |
| `runtime.scan_duration_s` | 实测扫描时长上界，影响年龄和未去畸变误差估计 |
| `aos.model_contract` | 最终权重旁边的 `observation_contract.json` 绝对路径，AOS 必填 |
| `aos.model_path` | 配套模型路径；当前SDK启动入口使用`models/v1/policy.onnx`，GOS mapper和可选Python adapter不加载模型 |
| `calibration_confirmed` | 完成上述坐标、时间、自体过滤校核后才设为 `true`；默认 false 会明确拒绝启动 |

LiDAR 外参满足列向量公式 `p_B = R_BL p_L + t_BL`；ROS 四元数一律 `x,y,z,w`，训练内部有 `w,x,y,z`，两者不能混用。**参考 baseline.yaml 的 lidar_imu 是 LiDAR/IMU 关系，不是已验证的 LiDAR/base 关系；训练的前后雷达 ±0.10 m/0.15 m 也只是合成感知假设，不能直接充当真机外参。**

一个输入话题可使用多种已配置的 cloud frame，适合驱动融合后的原始流。当前按扫描时间严格递增消费；两个独立雷达若各自发布相同时间戳，应先在驱动侧合并成统一 frame/扫描，或在上游完成明确的同步聚合，不要直接混发布。

### 位姿有两种模式

**推荐 `pose_mode="odometry"`**：输入 `nav_msgs/Odometry`，表示重力对齐世界系中的训练 base_link 位姿。必须是点云采集时的同步位姿；可以来自已有可靠 LIO/融合里程计，原始点云仍由本程序直接处理。没有逐点时间时按整帧变换并降低置信度；有逐点时间时按时间插值去畸变。逐点时间对应的位姿未到会等待，超时拒绝，不外推。

**`pose_mode="imu"`**：输入含有效 orientation 的 `sensor_msgs/Imu`。只有加速度/角速度而 orientation_covariance[0]=−1 的原始 IMU 不够用。此模式利用姿态得到当前扫描相对机体的重力对齐高度，不需要绝对 base_z；无法恢复平移、无法对齐历史扫描。因此关闭地图跨帧保留，限制可复用扫描年龄，平移畸变按配置运动上界计入误差。其完整性与置信度可能显著低于里程计模式，应先做静态点云检查；**不能称为具备 v1 的完整历史地图能力**。

如果原始点云已经是 world 注册点云，不要仅修改 frame 字符串冒充雷达原始坐标。应使用正确数据源或另写明确的坐标适配。

## 5. 板间网络与启动

沿用现场正常的 ROS 2 DDS 网络配置，两板设置相同 `ROS_DOMAIN_ID`。跨板通信时 `ROS_LOCALHOST_ONLY=0`。不要照抄旧工程 `direct_udp.xml` 里的固定网卡地址，也不要照抄其私有 domain 87。

```bash
export ROS_DOMAIN_ID=0       # 按现场权威域配置
export ROS_LOCALHOST_ONLY=0
ros2 topic info /m20/terrain/observation -v
ros2 topic info /你实际的位姿话题 -v
```

本机原始点云使用 reliable、volatile、有限队列，匹配实际 LIO 的 reliable 订阅；位姿订阅与地形输出使用 best-effort。外部消费者如果用 reliable 订阅本程序的 best-effort 地形输出，会不兼容；请按示例的 `sensor_qos()`/`rclcpp::SensorDataQoS()` 接入。原生输入链路与收件证明见 [原生 DDS 说明](docs/NATIVE_DDS_20261015.md)。必须检查真实 payload、header 时间与 frame，看到话题名不等于收到了数据。QoS 的兼容性依据见 [ROS 官方 REP-2003 讨论](https://discourse.ros.org/t/new-rep-2003-sensor-data-and-map-qos-settings/35758)。

两板的 ROS 时间必须同步（PTP/chrony 或现场已验证方案）；程序检查未来包、过旧包及单调时间，**不会靠扩大延迟阈值掩盖时钟偏差**。真实时钟发生回拨需重新建立输入流；回放应使用隔离域和正确的 `/clock`。

地形HOLD关闭后已有实机运行；本次轮速调整已应用，状态凭证相关Python包两板构建和实际预检通过，SDK已完成短时零速运行，因控制周期HOLD结束后停止；轮速行走效果未验证。最终进程状态见[迁移记录](evidence/migration_20261016/summary.json)。以下只列生产入口，操作条件见[步骤说明](docs/REAL_ROBOT_STEPS.md)。GOS纯感知：

```bash
sudo -E bash scripts/operator_start_gos.sh v1
```

AOS在板端tmux内启动，默认strict预检：

```bash
sudo -E bash scripts/operator_start_aos.sh v1
```

本轮显式`supported-init`入口和告警范围见[操作步骤](docs/REAL_ROBOT_STEPS.md#6-sdk-启动入口与当前操作)。SDK启动会发送初始化关节命令；GOS终端Ctrl+C只停止其感知进程组。没有自动起服、自动接管或自动重启运控行为。GOS只保留最新待处理扫描和一个后台计算任务，算不过来时丢掉中间扫描，年龄与缺测在输出中体现。

## 6. AOS 运控接线

生产SDK runner直接订阅`TerrainObservation`，从同一SDK本体快照拼接当前观测与历史，再执行ONNX；不经Python `aos_adapter`。本轮关闭覆盖及地形时效HOLD，地形缺失或超龄降为未知后继续推理；单格老化、非法时间、消息契约、本体时效、控制周期及所有权保护保留。未知输入不会伪装成可信平地；已有实机试验仍发生失稳，本次轮速调整尚未验证能否改善。参数见[唯一台账](docs/DEPLOYMENT_PARAMETERS.md)。

下述`Proprioception`/`PolicyObservation`为可选外接接口：控制循环每个实际策略周期发送一次`Proprioception`；header使用本体测量时间，`step`递增，控制器重启或重新进入策略时更换session。历史按策略周期维护，不按雷达频率维护。

当前 57 维依次为：

| 切片 | 内容 |
|---|---|
| `[0:3]` | 机体系角速度，clip(±100) 后乘 0.25 |
| `[3:6]` | 单位投影重力 `R_WBᵀ [0,0,−1]` |
| `[6:9]` | vx、vy、wz 命令 |
| `[9:25]` | 16 关节相对训练默认位姿；最后四个轮子的位置必须置 0 |
| `[25:41]` | 16 关节速度，clip(±100) 后乘 0.05 |
| `[41:57]` | 上一次策略**下发的归一化动作**，clip(±100)；不是测得关节位置，也不是经过 PD/延迟后的物理指令 |

关节顺序见 `contract.JOINT_NAMES`：FL/FR/HL/HR 各 hipx、hipy、knee，然后 FL/FR/HL/HR 四个 wheel。默认关节位姿由最终训练任务/权重配套配置提供，本工作区不猜测。

Python 可调用 `make_proprio(...)` 得到已缩放的 57 维，也可直接调用 `ObservationPacker` 在现有推理进程内拼接。Python ROS 2 发布/订阅示例：

```python
from m20_heightmap_msgs.msg import Proprioception, PolicyObservation
from m20_heightmap.ros_nodes import sensor_qos
from m20_heightmap.contract import make_proprio

pub = node.create_publisher(Proprioception, configured_proprio_topic, sensor_qos())
# 在现有 50Hz 循环中构造 msg：header、session_id、step 由该循环管理。
msg = Proprioception()
msg.header.stamp = measured_tick_stamp
msg.header.frame_id = configured_base_frame
msg.session_id, msg.step = controller_session, policy_step
msg.values = make_proprio(gyro_body, projected_gravity_body, velocity_command,
                         joint_q_in_training_order, joint_dq_in_training_order,
                         trained_default_q, previous_issued_action).tolist()
pub.publish(msg)

# 接收回调应将最新完整 PolicyObservation 放入有界缓存。
# 推理前重新验证 session、step、header 时效和维数；同一 step 只能执行一次。
sub = node.create_subscription(PolicyObservation, configured_policy_topic,
                               cache_latest_observation, sensor_qos())
```

C++ 消费者直接包含 `m20_heightmap_msgs/msg/policy_observation.hpp`，将 `values` 连续 float32 数据交给现有推理器；不要再次打乱或追加历史。自定义消息需要在两板各自编译并 source。

`aos_adapter` 在地形无效/过期、维度不符、时间跳变、缺失 tick 时停止发布 PolicyObservation 并清本体历史，重复包拒收不清防重放水位。**现有运控程序必须在没有收到本 tick 的新观测时走其已有安全退出/等待路径，不能无限使用上一次动作或观测。**本包不替代生产控制器的超时保护。

地形消息还携带 variance、age、session、sequence 和 query 时间，它们是验证元数据，**不进入模型**。AOS 接收时再增加传输年龄并降低置信度。GOS依据最新位姿重采样地图；GOS→AOS的短延迟不在AOS二次重投影。超过`max_alignment_age_s`的处理由地形时效开关决定，当前配置降为全未知；需结合实际速度和链路延迟验证这个界限。

### 写入权重后的 CPU 检查

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 scripts/check_model.py \
  --model /实际路径/student_v1.pt \
  --contract /实际路径/observation_contract.json --profile v1 --history-steps 4
```

这是 TorchScript 导出文件；原始训练 checkpoint 请先用训练工程导出。ONNX 文件使用同一命令并安装 CPU `onnxruntime`。脚本实际在 CPU 输入 `[1,789]`（v0 为 `[1,618]`），要求输出有限 `[1,16]`，打印 SHA256。可用 `--observation obs.npy` 检查录下的真实输入；它不证明实机闭环可用。

## 7. 离线验证与回放

```bash
bash scripts/test.sh
python3 scripts/audit_training.py --training ../rl_training
# 在有 Torch 的训练 Python 环境加 --torch-parity，对拍训练公式，不启动 Isaac/GPU。
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 scripts/benchmark.py --points 20000 --frames 30
```

本机 ROS 调度检查使用当前入口（隔离域、仅回环）：

```bash
bash scripts/test_ros_offline.sh
```

测试调用生产节点，使用受控阻塞验证位姿、地形发布和 IMU 转发能继续推进；不连接 AOS/GOS。常规测试保留在 `tests/` 和 `tests_ros/`，一次性诊断脚本与过时冒烟入口已清理。

NPZ 回放：

```bash
python3 scripts/replay_npz.py capture.npz --config config.local.json --output replay.npz
```

输入格式（禁止 pickle）：`points[F,P,3]` 原始雷达坐标，变长帧可用 NaN 补齐；`scan_stamps[F]`；`pose_stamps[K]`、`positions[K,3]`、`quaternions_xyzw[K,4]`；标量字符串 `cloud_frame`；可选 `point_offsets_s[F,P]`。均使用同一时间基准。输出 `observations[F,5,187]` 的五行为 h/m/c/variance/age，附同名 JSON 逐帧拒收原因。此脚本逐扫描查询，不模拟 AOS 的 50Hz 历史。

真实 rosbag 可在隔离域重放点云和位姿，使用相同 mapper 配置。若启用仿真时钟，两端都应传 `--ros-args -p use_sim_time:=true` 并保证 `/clock` 正常推进。生产 wrapper 默认使用真实时间。

## 8. 现场验收与故障定位

先在电机不由新策略控制时验证：平地、已知高度台阶、左右不对称物体、机体俯仰、转身、遮挡、停止点云。检查原始点数、有效率、点龄、compute_p95、输出频率和最终输入向量，不只看进程是否 active。左右不对称场景特别适合检查 Y 翻转与网格转置。

状态话题为配置的 `topics.status` 加 `/m20_heightmap_gos` 或 `/m20_heightmap_aos`。拒收原因会计数并每秒日志输出。常见原因：

| 现象 | 先查 |
|---|---|
| 话题有但零消息 | domain、DDS 网卡/发现、best-effort 兼容、订阅实际类型与 payload |
| `Uncalibrated cloud frame` | PointCloud2 frame 与外参字典键；不可随意重命名 frame |
| `Pose does not bracket capture time` | 点云/位姿时钟、缓冲覆盖、逐点时间起止含义 |
| `Pose timestamp stale/future` | 两板时钟偏差与原始驱动 timestamp 基准 |
| 置信度低但高度正确 | 未去畸变、扫描时长、位姿协方差、实测噪声；先标定，不靠降低方差“调绿” |
| 有效点少 | 真实遮挡/自体过滤/点密度/垂直面/多层；不能直接把最少点数改为 1 |
| `Policy tick missing...` | 本体发布周期与丢包；历史要从同一控制周期采样 |
| 模型契约不匹配 | v1 是否实际带历史、导出 sidecar 是否对应该权重 |

当前状态与验收边界见[迁移与启动记录](evidence/migration_20261016/summary.json)，旧狗证据见[历史记录](docs/TERRAIN_OFFLINE_FIX_20261016.md)。最新关闭地形HOLD的实机运行仍倒地，日志显示姿态超限后进入阻尼。仅提高轮目标速度上限的配置已应用，短时零速试验因控制周期HOLD提前结束，轮速行走效果未验证；不能宣称已解决倒地或通过连续行走。
