# 双 ER1 现场安装与外参标定手册

本轮交付安装和离线标定**工具**，不是已完成行车安装或现场外参。
运行 Ubuntu22.04 / ROS2 Humble / Python3.10；Windows 只做软件验证。
physical_output_enabled=false，automatic_control_enabled=false，camera_control_authority=false。
不启动 ADAM DO、executor、任务、NDT、Grab、Pit 或生产空间合并。

## 1. 现场顺序

临时安装 → 单雷达 canonical/raw 观察 → ROI 覆盖检查 → 调支架 → 固定 → 防松/标记
→ 建 mount_revision → 机械测量 → 静态采集 → GICP/VGICP → 多场景共识
→ 独立验证 → overlay → 定量报告 → 人工审核 → FIELD_VALIDATED。
每次支架再移动，从新的 mount_revision 和新的场景重新开始，不沿用旧外参。

## 2. 时间与网络前置

以 [S15 回传事实](../hardware/20261012_S15时间链封板事实.md) 为准：
204 单播 7799/6688，205 单播 6699/7788，目的主机192.168.1.102/enp3s0。
保持已通过 S15 的 PTP 服务和官方驱动配置；raw PTP 不减37，canonical 两种时间一起减37。
S15 的 LOCAL_VERIFIED 可做台架，不是生产 VALID；PTP 模式和 phase 必须持续验证。

仓库默认双雷达 pairing/geometry 参数未配置，安装 preview **不猜测**这些数值。
部署 owner 把经过 S15 验证的运行配置复制为仓库外 `CONFIG_ROOT`，
合入本轮 `runtime/` 与 `calibration/` 模板后逐项确认，不自动覆盖原部署。
必须保留 R2 relation/clock/first-point 配对约定；记录该配置 hash。

```bash
source /opt/ros/humble/setup.bash
source /path/to/exact-sha-workspace/install/setup.bash
source /path/to/vendor/install/setup.bash
# CONFIG_ROOT=/path/to/owner-reviewed-config （包含 runtime 和 sites/crane_01）
ros2 launch chili_crane_bringup lidar_installation_preview.launch.py \
  config_root:="$CONFIG_ROOT" profile:=installation_preview view:=204_only
```

另可选择 `view:=205_only` 或 `view:=rough`。这里 single-only 指显示视图，不关闭另一个时间通道。
RViz 预置 Top / Side / 3D；raw 原始时间证据通过 vendor_points 单独查看/保存。
AF_PACKET 权限由部署 owner 配置。不要给通用 python setcap、不要本轮改 ptp4l。
缺 vendor、权限或经过审核的 pairing 参数时明确报错；不能降级为伪同步。

## 3. Rig、机械 seed 和覆盖

配置 `calibration/sensor_rig.yaml`：父模式、rig frame、mount_revision、机械来源；
未知 parent 保持 NOT_CONFIGURED。动态父机构禁止静态 base→rig TF。
两个 `mechanical_seed_er1_*.yaml` 填测量 x/y/z 米、roll/pitch/yaw 弧度、方法、
不确定度、operator、date、同一 mount_revision。seed 保持 MECHANICAL_SEED / NOT_CALIBRATED。
公式 `T_204_205 = inverse(T_rig_204) * T_rig_205`，不允许空值 identity 兜底。

`installation_geometry.yaml` 指定 rig 坐标 ROI、coverage_cell_size_m、固定结构 ROI。
发布 `/crane_01/installation/diagnostics`：点数、range P10/P50/P90/P95、
204-only/205-only/overlap occupied cells、ROI 未观测比例、共同结构点数。
该比例是采样估计，不代表无盲区、自由空间或生产安全阈值。

## 4. 静止证据与采集

启动 `profile:=calibration_capture` 仍是显式 sensor-only pipeline，不自动录数据。
采集还需新鲜 `/crane_01/sensor_rig/stationarity`（具体 topic 以 rig 配置为准）。
消息 `RigStationarity` 必须包含：mount_revision、stationary=true、有效性、
verification_method、evidence_id、EvidenceMetadata 的 source/receive UTC 时间。
允许经 owner 验证的机械锁止 `MECHANICALLY_LOCKED_VERIFIED` 或独立运动测量
`MEASURED_RIG_MOTION`；不能仅凭云配准稳定推断整个 rig 静止。
发布 source_type=PHYSICAL，持续续新并在移动/解除锁止时立即无效。
**没有该现场来源就禁止最终场景采集**；不要用永久 true publisher 绕过门禁。

```bash
ros2 run chili_crane_slam capture_lidar_calibration_scene.py \
  --site "$CONFIG_ROOT/sites/crane_01" --repository /path/to/exact-sha-workspace \
  --scene-id scene01 --split CALIBRATION --frames 30 --stable-frames 20 \
  --max-pair-delta "$OWNER_MAX_PAIR_DELTA" --evidence-ttl "$OWNER_EVIDENCE_TTL" \
  --operator "$OPERATOR" --output /path/outside-repo/scene01 \
  --ros-args -r __ns:=/crane_01
```

或替换 `--frames 30` 为 `--seconds 2|3|4|5`，仍至少20帧、至多50帧。
pair delta/TTL 是 owner 已审核值，不把这里命令变量当现场建议阈值。
等待90秒仍不满足就退出且不形成完成 manifest。失步/phase丢失/运动清空积累。
目录必须新建、在仓库外；保存双 raw/canonical NPZ、每帧 header、点数、
时钟/网络/静止来源、Git SHA、配置 SHA256、四份 artifact SHA256。
大点云/bag 不提交；需审查的脱敏 manifest/hash 才可进入 Git。

## 5. 离线解算与独立验证

安装固定依赖（不从第三方仓库复制算法）：

```bash
python3 -m pip install -r requirements-calibration.txt
python3 -m pip install --no-deps --require-hashes -r third_party/small_gicp.requirements.txt
```

至少 scene01～scene05 CALIBRATION，另一个不参与拟合的 scene06 VALIDATION。
每个 scene 传感器 rig 静止，固定立柱/梁/墙角/坑沿/地面为主体；不要运动抓斗作特征。
另建 `solver-geometry.yaml`：coordinate_frame=crane_01/er1_204，明确该坐标 ROI；
六类动态 mask（grab/rope/person/vehicle/moving_trolley_structure/dynamic_material）
均 CONFIGURED + bounds，或 ABSENT_VERIFIED + evidence；未核实拒绝求解。
`solver_parameters.yaml` 数值默认 null；owner 在外部副本填参数和验收门限，
先 BENCH_EXPERIMENT，现场证据确认后才 FIELD_FROZEN；范围校验不是生产推荐值。

```bash
python3 tools/dual_lidar_extrinsic_solver.py \
  --site "$CONFIG_ROOT/sites/crane_01" --parameters /path/solver-parameters.yaml \
  --geometry /path/solver-geometry.yaml \
  --scene /path/scene01 --scene /path/scene02 --scene /path/scene03 \
  --scene /path/scene04 --scene /path/scene05 --scene /path/scene06 \
  --output /path/outside-repo/calibration-report
```

工具核对 hash/同 mount_revision/帧数/来源/scene split；有初值 GICP 为主解，VGICP交叉。
报告 `metrics.json`、`外参标定报告.md`、可用时 `candidate.yaml`，完整保留坐标约定、
translation/quaternion/matrix、每 scene GICP/VGICP、近邻与点面 P50/P90/P95、
法线夹角/plane offset、观测秩、XYZ/rotation dispersion、SE3 robust consensus 和 holdout。
不收敛或退化输出 REJECTED 和诊断，不产生可审核候选；单凭收敛不算定量通过。
任何定量项不通过仍保持候选待检查，禁止 FIELD_VALIDATED。

## 6. Overlay 与人工审核

保持 sensor-only 数据输入，另开：

```bash
ros2 launch chili_crane_bringup extrinsic_overlay.launch.py \
  site_config:="$CONFIG_ROOT/sites/crane_01" candidate:=/path/calibration-report/candidate.yaml
python3 tools/review_lidar_calibration.py --candidate /path/calibration-report/candidate.yaml \
  --state REVIEWED --operator "$OPERATOR" --mount-revision "$MOUNT_REVISION" \
  --confirm-report --output /path/calibration-report/reviewed.yaml
```

视觉核对柱/梁/墙/边/地面，204蓝、205橙；界面明确 CANDIDATE，不发布生产 TF。
REVIEWED 之后才可再对 PHYSICAL artifact 显式申请 FIELD_VALIDATED；synthetic 永不允许。
审核 artifact 不自动进入生产 profile；生产投用仍另行审核，不属于本轮。
支架移动用新 revision 使旧外参 INVALIDATED，并重新测量和采集。

## 7. 相机安装预览

owner 在现场 SDK确认 serial/model；相机本身预先配置 5 FPS、BayerRG8、GevIEEE1588=false。
工具先读回，不替 owner 改设备；未知/不符合拒绝启动。设置官方 MVS_PYTHON_PATH 后：

```bash
ros2 launch chili_crane_bringup camera_installation_preview.launch.py config_root:="$CONFIG_ROOT"
ros2 topic echo /crane_01/camera/hik_01/diagnostics
```

rqt 查看 image_raw；诊断显示实际 FPS、曝光 raw/unit、packet loss、timestamp mode。
不做相机PTP、标定、AI、Safety authority。SDK 未接入时不能报告实机出图 PASS。

## 8. 交接清单

- 临时/最终安装、锁止方法、parent 模式、mount_revision、防松记录：OWNER。
- measured seeds/ROI/静态结构与动态 mask：OWNER。
- paired timing profile、采集 TTL/配对范围、现场 solver/验收门限：OWNER。
- camera SDK serial/model 与图像参数读回：OWNER。
- PTP TX timestamp timeout 是否复发：OWNER 持续检查，不改冻结主链。
- 软件测试/CI 不代替以上现场事实；INSTALLATION / EXTRINSIC / CAMERA EXTRINSIC / ACTUATION 均 NOT_RUN。
