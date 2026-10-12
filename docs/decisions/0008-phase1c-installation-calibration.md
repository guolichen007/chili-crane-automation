# ADR 0008：安装观察、机械 seed 与离线外参标定

状态：已采用；仅覆盖 Phase1C 工具。基线 72019e0d464d4de6b38df74bc8c2097ae1829846。

## 边界

七包不变。设备协议仍归 hardware；slam 提供纯几何/采集门禁/离线 GICP、VGICP；
bringup 只组合显式传感器 profile。正常 profile driver=false；安装 profile 临时复制配置，
不修改原配置、不启动控制、不发布生产空间合并、不新增自动授权。
Phase1B TimeScale、observer、parser、normalizer、vendor lock 继续冻结。

## 坐标与生命周期

`T_A_B` 将 B 坐标点变到 A。机械输入是 `T_rig_204`、`T_rig_205`，
相对初值 `T_204_205 = inverse(T_rig_204) * T_rig_205`。
Euler 机械输入按 Rz(yaw)Ry(pitch)Rx(roll)，单位米/弧度；输出 matrix 与 xyzw quaternion。
`sensor_rig` 父关系必须 STATIC_PARENT / DYNAMIC_PARENT / NOT_CONFIGURED。
整个 rig 静止并不证明它永远相对 base 静止；本轮不发布 base→rig 静态 TF。
单雷达原坐标显示不需要未知外参；粗 overlay 需要测量 provenance，不能 identity 兜底。

NOT_CONFIGURED → MECHANICAL_SEED → CANDIDATE → REVIEWED → FIELD_VALIDATED。
seed 永远 NOT_CALIBRATED；候选/审核工具只生成外部 artifact，
即使 FIELD_VALIDATED 也不自动改生产 extrinsic YAML 或 dual_lidar_ready。
支架 mount_revision 改变立即 INVALIDATED / 拒绝 overlay、求解、采集。

## 采集与统计

每 scene 固定 rig，20～50 帧；原始与 canonical 按同帧 header、每点时间同时比对，
canonical 中点配对。新鲜 PTP、phase、双雷达和独立 RigStationarity 全部满足才采集。
PHYSICAL 不能接受 SYNTHETIC_FIXTURE 静止证据。未知/过期/失步/运动中止本次积累。
scene split、config hash、Git SHA、raw/canonical artifact hash 都保存；大数据不进 Git。

ROI/masks 在明确共同坐标系配置；预览默认 sensor_rig，求解默认 ER1-204，
不得拿 rig ROI 直接当 ER1-204 ROI。coverage 是采样占据栅格的未观测估计，
不是安全覆盖证书，更不是射线遮挡或可行驶区域证明。

GICP 是主解；相同机械初值 VGICP 是交叉检查；两种真实 native solver 均固定 small_gicp 1.0.1。
收敛不代表准确。近邻、局部面、法线、可观测性和解间差异同时验收；
近似 point-to-plane 信息矩阵区分 GOOD / WEAK_XY / WEAK_YAW / DEGENERATE。
至少五个标定场景，SE(3) log/exp Huber 共识，另设不参与拟合的验证场景。
模板数值 null；BENCH_EXPERIMENT 参数只证明软件，不冻结现场门限。

## 相机

独立只读安装入口，先读回确认 BayerRG8、GevIEEE1588=false、设备≤5FPS；
未确认拒绝开始。显示图像/实际 FPS/曝光/丢包/时间模式，时间仍 RECEIVE_ESTIMATE。
不写相机 PTP、不做雷达相机外参、不产生 camera control authority。
