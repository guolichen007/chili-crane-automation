# Phase2C-R2.1 工控机软件封板报告

> 日期：2026-10-15
> 阶段：Phase2C-R2.1（快照周期工程边界收口）
> 状态：`PASS_SOFTWARE_SHADOW_NOT_FIELD`

## 1. SHA 与分支

| 项 | 值 |
|---|---|
| 基线 BASE_SHA | `e552db4976a7e09d4aa3b347c4ceb0751f6996c1` |
| R2.1 代码提交 | `2d7103feb4c03c6bf9f98483f5ff0ad66cdbae57` |
| 封板分支 | `codex/phase2c-r2-1-final-seal` |
| 架构模式 | `SNAPSHOT_PER_CYCLE` |

## 2. 工控机环境

```text
Ubuntu 22.04.5 LTS
ROS2 Humble
Python 3.10.12
GCC 11.4.0
cmake 3.22.1
```

## 3. 本轮只修三项（R2.1-A/B/C）

| 项 | 内容 |
|---|---|
| R2.1-A | `grasp_interaction.py` 拆分配置必需参数：snapshot 模式不再强制 `collapse_margin_m`、`updated_clearance_margin_m` 两个已死 legacy refresh 边距；仍保留 `bound_clearance_margin_m`/`minimum_observed_rise_m`/`pit_clearance_margin_m`/`safe_lift_height_m` 等当前 cycle 安全证据 |
| R2.1-B | `grab_mask_pairs.py` 增加 `reset()`；`surface_runtime.py` 新 cycle SCAN 时重置 GrabMaskPairs，并对 `closest()` 增加 `minimum_epoch_ns` 硬时间合同（grab/clearance 时间必须晚于当前 `scan_start_ns`），杜绝上一 cycle 的 GrabMask 污染新 cycle 首帧 |
| R2.1-C | 增加统一 `architecture_mode` 配置字段与 `check_architecture_mode()` 合同；节点 `cycle_snapshot_mode` 参数与配置 `architecture_mode` 不一致时 `PHASE2C_ARCHITECTURE_MODE_MISMATCH` fail closed，杜绝静默混跑 |

## 4. 为什么只修这三项

R2 快照周期架构已冻结（`ARCHITECTURE_MODE=SNAPSHOT_PER_CYCLE`）。这三项是 R2 生命周期收口中的窄工程缺陷：A 消除快照模式下对已删除 continuous refresh 路径的死参数依赖；B 消除跨 cycle 的 GrabMask 泄漏；C 用配置合同关闭静默混跑风险。均不触碰 Phase2A/B 算法、TaskCycles、SafetyAuthority、生产阈值、授权链。

## 5. 新增反证测试（6 项）

```text
SNAPSHOT_WITH_REFRESH_FIELDS_UNSET=PASS
LEGACY_REFRESH_FIELDS_UNSET=REJECTED
OLD_CYCLE_GRAB_MASK_REJECTED=PASS（reset 丢弃上一 cycle 样本）
PRE_SCAN_GRAB_MASK_REJECTED=PASS（grab 时间早于 scan_start 被拒）
CURRENT_CYCLE_GRAB_MASK_ACCEPTED=PASS（scan_start 后新 pair 通过）
MIXED_R2_MODE_CONFIGURATION=REJECTED（config=SNAPSHOT + parameter=false → fail closed）
```

## 6. 静态测试

```text
unittest: 483 通过（22 skip）
pytest:   461 通过（22 skip）
```

## 7. 回归

```text
PHASE2A_REGRESSION=PASS（25 场景，200 序列，安全违例 0）
PHASE2B_REGRESSION=PASS（21 场景，64 序列，R1 regression）
PHASE2C_R2_1_REGRESSION=PASS（native + installed normal + installed symlink）
```

## 8. 构建与测试

```text
COLCON_NORMAL=PASS（8 包，colcon test 374 测试 0 失败 0 错误）
COLCON_SYMLINK=PASS
NORMAL_OVERLAY_PRESENT_IN_SYMLINK_ENV=NO
MIXED_PACKAGE_RESOLUTION=NO
```

## 9. 安装态 Phase2C R2.1

```text
INSTALLED_NORMAL=PASS
INSTALLED_SYMLINK=PASS
SCAN_ONLY / SURFACE_FREEZE_AFTER_PLAN 均 PASS
NO_ACTIVE_GRAB_SURFACE_MUTATION（ACTIVE_GRAB_SURFACE_MUTATION_COUNT=0）
NO_ACTIVE_GRAB_NEW_PROPOSAL（ACTIVE_GRAB_NEW_PROPOSAL_COUNT=0）
NEXT_CYCLE_RESCAN_REQUIRED=PASS
NO_OLD_SURFACE_NEXT_CYCLE_REUSE（NEXT_CYCLE_OLD_SURFACE_REUSE_COUNT=0）
```

## 10. flake probe 与进程泄漏

```text
NORMAL_FLAKE_PROBE=10/10_PASS
SYMLINK_FLAKE_PROBE=10/10_PASS
VALIDATOR_OWNED_PROCESS_LEAK_COUNT=0
ROS_DOMAIN_ISOLATED=YES（ROS_LOCALHOST_ONLY=1，显式 domain 42）
```

## 11. 软件安全零输出

```text
SAFETY_PERMIT_FALSE=PASS
AUTHORIZED_COMMAND_ZERO=PASS_0
ACTUATION_REQUEST_ZERO=PASS_0
PHYSICAL_OUTPUT_DISABLED=TRUE
AUTOMATIC_CONTROL_DISABLED=TRUE
CAMERA_CONTROL_AUTHORITY_FALSE=PASS
VALID_FOR_CONTROL_FALSE=PASS
CONTROL_TOPIC_MESSAGES=0
```

## 12. FIELD / 硬件尚未执行项目

```text
GRAB_DESCENT_TARGET_Z_FIELD_CALIBRATION=NOT_RUN
BENCH_DO_STATUS=NOT_RUN
FIELD_STATUS=NOT_RUN
FULL_AUTO=NOT_RUN
```

## 13. 结论

```text
PHASE2C_R2_1_STATUS=SEALED_SOFTWARE_SHADOW_NOT_FIELD
```

软件层为 Shadow（非现场）验收；未执行真实 ADAM DO、真实电机、自动控制、FIELD 标定。
