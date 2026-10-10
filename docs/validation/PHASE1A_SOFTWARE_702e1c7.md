# Phase 1A 精确 SHA 软件验证归档

EVIDENCE_SHA: 702e1c7fee09c1a90143065a0bf8c92e64db7158
BASE_SHA: d4b8443d40d0e3bbb7c46d631be4b234667a706b
EVIDENCE_SCOPE: WINDOWS_STATIC_AND_UBUNTU22_ROS2_CI_SOFTWARE_ONLY
BRANCH: codex/phase1a-algorithm-deployment-v1

## 已实际执行的检查

| 项目 | 实际结果 |
| --- | --- |
| Windows diff / repo contract / public secret scan | 通过 |
| Windows unittest | 163 项：162 通过，1 项依赖 ROS2 环境跳过 |
| Windows 架构场景 | 46 项通过；不与上一组相加宣称独立总数 |
| Ubuntu Python3.10 static unittest | 163 项通过，无跳过 |
| 七包 colcon build | 7 packages finished |
| 七包 colcon test | 104 tests，0 errors，0 failures，0 skipped |
| ROS2 双雷达合成已标定场景 | 204=52，205=37，merged=37，配对/TF/降级/接收时间不刷新通过 |
| ROS2 双雷达合成未标定场景 | 204=37，205=37，merged=0；不发布假 TF / ready |
| ROS2 mock | fail-closed、旧命令/STOP/无有效 energizing 输出通过 |
| 官方 SDK XYZIRT wrapper | 2 packages finished；固定 SHA 和最小补丁检查通过 |

数字是这次实际日志，不是现场帧率/性能承诺。合成场景的 calibration、clock、coverage
仅为 test fixture，绝不写回 site 文件。

## GitHub Actions 与 artifact

- [Static contracts / 38027177567](https://github.com/guolichen007/chili-crane-automation/actions/runs/38027177567)：success。
- [ROS2 + vendor / 38027177586](https://github.com/guolichen007/chili-crane-automation/actions/runs/38027177586)：两个 job 均 success。
- [ROS2 软件证据 / 11660487275](https://github.com/guolichen007/chili-crane-automation/actions/runs/38027177586/artifacts/11660487275)
  ZIP SHA256：36e34c5e1e9ad060a45b93b61794d91b71f477672aebd67f1b00eb3006379b22。
- [vendor 编译证据 / 11661035511](https://github.com/guolichen007/chili-crane-automation/actions/runs/38027177586/artifacts/11661035511)
  ZIP SHA256：5b261d80bf346f917cf43cafa9889df5a015f4f0864a13ded1f1b5dc43cd0e41。

GitHub artifact 有保留期限，部署负责人应下载校验后存到独立 artifact/NAS。
CI 日志归档 apt 包版本、编译环境与 exact source SHA；不将二进制放进普通 Git。
Humble container 使用 floating distro tag；本次日志记录实际镜像摘要，不能声称 tag 永久固定。

## 曾被验证抓到的缺陷

首次 46292d2 的七包构建/102项包内测试通过，但 ROS2 transport 失败：NumPy concatenate
自动 repack 将原32字节布局压缩为27字节，而 PointCloud2 仍声明32。保留失败证据，
4403bf1 修复 canonical dtype 并加入 buffer/offset/roundtrip 回归，远端再次通过。
702e1c7 再将 receive_stamp 固定为真实收帧事实，验证断一路后计时器不刷新该事实。
没有删除失败测试，没有重写公共历史。

## 现场状态（不得由上述软件证据提升）

PHASE05_R2_STATUS=PASS_SOFTWARE_ONLY
LIVE_ADAM_READONLY_STATUS=NOT_RUN
LIVE_PULLWIRE_STATUS=NOT_RUN
LIVE_ER1_204_STATUS=NOT_RUN
LIVE_ER1_205_STATUS=NOT_RUN
REAL_LIDAR_BAG_STATUS=NOT_RUN
EXTRINSIC_CALIBRATION_STATUS=NOT_RUN
X_SERVO_STATUS=NOT_CONFIGURED
PHYSICAL_ACTUATION_STATUS=NOT_RUN
FIELD_STATUS=NOT_RUN

physical_output_enabled=false；automatic_control_enabled=false。
main 未修改；未创建生产/现场验收 tag。硬件历史桌面验收只按脱敏交接文档原范围引用。
