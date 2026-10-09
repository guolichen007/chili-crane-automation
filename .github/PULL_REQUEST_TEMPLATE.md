## 目的与范围

说明需求、实现内容和明确不做的内容；链接ADR/验收矩阵。

## 版本与兼容性

BASE_SHA:
OUTPUT_SHA:
BRANCH:
BREAKING_CHANGE:
MIGRATION:

## 验证（提供精确SHA证据链接）

WINDOWS_STATIC:
CI_STATIC:
ROS2_BUILD:
ROS2_TEST:
MOCK_SCENARIOS:
NATIVE_UBUNTU:
BAG:
HARDWARE:
FIELD:

未执行填NOT_RUN，不把mock或云端CI当现场验收。

## 风险、未确认项与回退

OPEN_ITEMS:
ROLLBACK_TO_SHA:
PUBLIC_DATA_REVIEW:

## 检查清单

- [ ] 每次提交的3项静态检查通过；最新SHA的软件CI已核对
- [ ] 未硬编码未知参数；自动输出仍fail-closed
- [ ] 消息、配置、实现、测试和文档一致，破坏性变更明确
- [ ] 未上传密钥、本机上下文、现场隐私、录包、生成目录
- [ ] 未触碰NDT上游；复用代码有来源/许可证说明
- [ ] main、发布和硬件操作未扩大授权范围
