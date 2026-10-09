# 贡献与交付规范

本仓库是工业行车的软件框架，不是经现场验收的无人控制产品。
贡献者应先阅读 README、AGENTS、项目上下文、ADR 和接口契约。

## 分支与提交

- 日常工作使用独立 codex/<scope>-vN 或 ubuntu/validate-<sha> 分支。
- main 只接收经审查的合并；未经用户授权，不创建/合并PR、发布release或改仓库设置。
- 正常SSH push；不强推、不改写已公开历史、不上传工作区快照或临时记录。
- 新提交采用 Conventional Commits：type(scope): 摘要；描述可用中文。
  类型：feat/fix/refactor/docs/test/ci/build/chore/perf/style/revert。
  不兼容消息/语义变更用 ! 或 BREAKING CHANGE，说明消费方重建与迁移。
  每提交聚焦一个主题，代码、测试、配置与文档需完整对应。
- 历史提交保留，不为格式规范重写历史。
  规范来源：[Conventional Commits 1.0.0](https://www.conventionalcommits.org/en/v1.0.0/)。

## 提交前和推送前

每次提交前必须执行：

```text
git diff --check
python tools/check_repo_contracts.py
python -m unittest discover -s tests_static -p "test_*.py"
```

最终推送前：

```text
python tools/validate_architecture_scenarios.py
python tools/check_delivery.py --base-sha <40-character-base-sha>
git push -u origin <review-branch>
```

check_delivery 只检查，不自行push/merge。它检查干净工作区、提交标题、
基础SHA关系、SSH远程、公开范围及本机上下文不被跟踪。
首次push触发CI；CI结束后核对同一OUTPUT_SHA、测试日志与证据，再交付审查。
不能因为要先触发CI而宣称该SHA已通过。

## 公共资料

不提交密钥、token、密码、个人目录、本机记忆、现场照片/录包、客户敏感数据、
生成目录或未脱敏日志。设备清单仅记录明确授权公开的资产信息；
资产地址不等于驱动/生产参数已确认，未知角色/极性/标定保持NOT_CONFIGURED。
不要将聊天原文或AI计划直接充当工程文档，改为ADR、接口、验收矩阵和版本化配置。
提交前进行人工数据审查；自动门控不是完整秘密扫描或安全认证。

## 评审与证据

PR使用仓库模板，给出目的、范围、BASE/OUTPUT_SHA、破坏性变更、
验证分层、风险、OPEN项和回退方案。
README给外部读者项目定位；docs/review/REVIEW_GUIDE.md给技术/GPT审查入口。
CI容器、原生Ubuntu、真实bag、硬件、工厂验收必须分别报告PASS/FAIL/NOT_RUN。
不写“全部完成”掩盖接口桩，不用测试通过宣称工业安全认证。
NDT抽取需记录上游文件/SHA、许可证、改动和专项证据；厂商协议仅在hardware。

## 发布

当前Phase0.5仍是开发基线，不创建生产版本tag或release。
发布需另行授权、完整配置审查、精确SHA软件验证、现场验收与维护者批准。
GitHub分支保护/必需审查人/必需CI由维护者设置；
本轮只提供治理文件，不宣称已开启服务器端保护。
