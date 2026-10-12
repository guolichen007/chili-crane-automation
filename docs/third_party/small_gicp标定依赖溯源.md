# small_gicp 标定依赖溯源

仅用于离线标定，不作为生产导航/自动控制依赖。
项目：[koide3/small_gicp](https://github.com/koide3/small_gicp)，MIT；本轮不复制其源码或素材。
[PyPI 1.0.1](https://pypi.org/project/small-gicp/1.0.1/) 发布 provenance 对应
commit `57c1106daf83c2c79ee0c58a9c7ed0032298ff4e`。

Python binding 精确 `small-gicp==1.0.1`，安装使用 `--no-deps --require-hashes`。
hash 见仓库 `third_party/small_gicp.requirements.txt`，包括 cp310 Linux x86_64、cp311 Windows amd64 和 sdist。
numpy1.26.4、scipy1.15.3 在 requirements-calibration.txt 固定；Windows 在独立 venv 验证，
Ubuntu22/Humble Python3.10 在 CI 实际构建/运行。未安装时 solver 显式不可用，不能替换为伪 GICP。

`preprocess_points` 生成协方差；GICP 使用 PointCloud/KdTree，VGICP 使用 GaussianVoxelMap。
两者以同机械初值开始，报告迭代/收敛/mean cost/Hessian/对应数及独立几何指标。
原生 API 的 result.error 是总 cost，不是“精度”；归一化 mean cost 不单独承担验收。
粗体素可产生“已收敛但偏移”反例，已加入负测试；正测试参数只属于合成台架，现场模板保持 null。
