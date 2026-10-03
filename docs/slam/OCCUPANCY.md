# 双目 SLAM 的三维 OCC 优化

独立包 `winw.ai.slam` 已加入三维概率占据后端，实时驾驶视图默认显示体素障碍和已观测自由空间。它由双目几何深度驱动，不需要神经网络权重；未实现 Tesla 的神经网络、语义分类、运动流或遮挡场景补全。

## 资料与实现对应

| 原始资料 | 与本实现的关系 |
| --- | --- |
| Tesla 的 [视觉占据专利 WO2024054585A1](https://patents.google.com/patent/WO2024054585A1/en)、[官方 AI 页面](https://www.tesla.com/AI) | 公开描述基于图像的三维占据表达和可视化，以及时空视觉感知。专利不是学术论文；本项目参考三维空间表达，未复现其训练系统。 |
| [SurroundOcc，ICCV 2023](https://arxiv.org/abs/2303.09551)、[官方代码](https://github.com/weiyithu/SurroundOcc) | 多摄像头特征与三维占据预测；参考体素表达，不引入其注意力网络或训练流程。 |
| [OccNet / Scene as Occupancy](https://arxiv.org/abs/2306.02851)、[官方代码](https://github.com/OpenDriveLab/OccNet) | 时序三维占据与场景理解；本版本通过 SLAM 位姿对齐几何观测，不使用学习型时序解码器。 |
| [OccDepth](https://arxiv.org/abs/2302.13540)、[官方代码](https://github.com/megvii-research/OccDepth) | 针对双目深度感知的语义场景补全，适合作为后续模型接入依据；当前未运行该模型。 |
| [OctoMap 论文](https://www.arminhornung.de/Research/pub/hornung13auro.pdf)、[作者项目](https://octomap.github.io/) | 当前实际几何后端的主要算法依据：射线观测、占据概率、自由/占据/未知状态。Java 独立实现使用稀疏哈希体素，而非八叉树。 |

SurroundOcc、OccNet、OccDepth 是相关学术工作，不是 Tesla 发表的论文。没有将这些开源模型的能力作为本程序已经具备的功能。

## 已实现的处理链

```text
同步双目 → 校正 → SGBM 视差 → 米制深度与不确定度
                    + SLAM 当前位姿 / 关键帧锚点
                              ↓
三维 DDA 射线 → 自由证据 / 表面占据证据 / 遮挡未知
                              ↓
跨帧加权 log-odds → 有限时间窗 → 回环纠偏后重放
                              ↓
三维体素驾驶显示 + 二维鸟瞰投影 + CSV / PLY
```

- `DenseStereoMapper.measure` 统一计算深度，关键帧历史点云和滚动 OCC 共用测量，避免同帧重复 SGBM。
- `TemporalOccupancyMapper` 从相机到表面的射线清除可见空域，表面累积占据证据；表面后方和未采样位置保持未知。相机所在体素不会被直接判为自由。
- 同一帧同一体素最多记一次证据，命中优先于穿越。默认至少两个观测帧支持才确认占据，避免大量重复像素造成虚假置信度。
- 当前深度误差近似为 `sigmaZ = Z² * sigmaDisparity / (fx * baseline)`；置信权重为 `1 / (1 + (sigmaZ / resolution)²)`。它是启发式几何权重，未做统计标定。低权重观测不清空、不占据；靠近表面的不确定度带不提供自由证据。
- 以先验概率 0.5 累积 `L = Σ(wHit*logit(pHit) + wMiss*logit(pMiss))`。只在概率查询时将 L 限制到 [-3.5, 3.5]，过期观测能够精确撤销贡献，避免饱和累积无法清除旧障碍。
- 保留有限时间和帧数的观测；新自由射线可清除旧占据，旧证据也会过期。跟踪丢失时停止添加射线，但继续老化。没有动态物体分类或速度估计。
- 观测保存相对于锚定关键帧的位姿；回环后重放窗口内全部占据与自由证据，不仅移动障碍点。超出体素预算时撤销最老完整观测，窗口提示容量受限。

## 显示与坐标

驾驶模式显示占据体素的实际网格立方体；淡绿色代表已观测自由列，灰色背景保留未知。蓝线仍是历史轨迹。二维列只要存在占据体素就显示占据；只有 `view.minY` 到 `view.maxY` 内所有体素层都被观测为自由，才显示自由列。这是保守投影，并非可行驶区域检测。

世界坐标仍是首次初始化左相机：x 向右、y 向下、z 向前，单位米。没有 IMU、地面拟合或自动重力对齐；高度范围必须按相机安装位置配置。体素的概率是几何观测融合值，不能作为导航安全保证。

## 默认配置与运行

使用 [live-example.properties](live-example.properties) 并按 [设备接入文档](LIVE.md) 提供真实标定。OCC 在 `camera`、`dataset-view` 实时入口启用；普通 `dataset` 入口仍导出基础稀疏 SLAM。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `map.dense` / `occ.enabled` | true / true | 两者都开启才运行 OCC |
| `occ.resolution` | 0.25 m | 三维体素边长；OCC 不使用 `view.cellSize` |
| `occ.radius` | 15 m | 当前相机周围 X/Z 各方向的半宽，范围为正方形 |
| `occ.frameInterval` | 3 | 有效定位帧采样间隔；新增关键帧也采样 |
| `occ.maxAgeSeconds` / `occ.maxFrames` | 8 s / 20 | 滚动窗口时间与观测帧数上限 |
| `occ.maxVoxels` | 200000 | 存储预算 |
| `occ.minHitFrames` | 2 | 确认占据的最少命中帧数 |
| `occ.hitProbability` / `occ.missProbability` | 0.72 / 0.35 | 逆传感器模型参数 |
| `occ.disparitySigma` / `occ.minDepthConfidence` | 0.5 px / 0.2 | 深度误差估计与权重下限 |

判定阈值默认占据 0.65、自由 0.35，中间状态保持未知。增大体素、增加采样间隔或缩小半宽可减少计算量；尚未在真实摄像头上测得吞吐改善。

设置 `occ.enabled=false` 恢复历史点云显示。达到 SLAM 关键帧上限后，定位仍受已有地图范围限制，但 OCC 可继续融合有效定位帧；它不扩大 SLAM 的定位地图。

## 导出与验证

- `occupancy.csv`：当前高度范围内已存储体素中心、概率、状态及命中/自由帧数；未列出的体素为未知，文件头记录分辨率。
- `occupancy.ply`：确认占据的体素中心及概率。
- `environment.ply`：原有关键帧历史深度云，不是经过 OCC 时序过滤的结果。
- `bird-eye.png`：停止时的二维全图。驾驶视图最多绘制距离最近的 20000 个占据体素，此显示上限不截断占据数据导出。

`mvn -Pslam-tests test` 已通过 31 项测试，其中新增 12 项覆盖射线三态、同帧去重、旧障碍清除、时间过期、回环重放、弱深度抑制、不确定度带、容量约束、保守高度投影，以及真实调用 OpenCV 的合成双目到 OCC 集成与导出。`target/slam-occupancy-preview.png` 是合成测试预览，不是实机效果或精度评测。

单个双目摄像头只观测自身视野，不能得到 Tesla 多摄像头系统的即时 360° 信息。下一步应先用实测标定验证深度尺度、ATE/RPE 和帧耗时，再评估 OccDepth 等模型的 Java ONNX 接入；需核对输入校正、归一化、体素坐标与场景域，并有相应权重和评测数据后，才能增加语义与场景补全。
