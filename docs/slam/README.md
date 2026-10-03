# Java 双目 SLAM

本次新增独立包 `winw.ai.slam`，实现可运行的稀疏双目 SLAM 原型。算法调度、几何建模、地图管理及位姿图优化由 Java 实现；特征提取、匹配和 PnP 使用项目已有的 OpenCV JNI，因此运行时仍需要本地 OpenCV 库。

摄像头入口现已升级为实时 Swing 窗口，加入 SGBM 环境深度云和动态鸟瞰地图。接入设备、显示操作及输出说明见 [实时摄像头与鸟瞰图文档](LIVE.md)。新增 [三维 OCC 后端](OCCUPANCY.md)，支持占据/自由/未知表达、时序概率融合和体素驾驶显示。

## 项目分析

项目是 Maven 单模块工程，源码目标版本 Java 11，目前主代码能够编译。视觉代码集中在 `winw.ai.perception.visual`，已有 OpenCV 视差、特征匹配、光流和摄像头演示，以及 BoofCV 示例。这些代码适合验证单项视觉算法，但尚未形成连续定位与建图系统：

- `StereoMatcherDemo` 对单张左右图计算 SGBM 视差；相机参数、文件路径及显示逻辑与算法混在一起，没有连续位姿和地图。
- `StereoVision` 主要描述立体视觉理论；相关 demo 不包含关键帧、重定位、回环或优化后端。
- 现有 OpenCV 示例使用 `org.opencv.*`，POM 已有 Bytedeco OpenCV 4.9.0 和 Commons Math 3.6.1；本实现复用这些依赖，不再混入另一套 OpenCV Java API。
- 原有 Maven 配置默认跳过测试。启用全量测试时，旧的 `CitizenIdentificationNumberUtilTest` 引用不存在的类，编译失败。新增 `slam-tests` profile 只编译和运行本包测试，不修改旧测试。
- 当前目录未包含 `.git`，因此此次修改直接写入工作目录。

## 已实现流程

```text
同步左右图 → 去畸变/双目校正 → ORB 特征与描述子
                           ↓
左右 Hamming 匹配 → 比率检验 + 双向一致性 + 极线过滤
                           ↓
米制三角化 → 关键帧局部三维点 → 地图三维点与当前二维点匹配
                           ↓
PnP RANSAC → LM 位姿细化 → 重投影、正深度及图像覆盖检查
                           ↓
关键帧插入 / 跟踪失败后的历史关键帧重定位
                           ↓
旧关键帧候选 → 连续两帧几何一致性 → 回环边 → 位姿图优化
                           ↓
回环修正后的 TUM 轨迹与 PLY 稀疏点云
```

| 类 | 职责 |
| --- | --- |
| `StereoCalibration` | 校正后内参、基线及视差三角化 |
| `StereoRig` | 原始针孔双目校正或接收已校正图像 |
| `Pose` | 不可变刚体位姿、组合、逆变换、四元数 |
| `StereoSlam` | 跟踪状态机、关键帧地图、重定位、回环、导出 |
| `PoseGraph` | 固定首帧的刚体位姿图、数值雅可比、阻尼优化 |
| `SlamConfig` | 特征数量、内点阈值、关键帧和回环阈值 |
| `OpenCvNative` | 加载本地 OpenCV JNI |
| `StereoSlamDemo` | 离线图像序列和同步左右拼接摄像头入口 |

首个成功初始化的左相机坐标系作为世界坐标系，x 向右、y 向下、z 向前。`Pose` 表示 `T_world_camera`；PnP 返回 `T_camera_world`，程序取逆后才输出相机位置。基线和所有三维坐标单位为米。

对于左右水平内参相同的校正相机：

```text
d = u_left - u_right - (cx_left - cx_right)
Z = fx * baseline / d
X = (u_left - cx_left) * Z / fx
Y = (v_left - cy_left) * Z / fy
```

本版本通过关键帧三维地图跟踪，不只累积相邻帧的运动。地图点保存在所属关键帧的相机坐标系中；优化关键帧位姿后导出时重新变换。每个历史轨迹样本同样保存相对于锚定关键帧的变换，从而随回环修正。

## 标定与数据要求

先使用实测标定参数替换示例数值，示例不是你设备的真实标定。

- 已校正输入使用 `rectified-example.properties`，填写校正投影矩阵的 `fx, fy, cx, cy, rightCx` 和米制基线。左右必须具有相同 `fx, fy, cy`，且是水平双目。不要把原始内参当成校正后的内参。
- 原始针孔相机使用 `raw-example.properties`，填写左右 `K`、五参数畸变 `D=(k1,k2,p1,p2,k3)`、左右外参 `R,T`。定义为 `p_right = R * p_left + T`；常规左右布局下 `T.x` 为负数。程序调用 `stereoRectify`、`initUndistortRectifyMap` 和 `remap`，再从校正投影矩阵提取内参和基线。
- 图像尺寸必须与标定一致。缩放或裁剪后需要相应更新标定。
- 两侧需要同步曝光。单个设备输出的左右拼接流仅在设备本身保证同步时适用；两个普通 USB 相机顺序读取不能保证同步。本入口不实现双设备时间同步。
- 鱼眼、竖直双目、不等焦距的已校正输入不在当前模型内；鱼眼需要先使用对应模型外部校正。

## 运行

在项目根目录使用 PowerShell，要求 Java 11+、Maven 和匹配版本/架构的 OpenCV JNI。当前工作区已提供 `opencv_java490.dll`，自动加载根目录的库；其他环境可使用 `java.library.path` 或 `-Dslam.opencv.library=<绝对库路径>`。

```powershell
mvn compile
mvn -Pslam-tests test
```

离线清单每行三个空白分隔字段，路径相对于清单所在目录；时间戳单位为秒，严格递增。清单中的图片路径不支持空格。

```text
# timestamp left-image right-image
0.000 image_0/000000.png image_1/000000.png
0.100 image_0/000001.png image_1/000001.png
0.200 image_0/000002.png image_1/000002.png
```

运行离线序列：

```powershell
.\docs\slam\run-slam.ps1 -Mode dataset `
  -Calibration .\docs\slam\rectified-example.properties `
  -InputSource D:\datasets\sequence\manifest.txt `
  -OutputDirectory .\target\slam-output
```

运行单设备左右拼接双目摄像头，`0` 是设备编号，打开实时鸟瞰窗口，ESC 停止并保存：

```powershell
.\docs\slam\run-slam.ps1 -Mode camera `
  -Calibration .\docs\slam\raw-example.properties `
  -InputSource 0 -OutputDirectory .\target\slam-output
```

摄像头输出必须是 `left|right`，总尺寸为 `2*width × height`。驱动不一定接受程序请求的尺寸；实际尺寸不匹配时程序会报告错误。

脚本会构建并生成依赖 classpath。也可自行执行：

```powershell
mvn -q compile dependency:build-classpath '-Dmdep.outputFile=target/slam-classpath.txt'
$slamCp = 'target/classes;' + (Get-Content target/slam-classpath.txt -Raw).Trim()
java -cp $slamCp winw.ai.slam.StereoSlamDemo dataset calibration.properties manifest.txt target/slam-output
```

输出 `trajectory.tum` 的字段为 `timestamp tx ty tz qx qy qz qw`，`map.ply` 为 ASCII 点云，可用支持 PLY 的查看器打开。终端报告跟踪状态、PnP 内点、有效深度点、关键帧数量、回环和地图容量状态。`LOST` 时位姿为 null，该帧不会伪造轨迹样本。

API 接入已有同步相机采集代码：

```java
OpenCvNative.load();
try (StereoRig rig = StereoRig.load(calibrationPath);
     StereoSlam slam = new StereoSlam(rig.calibration, new SlamConfig())) {
    Mat rectLeft = new Mat(), rectRight = new Mat();
    try {
        // 在实际采集循环内执行；timestampSeconds 必须严格递增。
        rig.rectify(left, right, rectLeft, rectRight);
        StereoSlam.Result result = slam.process(rectLeft, rectRight, timestampSeconds);
        // 仅在 result.pose != null 时使用当前位置。
        slam.writeTrajectory(trajectoryPath);
        slam.writeMap(mapPath);
    } finally {
        rectLeft.release();
        rectRight.release();
    }
}
```

引擎采用单线程调用；调用者保留输入 Mat 的所有权。配置在构造时复制，之后修改原 `SlamConfig` 不影响已创建的引擎。

## 验证与边界

在当前工作区执行 `mvn -Pslam-tests test`，基础 SLAM 的 9 项测试、实时可视化的 7 项测试及驾驶视图的 3 项测试全部通过。基础覆盖深度尺度和左右主点差、位姿求逆与半周旋转、回环图降低漂移、原始双目校正、连续移动跟踪/丢失/恢复/导出、时间戳与标定检查、双帧验证的回环触发，以及数据集命令入口到轨迹/点云输出的集成测试。实时可视化测试说明见 [LIVE.md](LIVE.md)。视觉测试使用合成场景，真实调用 OpenCV ORB、匹配、PnP 与 SGBM。

本版本尚未使用实机或 KITTI/EuRoC 序列做精度与速度评测，测试通过不代表达到 ORB-SLAM2 的精度或实时性。当前工程边界：

- 采用关键帧局部点云，没有共享 MapPoint 观测图、地图点融合/剔除、局部或全局 BA；PLY 会保留跨关键帧重复点，也可能含未被多帧验证的双目错误点。
- 历史候选通过逐关键帧描述子匹配搜索，没有 DBoW2 词袋索引。场景重复纹理仍可能导致错误回环；双帧验证不能消除所有误匹配。
- 后端使用相对旋转向量和相对平移残差、统一权重、回环 Huber 损失；不是包含完整测量协方差和精确 SE(3) 对数映射的 g2o 后端。没有自动回滚错误回环。
- 位姿图是小规模稠密求解，默认最多 80 个关键帧。达到上限后继续定位于已有地图，但停止插入关键帧和添加新回环，不适合大范围长期运行。轨迹样本仍随运行时间增长。
- 未实现 IMU、动态物体过滤、曝光补偿、多地图合并和持久化地图加载。弱纹理、运动模糊、远距离小视差或快速转动可能丢失跟踪。

下一阶段应先用正确标定的真实短序列验证尺度和方向，再引入共享地图点与局部 BA、词袋检索、稀疏图求解和关键帧剔除。评估时记录 ATE/RPE、跟踪失败率和单帧耗时；双目已具备米制尺度，不应通过自由缩放的 Sim(3) 对齐掩盖尺度错误。

## GitHub 与论文依据

以下为算法参考；本包是独立 Java 简化实现，没有移植 ORB-SLAM2 的 C++ 代码或捆绑其词袋模型。

1. Mur-Artal, Tardós. **ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras**, IEEE T-RO, 2017, DOI: `10.1109/TRO.2017.2705103`。[论文](https://arxiv.org/abs/1610.06475)、[官方 GitHub](https://github.com/raulmur/ORB_SLAM2)。参考双目米制初始化、地图跟踪、关键帧、重定位与回环结构；原系统包含本原型尚未实现的 BA 等模块。
2. 官方源码的 [Tracking.cc](https://github.com/raulmur/ORB_SLAM2/blob/master/src/Tracking.cc) 与 [LoopClosing.cc](https://github.com/raulmur/ORB_SLAM2/blob/master/src/LoopClosing.cc)，用于核对职责和流程；本实现用 PnP 双帧验证替代其词袋、Sim3 与共视一致性等完整机制。
3. Kümmerle et al. **g2o: A General Framework for Graph Optimization**, ICRA, 2011。[作者项目及论文信息](https://openslam-org.github.io/g2o)、[官方 GitHub](https://github.com/RainerKuemmerle/g2o)。参考位姿图非线性最小二乘思路，本包用 Commons Math 独立实现小规模后端。
4. [OpenCV 官方 Calib3d Java 文档](https://docs.opencv.org/4.x/javadoc/org/opencv/calib3d/Calib3d.html)：校正、PnP RANSAC、LM 和重投影接口。该在线文档随 OpenCV 版本更新，本代码已在项目实际 4.9.0 依赖上编译和运行。
5. Geiger et al. **Are we ready for Autonomous Driving? The KITTI Vision Benchmark Suite**, CVPR, 2012。[原论文](https://www.cvlibs.net/projects/autonomous_vision_survey/literature/Geiger2012CVPR.pdf)、[官方里程计基准](https://www.cvlibs.net/datasets/kitti/eval_odometry.php)。用于后续实测评估设计，当前未报告该数据集结果。
