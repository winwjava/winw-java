# 不用棋盘格的自然场景自标定

支持 HBVCAM-W202011HD V33 这类单设备输出同步双目拼接图的普通相机。需要确认实际输出尺寸、左右顺序与曝光同步。没有预置该型号的内参，也没有把论文中的 61 mm 基线直接当成实测值。

## 直接启动：无需标定文件

推荐直接在 IDE 中运行 `StereoSlamDemo.main()`，不填写 Program arguments；Working directory 使用项目根目录。默认设备、单眼尺寸、拼接方式、帧率、视差、特征数和录制设置集中写在 `main` 的 `AutoStereoSettings defaults` 段中。默认单眼 640×480、设备 0、左右拼接、MJPG、30 FPS。

启动后界面上方可以调整这些参数，点击“应用并重启相机”生效。每次应用会先结束并保存上次采集，再打开新会话，旧录制不会混入新分辨率或左右顺序。参数不合法时留在当前会话；相机打开失败或尺寸不匹配时可以修改参数重试。支持调整采集后端、匹配窗口、唯一性阈值、校正验证误差、校正更新/录制间隔和录制上限；输出目录与可选校正缓存也可在界面选择。

想启动米制地图时，先录制至少 25 对（建议 80～120 对），在界面填写“实测基线 / mm”，然后点击“自然场景标定并启动地图”。它会停止并保存当前采集，自动准备所需 Python 组件、运行自标定并独立验证，成功后切换到地图窗口，不需要终端命令。基线默认 0（未填写），不把参考的 61 mm 当成实测参数；首次准备组件需要联网。失败会留在原窗口，保存原始序列与 `self-calibration.log`。该按钮使用本次实际采集的尺寸和左右顺序；相机参数编辑后须先点击“应用并重启相机”才能用于新的录制。

下面命令行入口仍保留，供脚本和已有流程使用：

在项目根目录运行：

```powershell
.\docs\slam\run-slam.ps1 -Mode camera-auto -InputSource 0 -EyeWidth 640 -EyeHeight 480 -LiveConfig .\docs\slam\live-example.properties
```

尺寸指**单眼**：左右拼接时上例请求总尺寸 1280×480。实际模式不符时会报告实际尺寸，请调整 `EyeWidth/EyeHeight`。`camera.layout=TOP_BOTTOM` 支持上下拼接，`camera.swapEyes=true` 可交换左右眼，配置沿用 `live-example.properties`。单个 USB 拼接设备仍需自身保证双目同步。

直接运行 `StereoSlamDemo.main()` 也默认进入 `camera-auto`，使用单眼 640×480。需要米制 SLAM 时仍可显式指定 `camera <真实标定文件> ...`。

窗口显示左右图、像素视差和质量状态。先对准有纹理且有远近层次的静态环境，缓慢移动相机。系统每 0.5 秒估计自然特征，使用 ORB 双向比率匹配、RANSAC 基础矩阵和 `stereoRectifyUncalibrated` 校正。匹配需要覆盖至少 8/12 个图像区域；主导平面、特征不足、形变过大或独立验证误差过大会拒绝候选。每对图像保留四分之一的匹配点用于验证，候选必须再通过后续两次观测。连续三次失败会暂停视差，等待重新估计。

视差图亮度表示当前校正坐标下的像素视差范围，黑色包括无效匹配和校正边界；它没有米制单位。任意投影校正后的视差不能直接代入现有的 `fx*baseline/d` 公式，也不保证跨参数更新时亮度可直接比较。此模式不会输出米制轨迹、点云或 OCC 地图。

每次运行创建独立的 `target/slam-output/auto-<UUID>/`，保存最多 120 对原始图像（每 0.5 秒一对）、离线清单 `manifest.txt` 和验证通过的 `rectification.properties`。原始图像没有被校正，能用于后续恢复内参。采满后仍可继续预览。停止后关闭窗口。

`rectification.properties` 仅包含投影单应矩阵，不能传给 `StereoRig`。如需复用缓存，可在 Java 启动前设置：

```powershell
$env:JAVA_TOOL_OPTIONS = '-Dslam.auto.rectificationCache=D:\path\to\rectification.properties'
```

缓存尺寸必须匹配，且读取后仍需三次当前场景验证，不直接启用。使用完按原配置恢复 `JAVA_TOOL_OPTIONS`。

## 从自然场景自动生成米制标定并启动地图

需要**实测左右镜头光心间距**作为尺度依据。不需要棋盘格。论文给出的该型号 61 mm 只能作参考，示例中的 `0.061` 应替换成你确认的基线。

```powershell
.\docs\slam\run-auto-slam.ps1 -BaselineMetres 0.061 -Device 0 -EyeWidth 640 -EyeHeight 480 -LiveConfig .\docs\slam\live-example.properties
```

这个入口会在项目 `target/scene-calibration-venv` 中准备独立的 Python 环境（首次需联网下载依赖），然后依次：

1. 打开无标定预览并录制自然场景。
2. 停止并关闭预览后，通过 PyCOLMAP 在 CPU 上估计内参、径向畸变和各帧相机位姿。
3. 检查左右刚性结构的一致性、相机运动与参数是否退化；用实测基线恢复尺度。
4. 在未参与重建的原始图像对上验证双目校正和正视差。
5. 生成现有 `StereoRig` 可读取的 `rectified=false` 文件，重新打开相机并运行原有 SLAM、稠密深度和 OCC。

拍摄至少 25 对，建议 80～120 对。相机需做左右、上下平移以及轻微转动；尽量保持焦距、自动对焦、分辨率和裁剪不变。只静止拍摄、只转动、只拍一面墙、动态物体或运动模糊都可能失败。自标定 CPU 计算可能持续数分钟，终端会显示进度。失败时保留全部原始序列与诊断，不启动米制地图。

已有序列可直接重试，免于重新录制：

```powershell
.\docs\slam\run-auto-slam.ps1 -BaselineMetres 0.061 -SequenceDirectory D:\path\to\auto-sequence -CalibrationOnly
```

也可直接使用 Python 处理：

```powershell
.\target\scene-calibration-venv\Scripts\python.exe .\docs\slam\self-calibrate.py D:\path\to\auto-sequence --baseline 0.061
```

默认使用已验证接口的 PyCOLMAP 4.2.1，无需另装 COLMAP 桌面程序。`--colmap <exe或COLMAP.bat>` 可切换为外部 CLI；GPU 参数名通过该 CLI 的帮助识别，仍使用 CPU。外部 CLI 路径没有在本环境做端到端验证。

`--model <COLMAP文本模型目录>` 可以导入已有模型，但报告会明确标记没有保留帧独立验证，不适合用它绕过质量检查。

## 质量检查与实际边界

内参使用两眼各自共享的 `SIMPLE_RADIAL` 模型：每眼一个焦距、中心主点和一阶径向畸变；主点固定在初始化值。不是任意镜头模型的全参数估计，明显鱼眼、复杂切向畸变或不同的水平/垂直焦距需要扩展模型。

重建只使用 80% 的图像对；至少恢复 12 对同步左右图。跨帧相对旋转误差第 90 百分位需低于 1°，平移不一致需低于 10%。相机平移要有至少两个方向的变化，并有至少 5° 转动。另 20% 图像对独立验证，至少 3 对且至少 60% 通过：匹配覆盖至少 8/12 区域，纵向误差第 80 百分位不超过 1.5 px，至少 85% 匹配具有正视差。质量报告保存在标定旁的 `.quality.json`；独立验证诊断也保存到序列目录。

校正可在线重新估计；完整米制标定目前在录制后计算，再启动一张新地图，尚不支持在旧地图中连续修改内参或尺度。弱畸变无标定预览可能先可用，明显畸变时也可能必须先完成多帧自标定。

通过质量检查不等于测距精度已获保证。首次实机运行应使用未参与标定的已知距离物体检查尺度和深度；这里没有该相机的实机精度评测。

## 验证

```powershell
mvn -Pslam-tests test
python .\docs\slam\test_self_calibrate.py
# 可选：真实运行 PyCOLMAP 的合成三维场景端到端测试
.\target\scene-calibration-venv\Scripts\python.exe .\docs\slam\test_scene_pipeline.py
```

最后一项生成的是合成纹理场景，输出保存在 `target/scene-pipeline-test-*`，不是相机实拍。

本次验证：37 项 Java 测试、4 项 Python 测试通过；实际 PyCOLMAP 重建恢复 24 对训练图像，6 对独立验证图像全部通过，纵向误差第 80 百分位约 0.93 px。生成的标定已通过 Java 数据集入口运行 30 帧，导出 30 个轨迹样本和 5,643 个点。上述数值来自合成场景，不代表 HBVCAM 实机精度。

参考：[OpenCV 双目几何接口](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html)、[COLMAP 自标定说明](https://github.com/colmap/colmap/blob/main/doc/faq.rst)、[COLMAP CLI](https://colmap.github.io/cli.html)、[COLMAP 输出坐标约定](https://colmap.github.io/format.html)。
