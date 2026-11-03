# 运行图片回放实测（2026-11-03）

对 `target/auto-b96cc826-19e0-4abe-b73d-5080c5495d18` 的 120 对原始图像实际执行了自动标定和顺序回放。

默认无参数启动进入 AutoStereoDemo，是校正/视差预览，没有调用 StereoSlam。`rectification.properties` 是投影对齐缓存，不能作为物理标定文件传入 StereoRig。

自动标定估计左右焦距约 518.72/515.72 px，但残差 0.00684 超过可靠性上限 0.005，拒绝导出。此次保留该检查，没有将候选参数当作真实标定。

## 更新：只输入图片和 65 毫米基线的 CPU 入口

用户补充实际基线 65 mm，明确不提供标定文件。新增入口无需用户内参、棋盘格、GPU 或神经网络模型：

```powershell
.\docs\slam\run-recording.ps1 -Sequence .\target\auto-b96cc826-19e0-4abe-b73d-5080c5495d18 -BaselineMm 65
```

程序从训练图片拟合右图纵向对齐，默认焦距先验为图宽的 0.8 倍（这不是测得的真实焦距），并用已知基线计算名义米制深度。保留极线、双向光流、PnP 和重投影验证，不因用户缺少标定文件而停止执行。

完整回放结果在 `target/cpu-auto-65mm`：31/120 有位姿，89/120 失败，0 回环；中位单帧约 58 ms。结果没有达到整段稳定 SLAM。基线解决尺度的一个约束，不能消除焦距先验与投影近似的误差。

曾实验增加非线性焦距/旋转拟合，得到 9/120，有效覆盖反而下降，已从新增代码中移除；没有把它作为自动入口。训练帧与几何留出帧分开，但这批数据已用于开发比较，不是独立测试集。

下面保留上一轮候选参数实验以便对照。后续工作不要求用户提供标定文件；仍需解决遮挡后恢复与弱纹理跨帧关联。

## 已修改

- ORB FAST 阈值降到 7，加入可选 SIFT；实拍对比后保留 ORB 为默认。
- 双目与近邻关键帧增加金字塔 LK 跟踪，检查双向误差、光度误差、边界；深度仍检查极线和范围，位姿仍经过 PnP RANSAC、LM、重投影、正深度和覆盖检查。
- 支持在跟踪内点较少时提前插入关键帧；离线评估采用每两帧关键帧间隔，适应约 2 Hz 的录制。
- Windows Maven profile 提供匹配版本的本地 OpenCV/OpenBLAS；缺少独立 JNI DLL 时用 Bytedeco Loader 加载已有发行版中的官方 Java JNI。
- 新增 RecordingSlamEvaluation，输出逐帧 CSV、TUM、PLY、统计报告，丢失帧保留 NaN，不填造位姿。

## 可复现实验

下列实验假设焦距 518 px、主点居中、零畸变、单位基线，在训练帧上拟合右图纵向仿射对齐。**这是投影近似诊断，不是可信米制 SLAM。** 每五帧的一帧不用于拟合对齐；在线回放仍可将其加入地图，因此不是独立冻结地图定位测试，也没有外部真值可计算 ATE/RPE。

```powershell
mvn -q compile dependency:build-classpath '-Dmdep.outputFile=target/slam-classpath.txt'
$slamCp = 'target/classes;' + (Get-Content target/slam-classpath.txt -Raw).Trim()
$manifest = 'target/auto-b96cc826-19e0-4abe-b73d-5080c5495d18/manifest.txt'
java -cp $slamCp winw.ai.slam.StereoSlamDemo evaluate $manifest candidate:518 target/recording-final orb
java -cp $slamCp winw.ai.slam.StereoSlamDemo evaluate $manifest candidate:518 target/recording-descriptor-control orb-descriptor
```

同一候选对齐、特征设置与关键帧策略下：

| 模式 | 有位姿帧 | 标定留出帧有位姿 | 回环 |
| --- | --- | --- | --- |
| 仅 ORB 描述子对照 | 1/120 | 1/24 | 0 |
| ORB + 双向 LK | 31/120 | 7/24 | 0 |

双向 LK 实验中位单帧耗时约 59 ms；数值受机器负载影响。地图和轨迹坐标为假设的基线单位，不可直接用作导航。89 帧仍无位姿，未达到完整序列稳定 SLAM，更没有达到“任意图片都可定位”。真实画面包含白墙、人物近距离遮挡和模糊；保存帧间隔约 0.5 秒。

有真实标定后，用标定文件路径替换 `candidate:518`，即可对原始左右图去畸变/物理校正并回放，不使用投影近似。需要实测镜头中心距、相机内参与畸变；只有中心距仍不足以恢复正确几何。建议保留同步原始视频/至少 15–30 Hz 图像，以保留跨帧跟踪信息。不可从已丢失的图像信息训练出可靠定位保证。

验证：`mvn -Pslam-tests test`。新增评估集成测试检查正常跟踪、空白帧 LOST、恢复 RELOCALIZED、逐帧 NaN 与轨迹不伪造。
