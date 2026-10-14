package winw.ai.slam;

import java.nio.file.*;

/** A snapshot of one capture session; UI edits are applied by reopening the camera. */
public final class AutoStereoSettings {
    public int device=0,width=640,height=480,backend=0,disparities=128;
    public int features=3500,blockSize=5,uniquenessRatio=12;
    public int observationMillis=500,recordMillis=500,maxSamples=120;
    public double fps=30,validationTolerance=1.5,baselineMm=0;
    public boolean swapEyes=false,record=true;
    public LiveSlamConfig.Layout layout=LiveSlamConfig.Layout.SIDE_BY_SIDE;
    public String fourcc="MJPG";
    public Path outputDirectory=Paths.get("target/slam-output"),rectificationCache;

    public AutoStereoSettings copy(){
        AutoStereoSettings s=new AutoStereoSettings();
        s.device=device;s.width=width;s.height=height;s.backend=backend;s.disparities=disparities;
        s.features=features;s.blockSize=blockSize;s.uniquenessRatio=uniquenessRatio;
        s.observationMillis=observationMillis;s.recordMillis=recordMillis;s.maxSamples=maxSamples;
        s.fps=fps;s.validationTolerance=validationTolerance;s.baselineMm=baselineMm;
        s.swapEyes=swapEyes;s.record=record;s.layout=layout;s.fourcc=fourcc;
        s.outputDirectory=outputDirectory;s.rectificationCache=rectificationCache;return s;
    }
    public LiveSlamConfig cameraConfig(){
        LiveSlamConfig c=new LiveSlamConfig();c.backend=backend;c.disparities=disparities;c.fps=fps;
        c.layout=layout;c.swapEyes=swapEyes;c.fourcc=fourcc;c.validate();return c;
    }
    public void validate(){
        if(device<0||device>100||width<160||height<120||width>8192||height>8192)throw new IllegalArgumentException("设备编号范围 0～100；单眼尺寸至少 160×120，最大边长 8192");
        if(disparities<16||disparities%16!=0||disparities>=width/2)throw new IllegalArgumentException("视差范围必须是 16 的倍数，并小于单眼宽度的一半");
        if(features<500||features>20000||blockSize<3||blockSize>21||blockSize%2==0||uniquenessRatio<0||uniquenessRatio>100)
            throw new IllegalArgumentException("特征数 500～20000；匹配窗口为 3～21 的奇数；唯一性阈值 0～100");
        if(observationMillis<100||observationMillis>10000||recordMillis<100||recordMillis>10000||maxSamples<25||maxSamples>1000
            ||!Double.isFinite(validationTolerance)||validationTolerance<.25||validationTolerance>5)
            throw new IllegalArgumentException("更新/录制间隔 100～10000 ms；录制上限 25～1000；验证误差 0.25～5 px");
        if(!Double.isFinite(baselineMm)||baselineMm<0||baselineMm>1000||outputDirectory==null||layout==null)
            throw new IllegalArgumentException("基线范围 0～1000 mm；请选择输出目录和拼接方式");
        if(fourcc==null||(!fourcc.isEmpty()&&fourcc.length()!=4)||!Double.isFinite(fps)||fps<1||fps>120||backend<0)
            throw new IllegalArgumentException("帧率范围 1～120；编码留空或填写四个字符，例如 MJPG");
        cameraConfig();
    }
}
