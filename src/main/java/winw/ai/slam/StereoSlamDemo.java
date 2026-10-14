package winw.ai.slam;

import lombok.extern.slf4j.Slf4j;

import java.io.*;
import java.nio.file.*;
import java.util.Arrays;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;

/** Dataset manifest or a single hardware-synchronized side-by-side camera. */
@Slf4j
public final class StereoSlamDemo {
    private StereoSlamDemo(){}
    public static void main(String[] args)throws Exception{
        if(args.length>0&&args[0].equals("camera-auto")){AutoStereoDemo.main(args);return;}
        if(args.length==0){
            // IDE 直接运行这里。常用默认参数集中在 main，启动后也可在界面调整。
            AutoStereoSettings defaults=new AutoStereoSettings();
            defaults.device=0;                          // 摄像头编号
            defaults.width=640;defaults.height=480;      // 单眼尺寸；左右拼接总尺寸 1280×480
            defaults.layout=LiveSlamConfig.Layout.SIDE_BY_SIDE;
            defaults.swapEyes=false;
            defaults.backend=0;                        // 0 自动；700 DSHOW；1400 MSMF
            defaults.fourcc="MJPG";defaults.fps=30;
            defaults.disparities=128;                  // 16 的倍数，小于单眼宽度的一半
            defaults.blockSize=5;defaults.uniquenessRatio=12;
            defaults.features=3500;defaults.validationTolerance=1.5;
            defaults.observationMillis=500;
            defaults.record=true;defaults.recordMillis=500;defaults.maxSamples=120;
            defaults.baselineMm=0;                     // 米制标定前在界面填写实测基线，0 表示未填写
            defaults.outputDirectory=Paths.get("target/slam-output");
            defaults.rectificationCache=null;         // 可选：已有自然场景校正缓存
            AutoStereoDemo.launch(defaults);return;
        }
        log.info("SLAM launch arguments: {}",Arrays.toString(args));
        if(args.length>=4&&(args[0].equals("camera")||args[0].equals("dataset-view"))){LiveStereoSlam.main(args);return;}
        if(args.length<4||(!args[0].equals("dataset")&&!args[0].equals("camera"))){
            log.info("dataset <calibration.properties> <manifest.txt> <output-directory>\n"
                +"camera <calibration.properties> <stereo-device-index> <output-directory> [live.properties]\n"
                +"dataset-view <calibration.properties> <manifest.txt> <output-directory> [live.properties]");return;
        }
        OpenCvNative.load();Path output=Paths.get(args[3]);Files.createDirectories(output);
        try(StereoRig rig=StereoRig.load(Paths.get(args[1]));StereoSlam slam=new StereoSlam(rig.calibration,new SlamConfig())){
            try{dataset(Paths.get(args[2]),rig,slam);}
            finally{slam.writeTrajectory(output.resolve("trajectory.tum"));slam.writeMap(output.resolve("map.ply"));}
        }
    }
    private static void dataset(Path manifest,StereoRig rig,StereoSlam slam)throws IOException{
        Path root=manifest.toAbsolutePath().getParent();
        try(BufferedReader reader=Files.newBufferedReader(manifest)){
            String line;int number=0;
            while((line=reader.readLine())!=null){number++;line=line.trim();if(line.isEmpty()||line.startsWith("#"))continue;
                String[] fields=line.split("\\s+");if(fields.length!=3)throw new IOException("Invalid manifest line "+number);
                Mat l=Imgcodecs.imread(root.resolve(fields[1]).toString(),Imgcodecs.IMREAD_GRAYSCALE);
                Mat r=Imgcodecs.imread(root.resolve(fields[2]).toString(),Imgcodecs.IMREAD_GRAYSCALE);
                try{run(l,r,Double.parseDouble(fields[0]),rig,slam);}finally{l.release();r.release();}
            }
        }
    }
    private static void run(Mat left,Mat right,double time,StereoRig rig,StereoSlam slam){
        Mat l=new Mat(),r=new Mat();
        try{rig.rectify(left,right,l,r);StereoSlam.Result s=slam.process(l,r,time);
            log.debug("timestamp={} state={} inliers={} stereo={} keyframes={} loop={} mapFull={} position={}",
                time,s.state,s.inliers,s.stereoPoints,s.keyframes,s.loopClosed,s.mapFull,s.pose);
        }finally{l.release();r.release();}
    }
}
