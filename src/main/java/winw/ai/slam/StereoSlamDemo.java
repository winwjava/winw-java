package winw.ai.slam;

import java.io.*;
import java.nio.file.*;
import java.util.Locale;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;

/** Dataset manifest or a single hardware-synchronized side-by-side camera. */
public final class StereoSlamDemo {
    private StereoSlamDemo(){}
    public static void main(String[] args)throws Exception{
        if(args.length>=4&&(args[0].equals("camera")||args[0].equals("dataset-view"))){LiveStereoSlam.main(args);return;}
        if(args.length<4||(!args[0].equals("dataset")&&!args[0].equals("camera"))){
            System.out.println("dataset <calibration.properties> <manifest.txt> <output-directory>\n"
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
            System.out.printf(Locale.ROOT,"%.3f %s inliers=%d stereo=%d keyframes=%d loop=%s mapFull=%s position=%s%n",
                time,s.state,s.inliers,s.stereoPoints,s.keyframes,s.loopClosed,s.mapFull,s.pose);
        }finally{l.release();r.release();}
    }
}
