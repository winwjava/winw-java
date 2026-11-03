package winw.ai.slam;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;
import org.opencv.calib3d.Calib3d;
import org.opencv.imgcodecs.Imgcodecs;

/** Reproducible replay. Candidate mode is explicitly experimental, with baseline units, not metres. */
public final class RecordingSlamEvaluation {
    public static void main(String[] args)throws Exception {
        if(args.length!=4)throw new IllegalArgumentException("<manifest> <auto:baselineMetres | calibration.properties | candidate:focalPixels> <output> <sift|orb|orb-descriptor>");
        OpenCvNative.load();Core.setRNGSeed(42);
        Path manifest=Paths.get(args[0]).toAbsolutePath(),root=manifest.getParent(),out=Paths.get(args[2]);
        Files.createDirectories(out);
        List<String[]> rows=new ArrayList<>();
        for(String line:Files.readAllLines(manifest)){line=line.trim();if(line.isEmpty()||line.startsWith("#"))continue;
            String[] f=line.split("\\s+");if(f.length!=3)throw new IOException("Invalid manifest: "+line);rows.add(f);}
        if(rows.isEmpty())throw new IOException("Empty sequence");
        boolean automatic=args[1].startsWith("auto:");
        boolean candidate=args[1].startsWith("candidate:");
        double baseline=automatic?Double.parseDouble(args[1].substring(5)):1;
        if(!Double.isFinite(baseline)||baseline<=0)throw new IllegalArgumentException("Positive baseline required");
        double focal=0;
        if(automatic){Mat image=Imgcodecs.imread(root.resolve(rows.get(0)[1]).toString(),0);
            try{if(image.empty())throw new IOException("Missing first image");focal=image.cols()*.8;}finally{image.release();}}
        Path calibration=automatic?candidate(root,rows,focal,baseline,out):candidate?candidate(root,rows,Double.parseDouble(args[1].substring(10)),1,out):Paths.get(args[1]);
        SlamConfig config=new SlamConfig();config.useSift=args[3].equals("sift");
        config.useOpticalFlow=!args[3].equals("orb-descriptor");
        if(!config.useSift&&!args[3].equals("orb")&&!args[3].equals("orb-descriptor"))throw new IllegalArgumentException("Expected sift or orb");
        config.features=3500;config.maxKeyframes=200;config.keyframeInterval=2;
        if(candidate||automatic){config.minDepth=.1*baseline;config.maxDepth=1000*baseline;config.keyframeTranslation=2*baseline;}
        int valid=0,heldout=0,heldoutValid=0,loops=0;long start=System.nanoTime();
        List<Double> elapsed=new ArrayList<>();
        try(StereoRig rig=StereoRig.load(calibration);StereoSlam slam=new StereoSlam(rig.calibration,config);
            BufferedWriter csv=Files.newBufferedWriter(out.resolve("frames.csv"))){
            csv.write("index,timestamp,split,state,inliers,stereo_points,keyframes,loop,ms,tx,ty,tz\n");
            for(int i=0;i<rows.size();i++){
                String[] f=rows.get(i);Mat l=Imgcodecs.imread(root.resolve(f[1]).toString(),0),r=Imgcodecs.imread(root.resolve(f[2]).toString(),0),a=new Mat(),b=new Mat();
                try{long t=System.nanoTime();if(candidate||automatic){
                        Properties warp=new Properties();try(Reader reader=Files.newBufferedReader(out.resolve("vertical-warp.properties"))){warp.load(reader);}
                        String[] v=warp.getProperty("matrix").split(",");Mat h=Mat.eye(3,3,CvType.CV_64F);
                        for(int j=0;j<9;j++)h.put(j/3,j%3,Double.parseDouble(v[j]));
                        try{org.opencv.imgproc.Imgproc.warpPerspective(r,b,h,r.size());l.copyTo(a);}finally{h.release();}
                    }else rig.rectify(l,r,a,b);StereoSlam.Result result=slam.process(a,b,Double.parseDouble(f[0]));
                    double ms=(System.nanoTime()-t)/1e6;elapsed.add(ms);boolean test=i%5==0;if(test)heldout++;
                    if(result.pose!=null){valid++;if(test)heldoutValid++;}if(result.loopClosed)loops++;
                    double[] p=result.pose==null?new double[]{Double.NaN,Double.NaN,Double.NaN}:result.pose.translation();
                    csv.write(String.format(Locale.ROOT,"%d,%s,%s,%s,%d,%d,%d,%s,%.3f,%.9f,%.9f,%.9f%n",i,f[0],test?"heldout-calibration":"training-calibration",result.state,result.inliers,result.stereoPoints,result.keyframes,result.loopClosed,ms,p[0],p[1],p[2]));csv.flush();
                }finally{l.release();r.release();a.release();b.release();}
            }
            slam.writeTrajectory(out.resolve("trajectory.tum"));slam.writeMap(out.resolve("map.ply"));
            if(candidate||automatic){Path trajectory=out.resolve("trajectory.tum");Files.writeString(trajectory,Files.readString(trajectory).replace("metres",automatic?"nominal metres from measured baseline; projective approximation":"EXPERIMENTAL baseline units; projective approximation"));}
        }
        Collections.sort(elapsed);
        String report="Frames: "+rows.size()+"\nValid poses: "+valid+"\nLost/uninitialized: "+(rows.size()-valid)+"\nCalibration-heldout valid: "+heldoutValid+"/"+heldout+"\nVerified loops: "+loops+"\nMedian ms: "+elapsed.get(elapsed.size()/2)+"\nTotal seconds: "+(System.nanoTime()-start)/1e9+"\n"+
            (automatic?"Image-only CPU mode: fitted vertical alignment, image-width focal prior (not measured focal), measured baseline; nominal metres, unverified geometry/metric accuracy.\n":candidate?"EXPERIMENTAL: projective vertical alignment, assumed focal, centered principal points, zero distortion, baseline=1. Coordinates are baseline units; NOT validated metric calibration.\n":"Uses supplied calibration.\n")+
            "Heldout refers only to calibration fitting. Replay is sequential online mapping, not independent localization against a frozen map. No ground truth: ATE/RPE and scale accuracy unknown.\n";
        Files.writeString(out.resolve("report.txt"),report);System.out.println(report);
    }
    private static Path candidate(Path root,List<String[]> rows,double focal,double baseline,Path out)throws Exception {
        if(!Double.isFinite(focal)||focal<=0)throw new IllegalArgumentException("Positive focal required");
        Mat first=Imgcodecs.imread(root.resolve(rows.get(0)[1]).toString(),0);int w=first.cols(),h=first.rows();first.release();
        if(w==0||h==0)throw new IOException("Missing first frame");
        NaturalSceneCalibration.Matches pooled=new NaturalSceneCalibration.Matches();
        try(NaturalSceneCalibration features=new NaturalSceneCalibration(w,h,3500,1.5)){
            // A bounded set of training-only stereo pairs; heldout images never enter this fit.
            for(int i=1;i<rows.size();i+=7){if(i%5==0)continue;String[] row=rows.get(i);
                Mat l=Imgcodecs.imread(root.resolve(row[1]).toString(),0),r=Imgcodecs.imread(root.resolve(row[2]).toString(),0);
                try(NaturalSceneCalibration.Features a=features.features(l);NaturalSceneCalibration.Features b=features.features(r)){
                    pooled.add(features.stereoInliers(features.stereoMatch(a,b)),250);
                }finally{l.release();r.release();}
            }
        }
        if(pooled.a.size()<60)throw new IOException("Insufficient stereo geometry");
        // Restricted vertical alignment preserves horizontal disparity. This is a projective
        // diagnostic model and deliberately does not claim a physical stereo calibration.
        MatOfPoint2f a=new MatOfPoint2f(),b=new MatOfPoint2f();
        List<Point> targets=new ArrayList<>();for(int i=0;i<pooled.a.size();i++)targets.add(new Point(pooled.b.get(i).x,pooled.a.get(i).y));
        a.fromList(pooled.b);b.fromList(targets);Mat mask=new Mat(),affine=null,hwarp=Mat.eye(3,3,CvType.CV_64F);
        try{affine=Calib3d.estimateAffine2D(a,b,mask,Calib3d.RANSAC,1.5,3000,.999,10);
            if(affine.empty()||Core.countNonZero(mask)<60)throw new IOException("Vertical alignment failed");
            for(int j=0;j<3;j++)hwarp.put(1,j,affine.get(1,j)[0]);
            Properties warp=new Properties();warp.setProperty("matrix",text(hwarp));
            try(Writer writer=Files.newBufferedWriter(out.resolve("vertical-warp.properties"))){warp.store(writer,"Training-only projective vertical alignment");}
            Properties p=new Properties();p.setProperty("rectified","true");p.setProperty("width",""+w);p.setProperty("height",""+h);
            p.setProperty("fx",""+focal);p.setProperty("fy",""+focal);p.setProperty("cx",""+(w/2.));p.setProperty("cy",""+(h/2.));p.setProperty("rightCx",""+(w/2.));p.setProperty("baseline",""+baseline);
            p.setProperty("calibration.method","EXPERIMENTAL-projective-image-geometry");
            Path file=out.resolve("experimental-calibration.properties");try(Writer writer=Files.newBufferedWriter(file)){p.store(writer,"NOT physical calibration; diagnostic replay requires vertical-warp.properties");}return file;
        }finally{for(Mat m:new Mat[]{a,b,mask,affine,hwarp})if(m!=null)m.release();}
    }
    private static String text(Mat m){StringJoiner s=new StringJoiner(",");for(int i=0;i<m.rows();i++)for(int j=0;j<m.cols();j++)s.add(""+m.get(i,j)[0]);return s.toString();}
}
