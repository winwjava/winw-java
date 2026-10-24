package winw.ai.slam;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import java.util.function.*;
import java.util.stream.*;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.features2d.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.video.Video;
import org.apache.commons.math3.linear.*;

/** Constrained natural-scene self-calibration: centered principal point, square pixels,
 * fixed focus and negligible distortion. Independent frames gate every export. */
public final class NaturalSceneCalibration implements AutoCloseable {
    /** Command line: sequence baseline-metres output [width height tolerance]. */
    public static void main(String[] args)throws Exception{
        if(args.length<3)throw new IllegalArgumentException("sequence baseline-metres output [width height tolerance]");
        OpenCvNative.load();Path sequence=Paths.get(args[0]),output=Paths.get(args[2]),log=sequence.resolve("self-calibration.log");
        int w=args.length>3?Integer.parseInt(args[3]):640,h=args.length>4?Integer.parseInt(args[4]):480;
        double tolerance=args.length>5?Double.parseDouble(args[5]):1.5;
        try(NaturalSceneCalibration estimator=new NaturalSceneCalibration(w,h,5000,tolerance)){
            estimator.run(sequence,Double.parseDouble(args[1]),output,log,()->false,System.out::println);
        }catch(Exception error){CalibrationRuntime.failure(log,error);throw error;}
    }
    private final int width,height;
    private final double tolerance;
    private final ORB orb;
    private final ORB lowContrastOrb;
    private final SIFT sift;
    private final org.opencv.imgproc.CLAHE contrast=org.opencv.imgproc.Imgproc.createCLAHE(2,new Size(8,8));
    private final BFMatcher matcher=BFMatcher.create(Core.NORM_HAMMING,false);
    private final BFMatcher siftMatcher=BFMatcher.create(Core.NORM_L2,false);
    private final List<Features> owned=new ArrayList<>();
    NaturalSceneCalibration(int width,int height,int features,double tolerance){
        this.width=width;this.height=height;this.tolerance=tolerance;orb=ORB.create(features);
        lowContrastOrb=ORB.create(features);lowContrastOrb.setFastThreshold(7);
        sift=SIFT.create(Math.min(3000,features),3,.015,10,1.6);
    }
    static final class Features implements AutoCloseable {
        final Mat descriptors=new Mat(),image=new Mat();final MatOfKeyPoint keys=new MatOfKeyPoint();
        final Mat siftDescriptors=new Mat();final MatOfKeyPoint siftKeys=new MatOfKeyPoint();
        boolean siftReady;String extraction="ORB";
        int originalFeatures;
        double brightness,sharpness;
        public void close(){descriptors.release();keys.release();image.release();siftDescriptors.release();siftKeys.release();}
    }
    static final class Matches {
        String method="ORB";
        final List<Point> a=new ArrayList<>(),b=new ArrayList<>();
        void add(Matches other,int limit){int step=Math.max(1,(other.a.size()+limit-1)/limit);
            for(int i=0;i<other.a.size();i+=step){a.add(other.a.get(i));b.add(other.b.get(i));}}
    }
    private Features features(Path file)throws IOException{
        Mat image=Imgcodecs.imread(file.toString(),Imgcodecs.IMREAD_GRAYSCALE);
        try{return features(image);}catch(IOException error){throw new IOException(file+": "+error.getMessage(),error);}finally{image.release();}
    }
    Features features(Mat image)throws IOException{
        Mat mask=new Mat();
        Features f=new Features();owned.add(f);
        try{if(image.empty()||image.cols()!=width||image.rows()!=height)throw new IOException("原始图像缺失或尺寸不一致");
            orb.detectAndCompute(image,mask,f.keys,f.descriptors);
            f.originalFeatures=f.descriptors.rows();
            if(f.descriptors.rows()<Math.max(500,orb.getMaxFeatures()/3)){
                Mat enhanced=new Mat(),descriptors=new Mat();MatOfKeyPoint keys=new MatOfKeyPoint();
                try{contrast.apply(image,enhanced);lowContrastOrb.detectAndCompute(enhanced,mask,keys,descriptors);
                    if(descriptors.rows()>f.descriptors.rows()){keys.copyTo(f.keys);descriptors.copyTo(f.descriptors);f.extraction="CLAHE+ORB-fast7";}
                }finally{enhanced.release();descriptors.release();keys.release();}
            }
            double scale=Math.min(1,960.0/Math.max(width,height));
            org.opencv.imgproc.Imgproc.resize(image,f.image,new Size(Math.round(width*scale),Math.round(height*scale)),0,0,org.opencv.imgproc.Imgproc.INTER_AREA);
            Mat laplacian=new Mat();MatOfDouble mean=new MatOfDouble(),deviation=new MatOfDouble();
            try{org.opencv.imgproc.Imgproc.Laplacian(image,laplacian,CvType.CV_64F);
                Core.meanStdDev(laplacian,mean,deviation);double sigma=deviation.toArray()[0];
                f.sharpness=sigma*sigma;f.brightness=Core.mean(image).val[0];
            }finally{laplacian.release();mean.release();deviation.release();}
            return f;
        }finally{mask.release();}
    }
    private Matches match(Features a,Features b){
        return match(a.descriptors,b.descriptors,a.keys.toArray(),b.keys.toArray(),matcher,"ORB");
    }
    private void ensureSift(Features f){
        if(f.siftReady)return;
        Mat enhanced=new Mat(),mask=new Mat();
        try{contrast.apply(f.image,enhanced);sift.detectAndCompute(enhanced,mask,f.siftKeys,f.siftDescriptors);
            KeyPoint[] points=f.siftKeys.toArray();for(KeyPoint p:points){p.pt.x*=((double)width/f.image.cols());p.pt.y*=((double)height/f.image.rows());}
            f.siftKeys.fromArray(points);f.siftReady=true;
        }finally{enhanced.release();mask.release();}
    }
    private Matches siftMatch(Features a,Features b){
        ensureSift(a);ensureSift(b);
        return match(a.siftDescriptors,b.siftDescriptors,a.siftKeys.toArray(),b.siftKeys.toArray(),siftMatcher,"CLAHE+SIFT");
    }
    Matches stereoMatch(Features a,Features b){
        Matches result=match(a,b);
        if(result.a.size()>=80&&coverage(result.a)&&coverage(result.b))return result;
        Matches fallback=siftMatch(a,b);
        // Selection uses descriptors/coverage only, never the calibration being validated.
        int coverage=Math.min(coverageCells(result.a),coverageCells(result.b));
        int fallbackCoverage=Math.min(coverageCells(fallback.a),coverageCells(fallback.b));
        if(fallback.a.size()>=60&&fallbackCoverage>=8&&(result.a.size()<60||coverage<8))return fallback;
        if(fallbackCoverage>=coverage&&fallback.a.size()>result.a.size())return fallback;
        return result;
    }
    private static Matches match(Mat ad,Mat bd,KeyPoint[] ak,KeyPoint[] bk,BFMatcher matcher,String method){
        Matches result=new Matches();result.method=method;if(ad.rows()<2||bd.rows()<2)return result;
        List<MatOfDMatch> forward=new ArrayList<>(),reverse=new ArrayList<>();
        try{matcher.knnMatch(ad,bd,forward,2);matcher.knnMatch(bd,ad,reverse,2);
            int[] back=new int[bd.rows()];Arrays.fill(back,-1);
            for(MatOfDMatch row:reverse){DMatch[] m=row.toArray();if(m.length==2&&m[0].distance<.75*m[1].distance)back[m[0].queryIdx]=m[0].trainIdx;}
            for(MatOfDMatch row:forward){DMatch[] m=row.toArray();if(m.length==2&&m[0].distance<.75*m[1].distance&&back[m[0].trainIdx]==m[0].queryIdx){
                result.a.add(ak[m[0].queryIdx].pt);result.b.add(bk[m[0].trainIdx].pt);}}
            return result;
        }finally{for(Mat m:forward)m.release();for(Mat m:reverse)m.release();}
    }
    private int coverageCells(List<Point> points){
        Set<Integer> cells=new HashSet<>();for(Point p:points)cells.add(Math.min(2,(int)(p.y*3/height))*4+Math.min(3,(int)(p.x*4/width)));
        return cells.size();
    }
    private boolean coverage(List<Point> points){return coverageCells(points)>=8;}
    /** Track spatially distributed corners, with a reverse pass to reject drift/occlusion. */
    static Matches track(Mat previous,Mat next){
        MatOfPoint corners=new MatOfPoint();Mat mask=new Mat();
        MatOfPoint2f a=new MatOfPoint2f(),b=new MatOfPoint2f(),back=new MatOfPoint2f();
        MatOfByte forwardStatus=new MatOfByte(),backStatus=new MatOfByte();
        MatOfFloat error=new MatOfFloat(),reverseError=new MatOfFloat();
        try{
            org.opencv.imgproc.Imgproc.goodFeaturesToTrack(previous,corners,1800,.008,7,mask,3,false,.04);
            Matches result=new Matches();if(corners.empty())return result;
            a.fromArray(corners.toArray());
            Video.calcOpticalFlowPyrLK(previous,next,a,b,forwardStatus,error,new Size(25,25),4);
            Video.calcOpticalFlowPyrLK(next,previous,b,back,backStatus,reverseError,new Size(25,25),4);
            Point[] pa=a.toArray(),pb=b.toArray(),pr=back.toArray();byte[] sa=forwardStatus.toArray(),sb=backStatus.toArray();
            float[] errors=error.toArray();
            for(int i=0;i<pa.length;i++)if(sa[i]!=0&&sb[i]!=0&&errors[i]<25&&Math.hypot(pa[i].x-pr[i].x,pa[i].y-pr[i].y)<.8
                    &&pb[i].x>=0&&pb[i].y>=0&&pb[i].x<next.cols()&&pb[i].y<next.rows()){
                result.a.add(pa[i]);result.b.add(pb[i]);}
            return result;
        }finally{for(Mat m:new Mat[]{corners,mask,a,b,back,forwardStatus,backStatus,error,reverseError})m.release();}
    }
    /** Include adjacent training frames; reserved frames are never tracking intermediates. */
    static List<int[]> temporalPairs(int count){
        List<Integer> training=new ArrayList<>();for(int i=0;i<count;i++)if(i%5!=0)training.add(i);
        List<int[]> pairs=new ArrayList<>();
        for(int i=0;i<training.size();i++)for(int gap:new int[]{1,2,3,5,8,12})if(i+gap<training.size())pairs.add(new int[]{training.get(i),training.get(i+gap)});
        return pairs;
    }
    /** Keep short contiguous bursts when bounding a long recording, not only far-apart frames. */
    static List<String> selectFrames(List<String> names){
        if(names.size()<=60)return new ArrayList<>(names);
        List<String> selected=new ArrayList<>();
        for(int burst=0;burst<12;burst++){
            int start=(int)((long)burst*(names.size()-5)/11);
            for(int i=0;i<5;i++)selected.add(names.get(start+i));
        }
        return selected;
    }
    private String geometryReason;
    private int geometryInliers;
    private double geometryPlaneFraction;
    private Mat rejectGeometry(String reason){geometryReason=reason;return new Mat();}
    private Mat temporalGeometry(Features a,Features b,String label,boolean near,Path log,Map<String,Integer> counts)throws IOException{
        long started=System.nanoTime();Matches points=match(a,b);Mat f=fundamental(points);
        try{
            logGeometry(log,label,"ORB",points,started,counts);
            if(f.empty()&&near){
                f.release();started=System.nanoTime();points=track(a.image,b.image);
                for(Point p:points.a){p.x*=((double)width/a.image.cols());p.y*=((double)height/a.image.rows());}
                for(Point p:points.b){p.x*=((double)width/b.image.cols());p.y*=((double)height/b.image.rows());}
                f=fundamental(points);
                logGeometry(log,label,"LK-forward-backward",points,started,counts);
            }
            if(f.empty()&&near){
                f.release();started=System.nanoTime();points=siftMatch(a,b);f=fundamental(points);
                logGeometry(log,label,points.method,points,started,counts);
            }
            return f;
        }catch(IOException|RuntimeException error){f.release();throw error;}
    }
    private void logGeometry(Path log,String label,String method,Matches points,long started,Map<String,Integer> counts)throws IOException{
        counts.merge(method+":"+geometryReason,1,Integer::sum);
        double[] movement=new double[points.a.size()];for(int i=0;i<movement.length;i++)movement[i]=Math.hypot(points.a.get(i).x-points.b.get(i).x,points.a.get(i).y-points.b.get(i).y);
        Arrays.sort(movement);
        CalibrationRuntime.log(log,String.format(Locale.ROOT,
            "PAIR %s method=%s matches=%d inliers=%d coverage=%d/12,%d/12 homographyFraction=%.3f motionP50=%.3f result=%s elapsedMs=%.1f",
            label,method,points.a.size(),geometryInliers,coverageCells(points.a),coverageCells(points.b),geometryPlaneFraction,
            movement.length==0?Double.NaN:movement[movement.length/2],geometryReason,(System.nanoTime()-started)/1e6));
    }
    /** Reject planar/pure-rotation observations before they can constrain focal length. */
    private Mat fundamental(Matches points){
        geometryReason="accepted";geometryInliers=0;geometryPlaneFraction=Double.NaN;
        if(points.a.size()<80)return rejectGeometry("too_few_matches");
        if(!coverage(points.a)||!coverage(points.b))return rejectGeometry("raw_coverage");
        MatOfPoint2f a=new MatOfPoint2f(),b=new MatOfPoint2f();Mat mask=new Mat(),hm=new Mat(),h=null,f=null;
        try{a.fromList(points.a);b.fromList(points.b);
            f=Calib3d.findFundamentalMat(a,b,Calib3d.FM_RANSAC,1.0,.999,mask);
            if(f.rows()!=3||f.cols()!=3){f.release();return rejectGeometry("fundamental_failed");}
            geometryInliers=Core.countNonZero(mask);
            if(geometryInliers<Math.max(60,points.a.size()*.6)){f.release();return rejectGeometry("ransac_inliers");}
            Matches good=new Matches();for(int i=0;i<points.a.size();i++)if(mask.get(i,0)[0]!=0){good.a.add(points.a.get(i));good.b.add(points.b.get(i));}
            a.fromList(good.a);b.fromList(good.b);h=Calib3d.findHomography(a,b,Calib3d.RANSAC,2,hm);
            if(!h.empty())geometryPlaneFraction=(double)Core.countNonZero(hm)/good.a.size();
            if(!coverage(good.a)||!coverage(good.b)){f.release();return rejectGeometry("inlier_coverage");}
            if(geometryPlaneFraction>.90){f.release();return rejectGeometry("homography_dominant");}
            Mat refined=Calib3d.findFundamentalMat(a,b,Calib3d.FM_8POINT);f.release();
            if(refined.rows()!=3||refined.cols()!=3||!Core.checkRange(refined)){refined.release();return rejectGeometry("refit_failed");}
            return refined;
        }finally{a.release();b.release();mask.release();hm.release();if(h!=null)h.release();if(f!=null)f.release();}
    }
    /** Clean each stereo observation before pooling; local models are only filters.
     * The pooled model must still pass the stricter geometry and held-out gates. */
    Matches stereoInliers(Matches points){
        Matches good=new Matches();good.method=points.method+"+RANSAC";
        if(points.a.size()!=points.b.size()||points.a.size()<16)return good;
        MatOfPoint2f a=new MatOfPoint2f(),b=new MatOfPoint2f();Mat mask=new Mat(),f=null;
        try{
            a.fromList(points.a);b.fromList(points.b);
            f=Calib3d.findFundamentalMat(a,b,Calib3d.FM_RANSAC,1.0,.999,mask);
            if(f.rows()!=3||f.cols()!=3||!Core.checkRange(f)||mask.total()!=points.a.size())return good;
            if(Core.countNonZero(mask)<Math.max(12,points.a.size()*.5))return good;
            for(int i=0;i<points.a.size();i++)if(mask.get(i,0)[0]!=0){
                good.a.add(points.a.get(i));good.b.add(points.b.get(i));
            }
            return good;
        }finally{a.release();b.release();mask.release();if(f!=null)f.release();}
    }
    static RealMatrix intrinsic(double focal,int width,int height){return MatrixUtils.createRealMatrix(new double[][]{{focal,0,width/2.},{0,focal,height/2.},{0,0,1}});}
    private static RealMatrix matrix(Mat m){double[][] v=new double[m.rows()][m.cols()];for(int i=0;i<m.rows();i++)for(int j=0;j<m.cols();j++)v[i][j]=m.get(i,j)[0];return MatrixUtils.createRealMatrix(v);}
    private static Mat mat(RealMatrix matrix){Mat m=new Mat(matrix.getRowDimension(),matrix.getColumnDimension(),CvType.CV_64F);for(int i=0;i<m.rows();i++)m.put(i,0,matrix.getRow(i));return m;}
    static double score(List<RealMatrix> fundamentals,double focal,int w,int h){
        RealMatrix k=intrinsic(focal,w,h);double[] errors=new double[fundamentals.size()];int i=0;
        for(RealMatrix f:fundamentals){double[] s=new SingularValueDecomposition(k.transpose().multiply(f).multiply(k)).getSingularValues();
            errors[i++]=Math.pow((s[0]-s[1])/Math.max(1e-15,s[0]+s[1]),2);}
        Arrays.sort(errors);double sum=0;int n=Math.max(1,(int)Math.ceil(errors.length*.8));for(i=0;i<n;i++)sum+=errors[i];return sum/n;
    }
    /** Multi-view essential-matrix singular-value constraint; reject unobservable focal length. */
    static double estimateFocal(List<RealMatrix> fundamentals,int w,int h)throws IOException{
        return estimateFocal(fundamentals,w,h,null,"");
    }
    private static double estimateFocal(List<RealMatrix> fundamentals,int w,int h,Path log,String eye)throws IOException{
        if(fundamentals.size()<6)throw new IOException("时序几何约束不足：接受 "+fundamentals.size()+" 组，需要至少 6 组；查看 PAIR 和 TEMPORAL_SUMMARY 中的拒绝原因");
        double lo=.3*Math.max(w,h),hi=3*Math.max(w,h),best=0,cost=Double.POSITIVE_INFINITY;
        for(int i=0;i<=240;i++){double f=lo*Math.pow(hi/lo,i/240.);double value=score(fundamentals,f,w,h);if(value<cost){cost=value;best=f;}}
        double a=best/1.02,b=best*1.02;
        for(int i=0;i<35;i++){double x=a+(b-a)/3,y=b-(b-a)/3;if(score(fundamentals,x,w,h)<score(fundamentals,y,w,h))b=y;else a=x;}
        best=(a+b)/2;cost=score(fundamentals,best,w,h);
        double separation=Math.min(score(fundamentals,best*.8,w,h),score(fundamentals,best*1.2,w,h))-cost;
        if(log!=null)CalibrationRuntime.log(log,"FOCAL eye="+eye+" constraints="+fundamentals.size()+" f="+best+" residual="+cost+" separation="+separation+" searchRange="+lo+","+hi);
        if(best<lo*1.05||best>hi/1.05||cost>.0025||separation<.00001)throw new IOException("焦距估计退化或不稳定：f="+best+", residual="+cost+", separation="+separation+"；增加平移及多方向转动，避免平面、纯平移、纯转动和动态人物");
        return best;
    }
    private static void check(BooleanSupplier cancelled)throws IOException{if(cancelled.getAsBoolean()||Thread.currentThread().isInterrupted())throw new IOException("标定已取消");}
    void run(Path sequence,double baseline,Path destination,Path log,BooleanSupplier cancelled,Consumer<String> progress)throws IOException{
        long started=System.nanoTime();
        CalibrationRuntime.log(log,"BEGIN run="+UUID.randomUUID()+" sequence="+sequence.toAbsolutePath()+" output="+destination.toAbsolutePath()
            +" size="+width+"x"+height+" baselineMetres="+baseline+" features="+orb.getMaxFeatures()+" validationTolerance="+tolerance
            +" OpenCV="+Core.VERSION+" Java="+System.getProperty("java.version"));
        if(!Double.isFinite(baseline)||baseline<=0||!Double.isFinite(tolerance)||tolerance<=0)throw new IOException("基线和验证阈值必须为正数");
        CalibrationRuntime.log(log,"Java/OpenCV natural scene calibration; centered principal point, square pixels, zero distortion model");
        List<String> names;
        try(Stream<Path> files=Files.list(sequence.resolve("images/left"))){names=files.filter(p->p.toString().endsWith(".png")).map(p->p.getFileName().toString()).sorted().collect(Collectors.toList());}
        Set<String> rightNames;
        try(Stream<Path> files=Files.list(sequence.resolve("images/right"))){rightNames=files.filter(p->p.toString().endsWith(".png")).map(p->p.getFileName().toString()).collect(Collectors.toSet());}
        if(names.size()<25||!rightNames.equals(new HashSet<>(names)))throw new IOException("需要至少 25 对同名同步原始图像");
        List<String> selected=selectFrames(names);int count=selected.size();
        CalibrationRuntime.log(log,"SEQUENCE recordedPairs="+names.size()+" selectedPairs="+count+" heldoutEvery=5 selection="+selected);
        CalibrationRuntime.log(log,"GATES matches>=80 inliers>=60 inlierFraction>=0.60 coverage>=8/12 homographyFraction<=0.90 RANSAC=1px; no threshold relaxation");
        List<Features> left=new ArrayList<>(),right=new ArrayList<>();List<Matches> stereo=new ArrayList<>();
        for(int i=0;i<count;i++){check(cancelled);progress.accept("Java/OpenCV 提取特征 "+(i+1)+"/"+count);
            left.add(features(sequence.resolve("images/left").resolve(selected.get(i))));right.add(features(sequence.resolve("images/right").resolve(selected.get(i))));
            stereo.add(stereoMatch(left.get(i),right.get(i)));
            Features l=left.get(i),r=right.get(i);
            CalibrationRuntime.log(log,String.format(Locale.ROOT,"FRAME %s heldout=%s features=%d,%d stereoMatches=%d brightness=%.1f,%.1f laplacianVariance=%.1f,%.1f extraction=%s,%s stereoMethod=%s siftFeatures=%d,%d rawORB=%d,%d",
                selected.get(i),i%5==0,l.descriptors.rows(),r.descriptors.rows(),stereo.get(i).a.size(),l.brightness,r.brightness,l.sharpness,r.sharpness,l.extraction,r.extraction,stereo.get(i).method,l.siftDescriptors.rows(),r.siftDescriptors.rows(),l.originalFeatures,r.originalFeatures));}
        int sufficientStereo=0,sufficientHeldout=0;double[] stereoCounts=new double[count];
        for(int i=0;i<count;i++){Matches m=stereo.get(i);stereoCounts[i]=m.a.size();
            if(m.a.size()>=60&&coverage(m.a)&&coverage(m.b)){sufficientStereo++;if(i%5==0)sufficientHeldout++;}}
        Arrays.sort(stereoCounts);
        CalibrationRuntime.log(log,"STEREO_SUMMARY pairs="+count+" medianMatches="+stereoCounts[count/2]+" sufficientMatchesAndCoverage="+sufficientStereo
            +" eligibleHeldout="+sufficientHeldout+"/"+((count+4)/5)+"; eligibility uses descriptors only, final geometry still unverified");
        List<RealMatrix> fl=new ArrayList<>(),fr=new ArrayList<>();Matches pooled=new Matches();int training=0;
        progress.accept("Java/OpenCV 跨时刻匹配与退化检查…");
        for(int i=0;i<count;i++)if(i%5!=0){
            check(cancelled);Matches raw=stereo.get(i),clean=stereoInliers(raw);
            CalibrationRuntime.log(log,"STEREO_FILTER frame="+selected.get(i)+" raw="+raw.a.size()
                +" retained="+clean.a.size()+" result="+(clean.a.isEmpty()?"rejected":"accepted"));
            if(!clean.a.isEmpty()){pooled.add(clean,250);training++;}
        }
        CalibrationRuntime.log(log,"STEREO_POOL acceptedFrames="+training+" trainingFrames="+(count-(count+4)/5)
            +" filteredMatches="+pooled.a.size()+"; heldout frames excluded");
        Map<String,Integer> leftCounts=new TreeMap<>(),rightCounts=new TreeMap<>();
        List<int[]> candidates=temporalPairs(count);int completed=0;
        for(int[] pair:candidates){check(cancelled);int i=pair[0],j=pair[1];
            for(int eye=0;eye<2;eye++){
                List<Features> frames=eye==0?left:right;
                String label="eye="+(eye==0?"left":"right")+" from="+selected.get(i)+" to="+selected.get(j);
                boolean near=names.indexOf(selected.get(j))-names.indexOf(selected.get(i))<=3;
                Mat f=temporalGeometry(frames.get(i),frames.get(j),label,near,log,eye==0?leftCounts:rightCounts);
                try{if(!f.empty()){(eye==0?fl:fr).add(matrix(f));CalibrationRuntime.log(log,"CONSTRAINT "+label+" F="+text(f));}}finally{f.release();}
            }
            progress.accept("Java/OpenCV 时序匹配 "+(++completed)+"/"+candidates.size()+"；有效约束 左 "+fl.size()+" / 右 "+fr.size());
        }
        CalibrationRuntime.log(log,"TEMPORAL_SUMMARY candidatesPerEye="+candidates.size()+" acceptedLeft="+fl.size()+" acceptedRight="+fr.size()+" left="+leftCounts+" right="+rightCounts);
        if(fl.size()<6||fr.size()<6)throw new IOException("时序几何约束不足：左 "+fl.size()+" / 右 "+fr.size()+"（各需至少 6）；具体筛选原因已输出到控制台和 self-calibration.log");
        if(training<3)throw new IOException("逐帧筛选后双目有效画面不足："+training+"（至少 3 帧），请查看 STEREO_FILTER 日志");
        check(cancelled);progress.accept("Java/OpenCV 多帧焦距估计与双目位姿恢复…");

        long stereoStarted=System.nanoTime();Mat f=fundamental(pooled);
        try{logGeometry(log,"stereo-pooled","ORB-or-SIFT",pooled,stereoStarted,new TreeMap<>());}catch(IOException error){f.release();throw error;}
        if(f.empty()){f.release();throw new IOException("双目几何估计被拒绝："+geometryReason+"，内点 "+geometryInliers+"/"+pooled.a.size()+"；查看 STEREO_FILTER 和 stereo-pooled 日志");}
        double[] focal;
        try{CalibrationRuntime.log(log,"STEREO_CONSTRAINT F="+text(f));focal=JointFocalEstimator.estimate(fl,fr,matrix(f),width,height,log);}
        catch(IOException|RuntimeException error){f.release();throw error;}
        double lf=focal[0],rf=focal[1];
        CalibrationRuntime.log(log,"training pairs="+training+"; temporal constraints="+fl.size()+","+fr.size()+"; focal="+lf+","+rf);
        Mat kl=mat(intrinsic(lf,width,height)),kr=mat(intrinsic(rf,width,height)),d=Mat.zeros(1,5,CvType.CV_64F);
        Mat r=new Mat(),t=new Mat(),e=null,identity=Mat.eye(3,3,CvType.CV_64F),poseMask=new Mat();
        MatOfPoint2f a=new MatOfPoint2f(),b=new MatOfPoint2f(),na=new MatOfPoint2f(),nb=new MatOfPoint2f();
        Mat rl=new Mat(),rr=new Mat(),pl=new Mat(),pr=new Mat(),q=new Mat();
        try{
            if(f.empty())throw new IOException("双目几何不稳定：纹理覆盖不足、场景接近平面或左右不同步");
            e=mat(matrix(kr).transpose().multiply(matrix(f)).multiply(matrix(kl)));
            a.fromList(pooled.a);b.fromList(pooled.b);Calib3d.undistortPoints(a,na,kl,d);Calib3d.undistortPoints(b,nb,kr,d);
            // Select geometrically consistent points before cheirality recovery.
            Mat essentialMask=new Mat(),fit=null;
            try{fit=Calib3d.findEssentialMat(na,nb,identity,Calib3d.RANSAC,.999,1.5/Math.max(lf,rf),2000,essentialMask);
                if(fit.empty())throw new IOException("本质矩阵估计失败");essentialMask.copyTo(poseMask);
                // Distances are in unit-baseline coordinates: the default 50 would
                // discard valid indoor points beyond 3 m on a 61 mm camera.
                int positive=Calib3d.recoverPose(e,na,nb,identity,r,t,1000,poseMask);
                CalibrationRuntime.log(log,"POSE essentialInliers="+Core.countNonZero(essentialMask)+" positiveDepth="+positive+" pooledMatches="+pooled.a.size());
                if(positive<60||positive<.6*Core.countNonZero(essentialMask))throw new IOException("三角化正深度不足，检查左右眼顺序与场景深度");
            }finally{essentialMask.release();if(fit!=null)fit.release();}
            Core.multiply(t,new Scalar(baseline/Core.norm(t)),t);
            CalibrationRuntime.log(log,"RIG R="+text(r)+" T="+text(t));
            if(t.get(0,0)[0]>=0||Math.abs(t.get(1,0)[0])>.3*baseline||Math.abs(t.get(2,0)[0])>.3*baseline)throw new IOException("非水平双目或左右眼顺序错误");
            Calib3d.stereoRectify(kl,d,kr,d,new Size(width,height),r,t,rl,rr,pl,pr,q,Calib3d.CALIB_ZERO_DISPARITY,0,new Size(width,height));
            int passed=0,total=0;
            for(int i=0;i<count;i+=5){check(cancelled);total++;Matches m=stereo.get(i);
                double error=Double.POSITIVE_INFINITY,positive=0;
                if(m.a.size()>=60&&coverage(m.a)&&coverage(m.b)){
                    a.fromList(m.a);b.fromList(m.b);Calib3d.undistortPoints(a,na,kl,d,rl,pl);Calib3d.undistortPoints(b,nb,kr,d,rr,pr);
                    Point[] ap=na.toArray(),bp=nb.toArray();double[] errors=new double[ap.length];int good=0;
                    for(int n=0;n<ap.length;n++){errors[n]=Math.abs(ap[n].y-bp[n].y);if(ap[n].x-bp[n].x>.25)good++;}
                    Arrays.sort(errors);error=errors[(int)(.8*(errors.length-1))];positive=(double)good/errors.length;
                }
                boolean ok=Double.isFinite(error)&&error<=tolerance&&positive>=.85;if(ok)passed++;
                CalibrationRuntime.log(log,"HELDOUT "+selected.get(i)+" matches="+m.a.size()+" verticalP80="+error+" positive="+positive+" passed="+ok);
            }
            CalibrationRuntime.log(log,"VALIDATION_SUMMARY passed="+passed+" total="+total+" tolerance="+tolerance);
            if(passed<3||passed<.6*total)throw new IOException("独立验证仅通过 "+passed+"/"+total+"，未导出标定；请增加场景纹理与运动，明显畸变需提供实测标定");
            check(cancelled);Properties p=new Properties();p.setProperty("rectified","false");p.setProperty("width",""+width);p.setProperty("height",""+height);
            p.setProperty("left.K",text(kl));p.setProperty("right.K",text(kr));p.setProperty("left.D",text(d));p.setProperty("right.D",text(d));p.setProperty("R",text(r));p.setProperty("T",text(t));
            p.setProperty("calibration.method","java-opencv-constrained-self-calibration");p.setProperty("calibration.assumptions","centered principal points; square pixels; negligible distortion; fixed focus");
            p.setProperty("validation.passedPairs",""+passed);p.setProperty("validation.totalPairs",""+total);p.setProperty("validation.tolerancePixels",""+tolerance);
            try(Writer writer=Files.newBufferedWriter(destination,StandardOpenOption.CREATE_NEW)){p.store(writer,"Estimated natural scene calibration; validate scale against independent distances");}
            CalibrationRuntime.log(log,"SUCCESS calibration="+destination+" elapsedSeconds="+(System.nanoTime()-started)/1e9);
        }finally{for(Mat m:new Mat[]{kl,kr,d,f,r,t,e,identity,poseMask,a,b,na,nb,rl,rr,pl,pr,q})if(m!=null)m.release();}
    }
    private static String text(Mat m){StringJoiner text=new StringJoiner(",");for(int i=0;i<m.rows();i++)for(int j=0;j<m.cols();j++)text.add(Double.toString(m.get(i,j)[0]));return text.toString();}
    @Override public void close(){for(Features f:owned)f.close();owned.clear();}
}
