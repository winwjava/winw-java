package winw.ai.slam;

import lombok.extern.slf4j.Slf4j;

import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.features2d.*;
import org.opencv.imgproc.Imgproc;

/** Single-threaded sparse stereo SLAM. Input must be synchronized and rectified. */
@Slf4j
public final class StereoSlam implements AutoCloseable {
    public enum State { INITIALIZING, TRACKING, RELOCALIZED, LOST }
    public static final class Result {
        public final State state;
        /** Null when tracking failed; never returns an old pose as a new estimate. */
        public final Pose pose;
        public final int inliers, stereoPoints, keyframes;
        public final boolean loopClosed, mapFull;
        Result(State s,Pose p,int n,int depth,int keys,boolean loop,boolean full){
            state=s;pose=p;inliers=n;stereoPoints=depth;keyframes=keys;loopClosed=loop;mapFull=full;
        }
    }
    private static final class Frame implements AutoCloseable {
        KeyPoint[] keys;
        Mat descriptors=new Mat(),image=new Mat();
        double[][] points;
        @Override public void close(){descriptors.release();image.release();}
    }
    private static final class Keyframe {
        Frame frame; int id, sequence;
        Keyframe(Frame f,int id,int seq){frame=f;this.id=id;sequence=seq;}
    }
    private static final class Estimate {
        Keyframe reference;Pose pose;int inliers;
        Estimate(Keyframe ref,Pose p,int n){reference=ref;pose=p;inliers=n;}
    }
    private static final class Sample {
        double time;int anchor;Pose relative;boolean startsSegment;
        Sample(double time,int anchor,Pose relative){this.time=time;this.anchor=anchor;this.relative=relative;}
    }
    private final StereoCalibration calibration;
    private final SlamConfig config;
    private final ORB orb;
    private final SIFT sift;
    private final BFMatcher matcher;
    private final Mat intrinsics;
    private final MatOfDouble distortion=new MatOfDouble(0,0,0,0,0);
    private final List<Keyframe> keyframes=new ArrayList<>();
    private final List<Sample> trajectory=new ArrayList<>();
    private final PoseGraph graph=new PoseGraph();
    private int sequence;
    private double lastTime=Double.NEGATIVE_INFINITY;
    private boolean closed;
    private long lastDiagnosticNanos;
    private State state=State.INITIALIZING;

    public StereoSlam(StereoCalibration calibration,SlamConfig config){
        config.validate();this.calibration=Objects.requireNonNull(calibration);this.config=config.copy();
        orb=ORB.create(config.features);orb.setFastThreshold(7);
        sift=SIFT.create(config.features,3,.01,10,1.6);
        matcher=BFMatcher.create(config.useSift?Core.NORM_L2:Core.NORM_HAMMING,false);
        intrinsics=calibration.cameraMatrix();
        log.info("SLAM calibration per-eye={}x{} fx={} fy={} cx={} cy={} rightCx={} baseline={}m; features={} minInliers={} epipolarTolerance={}px depth=[{},{}]m",calibration.width,calibration.height,calibration.fx,calibration.fy,calibration.cx,calibration.cy,calibration.rightCx,calibration.baseline,config.features,config.minInliers,config.epipolarTolerance,config.minDepth,config.maxDepth);
    }
    public Result process(Mat left,Mat right,double timestamp){
        if(closed)throw new IllegalStateException("Engine closed");
        if(!Double.isFinite(timestamp)||timestamp<=lastTime)throw new IllegalArgumentException("Timestamps must strictly increase");
        validate(left);validate(right);
        Frame frame=extract(left,right);boolean retained=false;
        lastTime=timestamp;sequence++;
        try {
            int depth=0;for(double[] p:frame.points)if(p!=null)depth++;
            if(keyframes.isEmpty()){
                if(depth<config.minInliers)return result(State.INITIALIZING,null,0,depth,false);
                addKeyframe(frame,Pose.identity());retained=true;
                trajectory.add(new Sample(timestamp,0,Pose.identity()));
                return result(State.TRACKING,Pose.identity(),depth,depth,false);
            }
            // Track against recent keyframe maps, then search older maps if tracking is lost.
            Estimate best=null;int recent=Math.max(0,keyframes.size()-3);
            for(int i=keyframes.size()-1;i>=recent;i--)best=better(best,estimate(keyframes.get(i),frame));
            boolean recovered=state==State.LOST;
            if(best==null){
                for(int i=0;i<recent;i++)best=better(best,estimate(keyframes.get(i),frame));
                recovered=best!=null;
            }
            if(best==null)return result(State.LOST,null,0,depth,false);
            Pose pose=best.pose;boolean loop=false;
            Keyframe latest=keyframes.get(keyframes.size()-1);
            boolean insert=depth>=config.minInliers&&keyframes.size()<config.maxKeyframes
                &&(sequence-latest.sequence>=config.keyframeInterval
                    || pose.distance(graph.pose(latest.id))>=config.keyframeTranslation
                    || (sequence-latest.sequence>=2&&best.inliers<config.minInliers*3));
            int anchor=best.reference.id;
            if(insert){
                // A loop requires two consecutive frames agreeing with the same old map.
                Estimate candidate=null;
                for(Keyframe old:keyframes)if(sequence-old.sequence>=config.loopGap&&old.id<recent)
                    candidate=better(candidate,estimate(old,frame));
                boolean verified=false;
                if(candidate!=null&&pendingLoop==candidate.reference.id){
                    Pose expected=pendingLoopPose.inverse().multiply(candidate.pose);
                    Pose observed=pendingTrackingPose.inverse().multiply(pose);
                    double[] error=expected.inverse().multiply(observed).vector();
                    double rotation=Math.sqrt(error[0]*error[0]+error[1]*error[1]+error[2]*error[2]);
                    double translation=Math.sqrt(error[3]*error[3]+error[4]*error[4]+error[5]*error[5]);
                    verified=rotation<.1&&translation<.2;
                }
                int id=addKeyframe(frame,pose);retained=true;anchor=id;
                graph.addEdge(latest.id,id,graph.pose(latest.id).inverse().multiply(pose),false);
                if(verified){
                    graph.addEdge(candidate.reference.id,id,
                        graph.pose(candidate.reference.id).inverse().multiply(candidate.pose),true);
                    graph.optimize(12);pose=graph.pose(id);loop=true;
                }
                pendingLoop=-1;
            } else {
                Estimate candidate=null;
                for(Keyframe old:keyframes)if(sequence-old.sequence>=config.loopGap&&old.id<recent)
                    candidate=better(candidate,estimate(old,frame));
                pendingLoop=candidate==null?-1:candidate.reference.id;
                pendingLoopPose=candidate==null?null:candidate.pose;pendingTrackingPose=pose;
            }
            Sample sample=new Sample(timestamp,anchor,graph.pose(anchor).inverse().multiply(pose));
            sample.startsSegment=recovered;
            trajectory.add(sample);
            return result(recovered?State.RELOCALIZED:State.TRACKING,pose,best.inliers,depth,loop);
        }finally{if(!retained)frame.close();}
    }
    private int pendingLoop=-1;
    private Pose pendingLoopPose,pendingTrackingPose;
    private Result result(State next,Pose pose,int n,int depth,boolean loop){
        long now=System.nanoTime();
        if(next!=state||sequence==1||now-lastDiagnosticNanos>=2_000_000_000L){
            if(pose==null)log.warn("SLAM frame={} state={} -> {} stereoPoints={} required={} inliers={} keyframes={}; no pose, occupancy fusion skipped",sequence,state,next,depth,config.minInliers,n,keyframes.size());
            else log.info("SLAM frame={} state={} -> {} stereoPoints={} inliers={} keyframes={} loop={} pose={}",sequence,state,next,depth,n,keyframes.size(),loop,pose);
            lastDiagnosticNanos=now;
        }
        state=next;if(next==State.LOST)pendingLoop=-1;
        return new Result(next,pose,n,depth,keyframes.size(),loop,keyframes.size()>=config.maxKeyframes);
    }
    private int addKeyframe(Frame frame,Pose pose){int id=graph.addPose(pose);keyframes.add(new Keyframe(frame,id,sequence));return id;}
    private Estimate better(Estimate a,Estimate b){return b!=null&&(a==null||b.inliers>a.inliers)?b:a;}
    private void validate(Mat m){
        if(m.empty()||m.cols()!=calibration.width||m.rows()!=calibration.height||m.depth()!=CvType.CV_8U
            ||(m.channels()!=1&&m.channels()!=3&&m.channels()!=4))
            throw new IllegalArgumentException("Image must be calibrated-size 8-bit gray/BGR/BGRA");
    }
    private Mat gray(Mat input){Mat out=new Mat();if(input.channels()==1)input.copyTo(out);
        else Imgproc.cvtColor(input,out,input.channels()==3?Imgproc.COLOR_BGR2GRAY:Imgproc.COLOR_BGRA2GRAY);return out;}
    private Frame extract(Mat left,Mat right){
        Frame f=new Frame();Mat l=gray(left),r=gray(right),rd=new Mat(),mask=new Mat();
        MatOfKeyPoint lk=new MatOfKeyPoint(),rk=new MatOfKeyPoint();
        try{
            Feature2D extractor=config.useSift?sift:orb;
            extractor.detectAndCompute(l,mask,lk,f.descriptors);extractor.detectAndCompute(r,mask,rk,rd);
            l.copyTo(f.image);
            f.keys=lk.toArray();KeyPoint[] rightKeys=rk.toArray();f.points=new double[f.keys.length][];
            List<DMatch> stereoMatches=matches(f.descriptors,rd);
            int verticalRejected=0,disparityRejected=0,depthRejected=0,accepted=0;
            double verticalSum=0,disparitySum=0;
            for(DMatch m:stereoMatches){
                Point a=f.keys[m.queryIdx].pt,b=rightKeys[m.trainIdx].pt;
                verticalSum+=Math.abs(a.y-b.y);disparitySum+=a.x-b.x-(calibration.cx-calibration.rightCx);
                if(Math.abs(a.y-b.y)>config.epipolarTolerance){verticalRejected++;continue;}
                double[] p=calibration.triangulate(a.x,a.y,b.x);
                if(p==null){disparityRejected++;continue;}
                if(p[2]<config.minDepth||p[2]>config.maxDepth){depthRejected++;continue;}
                f.points[m.queryIdx]=p;accepted++;
            }
            // Forward/backward patch tracking supplies depth for corners whose independently
            // detected right descriptors are absent. Geometry and photometric checks remain mandatory.
            List<Point> source=new ArrayList<>();for(KeyPoint key:f.keys)source.add(key.pt);
            Point[] tracked=config.useOpticalFlow?flow(l,r,source):new Point[source.size()];
            for(int i=0;i<tracked.length;i++)if(f.points[i]==null&&tracked[i]!=null){
                Point a=f.keys[i].pt,b=tracked[i];
                if(Math.abs(a.y-b.y)>config.epipolarTolerance)continue;
                double[] p=calibration.triangulate(a.x,a.y,b.x);
                if(p!=null&&p[2]>=config.minDepth&&p[2]<=config.maxDepth){f.points[i]=p;accepted++;}
            }
            String diagnostic="Stereo frame={} leftFeatures={} rightFeatures={} mutualRatioMatches={} rejectVertical={} rejectNonPositiveDisparity={} rejectDepth={} accepted={} meanAbsVertical={}px meanDisparity={}px";
            Object[] values={sequence+1,f.keys.length,rightKeys.length,stereoMatches.size(),verticalRejected,disparityRejected,depthRejected,accepted,stereoMatches.isEmpty()?0:verticalSum/stereoMatches.size(),stereoMatches.isEmpty()?0:disparitySum/stereoMatches.size()};
            if(accepted<config.minInliers&&(sequence==0||System.nanoTime()-lastDiagnosticNanos>=2_000_000_000L))log.warn(diagnostic,values);
            else log.debug(diagnostic,values);
            return f;
        }catch(RuntimeException e){f.close();throw e;}
        finally{l.release();r.release();rd.release();mask.release();lk.release();rk.release();}
    }
    /** Ratio test and mutual nearest-neighbor check prevent many-to-one correspondences. */
    private List<DMatch> matches(Mat a,Mat b){
        List<DMatch> result=new ArrayList<>();if(a.rows()<2||b.rows()<2)return result;
        List<MatOfDMatch> forward=new ArrayList<>(),reverse=new ArrayList<>();
        try{
            matcher.knnMatch(a,b,forward,2);matcher.knnMatch(b,a,reverse,2);
            int[] back=new int[b.rows()];Arrays.fill(back,-1);
            for(MatOfDMatch row:reverse){DMatch[] m=row.toArray();if(m.length==2&&m[0].distance<config.ratio*m[1].distance)
                back[m[0].queryIdx]=m[0].trainIdx;}
            for(MatOfDMatch row:forward){DMatch[] m=row.toArray();if(m.length==2&&m[0].distance<config.ratio*m[1].distance
                &&(config.useSift||m[0].distance<64)&&back[m[0].trainIdx]==m[0].queryIdx)result.add(m[0]);}
            return result;
        }finally{for(Mat m:forward)m.release();for(Mat m:reverse)m.release();}
    }
    private static Point[] flow(Mat a,Mat b,List<Point> source){
        Point[] result=new Point[source.size()];if(source.isEmpty())return result;
        MatOfPoint2f from=new MatOfPoint2f(),to=new MatOfPoint2f(),back=new MatOfPoint2f();
        MatOfByte status=new MatOfByte(),reverse=new MatOfByte();MatOfFloat error=new MatOfFloat(),re=new MatOfFloat();
        try{from.fromList(source);
            org.opencv.video.Video.calcOpticalFlowPyrLK(a,b,from,to,status,error,new Size(21,21),3);
            org.opencv.video.Video.calcOpticalFlowPyrLK(b,a,to,back,reverse,re,new Size(21,21),3);
            Point[] q=to.toArray(),p=back.toArray();byte[] ok=status.toArray(),rok=reverse.toArray();float[] err=error.toArray();
            for(int i=0;i<q.length;i++)if(ok[i]!=0&&rok[i]!=0&&err[i]<20&&Math.hypot(source.get(i).x-p[i].x,source.get(i).y-p[i].y)<.7
                &&q[i].x>=0&&q[i].y>=0&&q[i].x<b.cols()&&q[i].y<b.rows())result[i]=q[i];
            return result;
        }finally{for(Mat m:new Mat[]{from,to,back,status,reverse,error,re})m.release();}
    }
    private Estimate estimate(Keyframe ref,Frame current){
        Estimate descriptor=estimate(ref,current,false);
        // Optical flow complements descriptors for nearby frames; historic relocalization
        // still uses appearance matching to avoid drifting patches across large changes.
        return config.useOpticalFlow&&sequence-ref.sequence<=config.keyframeInterval*2?better(descriptor,estimate(ref,current,true)):descriptor;
    }
    private Estimate estimate(Keyframe ref,Frame current,boolean optical){
        List<Point3> objects=new ArrayList<>();List<Point> pixels=new ArrayList<>();
        Pose world=graph.pose(ref.id);
        if(optical){
            List<Point> source=new ArrayList<>();List<Integer> indices=new ArrayList<>();
            for(int i=0;i<ref.frame.keys.length;i++)if(ref.frame.points[i]!=null){source.add(ref.frame.keys[i].pt);indices.add(i);}
            Point[] tracked=flow(ref.frame.image,current.image,source);
            for(int i=0;i<tracked.length;i++)if(tracked[i]!=null){double[] p=world.transform(ref.frame.points[indices.get(i)]);
                objects.add(new Point3(p[0],p[1],p[2]));pixels.add(tracked[i]);}
        }else for(DMatch m:matches(ref.frame.descriptors,current.descriptors))if(ref.frame.points[m.queryIdx]!=null){
            double[] p=world.transform(ref.frame.points[m.queryIdx]);objects.add(new Point3(p[0],p[1],p[2]));
            pixels.add(current.keys[m.trainIdx].pt);
        }
        if(objects.size()<config.minInliers){log.debug("PnP reference={} correspondences={} required={}",ref.id,objects.size(),config.minInliers);return null;}
        MatOfPoint3f op=new MatOfPoint3f();MatOfPoint2f ip=new MatOfPoint2f();
        Mat rv=new Mat(),tv=new Mat(),inliers=new Mat(),rotation=new Mat();
        MatOfPoint3f goodObjects=new MatOfPoint3f();MatOfPoint2f goodPixels=new MatOfPoint2f(),projected=new MatOfPoint2f();
        try{
            op.fromList(objects);ip.fromList(pixels);
            boolean ok=Calib3d.solvePnPRansac(op,ip,intrinsics,distortion,rv,tv,false,150,
                (float)config.reprojectionError,.999,inliers,Calib3d.SOLVEPNP_EPNP);
            if(!ok||inliers.rows()<config.minInliers||inliers.rows()<objects.size()*config.minInlierRatio){log.debug("PnP rejected reference={} ok={} correspondences={} inliers={} required={} minRatio={}",ref.id,ok,objects.size(),inliers.rows(),config.minInliers,config.minInlierRatio);return null;}
            List<Point3> gp=new ArrayList<>();List<Point> gi=new ArrayList<>();
            for(int i=0;i<inliers.rows();i++){int j=(int)inliers.get(i,0)[0];gp.add(objects.get(j));gi.add(pixels.get(j));}
            goodObjects.fromList(gp);goodPixels.fromList(gi);
            Calib3d.solvePnPRefineLM(goodObjects,goodPixels,intrinsics,distortion,rv,tv);
            Calib3d.Rodrigues(rv,rotation);double[] rr=new double[9],tt=new double[3];rotation.get(0,0,rr);tv.get(0,0,tt);
            Pose cameraWorld=new Pose(rr,tt);
            Calib3d.projectPoints(goodObjects,rv,tv,intrinsics,distortion,projected);
            Point[] reprojections=projected.toArray();int accepted=0;
            double minX=Double.POSITIVE_INFINITY,maxX=0,minY=Double.POSITIVE_INFINITY,maxY=0;
            for(int i=0;i<gp.size();i++){
                Point3 p=gp.get(i);Point uv=gi.get(i),q=reprojections[i];
                double[] pc=cameraWorld.transform(new double[]{p.x,p.y,p.z});
                if(pc[2]>0&&Math.hypot(uv.x-q.x,uv.y-q.y)<=config.reprojectionError){accepted++;
                    minX=Math.min(minX,uv.x);maxX=Math.max(maxX,uv.x);minY=Math.min(minY,uv.y);maxY=Math.max(maxY,uv.y);}
            }
            if(accepted<config.minInliers||accepted<objects.size()*config.minInlierRatio
                ||maxX-minX<calibration.width*.15||maxY-minY<calibration.height*.15)return null;
            return new Estimate(ref,cameraWorld.inverse(),accepted);
        }finally{op.release();ip.release();rv.release();tv.release();inliers.release();rotation.release();
            goodObjects.release();goodPixels.release();projected.release();}
    }
    /** Reconstructs historic poses using their corrected keyframe anchors. */
    public SlamSnapshot snapshot(){
        return snapshot(true);
    }
    public SlamSnapshot snapshot(boolean includeSparsePoints){
        if(closed)throw new IllegalStateException("Engine closed");
        List<SlamSnapshot.MapPoint> points=new ArrayList<>();List<Pose> poses=new ArrayList<>();
        List<SlamSnapshot.TrackPoint> tracks=new ArrayList<>();
        for(Keyframe k:keyframes){Pose pose=graph.pose(k.id);poses.add(pose);
            if(includeSparsePoints)for(double[] local:k.frame.points)if(local!=null){double[] p=pose.transform(local);
                points.add(new SlamSnapshot.MapPoint(p[0],p[1],p[2]));}}
        // Bound the live display cost. Export still preserves the full trajectory.
        int begin=Math.max(0,trajectory.size()-10000);
        for(int i=begin;i<trajectory.size();i++){Sample s=trajectory.get(i);
            tracks.add(new SlamSnapshot.TrackPoint(graph.pose(s.anchor).multiply(s.relative),i==begin||s.startsSegment));}
        return new SlamSnapshot(points,tracks,poses);
    }
    public void writeTrajectory(Path file)throws IOException{
        try(BufferedWriter w=Files.newBufferedWriter(file)){
            w.write("# timestamp tx ty tz qx qy qz qw (T_world_camera, metres)\n");
            for(Sample s:trajectory){Pose p=graph.pose(s.anchor).multiply(s.relative);double[] t=p.translation(),q=p.quaternion();
                w.write(String.format(Locale.ROOT,"%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f%n",
                    s.time,t[0],t[1],t[2],q[0],q[1],q[2],q[3]));}
        }
    }
    /** Sparse keyframe cloud; repeated landmarks across keyframes are retained. */
    public void writeMap(Path file)throws IOException{
        int count=0;for(Keyframe k:keyframes)for(double[] p:k.frame.points)if(p!=null)count++;
        try(BufferedWriter w=Files.newBufferedWriter(file)){
            w.write("ply\nformat ascii 1.0\nelement vertex "+count+"\nproperty float x\nproperty float y\nproperty float z\nend_header\n");
            for(Keyframe k:keyframes)for(double[] p:k.frame.points)if(p!=null){double[] q=graph.pose(k.id).transform(p);
                w.write(String.format(Locale.ROOT,"%.6f %.6f %.6f%n",q[0],q[1],q[2]));}
        }
    }
    @Override public void close(){if(closed)return;closed=true;for(Keyframe k:keyframes)k.frame.close();
        // ORB has no retained image buffers. Its wrapper owns its native lifetime;
        // Algorithm.clear() is not supported by this repository's 4.9 JNI build.
        intrinsics.release();distortion.release();matcher.clear();}
}
