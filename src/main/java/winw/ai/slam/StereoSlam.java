package winw.ai.slam;

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
        Mat descriptors=new Mat();
        double[][] points;
        @Override public void close(){descriptors.release();}
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
    private final BFMatcher matcher=BFMatcher.create(Core.NORM_HAMMING,false);
    private final Mat intrinsics;
    private final MatOfDouble distortion=new MatOfDouble(0,0,0,0,0);
    private final List<Keyframe> keyframes=new ArrayList<>();
    private final List<Sample> trajectory=new ArrayList<>();
    private final PoseGraph graph=new PoseGraph();
    private int sequence;
    private double lastTime=Double.NEGATIVE_INFINITY;
    private boolean closed;
    private State state=State.INITIALIZING;

    public StereoSlam(StereoCalibration calibration,SlamConfig config){
        config.validate();this.calibration=Objects.requireNonNull(calibration);this.config=config.copy();
        orb=ORB.create(config.features);intrinsics=calibration.cameraMatrix();
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
                    || pose.distance(graph.pose(latest.id))>=config.keyframeTranslation);
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
            orb.detectAndCompute(l,mask,lk,f.descriptors);orb.detectAndCompute(r,mask,rk,rd);
            f.keys=lk.toArray();KeyPoint[] rightKeys=rk.toArray();f.points=new double[f.keys.length][];
            for(DMatch m:matches(f.descriptors,rd)){
                Point a=f.keys[m.queryIdx].pt,b=rightKeys[m.trainIdx].pt;
                if(Math.abs(a.y-b.y)>config.epipolarTolerance)continue;
                double[] p=calibration.triangulate(a.x,a.y,b.x);
                if(p!=null&&p[2]>=config.minDepth&&p[2]<=config.maxDepth)f.points[m.queryIdx]=p;
            }
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
                &&m[0].distance<64&&back[m[0].trainIdx]==m[0].queryIdx)result.add(m[0]);}
            return result;
        }finally{for(Mat m:forward)m.release();for(Mat m:reverse)m.release();}
    }
    private Estimate estimate(Keyframe ref,Frame current){
        List<Point3> objects=new ArrayList<>();List<Point> pixels=new ArrayList<>();
        Pose world=graph.pose(ref.id);
        for(DMatch m:matches(ref.frame.descriptors,current.descriptors))if(ref.frame.points[m.queryIdx]!=null){
            double[] p=world.transform(ref.frame.points[m.queryIdx]);objects.add(new Point3(p[0],p[1],p[2]));
            pixels.add(current.keys[m.trainIdx].pt);
        }
        if(objects.size()<config.minInliers)return null;
        MatOfPoint3f op=new MatOfPoint3f();MatOfPoint2f ip=new MatOfPoint2f();
        Mat rv=new Mat(),tv=new Mat(),inliers=new Mat(),rotation=new Mat();
        MatOfPoint3f goodObjects=new MatOfPoint3f();MatOfPoint2f goodPixels=new MatOfPoint2f(),projected=new MatOfPoint2f();
        try{
            op.fromList(objects);ip.fromList(pixels);
            boolean ok=Calib3d.solvePnPRansac(op,ip,intrinsics,distortion,rv,tv,false,150,
                (float)config.reprojectionError,.999,inliers,Calib3d.SOLVEPNP_EPNP);
            if(!ok||inliers.rows()<config.minInliers||inliers.rows()<objects.size()*config.minInlierRatio)return null;
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
