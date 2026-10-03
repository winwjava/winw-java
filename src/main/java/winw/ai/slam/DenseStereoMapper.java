package winw.ai.slam;

import java.util.*;
import org.opencv.calib3d.StereoSGBM;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

/** SGBM depth observations anchored to keyframes so loop corrections also move the dense cloud. */
public final class DenseStereoMapper implements AutoCloseable {
    private final StereoCalibration calibration;
    private final LiveSlamConfig config;
    private final StereoSGBM matcher;
    private final Map<Integer,List<SlamSnapshot.MapPoint>> clouds=new LinkedHashMap<>();
    public DenseStereoMapper(StereoCalibration calibration,LiveSlamConfig config){
        config.validate();this.calibration=calibration;this.config=config;
        if(config.disparities>=calibration.width/2)throw new IllegalArgumentException("map.disparities must be less than half the image width");
        int minimum=(int)Math.floor(calibration.cx-calibration.rightCx);
        matcher=StereoSGBM.create(minimum,config.disparities,5,8*25,32*25,1,31,12,100,2,StereoSGBM.MODE_SGBM_3WAY);
    }
    public void addKeyframe(int id,Mat left,Mat right){
        if(clouds.containsKey(id))return;
        List<SlamSnapshot.MapPoint> points=new ArrayList<>();
        for(TemporalOccupancyMapper.DepthSample sample:measure(left,right))points.add(new SlamSnapshot.MapPoint(sample.x,sample.y,sample.z));
        rememberKeyframe(id,points);
    }
    public void rememberKeyframe(int id,List<SlamSnapshot.MapPoint> points){
        clouds.putIfAbsent(id,Collections.unmodifiableList(new ArrayList<>(points)));
    }
    /** The same depth calculation feeds historical point clouds and the rolling occupancy window. */
    public List<TemporalOccupancyMapper.DepthSample> measure(Mat left,Mat right){
        Mat l=gray(left),r=gray(right),disparity=new Mat();
        try{
            matcher.compute(l,r,disparity);short[] values=new short[(int)disparity.total()];disparity.get(0,0,values);
            List<TemporalOccupancyMapper.DepthSample> points=new ArrayList<>();
            int step=Math.max(config.sampleStep,(int)Math.ceil(Math.sqrt((double)left.total()/config.maxPointsPerKeyframe)));
            int minimum=matcher.getMinDisparity();
            for(int y=step;y<l.rows()-step;y+=step)for(int x=step;x<l.cols()-step;x+=step){
                double d=values[y*l.cols()+x]/16.0;
                if(d<=minimum)continue;
                double[] p=calibration.triangulate(x,y,x-d);
                if(p!=null&&p[2]>=config.minDepth&&p[2]<=config.maxDepth)
                    {
                        // Stereo disparity uncertainty grows quadratically with depth.
                        double sigma=p[2]*p[2]*config.occupancy.disparitySigma/(calibration.fx*calibration.baseline);
                        double confidence=1/(1+Math.pow(sigma/config.occupancy.resolution,2));
                        points.add(new TemporalOccupancyMapper.DepthSample(p[0],p[1],p[2],confidence,sigma));
                    }
            }
            return Collections.unmodifiableList(points);
        }finally{l.release();r.release();disparity.release();}
    }
    private Mat gray(Mat input){Mat m=new Mat();if(input.channels()==1)input.copyTo(m);
        else Imgproc.cvtColor(input,m,input.channels()==3?Imgproc.COLOR_BGR2GRAY:Imgproc.COLOR_BGRA2GRAY);return m;}
    public List<SlamSnapshot.MapPoint> worldPoints(List<Pose> keyframes){
        List<SlamSnapshot.MapPoint> result=new ArrayList<>();
        for(Map.Entry<Integer,List<SlamSnapshot.MapPoint>> entry:clouds.entrySet()){
            if(entry.getKey()>=keyframes.size())continue;Pose pose=keyframes.get(entry.getKey());
            for(SlamSnapshot.MapPoint p:entry.getValue()){
                double[] q=pose.transform(new double[]{p.x,p.y,p.z});result.add(new SlamSnapshot.MapPoint(q[0],q[1],q[2]));}
        }
        return result;
    }
    @Override public void close(){clouds.clear(); /* Native matcher ownership follows its Java wrapper. */}
}
