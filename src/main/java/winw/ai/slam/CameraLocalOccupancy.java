package winw.ai.slam;

import java.util.*;
import org.opencv.core.*;

/** Camera-local OCC from current disparity; independent of global localization availability. */
public final class CameraLocalOccupancy {
    private CameraLocalOccupancy(){}
    public static OccupancySnapshot measure(Mat disparity,Mat leftMask,Mat rightMask,int minimum,double baseline){
        OccupancyConfig config=new OccupancyConfig();config.resolution=.05;config.radius=7;config.minHitFrames=1;config.maxVoxels=100000;
        TemporalOccupancyMapper mapper=new TemporalOccupancyMapper(config,-3,3);
        List<TemporalOccupancyMapper.DepthSample> samples=new ArrayList<>();
        if(disparity.empty()||!Double.isFinite(baseline)||baseline<=0)return mapper.snapshot();
        int width=disparity.cols(),height=disparity.rows();double focal=.8*width;
        short[] values=new short[width*height];disparity.get(0,0,values);byte[] lm=new byte[values.length],rm=new byte[values.length];leftMask.get(0,0,lm);rightMask.get(0,0,rm);
        int step=Math.max(4,(int)Math.ceil(Math.sqrt((double)values.length/10000)));
        for(int y=step;y<height-step;y+=step)for(int x=step;x<width-step;x+=step){int i=y*width+x;double d=values[i]/16.;int rx=(int)Math.round(x-d);
            if(d<=Math.max(0,minimum)||rx<3||rx>=width-3||lm[i]==0||rm[y*width+rx]==0)continue;
            boolean border=false;for(int offset=-2;offset<=2;offset++)if(lm[i+offset]==0||rm[y*width+rx+offset]==0||lm[i+offset*width]==0||rm[(y+offset)*width+rx]==0){border=true;break;}if(border)continue;
            double z=focal*baseline/d;if(z<.15||z>6)continue;
            double sigma=z*z*.5/(focal*baseline),confidence=1/(1+Math.pow(sigma/config.resolution,2));
            samples.add(new TemporalOccupancyMapper.DepthSample((x-width/2.)*z/focal,(y-height/2.)*z/focal,z,confidence,sigma));
        }
        mapper.observe(0,0,Pose.identity(),samples,Collections.singletonList(Pose.identity()),Pose.identity());return mapper.snapshot();
    }
}
