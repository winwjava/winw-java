package winw.ai.slam;

import java.util.*;
import org.opencv.core.*;
import org.opencv.calib3d.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.ximgproc.*;

/** CPU bidirectional stereo, edge-aware disparity and image-organized surface triangles. */
public final class StereoSurfaceReconstructor implements AutoCloseable {
    public static final class Result {
        public final OccupancySurfaceMesh surface;
        public final short[] disparity;
        public final float[] confidence;
        public final int rawValid,regularizedValid,trustedPixels,triangles;
        public final double milliseconds;
        Result(OccupancySurfaceMesh mesh,short[] d,float[] c,int raw,int valid,int trusted,double ms){surface=mesh;disparity=d;confidence=c;rawValid=raw;regularizedValid=valid;trustedPixels=trusted;triangles=mesh.faces.size();milliseconds=ms;}
    }
    private final StereoSGBM leftMatcher;
    private StereoMatcher rightMatcher;
    private DisparityWLSFilter filter;
    private final int disparities,step;
    private int minimum=Integer.MIN_VALUE;
    public StereoSurfaceReconstructor(int disparities,int step){
        if(disparities<16||disparities%16!=0||step<1)throw new IllegalArgumentException("Stereo surface configuration");
        this.disparities=disparities;this.step=step;
        leftMatcher=StereoSGBM.create(0,disparities,3,8*9,32*9,1,31,5,0,0,StereoSGBM.MODE_SGBM_3WAY);
    }
    private void configure(int min){if(min==minimum)return;minimum=min;leftMatcher.setMinDisparity(min);
        rightMatcher=Ximgproc.createRightMatcher(leftMatcher);filter=Ximgproc.createDisparityWLSFilter(leftMatcher);
        filter.setLambda(8000);filter.setSigmaColor(1.2);filter.setLRCthresh(24);filter.setDepthDiscontinuityRadius(3);
    }
    public Result compute(Mat left,Mat right,Mat leftMask,Mat rightMask,int min,double baseline){
        if(left.empty()||right.empty()||left.cols()!=right.cols()||left.rows()!=right.rows()||!Double.isFinite(baseline)||baseline<=0)throw new IllegalArgumentException("Valid stereo images and baseline required");
        long started=System.nanoTime();configure(min);int width=left.cols(),height=left.rows();
        Mat l=gray(left),r=gray(right),dl=new Mat(),dr=new Mat(),filtered=new Mat(),confidence=null;
        try{leftMatcher.compute(l,r,dl);rightMatcher.compute(r,l,dr);filter.filter(dl,left,filtered,dr);confidence=filter.getConfidenceMap();
            short[] raw=new short[width*height],values=new short[raw.length];float[] weights=new float[raw.length];
            dl.get(0,0,raw);filtered.get(0,0,values);confidence.get(0,0,weights);
            byte[] lm=mask(leftMask,values.length),rm=mask(rightMask,values.length),pixels=new byte[values.length*3];
            Mat color=new Mat();boolean[] supported;try{if(left.channels()==1)Imgproc.cvtColor(left,color,Imgproc.COLOR_GRAY2BGR);else if(left.channels()==4)Imgproc.cvtColor(left,color,Imgproc.COLOR_BGRA2BGR);else left.copyTo(color);color.get(0,0,pixels);supported=RegionDisparityCompletion.complete(color,values,weights,pixels,width,height);}finally{color.release();}
            int rawCount=0,valid=0,trusted=0;double focal=width*.8;double[][] vertices=new double[values.length][];
            for(int y=3;y<height-3;y++)for(int x=3;x<width-3;x++){int i=y*width+x;double d=values[i]/16.;int rx=(int)Math.round(x-d);
                if(raw[i]>Math.max(0,min)*16&&lm[i]!=0)rawCount++;
                if(!supported[i]||d<=Math.max(0,min)||lm[i]==0||rx<3||rx>=width-3||rm[y*width+rx]==0){values[i]=(short)((min-1)*16);continue;}
                double z=focal*baseline/d;if(z<.15||z>6){values[i]=(short)((min-1)*16);continue;}valid++;if(weights[i]>=128)trusted++;
                // Keep regularized low-confidence geometry separate from trusted OCC evidence.
                if(x%step==0&&y%step==0)vertices[i]=new double[]{(x-width/2.)*z/focal,(y-height/2.)*z/focal,z};
            }
            List<OccupancySurfaceMesh.Face> triangles=new ArrayList<>();
            double[] lo={Double.POSITIVE_INFINITY,Double.POSITIVE_INFINITY,Double.POSITIVE_INFINITY},hi={Double.NEGATIVE_INFINITY,Double.NEGATIVE_INFINITY,Double.NEGATIVE_INFINITY};
            for(int y=step;y<height-step;y+=step)for(int x=step;x<width-step;x+=step){int a=y*width+x,b=a+step,c=a+step*width,d=c+step;
                triangle(triangles,vertices,pixels,weights,a,c,b,width,lo,hi);triangle(triangles,vertices,pixels,weights,b,c,d,width,lo,hi);
            }
            if(triangles.isEmpty()){Arrays.fill(lo,0);Arrays.fill(hi,0);}
            OccupancySurfaceMesh mesh=OccupancySurfaceMesh.surface(triangles,lo,hi);
            return new Result(mesh,values,weights,rawCount,valid,trusted,(System.nanoTime()-started)/1e6);
        }finally{for(Mat m:new Mat[]{l,r,dl,dr,filtered,confidence})if(m!=null)m.release();}
    }
    private static void triangle(List<OccupancySurfaceMesh.Face> out,double[][] vertices,byte[] colors,float[] confidence,int a,int b,int c,int width,double[] lo,double[] hi){
        double[] p=vertices[a],q=vertices[b],r=vertices[c];if(p==null||q==null||r==null)return;
        double min=Math.min(p[2],Math.min(q[2],r[2])),max=Math.max(p[2],Math.max(q[2],r[2]));
        // Never bridge foreground/background depth discontinuities, even when all pixels exist.
        if(max-min>.008+min*.025)return;
        int[] ids={a,b,c};double[][] points={p,q,r};
        for(int edge=0;edge<3;edge++){int next=(edge+1)%3;double pixelsApart=Math.hypot(ids[edge]%width-ids[next]%width,ids[edge]/width-ids[next]/width);
            double length=0;for(int k=0;k<3;k++)length+=Math.pow(points[edge][k]-points[next][k],2);
            if(Math.sqrt(length)>pixelsApart*min/(width*.8)*4)return;
        }
        int rgb=0;for(int channel=0;channel<3;channel++){int value=((colors[a*3+channel]&255)+(colors[b*3+channel]&255)+(colors[c*3+channel]&255))/3;rgb|=value<<(channel==0?0:channel==1?8:16);}
        double weight=Math.min(confidence[a],Math.min(confidence[b],confidence[c]))/255.;
        out.add(new OccupancySurfaceMesh.Face(new double[][]{p,q,r},0,1,rgb,weight));
        for(double[] point:new double[][]{p,q,r})for(int k=0;k<3;k++){lo[k]=Math.min(lo[k],point[k]);hi[k]=Math.max(hi[k],point[k]);}
    }
    private static byte[] mask(Mat mask,int count){byte[] values=new byte[count];if(mask==null||mask.empty())Arrays.fill(values,(byte)255);else mask.get(0,0,values);return values;}
    private static Mat gray(Mat input){Mat out=new Mat();if(input.channels()==1)input.copyTo(out);else Imgproc.cvtColor(input,out,input.channels()==3?Imgproc.COLOR_BGR2GRAY:Imgproc.COLOR_BGRA2GRAY);return out;}
    public void close(){/* Native algorithms follow Java wrapper ownership. Temporary Mats are released per frame. */}
}
