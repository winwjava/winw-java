package winw.ai.slam;

import java.io.IOException;
import java.io.Reader;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Properties;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

/** Calibration of RECTIFIED stereo images, with equal vertical intrinsics. */
public final class StereoCalibration {
    public final int width,height;
    public final double fx,fy,cx,cy,rightCx,baseline;
    public StereoCalibration(int width,int height,double fx,double fy,double cx,double cy,double rightCx,double baseline) {
        if(width<=0 || height<=0 || !positive(fx) || !positive(fy) || !positive(baseline)
            || !Double.isFinite(cx) || !Double.isFinite(cy) || !Double.isFinite(rightCx))
            throw new IllegalArgumentException("Invalid rectified calibration (baseline must be in metres)");
        this.width=width; this.height=height; this.fx=fx; this.fy=fy; this.cx=cx; this.cy=cy;
        this.rightCx=rightCx; this.baseline=baseline;
    }
    private static boolean positive(double v) {return Double.isFinite(v)&&v>0;}
    public static StereoCalibration load(Path file) throws IOException {
        Properties p=new Properties(); try(Reader r=Files.newBufferedReader(file)){p.load(r);}
        return new StereoCalibration(Integer.parseInt(p.getProperty("width")),Integer.parseInt(p.getProperty("height")),
            value(p,"fx"),value(p,"fy"),value(p,"cx"),value(p,"cy"),
            Double.parseDouble(p.getProperty("rightCx",p.getProperty("cx"))),value(p,"baseline"));
    }
    private static double value(Properties p,String k){return Double.parseDouble(p.getProperty(k));}
    public Mat cameraMatrix() {
        Mat m=Mat.eye(3,3,CvType.CV_64F); m.put(0,0,fx);m.put(1,1,fy);m.put(0,2,cx);m.put(1,2,cy);return m;
    }
    public double[] triangulate(double u,double v,double rightU) {
        double disparity=u-rightU-(cx-rightCx);
        if(!Double.isFinite(disparity) || disparity<=0) return null;
        double z=fx*baseline/disparity;
        return new double[]{(u-cx)*z/fx,(v-cy)*z/fy,z};
    }
}
