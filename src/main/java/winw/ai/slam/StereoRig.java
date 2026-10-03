package winw.ai.slam;

import java.io.*;
import java.nio.file.*;
import java.util.Properties;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

/** Optional pinhole stereo rectification. Fisheye input requires external rectification. */
public final class StereoRig implements AutoCloseable {
    public final StereoCalibration calibration;
    private final Mat lx=new Mat(),ly=new Mat(),rx=new Mat(),ry=new Mat();
    private boolean remap;
    private StereoRig(StereoCalibration c){calibration=c;}
    public static StereoRig load(Path path)throws IOException{
        Properties p=new Properties();try(Reader r=Files.newBufferedReader(path)){p.load(r);}
        String mode=p.getProperty("rectified","true");
        if(!mode.equals("true")&&!mode.equals("false"))throw new IllegalArgumentException("rectified must be true or false");
        if(mode.equals("true"))return new StereoRig(StereoCalibration.load(path));
        int width=Integer.parseInt(p.getProperty("width")),height=Integer.parseInt(p.getProperty("height"));
        Mat kl=null,kr=null,dl=null,dr=null,r=null,t=null;
        Mat rl=new Mat(),rr=new Mat(),pl=new Mat(),pr=new Mat(),q=new Mat();
        StereoRig rig=null;
        try{
            kl=matrix(p,"left.K",3,3);kr=matrix(p,"right.K",3,3);
            dl=matrix(p,"left.D",1,5);dr=matrix(p,"right.D",1,5);
            r=matrix(p,"R",3,3);t=matrix(p,"T",3,1);
            Size size=new Size(width,height);
            Calib3d.stereoRectify(kl,dl,kr,dr,size,r,t,rl,rr,pl,pr,q,Calib3d.CALIB_ZERO_DISPARITY,0,size);
            double fx=pl.get(0,0)[0],baseline=-pr.get(0,3)[0]/pr.get(0,0)[0];
            if(Math.abs(pr.get(1,3)[0])>1e-6)throw new IllegalArgumentException("Only horizontal stereo rigs supported");
            rig=new StereoRig(new StereoCalibration(width,height,fx,pl.get(1,1)[0],pl.get(0,2)[0],pl.get(1,2)[0],pr.get(0,2)[0],baseline));
            Calib3d.initUndistortRectifyMap(kl,dl,rl,pl,size,CvType.CV_32FC1,rig.lx,rig.ly);
            Calib3d.initUndistortRectifyMap(kr,dr,rr,pr,size,CvType.CV_32FC1,rig.rx,rig.ry);
            rig.remap=true;return rig;
        }catch(RuntimeException e){if(rig!=null)rig.close();throw e;}
        finally{for(Mat m:new Mat[]{kl,kr,dl,dr,r,t,rl,rr,pl,pr,q})if(m!=null)m.release();}
    }
    private static Mat matrix(Properties p,String name,int rows,int cols){
        String[] tokens=p.getProperty(name,"").trim().split("[,\\s]+");
        if(tokens.length!=rows*cols)throw new IllegalArgumentException("Expected "+rows*cols+" values for "+name);
        double[] values=new double[tokens.length];for(int i=0;i<values.length;i++){
            values[i]=Double.parseDouble(tokens[i]);if(!Double.isFinite(values[i]))throw new IllegalArgumentException(name+" is nonfinite");}
        Mat m=new Mat(rows,cols,CvType.CV_64F);m.put(0,0,values);return m;
    }
    public void rectify(Mat left,Mat right,Mat outputLeft,Mat outputRight){
        if(left.empty()||right.empty()||left.cols()!=calibration.width||right.cols()!=calibration.width
            ||left.rows()!=calibration.height||right.rows()!=calibration.height)
            throw new IllegalArgumentException("Input dimensions do not match calibration");
        if(remap){Imgproc.remap(left,outputLeft,lx,ly,Imgproc.INTER_LINEAR);Imgproc.remap(right,outputRight,rx,ry,Imgproc.INTER_LINEAR);}
        else{left.copyTo(outputLeft);right.copyTo(outputRight);}
    }
    @Override public void close(){lx.release();ly.release();rx.release();ry.release();}
}
