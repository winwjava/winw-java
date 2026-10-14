package winw.ai.slam;

import java.awt.image.BufferedImage;
import java.io.*;
import java.nio.file.*;
import java.util.Locale;
import java.util.concurrent.atomic.*;
import javax.swing.*;
import org.opencv.calib3d.StereoSGBM;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;

/** No calibration file: targetless stereo preview and raw sequence capture for self-calibration. */
public final class AutoStereoDemo {
    private AutoStereoDemo(){}
    static final class Frame {
        final BufferedImage left,right,disparity;final String status;
        Frame(Mat l,Mat r,Mat d,String status){left=LiveStereoSlam.bufferedImage(l);right=LiveStereoSlam.bufferedImage(r);
            disparity=d.empty()?null:LiveStereoSlam.bufferedImage(d);this.status=status;}
    }
    public static void main(String[] args)throws Exception{
        if(args.length<5)throw new IllegalArgumentException("camera-auto <eye-width> <eye-height> <device> <output-directory> [live.properties]");
        LiveSlamConfig config=LiveSlamConfig.load(args.length>5?Paths.get(args[5]):null);
        AutoStereoSettings s=new AutoStereoSettings();s.width=Integer.parseInt(args[1]);s.height=Integer.parseInt(args[2]);s.device=Integer.parseInt(args[3]);
        s.outputDirectory=Paths.get(args[4]);s.backend=config.backend;s.fourcc=config.fourcc;s.fps=config.fps;
        s.layout=config.layout;s.swapEyes=config.swapEyes;s.disparities=config.disparities;
        String cache=System.getProperty("slam.auto.rectificationCache");if(cache!=null)s.rectificationCache=Paths.get(cache);
        launch(s);
    }
    public static void launch(AutoStereoSettings settings)throws Exception{
        AutoStereoSettings initial=settings.copy();initial.validate();OpenCvNative.load();
        Runnable show=()->new AutoStereoWindow(initial).show();
        if(SwingUtilities.isEventDispatchThread())show.run();else SwingUtilities.invokeAndWait(show);
    }
    static void capture(AutoStereoSettings settings,Path output,AtomicBoolean stop,AtomicReference<Frame> latest)throws Exception{
        int width=settings.width,height=settings.height;LiveSlamConfig config=settings.cameraConfig();
        Mat l=new Mat(),r=new Mat(),rl=new Mat(),rr=new Mat(),disparity=new Mat(),visual=new Mat(),lm=new Mat(),rm=new Mat();
        int area=settings.blockSize*settings.blockSize;
        StereoSGBM sgbm=StereoSGBM.create(0,config.disparities,settings.blockSize,8*area,32*area,1,31,settings.uniquenessRatio,100,2,StereoSGBM.MODE_SGBM_3WAY);
        int samples=0;long started=System.nanoTime(),lastObservation=0,lastRecord=0,lastPublish=0;
        try(SceneStereoRectifier rectifier=new SceneStereoRectifier(width,height,settings.features,settings.validationTolerance);
            StereoCamera camera=new StereoCamera(settings.device,width,height,config);
            BufferedWriter manifest=Files.newBufferedWriter(output.resolve("manifest.txt"))){
            if(settings.rectificationCache!=null)rectifier.loadCandidate(settings.rectificationCache);
            manifest.write("# timestamp left right; raw images, NOT rectified\n");
            while(!stop.get()){
                if(!camera.read(l,r))throw new IOException("摄像头读取失败");long now=System.nanoTime();
                if(settings.record&&now-lastRecord>=settings.recordMillis*1_000_000L&&samples<settings.maxSamples){
                    String name=String.format(Locale.ROOT,"%06d.png",samples);
                    if(!Imgcodecs.imwrite(output.resolve("images/left/"+name).toString(),l)||!Imgcodecs.imwrite(output.resolve("images/right/"+name).toString(),r))
                        throw new IOException("原始图像写入失败");
                    manifest.write(String.format(Locale.ROOT,"%.6f images/left/%s images/right/%s%n",(now-started)/1e9,name,name));manifest.flush();samples++;lastRecord=now;
                }
                if(now-lastObservation>=settings.observationMillis*1_000_000L){
                    if(rectifier.observe(l,r))rectifier.save(output.resolve("rectification.properties"));
                    lastObservation=now;
                }
                if(now-lastPublish<100_000_000L)continue;lastPublish=now;
                visual.release();
                if(rectifier.ready()){
                    rectifier.rectify(l,r,rl,rr);rectifier.masks(lm,rm);
                    sgbm.setMinDisparity(rectifier.minimumDisparity());
                    Mat gl=SceneStereoRectifier.gray(rl),gr=SceneStereoRectifier.gray(rr);
                    try{sgbm.compute(gl,gr,disparity);renderDisparity(disparity,lm,rm,sgbm.getMinDisparity(),config.disparities,visual);}
                    finally{gl.release();gr.release();}
                }else{l.copyTo(rl);r.copyTo(rr);}
                latest.set(new Frame(rl,rr,visual,String.format(Locale.ROOT,"%s | 匹配 %d / 内点 %d / 验证误差 %.2f px | 已录制 %d/%d 对%s",
                    rectifier.status,rectifier.matches,rectifier.inliers,rectifier.verticalError,samples,settings.maxSamples,settings.record?"":"（录制关闭）")));
            }
        }finally{for(Mat m:new Mat[]{l,r,rl,rr,disparity,visual,lm,rm})m.release();}
    }
    static void renderDisparity(Mat disparity,Mat lm,Mat rm,int minimum,int range,Mat output){
        int width=disparity.cols(),height=disparity.rows();short[] d=new short[width*height];disparity.get(0,0,d);
        byte[] a=new byte[width*height],b=new byte[width*height],pixels=new byte[width*height];lm.get(0,0,a);rm.get(0,0,b);
        for(int y=3;y<height-3;y++)for(int x=3;x<width-3;x++){
            int i=y*width+x;double value=d[i]/16.;int rx=(int)Math.round(x-value);
            if(value<=minimum||rx<3||rx>=width-3||a[i]==0||b[y*width+rx]==0)continue;
            boolean border=false;
            for(int offset=-2;offset<=2;offset++)if(a[i+offset]==0||b[y*width+rx+offset]==0||a[i+offset*width]==0||b[(y+offset)*width+rx]==0){border=true;break;}
            if(!border)pixels[i]=(byte)Math.min(255,Math.max(1,1+254*(value-minimum)/range));
        }
        output.create(height,width,CvType.CV_8UC1);output.put(0,0,pixels);
    }
}
