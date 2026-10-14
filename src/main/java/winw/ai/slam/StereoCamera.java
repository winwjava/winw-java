package winw.ai.slam;

import lombok.extern.slf4j.Slf4j;

import org.opencv.core.*;
import org.opencv.videoio.*;

/** One device carrying a synchronized stereo pair, never two independently timed USB reads. */
@Slf4j
public final class StereoCamera implements AutoCloseable {
    private final VideoCapture capture=new VideoCapture();
    private final Mat pair=new Mat();
    private boolean firstFrame=true;
    private final int eyeWidth,eyeHeight;
    private final LiveSlamConfig config;
    public StereoCamera(int device,StereoCalibration calibration,LiveSlamConfig config){
        this(device,calibration.width,calibration.height,config);
    }
    /** Capture dimensions do not require invented focal lengths or baseline. */
    public StereoCamera(int device,int eyeWidth,int eyeHeight,LiveSlamConfig config){
        if(eyeWidth<=0||eyeHeight<=0)throw new IllegalArgumentException("Invalid per-eye dimensions");
        this.eyeWidth=eyeWidth;this.eyeHeight=eyeHeight;this.config=config;config.validate();
        try{
            if(!capture.open(device,config.backend))throw new IllegalStateException("Cannot open camera device "+device);
            if(!config.fourcc.isEmpty())capture.set(Videoio.CAP_PROP_FOURCC,VideoWriter.fourcc(
                config.fourcc.charAt(0),config.fourcc.charAt(1),config.fourcc.charAt(2),config.fourcc.charAt(3)));
            capture.set(Videoio.CAP_PROP_FRAME_WIDTH,eyeWidth*(config.layout==LiveSlamConfig.Layout.SIDE_BY_SIDE?2:1));
            capture.set(Videoio.CAP_PROP_FRAME_HEIGHT,eyeHeight*(config.layout==LiveSlamConfig.Layout.TOP_BOTTOM?2:1));
            capture.set(Videoio.CAP_PROP_FPS,config.fps);
            log.info("Camera device={} backend={} layout={} swapEyes={} negotiated={}x{} fps={} fourcc={}",device,config.backend,config.layout,config.swapEyes,capture.get(Videoio.CAP_PROP_FRAME_WIDTH),capture.get(Videoio.CAP_PROP_FRAME_HEIGHT),capture.get(Videoio.CAP_PROP_FPS),capture.get(Videoio.CAP_PROP_FOURCC));
        }catch(RuntimeException e){close();throw e;}
    }
    public boolean read(Mat left,Mat right){
        if(!capture.read(pair)||pair.empty())return false;
        if(firstFrame){log.info("First captured stereo frame: {}x{} channels={}",pair.cols(),pair.rows(),pair.channels());firstFrame=false;}
        split(pair,eyeWidth,eyeHeight,config.layout,config.swapEyes,left,right);return true;
    }
    static void split(Mat pair,StereoCalibration calibration,LiveSlamConfig.Layout layout,boolean swap,Mat left,Mat right){
        split(pair,calibration.width,calibration.height,layout,swap,left,right);
    }
    static void split(Mat pair,int eyeWidth,int eyeHeight,LiveSlamConfig.Layout layout,boolean swap,Mat left,Mat right){
        boolean horizontal=layout==LiveSlamConfig.Layout.SIDE_BY_SIDE;
        int width=eyeWidth*(horizontal?2:1),height=eyeHeight*(horizontal?1:2);
        if(pair.cols()!=width||pair.rows()!=height)throw new IllegalArgumentException(
            "Camera frame is "+pair.cols()+"x"+pair.rows()+", expected "+width+"x"+height+" ("+layout+"). Check camera mode and calibration.");
        Mat first=horizontal?pair.colRange(0,eyeWidth):pair.rowRange(0,eyeHeight);
        Mat second=horizontal?pair.colRange(eyeWidth,width):pair.rowRange(eyeHeight,height);
        try{(swap?second:first).copyTo(left);(swap?first:second).copyTo(right);}
        finally{first.release();second.release();}
    }
    @Override public void close(){capture.release();pair.release();}
}
