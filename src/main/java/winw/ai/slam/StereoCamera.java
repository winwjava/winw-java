package winw.ai.slam;

import org.opencv.core.*;
import org.opencv.videoio.*;

/** One device carrying a synchronized stereo pair, never two independently timed USB reads. */
public final class StereoCamera implements AutoCloseable {
    private final VideoCapture capture=new VideoCapture();
    private final Mat pair=new Mat();
    private final StereoCalibration calibration;
    private final LiveSlamConfig config;
    public StereoCamera(int device,StereoCalibration calibration,LiveSlamConfig config){
        this.calibration=calibration;this.config=config;config.validate();
        try{
            if(!capture.open(device,config.backend))throw new IllegalStateException("Cannot open camera device "+device);
            if(!config.fourcc.isEmpty())capture.set(Videoio.CAP_PROP_FOURCC,VideoWriter.fourcc(
                config.fourcc.charAt(0),config.fourcc.charAt(1),config.fourcc.charAt(2),config.fourcc.charAt(3)));
            capture.set(Videoio.CAP_PROP_FRAME_WIDTH,calibration.width*(config.layout==LiveSlamConfig.Layout.SIDE_BY_SIDE?2:1));
            capture.set(Videoio.CAP_PROP_FRAME_HEIGHT,calibration.height*(config.layout==LiveSlamConfig.Layout.TOP_BOTTOM?2:1));
            capture.set(Videoio.CAP_PROP_FPS,config.fps);
        }catch(RuntimeException e){close();throw e;}
    }
    public boolean read(Mat left,Mat right){
        if(!capture.read(pair)||pair.empty())return false;
        split(pair,calibration,config.layout,config.swapEyes,left,right);return true;
    }
    static void split(Mat pair,StereoCalibration calibration,LiveSlamConfig.Layout layout,boolean swap,Mat left,Mat right){
        boolean horizontal=layout==LiveSlamConfig.Layout.SIDE_BY_SIDE;
        int width=calibration.width*(horizontal?2:1),height=calibration.height*(horizontal?1:2);
        if(pair.cols()!=width||pair.rows()!=height)throw new IllegalArgumentException(
            "Camera frame is "+pair.cols()+"x"+pair.rows()+", expected "+width+"x"+height+" ("+layout+"). Check camera mode and calibration.");
        Mat first=horizontal?pair.colRange(0,calibration.width):pair.rowRange(0,calibration.height);
        Mat second=horizontal?pair.colRange(calibration.width,width):pair.rowRange(calibration.height,height);
        try{(swap?second:first).copyTo(left);(swap?first:second).copyTo(right);}
        finally{first.release();second.release();}
    }
    @Override public void close(){capture.release();pair.release();}
}
