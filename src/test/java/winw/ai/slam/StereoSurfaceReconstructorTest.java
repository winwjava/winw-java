package winw.ai.slam;

import java.nio.file.*;
import java.util.*;
import org.junit.Test;
import org.opencv.core.*;
import static org.junit.Assert.*;

public class StereoSurfaceReconstructorTest {
    @Test public void recoversKnownStereoShiftAndRoundTripsSurface()throws Exception {
        OpenCvNative.load();Core.setRNGSeed(93);
        Mat l=new Mat(180,320,CvType.CV_8UC1),r=Mat.zeros(180,320,CvType.CV_8UC1);Core.randu(l,0,255);
        Mat source=l.submat(new Rect(16,0,304,180)),destination=r.submat(new Rect(0,0,304,180));source.copyTo(destination);source.release();destination.release();
        Path file=Files.createTempFile("stereo-surface",".surf");
        try(StereoSurfaceReconstructor reconstructor=new StereoSurfaceReconstructor(64,4)){
            StereoSurfaceReconstructor.Result result=reconstructor.compute(l,r,null,null,0,.065);
            assertTrue(result.trustedPixels>5000);assertTrue(result.triangles>1000);
            List<Double> disparity=new ArrayList<>();for(int i=0;i<result.disparity.length;i++)if(result.confidence[i]>=128&&result.disparity[i]>0)disparity.add(result.disparity[i]/16.);
            Collections.sort(disparity);assertEquals(16,disparity.get(disparity.size()/2),.3);
            result.surface.write(file);OccupancySurfaceMesh decoded=OccupancySurfaceMesh.read(file);
            assertEquals(result.triangles,decoded.faces.size());assertEquals(result.surface.faces.get(0).rgb,decoded.faces.get(0).rgb);
            assertEquals(result.surface.faces.get(0).confidence,decoded.faces.get(0).confidence,1e-6);
        }finally{l.release();r.release();Files.delete(file);}
    }
    @Test public void blankImagesDoNotCreateAWorldByRegularization(){
        OpenCvNative.load();Mat blank=Mat.zeros(180,320,CvType.CV_8UC1);
        try(StereoSurfaceReconstructor reconstructor=new StereoSurfaceReconstructor(64,4)){
            StereoSurfaceReconstructor.Result result=reconstructor.compute(blank,blank,null,null,0,.065);
            assertEquals(0,result.trustedPixels);assertTrue(result.surface.faces.isEmpty());
        }finally{blank.release();}
    }
}
