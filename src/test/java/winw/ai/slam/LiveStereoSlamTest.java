package winw.ai.slam;

import static org.junit.Assert.*;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.nio.file.*;
import java.util.*;
import javax.imageio.ImageIO;
import javax.swing.SwingUtilities;
import org.junit.*;
import org.opencv.core.*;

public class LiveStereoSlamTest {
    @BeforeClass public static void nativeLibrary(){OpenCvNative.load();}
    @Test public void splitsBothLayoutsAndSwapsEyes(){
        StereoCalibration calibration=new StereoCalibration(4,3,10,10,2,1,2,.1);
        Mat horizontal=new Mat(3,8,CvType.CV_8UC1,new Scalar(21)),vertical=new Mat(6,4,CvType.CV_8UC1,new Scalar(31));
        Mat second=horizontal.colRange(4,8),bottom=vertical.rowRange(3,6),l=new Mat(),r=new Mat();
        try{second.setTo(new Scalar(99));bottom.setTo(new Scalar(88));
            StereoCamera.split(horizontal,calibration,LiveSlamConfig.Layout.SIDE_BY_SIDE,false,l,r);
            assertEquals(21,l.get(1,1)[0],0);assertEquals(99,r.get(1,1)[0],0);
            StereoCamera.split(vertical,calibration,LiveSlamConfig.Layout.TOP_BOTTOM,true,l,r);
            assertEquals(88,l.get(1,1)[0],0);assertEquals(31,r.get(1,1)[0],0);
        }finally{horizontal.release();vertical.release();second.release();bottom.release();l.release();r.release();}
    }
    @Test public void heightFilteringAndNegativeGridCoordinates(){
        LiveSlamConfig config=new LiveSlamConfig();config.cellSize=.1;config.minY=-1;config.maxY=1;
        BirdEyeMap map=BirdEyeMap.build(Arrays.asList(new SlamSnapshot.MapPoint(-.01,0,-.01),
            new SlamSnapshot.MapPoint(-.02,.1,-.02),new SlamSnapshot.MapPoint(5,2,5),new SlamSnapshot.MapPoint(Double.NaN,0,0)),config);
        assertEquals(2,map.visiblePoints);assertEquals(1,map.cells.size());
        assertEquals(-.05,map.cells.get(0).x,1e-8);assertEquals(-.05,map.cells.get(0).z,1e-8);assertEquals(2,map.cells.get(0).observations);
        assertEquals(0,map.cells.get(0).minY,0);assertEquals(.1,map.cells.get(0).maxY,1e-8);
    }
    @Test public void denseDepthHasMetricScaleAndMovesWithCorrectedAnchor(){
        StereoCalibration calibration=new StereoCalibration(320,240,300,300,160,120,160,.12);
        LiveSlamConfig config=new LiveSlamConfig();config.disparities=64;config.sampleStep=4;
        Mat l=new Mat(240,320,CvType.CV_8UC1),r=Mat.zeros(240,320,CvType.CV_8UC1);
        byte[] texture=new byte[320*240];new Random(3481).nextBytes(texture);l.put(0,0,texture);
        Mat source=l.colRange(20,320),target=r.colRange(0,300);
        try(DenseStereoMapper mapper=new DenseStereoMapper(calibration,config)){
            source.copyTo(target);mapper.addKeyframe(0,l,r);
            List<SlamSnapshot.MapPoint> points=mapper.worldPoints(Collections.singletonList(Pose.identity()));
            assertTrue("Expected dense textured surface",points.size()>500);assertTrue(points.size()<=config.maxPointsPerKeyframe);
            double[] depths=points.stream().mapToDouble(p->p.z).sorted().toArray();assertEquals(1.8,depths[depths.length/2],.03);
            List<SlamSnapshot.MapPoint> corrected=mapper.worldPoints(Collections.singletonList(Pose.fromVector(new double[]{0,0,0,.4,0,.2})));
            assertEquals(points.get(0).x+.4,corrected.get(0).x,1e-8);assertEquals(points.get(0).z+.2,corrected.get(0).z,1e-8);
        }finally{source.release();target.release();l.release();r.release();}
    }
    @Test public void birdEyeProjectionAndLostPoseRendering()throws Exception{
        SwingUtilities.invokeAndWait(()->{
            LiveSlamConfig config=new LiveSlamConfig();BirdEyeMap map=BirdEyeMap.build(Collections.singletonList(new SlamSnapshot.MapPoint(1,0,1)),config);
            BirdEyePanel panel=new BirdEyePanel(45);panel.showOrigin();panel.update(map,SlamSnapshot.empty(),Pose.identity());
            BufferedImage image=panel.image(900,680);assertEquals(45,new Color(image.getRGB(496,294)).getRed());
            Color camera=new Color(image.getRGB(450,329));assertTrue(camera.getRed()>200&&camera.getGreen()<160);
            panel.update(map,SlamSnapshot.empty(),null);BufferedImage lost=panel.image(900,680);
            assertTrue(new Color(lost.getRGB(450,329)).getRed()<100);
        });
    }
    @Test public void previewArtifact()throws Exception{
        List<SlamSnapshot.MapPoint> points=new ArrayList<>();
        for(double z=0;z<8;z+=.025)for(int i=0;i<3;i++){
            points.add(new SlamSnapshot.MapPoint(-2.5,0,z));points.add(new SlamSnapshot.MapPoint(2.5,0,z));}
        for(double x=-2.5;x<2.5;x+=.025)for(int i=0;i<3;i++)points.add(new SlamSnapshot.MapPoint(x,0,8));
        for(double a=0;a<Math.PI*2;a+=.01)points.add(new SlamSnapshot.MapPoint(.8+.45*Math.cos(a),0,4.2+.45*Math.sin(a)));
        List<SlamSnapshot.TrackPoint> trajectory=new ArrayList<>();List<Pose> keyframes=new ArrayList<>();
        for(int i=0;i<100;i++){Pose p=Pose.fromVector(new double[]{0,.25*Math.sin(i*.03),0,.3*Math.sin(i*.04),0,i*.055});
            trajectory.add(new SlamSnapshot.TrackPoint(p,i==0));if(i%10==0)keyframes.add(p);}
        LiveSlamConfig config=new LiveSlamConfig();BirdEyeMap map=BirdEyeMap.build(points,config);
        Path output=Paths.get("target/slam-bird-eye-preview.png");Files.createDirectories(output.getParent());
        SwingUtilities.invokeAndWait(()->{
            BirdEyePanel panel=new BirdEyePanel(65);panel.setSize(1000,760);
            panel.update(map,new SlamSnapshot(points,trajectory,keyframes),trajectory.get(99).pose);panel.fitMap();
            BufferedImage image=panel.image(1000,760);Graphics2D graphics=image.createGraphics();
            graphics.setColor(Color.WHITE);graphics.drawString("SYNTHETIC DISPLAY EXAMPLE - NOT A CAMERA CAPTURE",18,90);graphics.dispose();
            try{ImageIO.write(image,"png",output.toFile());}catch(java.io.IOException e){throw new java.io.UncheckedIOException(e);}
        });
        assertTrue(Files.size(output)>5000);
    }
    @Test public void previewImagePreservesBgrColours(){
        Mat m=new Mat(2,2,CvType.CV_8UC3,new Scalar(10,20,200));
        try{Color color=new Color(LiveStereoSlam.bufferedImage(m).getRGB(0,0));
            assertEquals(200,color.getRed());assertEquals(20,color.getGreen());assertEquals(10,color.getBlue());
        }finally{m.release();}
    }
    @Test(expected=IllegalArgumentException.class)public void rejectsInvalidDenseDisparity(){
        LiveSlamConfig config=new LiveSlamConfig();config.disparities=63;config.validate();
    }
}
