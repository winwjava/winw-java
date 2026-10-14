package winw.ai.slam;

import static org.junit.Assert.*;
import java.util.*;
import java.nio.file.*;
import org.junit.*;
import org.opencv.core.*;

public class SceneStereoRectifierTest {
    @BeforeClass public static void load(){OpenCvNative.load();}
    private static List<Point>[] correspondences(int seed,boolean planar){
        Random random=new Random(seed);List<Point> l=new ArrayList<>(),r=new ArrayList<>();
        double roll=.012,yaw=.015;
        for(int i=0;i<400;i++){
            double u=45+random.nextDouble()*540,v=35+random.nextDouble()*400,z=planar?4:1.5+random.nextDouble()*6;
            double x=(u-320)*z/500,y=(v-240)*z/500;
            double xx=Math.cos(yaw)*x+Math.sin(yaw)*z-.12,zz=-Math.sin(yaw)*x+Math.cos(yaw)*z;
            double rx=Math.cos(roll)*xx-Math.sin(roll)*y,ry=Math.sin(roll)*xx+Math.cos(roll)*y;
            l.add(new Point(u,v));r.add(new Point(500*rx/zz+320+random.nextGaussian()*.1,500*ry/zz+240+random.nextGaussian()*.1));
        }
        @SuppressWarnings("unchecked") List<Point>[] points=new List[]{l,r};return points;
    }
    @Test public void perspectiveGeometryNeedsSubsequentFramesAndRejectsLoss(){
        try(SceneStereoRectifier rectifier=new SceneStereoRectifier(640,480)){
            for(int i=0;i<3;i++){
                List<Point>[] points=correspondences(170+i,false);
                assertEquals(rectifier.status,i==2,rectifier.estimate(points[0],points[1]));
                assertEquals(rectifier.status,i==2,rectifier.ready());
            }
            assertTrue(rectifier.verticalError<1.5);
            Mat image=new Mat(480,640,CvType.CV_8UC1,new Scalar(255)),l=new Mat(),r=new Mat();
            try{rectifier.rectify(image,image,l,r);assertEquals(640,l.cols());assertTrue(Core.countNonZero(l)>640*480*.55);}
            finally{image.release();l.release();r.release();}
            for(int i=0;i<3;i++)rectifier.estimate(Collections.emptyList(),Collections.emptyList());
            assertFalse(rectifier.ready());
        }
    }
    @Test public void rejectsPlanarAndPoorlyDistributedScenes(){
        try(SceneStereoRectifier rectifier=new SceneStereoRectifier(640,480)){
            List<Point>[] plane=correspondences(42,true);
            assertFalse(rectifier.estimate(plane[0],plane[1]));assertFalse(rectifier.ready());
            assertTrue(rectifier.status,rectifier.status.contains("平面"));
            List<Point>[] narrow=correspondences(21,false);
            for(Point p:narrow[0]){p.x=300+(p.x-320)*.05;p.y=220+(p.y-240)*.05;}
            assertFalse(rectifier.estimate(narrow[0],narrow[1]));
        }
    }
    @Test public void matchesNaturalTextureAcrossChangingViews(){
        try(SceneStereoRectifier rectifier=new SceneStereoRectifier(640,480)){
            for(int i=0;i<6&&!rectifier.ready();i++){
                Mat l=StereoSlamTest.scene(i*.015),r=StereoSlamTest.scene(i*.015+.12);
                try{rectifier.observe(l,r);}finally{l.release();r.release();}
            }
            assertTrue(rectifier.status+" matches="+rectifier.matches+" inliers="+rectifier.inliers,rectifier.ready());
            assertTrue(rectifier.inliers>50);
        }
    }
    @Test public void independentBadCorrespondencesPreventAcceptance(){
        try(SceneStereoRectifier rectifier=new SceneStereoRectifier(640,480)){
            for(int i=0;i<3;i++){
                List<Point>[] points=correspondences(800+i,false);
                for(int j=0;j<points[1].size();j+=4)points[1].get(j).y+=25;
                assertFalse(rectifier.estimate(points[0],points[1]));
            }
            assertFalse(rectifier.ready());
        }
    }
    @Test public void savedCacheMustBeValidatedInCurrentScene()throws Exception{
        Path file=Files.createTempFile("scene-rectification-", ".properties");
        try(SceneStereoRectifier first=new SceneStereoRectifier(640,480);
            SceneStereoRectifier cached=new SceneStereoRectifier(640,480)){
            for(int i=0;i<3;i++){List<Point>[] p=correspondences(170+i,false);first.estimate(p[0],p[1]);}
            assertTrue(first.ready());first.save(file);cached.loadCandidate(file);assertFalse(cached.ready());
            for(int i=0;i<3;i++){List<Point>[] p=correspondences(173+i,false);cached.estimate(p[0],p[1]);assertEquals(i==2,cached.ready());}
        }finally{Files.deleteIfExists(file);}
    }
    @Test public void disparityMasksCheckCorrespondingRightPixelAndInvalidValue(){
        Mat disparity=new Mat(12,30,CvType.CV_16SC1,new Scalar(4*16));
        Mat l=new Mat(12,30,CvType.CV_8UC1,new Scalar(255)),r=l.clone(),out=new Mat();
        try{
            r.put(6,10,new byte[]{0});disparity.put(6,20,new short[]{-16});
            AutoStereoDemo.renderDisparity(disparity,l,r,0,64,out);
            assertEquals(0,out.get(6,14)[0],0);assertEquals(0,out.get(6,20)[0],0);
            assertTrue(out.get(6,24)[0]>0);assertEquals(0,out.get(6,1)[0],0);
        }finally{disparity.release();l.release();r.release();out.release();}
    }
}
