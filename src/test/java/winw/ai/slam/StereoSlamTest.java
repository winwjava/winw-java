package winw.ai.slam;

import static org.junit.Assert.*;
import java.nio.file.*;
import java.util.*;
import org.junit.BeforeClass;
import org.junit.Test;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgcodecs.Imgcodecs;

public class StereoSlamTest {
    private static final StereoCalibration CAMERA=new StereoCalibration(640,480,500,500,320,240,320,.12);
    @BeforeClass public static void nativeLibrary(){OpenCvNative.load();Core.setRNGSeed(42);}
    @Test public void metricTriangulation(){
        assertArrayEquals(new double[]{.6,.3,3},CAMERA.triangulate(420,290,400),1e-10);
        assertNull(CAMERA.triangulate(100,100,101));
        StereoCalibration c=new StereoCalibration(640,480,500,500,320,240,310,.12);
        assertEquals(3,c.triangulate(420,290,390)[2],1e-10);
    }
    @Test public void inverseCompositionAndHalfTurn(){
        Pose p=Pose.fromVector(new double[]{.2,-.3,.1,1,2,3});
        assertArrayEquals(Pose.identity().rotation(),p.multiply(p.inverse()).rotation(),1e-10);
        assertArrayEquals(new double[]{1,2,3},p.inverse().transform(p.transform(new double[]{1,2,3})),1e-10);
        Pose half=Pose.fromVector(new double[]{Math.PI,0,0,0,0,0});
        assertArrayEquals(half.rotation(),Pose.fromVector(half.vector()).rotation(),1e-10);
    }
    @Test public void loopGraphReducesDrift(){
        PoseGraph graph=new PoseGraph();graph.addPose(Pose.identity());
        for(int i=1;i<6;i++){graph.addPose(Pose.fromVector(new double[]{0,0,0,i*1.1,0,0}));
            graph.addEdge(i-1,i,Pose.fromVector(new double[]{0,0,0,1.1,0,0}),false);}
        graph.addEdge(0,5,Pose.fromVector(new double[]{0,0,0,5,0,0}),true);
        double before=graph.cost();graph.optimize(15);
        assertTrue(graph.cost()<before*.3);assertTrue(graph.pose(5).translation()[0]<5.2);
        assertArrayEquals(new double[3],graph.pose(0).translation(),0);
    }
    @Test public void rawRectificationMatchesPinholeCalibration()throws Exception{
        try(StereoRig rig=StereoRig.load(Paths.get("docs/slam/raw-example.properties"))){
            assertEquals(500,rig.calibration.fx,1e-5);assertEquals(.12,rig.calibration.baseline,1e-8);
            Mat image=scene(0),left=new Mat(),right=new Mat();
            try{rig.rectify(image,image,left,right);assertEquals(0,Core.norm(image,left,Core.NORM_INF),1e-8);}
            finally{image.release();left.release();right.release();}
        }
    }
    @Test public void stereoTrackingLostRecoveryAndExports()throws Exception{
        SlamConfig config=new SlamConfig();config.features=2500;config.keyframeInterval=2;config.maxKeyframes=3;
        try(StereoSlam slam=new StereoSlam(CAMERA,config)){
            Mat blank=Mat.zeros(480,640,CvType.CV_8UC1);
            try{
                assertEquals(StereoSlam.State.INITIALIZING,slam.process(blank,blank,0).state);
                for(int i=0;i<7;i++){
                    Mat l=scene(i*.02),r=scene(i*.02+.12);
                    try{StereoSlam.Result result=slam.process(l,r,i+1);
                        assertNotNull("Tracking frame "+i+" state="+result.state+" points="+result.stereoPoints,result.pose);
                        assertEquals(i*.02,result.pose.translation()[0],.045);
                        assertEquals(0,result.pose.translation()[2],.05);
                        if(i==6)assertTrue(result.mapFull);
                    }finally{l.release();r.release();}
                }
                StereoSlam.Result lost=slam.process(blank,blank,8);
                assertEquals(StereoSlam.State.LOST,lost.state);assertNull(lost.pose);
                Mat l=scene(0),r=scene(.12);
                try{assertEquals(StereoSlam.State.RELOCALIZED,slam.process(l,r,9).state);}
                finally{l.release();r.release();}
                Path trajectory=Files.createTempFile("slam-test-trajectory",".tum"),map=Files.createTempFile("slam-test-map",".ply");
                try{slam.writeTrajectory(trajectory);slam.writeMap(map);
                    assertEquals(9,Files.readAllLines(trajectory).size());
                    assertTrue(Files.readString(map).startsWith("ply\nformat ascii 1.0"));
                }finally{Files.deleteIfExists(trajectory);Files.deleteIfExists(map);}
            }finally{blank.release();}
        }
    }
    @Test(expected=IllegalArgumentException.class)public void rejectsRepeatedTime(){
        try(StereoSlam slam=new StereoSlam(CAMERA,new SlamConfig())){
            Mat m=Mat.zeros(480,640,CvType.CV_8UC1);try{slam.process(m,m,1);slam.process(m,m,1);}finally{m.release();}
        }
    }
    @Test public void verifiedLoopClosesOnOverlappingKeyframes(){
        SlamConfig config=new SlamConfig();config.features=2500;config.keyframeInterval=3;config.loopGap=8;
        boolean closedLoop=false;
        try(StereoSlam slam=new StereoSlam(CAMERA,config)){
            for(int i=0;i<28;i++){
                double x=(i<=14?i:28-i)*.01;Mat l=scene(x),r=scene(x+.12);
                try{StereoSlam.Result result=slam.process(l,r,i);assertNotNull(result.pose);
                    assertEquals(x,result.pose.translation()[0],.05);closedLoop|=result.loopClosed;
                }finally{l.release();r.release();}
            }
        }
        assertTrue("Two-frame geometric verification should generate a loop constraint",closedLoop);
    }
    @Test public void datasetEntryPointWritesTrajectoryAndMap()throws Exception{
        Path directory=Files.createTempDirectory("slam-dataset-test");
        try{
            for(int i=0;i<2;i++){
                Mat l=scene(i*.02),r=scene(i*.02+.12);
                try{assertTrue(Imgcodecs.imwrite(directory.resolve("l"+i+".png").toString(),l));
                    assertTrue(Imgcodecs.imwrite(directory.resolve("r"+i+".png").toString(),r));
                }finally{l.release();r.release();}
            }
            Path manifest=directory.resolve("manifest.txt");
            Files.writeString(manifest,"# sample\n0 l0.png r0.png\n0.1 l1.png r1.png\n");
            StereoSlamDemo.main(new String[]{"dataset","docs/slam/rectified-example.properties",manifest.toString(),directory.toString()});
            assertEquals(3,Files.readAllLines(directory.resolve("trajectory.tum")).size());
            assertTrue(Files.size(directory.resolve("map.ply"))>1000);
        }finally{
            try(java.util.stream.Stream<Path> files=Files.list(directory)){
                for(Path file:(Iterable<Path>)files::iterator)Files.delete(file);
            }
            Files.delete(directory);
        }
    }
    @Test(expected=IllegalArgumentException.class)public void rejectsInvalidCalibration(){
        new StereoCalibration(640,480,500,500,320,240,320,Double.NaN);
    }
    /** Seeded textured frontoparallel patches at multiple depths, rendered in both cameras. */
    static Mat scene(double cameraX){
        Mat image=new Mat(480,640,CvType.CV_8UC1,new Scalar(30));Random random=new Random(93281);
        for(int y=0;y<5;y++)for(int x=0;x<7;x++){
            double z=3+(x+y)%4;
            double wx=(75+x*78-320)*z/500,wy=(65+y*78-240)*z/500;
            int u=(int)Math.round(500*(wx-cameraX)/z+320),v=(int)Math.round(500*wy/z+240);
            for(int dy=0;dy<10;dy++)for(int dx=0;dx<10;dx++){
                int value=random.nextBoolean()?230:65;
                Imgproc.rectangle(image,new Point(u+dx*5,v+dy*5),new Point(u+dx*5+4,v+dy*5+4),new Scalar(value),-1);
            }
        }
        return image;
    }
}
