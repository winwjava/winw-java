package winw.ai.slam;

import java.nio.file.*;
import java.util.*;
import org.junit.*;
import static org.junit.Assert.*;
import org.apache.commons.math3.linear.*;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgcodecs.Imgcodecs;

public class NaturalSceneCalibrationTest {
    @BeforeClass public static void load(){OpenCvNative.load();}
    @Test public void weakContrastStereoRecoversRealCorrespondences()throws Exception{
        Mat a=new Mat(480,640,CvType.CV_8UC1),b=new Mat(),transform=Mat.eye(2,3,CvType.CV_64F);
        org.opencv.features2d.ORB baseline=org.opencv.features2d.ORB.create(3500);
        MatOfKeyPoint rawKeys=new MatOfKeyPoint();Mat rawDescriptors=new Mat(),mask=new Mat();
        try(NaturalSceneCalibration estimator=new NaturalSceneCalibration(640,480,3500,1.5)){
            Core.setRNGSeed(7741);Core.randu(a,0,255);Imgproc.GaussianBlur(a,a,new Size(5,5),1);
            a.convertTo(a,-1,.12,90);transform.put(0,2,-17);transform.put(1,2,3);
            Imgproc.warpAffine(a,b,transform,a.size());Core.add(b,new Scalar(20),b);
            baseline.detectAndCompute(a,mask,rawKeys,rawDescriptors);
            NaturalSceneCalibration.Features fa=estimator.features(a),fb=estimator.features(b);
            NaturalSceneCalibration.Matches matches=estimator.stereoMatch(fa,fb);
            assertTrue("Baseline should reproduce sparse default-ORB detection",rawDescriptors.rows()<80);
            assertTrue("Enhanced matching must preserve scene correspondences",matches.a.size()>=80);
            double[] errors=new double[matches.a.size()];
            for(int i=0;i<errors.length;i++)errors[i]=Math.hypot(matches.b.get(i).x-matches.a.get(i).x+17,matches.b.get(i).y-matches.a.get(i).y-3);
            Arrays.sort(errors);assertTrue("More matches must be correct",errors[(int)(errors.length*.8)]<1.5);
        }finally{a.release();b.release();transform.release();rawKeys.release();rawDescriptors.release();mask.release();}
    }
    @Test public void jointCalibrationUsesStrongEyeToResolveWeakEye()throws Exception{
        List<RealMatrix> weak=new ArrayList<>();for(int i=0;i<12;i++)weak.add(horizontalFundamental(530,530));
        // A pure-translation eye alone gives no absolute focal constraint.
        try{NaturalSceneCalibration.estimateFocal(weak,640,480);fail();}catch(java.io.IOException expected){}
        double[] focal=JointFocalEstimator.estimate(weak,motionFundamentals(700),horizontalFundamental(530,700),640,480,null);
        assertEquals(530,focal[0],.2);assertEquals(700,focal[1],.2);
    }
    @Test public void siftFallbackMatchesRotatedSceneWithoutUsingCalibration()throws Exception{
        Mat a=StereoSlamTest.scene(0),b=new Mat(),transform=Imgproc.getRotationMatrix2D(new Point(320,240),18,1.12);
        try(NaturalSceneCalibration estimator=new NaturalSceneCalibration(640,480,3500,1.5)){
            Imgproc.warpAffine(a,b,transform,a.size());Core.add(b,new Scalar(15),b);
            NaturalSceneCalibration.Features fa=estimator.features(a),fb=estimator.features(b);
            // Exercise the fallback when the binary descriptor path has no usable data.
            fa.descriptors.release();fb.descriptors.release();
            NaturalSceneCalibration.Matches matches=estimator.stereoMatch(fa,fb);
            assertEquals("CLAHE+SIFT",matches.method);assertTrue(matches.a.size()>80);
            double[] errors=new double[matches.a.size()];
            for(int i=0;i<errors.length;i++){
                Point p=matches.a.get(i),q=matches.b.get(i);
                double x=transform.get(0,0)[0]*p.x+transform.get(0,1)[0]*p.y+transform.get(0,2)[0];
                double y=transform.get(1,0)[0]*p.x+transform.get(1,1)[0]*p.y+transform.get(1,2)[0];
                errors[i]=Math.hypot(x-q.x,y-q.y);
            }
            Arrays.sort(errors);assertTrue("Fallback must match known geometry",errors[(int)(errors.length*.8)]<1.5);
        }finally{a.release();b.release();transform.release();}
    }
    @Test public void jointProfileRejectsCorrectRatioWithUnknownAbsoluteScale()throws Exception{
        List<RealMatrix> left=new ArrayList<>(),right=new ArrayList<>();
        for(int i=0;i<12;i++){left.add(horizontalFundamental(530,530));right.add(horizontalFundamental(700,700));}
        try{JointFocalEstimator.estimate(left,right,horizontalFundamental(530,700),640,480,null);fail("A known focal ratio does not fix metric depth");}
        catch(java.io.IOException expected){assertTrue(expected.getMessage(),expected.getMessage().contains("profileSeparation"));}
    }
    private static RealMatrix horizontalFundamental(double left,double right){
        RealMatrix kl=new LUDecomposition(NaturalSceneCalibration.intrinsic(left,640,480)).getSolver().getInverse();
        RealMatrix kr=new LUDecomposition(NaturalSceneCalibration.intrinsic(right,640,480)).getSolver().getInverse();
        return kr.transpose().multiply(MatrixUtils.createRealMatrix(new double[][]{{0,0,0},{0,0,1},{0,-1,0}})).multiply(kl);
    }
    private static List<RealMatrix> motionFundamentals(double focal){
        List<RealMatrix> fundamentals=new ArrayList<>();
        RealMatrix inverse=new LUDecomposition(NaturalSceneCalibration.intrinsic(focal,640,480)).getSolver().getInverse();
        for(int i=0;i<12;i++){
            double x=.04+i*.013,y=-.09+i*.016;
            RealMatrix rx=MatrixUtils.createRealMatrix(new double[][]{{1,0,0},{0,Math.cos(x),-Math.sin(x)},{0,Math.sin(x),Math.cos(x)}});
            RealMatrix ry=MatrixUtils.createRealMatrix(new double[][]{{Math.cos(y),0,Math.sin(y)},{0,1,0},{-Math.sin(y),0,Math.cos(y)}});
            double ty=.03*Math.sin(i),tz=.02*Math.cos(i);
            RealMatrix cross=MatrixUtils.createRealMatrix(new double[][]{{0,-tz,ty},{tz,0,.1},{-ty,-.1,0}});
            fundamentals.add(inverse.transpose().multiply(cross.multiply(rx.multiply(ry))).multiply(inverse));
        }
        return fundamentals;
    }
    @Test public void longRecordingKeepsNearbyTrainingFramesAndHeldoutNeverLeaks(){
        List<String> names=new ArrayList<>();for(int i=0;i<1000;i++)names.add(String.format(Locale.ROOT,"%06d.png",i));
        List<String> selected=NaturalSceneCalibration.selectFrames(names);
        assertEquals(60,selected.size());assertEquals(60,new HashSet<>(selected).size());
        assertEquals(names.get(0),selected.get(0));assertEquals(names.get(999),selected.get(59));
        int adjacent=0;
        for(int[] pair:NaturalSceneCalibration.temporalPairs(selected.size())){
            assertNotEquals(0,pair[0]%5);assertNotEquals(0,pair[1]%5);
            int first=names.indexOf(selected.get(pair[0])),second=names.indexOf(selected.get(pair[1]));
            if(second-first==1)adjacent++;
        }
        assertTrue("Long recordings must retain at least two near-neighbor constraints per burst",adjacent>=24);
    }
    @Test public void forwardBackwardFlowRecoversSubpixelMotionAndRejectsBlankImage(){
        Mat a=new Mat(240,320,CvType.CV_8UC1),b=new Mat(),transform=Mat.eye(2,3,CvType.CV_64F),blank=Mat.zeros(240,320,CvType.CV_8UC1);
        try{Core.setRNGSeed(1729);Core.randu(a,0,255);Imgproc.GaussianBlur(a,a,new Size(5,5),1);
            transform.put(0,2,6.25);transform.put(1,2,-3.5);Imgproc.warpAffine(a,b,transform,a.size());
            NaturalSceneCalibration.Matches matches=NaturalSceneCalibration.track(a,b);
            assertTrue("Tracking should preserve texture",matches.a.size()>100);
            double[] errors=new double[matches.a.size()];
            for(int i=0;i<errors.length;i++)errors[i]=Math.hypot(matches.b.get(i).x-matches.a.get(i).x-6.25,matches.b.get(i).y-matches.a.get(i).y+3.5);
            Arrays.sort(errors);assertTrue("Subpixel tracking error",errors[(int)(errors.length*.9)]<.3);
            assertTrue(NaturalSceneCalibration.track(a,blank).a.size()<10);
        }finally{a.release();b.release();transform.release();blank.release();}
    }
    @Test public void rejectedSequenceLogsActualScreeningReasonsBeforeFocalEstimation()throws Exception{
        Path root=Files.createTempDirectory("calibration-diagnostics"),left=Files.createDirectories(root.resolve("images/left")),right=Files.createDirectories(root.resolve("images/right"));
        Path log=root.resolve("self-calibration.log"),output=root.resolve("result.properties");
        List<Path> created=new ArrayList<>();Mat blank=Mat.zeros(120,160,CvType.CV_8UC1);
        try{
            for(int i=0;i<25;i++)for(Path eye:Arrays.asList(left,right)){
                Path image=eye.resolve(String.format(Locale.ROOT,"%06d.png",i));created.add(image);assertTrue(Imgcodecs.imwrite(image.toString(),blank));}
            try(NaturalSceneCalibration estimator=new NaturalSceneCalibration(160,120,1000,1.5)){
                try{estimator.run(root,.065,output,log,()->false,s->{});fail("Blank sequence must not export");}
                catch(java.io.IOException expected){assertTrue(expected.getMessage(),expected.getMessage().contains("左 0 / 右 0"));}
            }
            String diagnostics=Files.readString(log);
            assertTrue(diagnostics.contains("TEMPORAL_SUMMARY"));assertTrue(diagnostics.contains("too_few_matches"));
            assertTrue(diagnostics.contains("LK-forward-backward"));assertTrue(diagnostics.contains("features=0,0"));assertFalse(Files.exists(output));
        }finally{blank.release();for(Path image:created)Files.deleteIfExists(image);Files.deleteIfExists(log);Files.deleteIfExists(output);
            Files.delete(left);Files.delete(right);Files.delete(root.resolve("images"));Files.delete(root);}
    }
    @Test public void recoversFocalFromIndependentCameraMotions()throws Exception{
        List<RealMatrix> fundamentals=new ArrayList<>();
        RealMatrix inverse=new LUDecomposition(NaturalSceneCalibration.intrinsic(530,640,480)).getSolver().getInverse();
        for(int i=0;i<12;i++){
            double x=.04+i*.013,y=-.09+i*.016;
            RealMatrix rx=MatrixUtils.createRealMatrix(new double[][]{{1,0,0},{0,Math.cos(x),-Math.sin(x)},{0,Math.sin(x),Math.cos(x)}});
            RealMatrix ry=MatrixUtils.createRealMatrix(new double[][]{{Math.cos(y),0,Math.sin(y)},{0,1,0},{-Math.sin(y),0,Math.cos(y)}});
            double ty=.03*Math.sin(i),tz=.02*Math.cos(i);
            RealMatrix cross=MatrixUtils.createRealMatrix(new double[][]{{0,-tz,ty},{tz,0,.1},{-ty,-.1,0}});
            fundamentals.add(inverse.transpose().multiply(cross.multiply(rx.multiply(ry))).multiply(inverse));
        }
        assertEquals(530,NaturalSceneCalibration.estimateFocal(fundamentals,640,480),.1);
    }
    @Test public void rejectsUnobservableFocalInPureTranslation()throws Exception{
        List<RealMatrix> fundamentals=new ArrayList<>();
        for(int i=0;i<8;i++)fundamentals.add(MatrixUtils.createRealMatrix(new double[][]{{0,0,0},{0,0,1},{0,-1,0}}));
        try{NaturalSceneCalibration.estimateFocal(fundamentals,640,480);fail("Unobservable focal must not be guessed");}catch(java.io.IOException expected){}
    }
    @Test public void logExistsEvenWhenSequenceIsMissing()throws Exception{
        OpenCvNative.load();Path sequence=Files.createTempDirectory("java-calibration-test");Path log=sequence.resolve("self-calibration.log");
        try(NaturalSceneCalibration estimator=new NaturalSceneCalibration(640,480,3500,1.5)){
            try{estimator.run(sequence,.065,sequence.resolve("result.properties"),log,()->false,s->{});fail();}catch(java.io.IOException expected){}
            assertTrue(Files.size(log)>0);assertFalse(Files.exists(sequence.resolve("result.properties")));
        }finally{Files.deleteIfExists(log);Files.deleteIfExists(sequence);}
    }
}
