package winw.ai.slam;

import java.nio.file.*;
import java.util.*;
import org.junit.Test;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import static org.junit.Assert.*;

public class RecordingSlamEvaluationTest {
    @Test public void imageOnlyEntryNeedsOnlyBaselineAndImages()throws Exception {
        OpenCvNative.load();Path dir=Files.createTempDirectory("slam-image-only");
        try {
            StringBuilder manifest=new StringBuilder();
            for(int i=0;i<4;i++){
                Mat l=StereoSlamTest.scene(i*.01),r=StereoSlamTest.scene(i*.01+.12);
                try{assertTrue(Imgcodecs.imwrite(dir.resolve("l"+i+".png").toString(),l));assertTrue(Imgcodecs.imwrite(dir.resolve("r"+i+".png").toString(),r));}
                finally{l.release();r.release();}
                manifest.append(i+" l"+i+".png r"+i+".png\n");
            }
            Files.writeString(dir.resolve("manifest.txt"),manifest);
            RecordingSlamEvaluation.main(new String[]{dir.resolve("manifest.txt").toString(),"auto:0.065",dir.resolve("output").toString(),"orb"});
            Properties model=new Properties();try(java.io.Reader reader=Files.newBufferedReader(dir.resolve("output/experimental-calibration.properties"))){model.load(reader);}
            assertEquals(.065,Double.parseDouble(model.getProperty("baseline")),0);
            assertTrue(Files.readString(dir.resolve("output/report.txt")).contains("image-width focal prior"));
            assertTrue(Files.readAllLines(dir.resolve("output/trajectory.tum")).size()>1);
            assertTrue(Files.readAllLines(dir.resolve("output/trajectory.tum")).get(0).contains("projective approximation"));
        }finally{try(java.util.stream.Stream<Path> files=Files.walk(dir)){for(Path p:(Iterable<Path>)files.sorted(Comparator.reverseOrder())::iterator)Files.delete(p);}}
    }
    @Test public void reportsLostFramesWithoutInventingTrajectory()throws Exception {
        OpenCvNative.load();Path dir=Files.createTempDirectory("slam-evaluation");
        try {
            StringBuilder manifest=new StringBuilder();
            for(int i=0;i<3;i++) {
                Mat l=i==1?Mat.zeros(480,640,CvType.CV_8UC1):StereoSlamTest.scene(0);
                Mat r=i==1?Mat.zeros(480,640,CvType.CV_8UC1):StereoSlamTest.scene(.12);
                try{assertTrue(Imgcodecs.imwrite(dir.resolve("l"+i+".png").toString(),l));assertTrue(Imgcodecs.imwrite(dir.resolve("r"+i+".png").toString(),r));}
                finally{l.release();r.release();}
                manifest.append(i+" l"+i+".png r"+i+".png\n");
            }
            Files.writeString(dir.resolve("manifest.txt"),manifest);
            RecordingSlamEvaluation.main(new String[]{dir.resolve("manifest.txt").toString(),"docs/slam/rectified-example.properties",dir.resolve("output").toString(),"orb"});
            List<String> rows=Files.readAllLines(dir.resolve("output/frames.csv"));
            assertEquals(4,rows.size());assertTrue(rows.get(2).contains(",LOST,"));assertTrue(rows.get(2).endsWith("NaN,NaN,NaN"));
            assertTrue(rows.get(3).contains(",RELOCALIZED,"));
            assertEquals(3,Files.readAllLines(dir.resolve("output/trajectory.tum")).size());
        }finally{try(java.util.stream.Stream<Path> files=Files.walk(dir)){for(Path p:(Iterable<Path>)files.sorted(Comparator.reverseOrder())::iterator)Files.delete(p);}}
    }
}
