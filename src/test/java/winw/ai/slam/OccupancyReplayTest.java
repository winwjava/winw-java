package winw.ai.slam;

import java.nio.file.*;
import java.util.*;
import org.junit.Test;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import static org.junit.Assert.*;

public class OccupancyReplayTest {
    @Test public void emptyImageDoesNotProduceOccupiedVolume()throws Exception {
        OpenCvNative.load();Path dir=Files.createTempDirectory("occ-replay");
        try {
            StringBuilder manifest=new StringBuilder();
            for(int i=0;i<3;i++){
                Mat l=i==1?Mat.zeros(480,640,CvType.CV_8UC1):StereoSlamTest.scene(0);
                Mat r=i==1?Mat.zeros(480,640,CvType.CV_8UC1):StereoSlamTest.scene(.12);
                try{assertTrue(Imgcodecs.imwrite(dir.resolve("l"+i+".png").toString(),l));assertTrue(Imgcodecs.imwrite(dir.resolve("r"+i+".png").toString(),r));}
                finally{l.release();r.release();}
                manifest.append(i+" l"+i+".png r"+i+".png\n");
            }
            Files.writeString(dir.resolve("manifest.txt"),manifest);
            Files.copy(Paths.get("docs/slam/rectified-example.properties"),dir.resolve("experimental-calibration.properties"));
            Files.writeString(dir.resolve("vertical-warp.properties"),"matrix=1,0,0,0,1,0,0,0,1\n");
            OccupancyReplay.main(new String[]{dir.resolve("manifest.txt").toString(),dir.toString(),dir.resolve("out").toString()});
            assertEquals(2,Files.readAllLines(dir.resolve("out/volumes/frame-001.csv")).size());
            assertTrue(Files.readAllLines(dir.resolve("out/volumes/frame-000.csv")).size()>2);
            String data=Files.readString(dir.resolve("out/occupancy-data.json"));
            assertTrue(data.contains("\"tracking\":\"LOST\",\"occupied\":0"));
            assertTrue(data.contains("\"thumbnail\":\"/9j/"));
            assertFalse(Files.exists(dir.resolve("out/map.ply")));
        }finally{try(java.util.stream.Stream<Path> files=Files.walk(dir)){for(Path p:(Iterable<Path>)files.sorted(Comparator.reverseOrder())::iterator)Files.delete(p);}}
    }
}
