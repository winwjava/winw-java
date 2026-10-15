package winw.ai.slam;

import java.nio.file.*;
import java.util.*;
import org.junit.*;
import static org.junit.Assert.*;
import org.apache.commons.math3.linear.*;

public class NaturalSceneCalibrationTest {
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
