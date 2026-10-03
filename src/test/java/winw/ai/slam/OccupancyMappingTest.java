package winw.ai.slam;

import static org.junit.Assert.*;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.nio.file.*;
import java.util.*;
import java.util.List;
import javax.imageio.ImageIO;
import javax.swing.SwingUtilities;
import org.junit.Test;
import org.opencv.core.Mat;
import winw.ai.slam.OccupancySnapshot.State;
import winw.ai.slam.TemporalOccupancyMapper.DepthSample;

public class OccupancyMappingTest {
    private static final List<Pose> ANCHORS=Collections.singletonList(Pose.identity());
    private TemporalOccupancyMapper mapper(){return new TemporalOccupancyMapper(new OccupancyConfig(),-1,1);}
    private DepthSample point(double z){return new DepthSample(.1,.1,z,1);}
    private void observe(TemporalOccupancyMapper mapper,double time,List<DepthSample> points){
        mapper.observe(time,0,Pose.identity(),points,ANCHORS,Pose.identity());
    }
    @Test public void rayDistinguishesOccupiedFreeAndOccludedUnknown(){
        TemporalOccupancyMapper mapper=mapper();observe(mapper,0,Collections.singletonList(point(4.1)));observe(mapper,1,Collections.singletonList(point(4.1)));
        OccupancySnapshot map=mapper.snapshot();assertEquals(State.OCCUPIED,map.stateAt(.1,.1,4.1));
        assertEquals(State.FREE,map.stateAt(.1,.1,2.1));assertEquals(State.UNKNOWN,map.stateAt(.1,.1,5.1));
        assertEquals(State.UNKNOWN,map.stateAt(-2,.1,2));assertEquals(.5,map.probabilityAt(-2,.1,2),0);
        assertEquals(State.UNKNOWN,map.stateAt(0,0,0));
    }
    @Test public void duplicatePixelsCannotReplaceMultipleFrameConfirmation(){
        TemporalOccupancyMapper mapper=mapper();observe(mapper,0,Collections.nCopies(1000,point(4.1)));
        OccupancySnapshot map=mapper.snapshot();assertEquals(State.UNKNOWN,map.stateAt(.1,.1,4.1));
        assertEquals(.72,map.probabilityAt(.1,.1,4.1),1e-8);
        assertEquals(1,map.voxels.stream().filter(v->v.hitFrames>0).findFirst().get().hitFrames);
    }
    @Test public void hitWinsOverCrossingFreeRayInSameFrame(){
        TemporalOccupancyMapper mapper=mapper();observe(mapper,0,Arrays.asList(point(4.1),point(8.1)));
        assertEquals(.72,mapper.snapshot().probabilityAt(.1,.1,4.1),1e-8);
    }
    @Test public void repeatedFreeObservationsClearAnOldObstacle(){
        TemporalOccupancyMapper mapper=mapper();observe(mapper,0,Collections.singletonList(point(4.1)));observe(mapper,1,Collections.singletonList(point(4.1)));
        for(int i=2;i<7;i++)observe(mapper,i,Collections.singletonList(point(8.1)));
        assertEquals(State.FREE,mapper.snapshot().stateAt(.1,.1,4.1));
    }
    @Test public void staleEvidenceExpiresWhileTrackingIsLost(){
        TemporalOccupancyMapper mapper=mapper();observe(mapper,0,Collections.singletonList(point(4.1)));observe(mapper,1,Collections.singletonList(point(4.1)));
        mapper.refresh(20,ANCHORS,null);OccupancySnapshot map=mapper.snapshot();
        assertEquals(0,map.activeFrames);assertEquals(0,map.voxels.size());assertEquals(.5,map.probabilityAt(.1,.1,4.1),0);
    }
    @Test public void loopCorrectionReplaysBothOccupiedAndFreeEvidence(){
        TemporalOccupancyMapper mapper=mapper();observe(mapper,0,Collections.singletonList(point(4.1)));observe(mapper,1,Collections.singletonList(point(4.1)));
        Pose shift=Pose.fromVector(new double[]{0,0,0,1,0,0});mapper.refresh(2,Collections.singletonList(shift),shift);
        OccupancySnapshot map=mapper.snapshot();assertEquals(State.UNKNOWN,map.stateAt(.1,.1,4.1));
        assertEquals(State.OCCUPIED,map.stateAt(1.1,.1,4.1));assertEquals(State.FREE,map.stateAt(1.1,.1,2.1));
        assertEquals(State.UNKNOWN,map.stateAt(.1,.1,2.1));
    }
    @Test public void weakDepthCannotMarkFreeOrOccupied(){
        TemporalOccupancyMapper mapper=mapper();observe(mapper,0,Collections.singletonList(new DepthSample(.1,.1,4.1,.05)));
        assertEquals(0,mapper.snapshot().voxels.size());
    }
    @Test public void uncertainSurfaceKeepsNearbyVoxelsUnknown(){
        TemporalOccupancyMapper mapper=mapper();
        observe(mapper,0,Collections.singletonList(new DepthSample(.1,.1,4.1,1,.5)));
        assertEquals(State.FREE,mapper.snapshot().stateAt(.1,.1,2.1));
        assertEquals(State.UNKNOWN,mapper.snapshot().stateAt(.1,.1,3.6));
    }
    @Test public void voxelBudgetRetractsOldScansInsteadOfDroppingArbitraryCells(){
        OccupancyConfig config=new OccupancyConfig();config.maxVoxels=100;config.radius=15;
        TemporalOccupancyMapper mapper=new TemporalOccupancyMapper(config,-1,1);
        for(int i=0;i<10;i++)observe(mapper,i,Collections.singletonList(new DepthSample(i-5,.1,10.1,1)));
        OccupancySnapshot map=mapper.snapshot();assertTrue(map.voxels.size()<=100);assertTrue(map.capacityLimited);
    }
    @Test public void bevFreeRequiresEveryHeightLayerToBeObservedFree(){
        List<OccupancySnapshot.Voxel> voxels=new ArrayList<>();double r=.25;
        voxels.add(new OccupancySnapshot.Voxel(new OccupancySnapshot.VoxelKey(0,0,1),r,.2,0,3,State.FREE));
        OccupancySnapshot partial=new OccupancySnapshot(voxels,r,3,false);
        assertEquals(0,BirdEyeMap.fromOccupancy(partial,0,.5).freeCells.size());
        voxels.add(new OccupancySnapshot.Voxel(new OccupancySnapshot.VoxelKey(0,1,1),r,.2,0,3,State.FREE));
        assertEquals(1,BirdEyeMap.fromOccupancy(new OccupancySnapshot(voxels,r,3,false),0,.5).freeCells.size());
        voxels.set(1,new OccupancySnapshot.Voxel(new OccupancySnapshot.VoxelKey(0,1,1),r,.85,3,0,State.OCCUPIED));
        BirdEyeMap occupied=BirdEyeMap.fromOccupancy(new OccupancySnapshot(voxels,r,3,false),0,.5);
        assertEquals(0,occupied.freeCells.size());assertEquals(1,occupied.cells.size());
    }
    @Test public void realStereoAndSlamFeedOccupancyAndExports()throws Exception{
        OpenCvNative.load();StereoCalibration camera=new StereoCalibration(640,480,500,500,320,240,320,.12);
        LiveSlamConfig config=new LiveSlamConfig();config.maxDepth=12;config.occupancy.disparitySigma=.3;
        TemporalOccupancyMapper occupancy=new TemporalOccupancyMapper(config.occupancy,-2,.8);
        try(StereoSlam slam=new StereoSlam(camera,new SlamConfig());DenseStereoMapper depth=new DenseStereoMapper(camera,config)){
            for(int i=0;i<4;i++){Mat left=StereoSlamTest.scene(i*.01),right=StereoSlamTest.scene(i*.01+.12);
                try{StereoSlam.Result result=slam.process(left,right,i);assertNotNull(result.pose);
                    SlamSnapshot snapshot=slam.snapshot(false);int anchor=snapshot.keyframes.size()-1;
                    occupancy.observe(i,anchor,snapshot.keyframes.get(anchor).inverse().multiply(result.pose),depth.measure(left,right),snapshot.keyframes,result.pose);
                }finally{left.release();right.release();}}
            OccupancySnapshot map=occupancy.snapshot();assertTrue(map.occupiedCount>100);assertTrue(map.freeCount>100);
            Path csv=Files.createTempFile("occupancy-test",".csv"),ply=Files.createTempFile("occupancy-test",".ply");
            try{map.writeCsv(csv);map.writeOccupiedPly(ply);assertEquals(map.voxels.size()+2,Files.readAllLines(csv).size());
                assertTrue(Files.readString(ply).contains("element vertex "+map.occupiedCount));}
            finally{Files.deleteIfExists(csv);Files.deleteIfExists(ply);}
        }
    }
    @Test public void occupancyPreviewArtifact()throws Exception{
        OccupancyConfig config=new OccupancyConfig();config.radius=18;config.resolution=.25;
        TemporalOccupancyMapper mapper=new TemporalOccupancyMapper(config,-.5,.5);
        // Synthetic measured rays to wall/obstacle surfaces, with several camera heights.
        for(int frame=0;frame<4;frame++){
            List<DepthSample> points=new ArrayList<>();
            for(double x=-5;x<=5;x+=.15)for(double y=-.4;y<=.4;y+=.18){
                double z=(x>-.8&&x<.8)?5.1:12.1;points.add(new DepthSample(x,y,z,1));}
            observe(mapper,frame,points);
        }
        OccupancySnapshot snapshot=mapper.snapshot();BirdEyeMap map=BirdEyeMap.fromOccupancy(snapshot,-.5,.5);
        Path imagePath=Paths.get("target/slam-occupancy-preview.png");Files.createDirectories(imagePath.getParent());
        snapshot.writeCsv(Paths.get("target/slam-occupancy-preview.csv"));
        SwingUtilities.invokeAndWait(()->{
            BirdEyePanel panel=new BirdEyePanel(43);panel.setSize(1000,900);panel.setStyle(BirdEyePanel.Style.DRIVING);
            panel.update(map,SlamSnapshot.empty(),Pose.identity());BufferedImage image=panel.image(1000,900);Graphics2D g=image.createGraphics();
            g.setColor(new Color(250,251,253,240));g.fillRoundRect(18,105,430,32,8,8);g.setColor(new Color(90,110,133));
            g.drawString("SYNTHETIC OCC TEST - NO NEURAL NETWORK INFERENCE",30,126);g.dispose();
            try{ImageIO.write(image,"png",imagePath.toFile());}catch(java.io.IOException error){throw new java.io.UncheckedIOException(error);}
        });
        assertTrue(snapshot.occupiedCount>100);assertTrue(Files.size(imagePath)>10000);
    }
}
