package winw.ai.slam;

import static org.junit.Assert.*;
import java.awt.*;
import java.awt.geom.Point2D;
import java.awt.image.BufferedImage;
import java.nio.file.*;
import java.util.*;
import java.util.List;
import javax.imageio.ImageIO;
import javax.swing.SwingUtilities;
import org.junit.Test;

public class DrivingViewTest {
    @Test public void headingProjectionHandlesNinetyDegreeTurn(){
        DrivingProjection projection=new DrivingProjection(10,20,Math.PI/2,45,900,680);
        assertArrayEquals(new double[]{0,5},projection.local(15,20),1e-10);
        assertArrayEquals(new double[]{1,0},projection.local(10,19),1e-10);
        Point2D origin=projection.project(10,20),forward=projection.project(15,20),right=projection.project(10,19);
        assertEquals(450,forward.getX(),1e-10);assertTrue(forward.getY()<origin.getY());assertTrue(right.getX()>origin.getX());
        Pose pose=Pose.fromVector(new double[]{0,Math.PI/2,0,10,0,20});assertEquals(Math.PI/2,DrivingProjection.yaw(pose),1e-10);
    }
    @Test public void lostPoseHidesCarrierAndFullMapSwitchesMode()throws Exception{
        SwingUtilities.invokeAndWait(()->{
            BirdEyePanel panel=new BirdEyePanel(45);panel.setSize(900,680);panel.setStyle(BirdEyePanel.Style.DRIVING);
            panel.update(null,SlamSnapshot.empty(),Pose.identity());BufferedImage tracking=panel.image(900,680);
            assertTrue(new Color(tracking.getRGB(450,525)).getRed()<160);
            panel.update(null,SlamSnapshot.empty(),null);BufferedImage lost=panel.image(900,680);
            assertTrue(new Color(lost.getRGB(450,525)).getRed()>200);
            panel.fitMap();assertEquals(BirdEyePanel.Style.MAP,panel.style());
        });
    }
    @Test public void drivingPreviewArtifact()throws Exception{
        List<SlamSnapshot.MapPoint> points=new ArrayList<>();
        // Synthetic static surfaces. These are neither detected lanes nor classified cars.
        for(double z=-12;z<38;z+=.04)for(int i=0;i<3;i++){
            double bend=Math.max(0,z-10)*.025;points.add(new SlamSnapshot.MapPoint(-4.2+bend,0,z));points.add(new SlamSnapshot.MapPoint(4.2+bend,0,z));}
        rectangle(points,-2,10,1.6,3);rectangle(points,2.2,19,1.7,2.8);rectangle(points,-3,27,.8,1.1);
        List<SlamSnapshot.TrackPoint> track=new ArrayList<>();
        for(int i=0;i<=100;i++)track.add(new SlamSnapshot.TrackPoint(Pose.fromVector(new double[]{0,0,0,.12*Math.sin(i*.05),0,-12+i*.12}),i==0));
        SlamSnapshot snapshot=new SlamSnapshot(points,track,Collections.emptyList());BirdEyeMap map=BirdEyeMap.build(points,new LiveSlamConfig());
        Path output=Paths.get("target/slam-driving-view-preview.png");Files.createDirectories(output.getParent());
        SwingUtilities.invokeAndWait(()->{
            BirdEyePanel panel=new BirdEyePanel(38);panel.setSize(1000,900);panel.setStyle(BirdEyePanel.Style.DRIVING);panel.update(map,snapshot,Pose.identity());
            BufferedImage image=panel.image(1000,900);Graphics2D g=image.createGraphics();
            g.setColor(new Color(248,250,253,245));g.fillRoundRect(18,108,440,32,8,8);
            g.setFont(new Font(Font.SANS_SERIF,Font.BOLD,11));g.setColor(new Color(105,118,135));g.drawString("SYNTHETIC GEOMETRY EXAMPLE - NOT CAMERA RESULTS",30,128);g.dispose();
            try{ImageIO.write(image,"png",output.toFile());}catch(java.io.IOException error){throw new java.io.UncheckedIOException(error);}
        });
        assertTrue(Files.size(output)>10000);
    }
    private void rectangle(List<SlamSnapshot.MapPoint> points,double x,double z,double width,double length){
        for(double a=0;a<width;a+=.04)for(int i=0;i<3;i++){
            points.add(new SlamSnapshot.MapPoint(x+a,-.9+i*.45,z));points.add(new SlamSnapshot.MapPoint(x+a,-.9+i*.45,z+length));}
        for(double a=0;a<length;a+=.04)for(int i=0;i<3;i++){
            points.add(new SlamSnapshot.MapPoint(x,-.9+i*.45,z+a));points.add(new SlamSnapshot.MapPoint(x+width,-.9+i*.45,z+a));}
    }
}
