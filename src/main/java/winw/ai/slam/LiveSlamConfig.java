package winw.ai.slam;

import java.io.*;
import java.nio.file.*;
import java.util.Properties;

/** Camera and bird's-eye display settings; independent of measured calibration. */
public final class LiveSlamConfig {
    public BirdEyePanel.Style style=BirdEyePanel.Style.DRIVING;
    public final OccupancyConfig occupancy=new OccupancyConfig();
    public enum Layout { SIDE_BY_SIDE, TOP_BOTTOM }
    public Layout layout=Layout.SIDE_BY_SIDE;
    public boolean swapEyes=false,dense=true;
    public int backend=0,disparities=128,sampleStep=6,maxPointsPerKeyframe=5000,maxKeyframes=80;
    public double fps=30,pixelsPerMetre=45,minDepth=.3,maxDepth=15,minY=-2,maxY=.8,cellSize=.05;
    public String fourcc="MJPG";
    public static LiveSlamConfig load(Path file)throws IOException{
        LiveSlamConfig c=new LiveSlamConfig();if(file==null)return c;
        Properties p=new Properties();try(Reader r=Files.newBufferedReader(file)){p.load(r);}
        c.layout=Layout.valueOf(p.getProperty("camera.layout",c.layout.name()));
        c.style=BirdEyePanel.Style.valueOf(p.getProperty("view.mode",c.style.name()));
        c.swapEyes=bool(p,"camera.swapEyes",c.swapEyes);c.backend=integer(p,"camera.backend",c.backend);
        c.fps=number(p,"camera.fps",c.fps);c.fourcc=p.getProperty("camera.fourcc",c.fourcc);
        c.dense=bool(p,"map.dense",c.dense);c.disparities=integer(p,"map.disparities",c.disparities);
        c.sampleStep=integer(p,"map.sampleStep",c.sampleStep);c.maxPointsPerKeyframe=integer(p,"map.maxPointsPerKeyframe",c.maxPointsPerKeyframe);
        c.maxKeyframes=integer(p,"slam.maxKeyframes",c.maxKeyframes);
        c.pixelsPerMetre=number(p,"view.pixelsPerMetre",c.pixelsPerMetre);
        c.minDepth=number(p,"map.minDepth",c.minDepth);c.maxDepth=number(p,"map.maxDepth",c.maxDepth);
        c.minY=number(p,"view.minY",c.minY);c.maxY=number(p,"view.maxY",c.maxY);c.cellSize=number(p,"view.cellSize",c.cellSize);
        c.occupancy.enabled=bool(p,"occ.enabled",c.occupancy.enabled);
        c.occupancy.resolution=number(p,"occ.resolution",c.occupancy.resolution);c.occupancy.radius=number(p,"occ.radius",c.occupancy.radius);
        c.occupancy.maxAgeSeconds=number(p,"occ.maxAgeSeconds",c.occupancy.maxAgeSeconds);
        c.occupancy.maxFrames=integer(p,"occ.maxFrames",c.occupancy.maxFrames);c.occupancy.maxVoxels=integer(p,"occ.maxVoxels",c.occupancy.maxVoxels);
        c.occupancy.frameInterval=integer(p,"occ.frameInterval",c.occupancy.frameInterval);c.occupancy.minHitFrames=integer(p,"occ.minHitFrames",c.occupancy.minHitFrames);
        c.occupancy.hitProbability=number(p,"occ.hitProbability",c.occupancy.hitProbability);c.occupancy.missProbability=number(p,"occ.missProbability",c.occupancy.missProbability);
        c.occupancy.disparitySigma=number(p,"occ.disparitySigma",c.occupancy.disparitySigma);
        c.occupancy.minDepthConfidence=number(p,"occ.minDepthConfidence",c.occupancy.minDepthConfidence);
        c.validate();return c;
    }
    public void validate(){
        occupancy.validate();
        if(backend<0||disparities<16||disparities%16!=0||sampleStep<1||maxPointsPerKeyframe<1||maxKeyframes<2||maxKeyframes>200
            ||!positive(fps)||!positive(pixelsPerMetre)||!positive(minDepth)||!positive(maxDepth)||maxDepth<=minDepth
            ||!Double.isFinite(minY)||!Double.isFinite(maxY)||maxY<=minY||!positive(cellSize)
            ||(!fourcc.isEmpty()&&fourcc.length()!=4))throw new IllegalArgumentException("Invalid live SLAM configuration");
    }
    private static boolean positive(double v){return Double.isFinite(v)&&v>0;}
    private static int integer(Properties p,String key,int fallback){return Integer.parseInt(p.getProperty(key,Integer.toString(fallback)));}
    private static double number(Properties p,String key,double fallback){return Double.parseDouble(p.getProperty(key,Double.toString(fallback)));}
    private static boolean bool(Properties p,String key,boolean fallback){String value=p.getProperty(key,Boolean.toString(fallback));
        if(!value.equals("true")&&!value.equals("false"))throw new IllegalArgumentException("Invalid boolean: "+key);return Boolean.parseBoolean(value);}
}
