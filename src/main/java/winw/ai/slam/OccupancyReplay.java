package winw.ai.slam;

import java.io.*;
import java.nio.file.*;
import java.util.*;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

/** Replay images into actual occupied/free/unknown voxel volumes, including local views during LOST. */
public final class OccupancyReplay {
    public static void main(String[] args)throws Exception {
        if(args.length!=3)throw new IllegalArgumentException("manifest image-only-replay-directory output-directory");
        OpenCvNative.load();Core.setRNGSeed(42);
        Path manifest=Paths.get(args[0]).toAbsolutePath(),root=manifest.getParent(),replay=Paths.get(args[1]),out=Paths.get(args[2]);
        Files.createDirectories(out.resolve("frames"));Files.createDirectories(out.resolve("volumes"));
        Properties warp=new Properties();try(Reader reader=Files.newBufferedReader(replay.resolve("vertical-warp.properties"))){warp.load(reader);}
        Mat h=Mat.eye(3,3,CvType.CV_64F);String[] v=warp.getProperty("matrix").split(",");for(int j=0;j<9;j++)h.put(j/3,j%3,Double.parseDouble(v[j]));
        LiveSlamConfig settings=new LiveSlamConfig();settings.sampleStep=4;settings.maxPointsPerKeyframe=16000;
        settings.minDepth=.15;settings.maxDepth=6;settings.minY=-3;settings.maxY=3;
        OccupancyConfig occ=settings.occupancy;occ.resolution=.05;occ.radius=7;occ.minHitFrames=1;occ.maxFrames=20;occ.maxVoxels=150000;occ.maxAgeSeconds=8;
        Properties model=new Properties();try(Reader reader=Files.newBufferedReader(replay.resolve("experimental-calibration.properties"))){model.load(reader);}
        double baseline=Double.parseDouble(model.getProperty("baseline"));
        SlamConfig tracking=new SlamConfig();tracking.features=3500;tracking.maxKeyframes=200;tracking.keyframeInterval=2;
        tracking.minDepth=.1*baseline;tracking.maxDepth=1000*baseline;tracking.keyframeTranslation=2*baseline;
        TemporalOccupancyMapper global=new TemporalOccupancyMapper(occ,-3,3);
        OccupancySnapshot best=null;int bestFrame=-1,index=0,posed=0,localNonempty=0;long start=System.nanoTime();
        StringBuilder json=new StringBuilder("{\"resolution\":0.05,\"frames\":[");
        try(StereoRig rig=StereoRig.load(replay.resolve("experimental-calibration.properties"));DenseStereoMapper depth=new DenseStereoMapper(rig.calibration,settings);StereoSlam slam=new StereoSlam(rig.calibration,tracking)){
            for(String line:Files.readAllLines(manifest)){line=line.trim();if(line.isEmpty()||line.startsWith("#"))continue;
                String[] f=line.split("\\s+");double timestamp=Double.parseDouble(f[0]);
                Mat l=Imgcodecs.imread(root.resolve(f[1]).toString()),r=Imgcodecs.imread(root.resolve(f[2]).toString()),right=new Mat();
                try{Imgproc.warpPerspective(r,right,h,r.size());List<TemporalOccupancyMapper.DepthSample> samples=depth.measure(l,right);
                    TemporalOccupancyMapper local=new TemporalOccupancyMapper(occ,-3,3);
                    local.observe(timestamp,0,Pose.identity(),samples,Collections.singletonList(Pose.identity()),Pose.identity());
                    OccupancySnapshot snapshot=local.snapshot();if(snapshot.occupiedCount>0)localNonempty++;
                    snapshot.writeCsv(out.resolve(String.format(Locale.ROOT,"volumes/frame-%03d.csv",index)));
                    StereoSlam.Result result=slam.process(l,right,timestamp);SlamSnapshot map=slam.snapshot(false);
                    if(result.pose!=null){posed++;int anchor=map.keyframes.size()-1;
                        global.observe(timestamp,anchor,map.keyframes.get(anchor).inverse().multiply(result.pose),samples,map.keyframes,result.pose);
                    }else global.refresh(timestamp,map.keyframes,null);
                    OccupancySnapshot fused=global.snapshot();if(best==null||fused.occupiedCount>best.occupiedCount){best=fused;bestFrame=index;}
                    Files.copy(root.resolve(f[1]),out.resolve(String.format(Locale.ROOT,"frames/%03d.png",index)),StandardCopyOption.REPLACE_EXISTING);
                    if(index>0)json.append(',');json.append("{\"index\":").append(index).append(",\"time\":").append(timestamp).append(",\"tracking\":\"").append(result.state).append("\",\"occupied\":").append(snapshot.occupiedCount).append(",\"free\":").append(snapshot.freeCount).append(",\"samples\":").append(samples.size()).append(",\"voxels\":");
                    appendVoxels(json,snapshot,l,rig.calibration,7000);
                    json.append(",\"thumbnail\":\"").append(thumbnail(l)).append("\"}");index++;
                }finally{l.release();r.release();right.release();}
            }
            json.append("],\"globalFree\":").append(best.freeCount).append(",\"globalFrame\":").append(bestFrame).append(",\"globalVoxels\":");appendVoxels(json,best,null,rig.calibration,14000);json.append('}');
            best.writeCsv(out.resolve("global-best-occupancy.csv"));
        }finally{h.release();}
        Files.writeString(out.resolve("occupancy-data.json"),json);
        String report="Frames: "+index+"\nFrames with local occupied voxels: "+localNonempty+"\nPosed frames eligible for global fusion: "+posed+"\nBest global snapshot frame: "+bestFrame+"\nBest global occupied/free: "+best.occupiedCount+"/"+best.freeCount+"\nResolution: 0.05 nominal metres\nSeconds: "+(System.nanoTime()-start)/1e9+"\nLocal views use one observation, not temporal confirmation. Global snapshot is the richest historical snapshot, not final-time occupancy. Unobserved/occluded volume remains UNKNOWN. Image-only approximate geometry, no semantic labels or learned Tesla model. Viewer caps rendered voxels; CSV preserves all observed cells.\n";
        Files.writeString(out.resolve("report.txt"),report);System.out.println(report);
    }
    private static String thumbnail(Mat image)throws IOException {
        java.awt.image.BufferedImage thumb=new java.awt.image.BufferedImage(200,150,java.awt.image.BufferedImage.TYPE_INT_RGB);
        java.awt.Graphics2D g=thumb.createGraphics();
        try{g.drawImage(LiveStereoSlam.bufferedImage(image),0,0,200,150,null);}finally{g.dispose();}
        ByteArrayOutputStream bytes=new ByteArrayOutputStream();javax.imageio.ImageIO.write(thumb,"jpg",bytes);
        return Base64.getEncoder().encodeToString(bytes.toByteArray());
    }
    private static void appendVoxels(StringBuilder json,OccupancySnapshot snapshot,Mat image,StereoCalibration c,int limit){
        List<OccupancySnapshot.Voxel> cells=new ArrayList<>();for(OccupancySnapshot.Voxel voxel:snapshot.voxels)if(voxel.state==OccupancySnapshot.State.OCCUPIED)cells.add(voxel);
        cells.sort(Comparator.comparingInt((OccupancySnapshot.Voxel a)->a.iz).thenComparingInt(a->a.iy).thenComparingInt(a->a.ix));
        int step=Math.max(1,(cells.size()+limit-1)/limit);json.append('[');boolean first=true;
        for(int i=0;i<cells.size();i+=step){OccupancySnapshot.Voxel cell=cells.get(i);int color=0x4baea4;
            if(image!=null&&cell.z>0){int x=(int)Math.round(c.fx*cell.x/cell.z+c.cx),y=(int)Math.round(c.fy*cell.y/cell.z+c.cy);
                if(x>=0&&x<image.cols()&&y>=0&&y<image.rows()){double[] bgr=image.get(y,x);color=((int)bgr[2]<<16)|((int)bgr[1]<<8)|(int)bgr[0];}}
            if(!first)json.append(',');first=false;json.append('[').append(cell.ix).append(',').append(cell.iy).append(',').append(cell.iz).append(',').append(color).append(']');
        }json.append(']');
    }
}
