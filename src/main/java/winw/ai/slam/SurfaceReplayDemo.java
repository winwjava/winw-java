package winw.ai.slam;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.nio.file.*;
import java.util.*;
import java.io.*;
import javax.imageio.ImageIO;
import javax.swing.*;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

/** Full-resolution stereo quality replay with reproducible per-frame coverage and surface export. */
public final class SurfaceReplayDemo {
    public static void main(String[] args)throws Exception {
        if(args.length<3)throw new IllegalArgumentException("manifest image-only-replay-directory output [limit]");
        OpenCvNative.load();Path manifest=Paths.get(args[0]).toAbsolutePath(),root=manifest.getParent(),model=Paths.get(args[1]),out=Paths.get(args[2]);
        Files.createDirectories(out.resolve("surfaces"));Files.createDirectories(out.resolve("frames"));
        Properties p=new Properties();try(Reader r=Files.newBufferedReader(model.resolve("experimental-calibration.properties"))){p.load(r);}double baseline=Double.parseDouble(p.getProperty("baseline"));
        Properties warp=new Properties();try(Reader r=Files.newBufferedReader(model.resolve("vertical-warp.properties"))){warp.load(r);}
        Mat h=Mat.eye(3,3,CvType.CV_64F);String[] values=warp.getProperty("matrix").split(",");for(int i=0;i<9;i++)h.put(i/3,i%3,Double.parseDouble(values[i]));
        int limit=args.length>3?Integer.parseInt(args[3]):Integer.MAX_VALUE,index=0;long started=System.nanoTime();double totalMs=0;
        try(StereoSurfaceReconstructor reconstructor=new StereoSurfaceReconstructor(128,4);BufferedWriter csv=Files.newBufferedWriter(out.resolve("quality.csv"))){
            csv.write("frame,raw_valid,regularized_valid,trusted_pixels,surface_triangles,ms\n");
            for(String line:Files.readAllLines(manifest)){line=line.trim();if(line.isEmpty()||line.startsWith("#"))continue;if(index>=limit)break;
                String[] fields=line.split("\\s+");Mat left=Imgcodecs.imread(root.resolve(fields[1]).toString()),right=Imgcodecs.imread(root.resolve(fields[2]).toString()),rectRight=new Mat(),ones=new Mat(left.size(),CvType.CV_8U,new Scalar(255)),rightMask=new Mat();
                try{Imgproc.warpPerspective(right,rectRight,h,right.size());Imgproc.warpPerspective(ones,rightMask,h,right.size());
                    StereoSurfaceReconstructor.Result result=reconstructor.compute(left,rectRight,ones,rightMask,0,baseline);
                    result.surface.write(out.resolve(String.format(Locale.ROOT,"surfaces/%03d.surf",index)));
                    Files.copy(root.resolve(fields[1]),out.resolve(String.format(Locale.ROOT,"frames/%03d.png",index)),StandardCopyOption.REPLACE_EXISTING);
                    csv.write(String.format(Locale.ROOT,"%d,%d,%d,%d,%d,%.3f%n",index,result.rawValid,result.regularizedValid,result.trustedPixels,result.triangles,result.milliseconds));csv.flush();totalMs+=result.milliseconds;
                    if(index==12||index==0&&limit==1)preview(result.surface,left,out,index);
                }finally{left.release();right.release();rectRight.release();ones.release();rightMask.release();}index++;
            }
        }finally{h.release();}
        Files.writeString(out.resolve("report.txt"),"Frames: "+index+"\nMean stereo + mesh ms: "+totalMs/Math.max(1,index)+"\nTotal seconds: "+(System.nanoTime()-started)/1e9+"\nCPU SGBM left/right + WLS + image-organized triangles. Regularized low-confidence geometry is inference, not confirmed occupancy. No ground-truth depth or metric accuracy test available.\n");
    }
    private static void preview(OccupancySurfaceMesh surface,Mat source,Path out,int index)throws Exception {
        BufferedImage camera=LiveStereoSlam.bufferedImage(source);
        SwingUtilities.invokeAndWait(()->{try{Scene3DPanel panel=new Scene3DPanel();panel.updateSurface(surface,"实拍第 "+index+" 帧 · 双向视差 / 边缘保持滤波 · 含平滑推断");
            BufferedImage image=new BufferedImage(1500,900,BufferedImage.TYPE_INT_RGB);Graphics2D g=image.createGraphics();try{g.setColor(new Color(5,7,10));g.fillRect(0,0,1500,900);g.drawImage(panel.image(1160,900),0,0,null);g.drawImage(camera,1170,90,320,240,null);g.setColor(Color.LIGHT_GRAY);g.setFont(new Font("Microsoft YaHei",Font.PLAIN,13));g.drawString("对应原图 / 第 "+index+" 帧",1170,70);g.drawString("保留逐像素深度后再重建表面",1170,365);g.drawString("低置信区域来自正则化推断",1170,390);ImageIO.write(image,"png",out.resolve("surface-preview.png").toFile());panel.setTextured(false);g.drawImage(panel.image(1160,900),0,0,null);ImageIO.write(image,"png",out.resolve("surface-gray-preview.png").toFile());}finally{g.dispose();}
        }catch(Exception e){throw new RuntimeException(e);}});
    }
}
