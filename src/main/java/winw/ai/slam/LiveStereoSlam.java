package winw.ai.slam;

import java.awt.*;
import java.awt.event.*;
import java.awt.image.*;
import java.io.*;
import java.nio.file.*;
import java.util.*;
import java.util.List;
import java.util.concurrent.atomic.*;
import javax.imageio.ImageIO;
import javax.swing.*;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

/** Live native Java dashboard. Capture/SLAM on one worker, latest-frame rendering on the EDT. */
public final class LiveStereoSlam {
    private LiveStereoSlam(){}
    private static final class Display {
        final BufferedImage left,right;
        final BirdEyeMap map;
        final SlamSnapshot snapshot;
        final StereoSlam.Result result;
        final double milliseconds,fps;
        Display(BufferedImage left,BufferedImage right,BirdEyeMap map,SlamSnapshot snapshot,StereoSlam.Result result,double ms,double fps){
            this.left=left;this.right=right;this.map=map;this.snapshot=snapshot;this.result=result;milliseconds=ms;this.fps=fps;
        }
    }
    private static final class ImagePanel extends JPanel {
        private BufferedImage image;private final String label;
        ImagePanel(String label){this.label=label;setPreferredSize(new Dimension(360,230));setBackground(new Color(20,30,42));}
        void image(BufferedImage next){image=next;repaint();}
        @Override protected void paintComponent(Graphics graphics){super.paintComponent(graphics);
            Graphics2D g=(Graphics2D)graphics.create();try{
                if(image!=null){double scale=Math.min((double)getWidth()/image.getWidth(),(double)(getHeight()-28)/image.getHeight());
                    int w=(int)(image.getWidth()*scale),h=(int)(image.getHeight()*scale);
                    g.drawImage(image,(getWidth()-w)/2,28+(getHeight()-28-h)/2,w,h,null);}
                g.setColor(Color.WHITE);g.drawString(label,12,19);
            }finally{g.dispose();}}
    }
    private static final class Dashboard {
        final JFrame frame=new JFrame("双目 SLAM · 实时鸟瞰地图");
        final BirdEyePanel birdEye;
        final ImagePanel left=new ImagePanel("LEFT / rectified"),right=new ImagePanel("RIGHT / rectified");
        final JLabel status=new JLabel("正在启动相机和 SLAM…"),details=new JLabel("等待同步双目图像");
        final AtomicBoolean stop=new AtomicBoolean(),finished=new AtomicBoolean();
        final AtomicReference<Display> latest=new AtomicReference<>();
        final JButton stopButton=new JButton("停止并保存");
        final javax.swing.Timer timer;
        Display shown;
        Dashboard(LiveSlamConfig config,Path output){
            birdEye=new BirdEyePanel(config.pixelsPerMetre);
            birdEye.setStyle(config.style);
            JPanel header=new JPanel(new BorderLayout(10,8));header.setBorder(BorderFactory.createEmptyBorder(12,16,10,16));
            JLabel title=new JLabel("双目 SLAM · 驾驶场景");title.setFont(new Font(Font.SANS_SERIF,Font.BOLD,20));header.add(title,BorderLayout.WEST);
            JPanel buttons=new JPanel(new FlowLayout(FlowLayout.RIGHT));
            JButton driving=new JButton("驾驶视图"),mapping=new JButton("地图视图");
            driving.addActionListener(e->birdEye.setStyle(BirdEyePanel.Style.DRIVING));mapping.addActionListener(e->birdEye.setStyle(BirdEyePanel.Style.MAP));
            JButton follow=new JButton("跟随"),origin=new JButton("原点"),fit=new JButton("全图"),save=new JButton("保存图像");
            follow.addActionListener(e->birdEye.followCamera());origin.addActionListener(e->birdEye.showOrigin());fit.addActionListener(e->birdEye.fitMap());
            save.addActionListener(e->{try{Files.createDirectories(output);ImageIO.write(birdEye.image(Math.max(800,birdEye.getWidth()),Math.max(600,birdEye.getHeight())),"png",output.resolve("bird-eye.png").toFile());
                status.setText("鸟瞰图已保存："+output.resolve("bird-eye.png"));}catch(IOException error){status.setText("保存失败："+error.getMessage());}});
            stopButton.addActionListener(e->{if(finished.get())frame.dispose();else requestStop();});
            for(JButton button:new JButton[]{driving,mapping,follow,origin,fit,save,stopButton})buttons.add(button);header.add(buttons,BorderLayout.EAST);
            JPanel previews=new JPanel(new GridLayout(2,1,0,8));previews.add(left);previews.add(right);
            previews.setBorder(BorderFactory.createEmptyBorder(0,8,0,8));
            JSplitPane split=new JSplitPane(JSplitPane.HORIZONTAL_SPLIT,birdEye,previews);split.setResizeWeight(.7);split.setDividerLocation(850);
            JPanel footer=new JPanel(new GridLayout(3,1,0,4));footer.setBorder(BorderFactory.createEmptyBorder(8,16,10,16));
            footer.add(status);footer.add(details);footer.add(new JLabel(String.format(Locale.ROOT,
                "滚轮缩放 · 拖动平移 · y=[%.2f, %.2f] m · OCC：体素障碍 / 淡绿已观测自由空间 / 蓝色历史轨迹；车辆图标表示相机载体",config.minY,config.maxY)));
            frame.add(header,BorderLayout.NORTH);frame.add(split,BorderLayout.CENTER);frame.add(footer,BorderLayout.SOUTH);
            frame.setSize(1320,820);frame.setMinimumSize(new Dimension(1050,650));frame.setLocationByPlatform(true);
            frame.setDefaultCloseOperation(WindowConstants.DO_NOTHING_ON_CLOSE);
            timer=new javax.swing.Timer(100,e->refresh());timer.start();
            frame.addWindowListener(new WindowAdapter(){
                @Override public void windowClosing(WindowEvent e){if(finished.get())frame.dispose();else requestStop();}
                @Override public void windowClosed(WindowEvent e){stop.set(true);timer.stop();}
            });
            frame.getRootPane().registerKeyboardAction(e->requestStop(),KeyStroke.getKeyStroke(KeyEvent.VK_ESCAPE,0),JComponent.WHEN_IN_FOCUSED_WINDOW);
            frame.setVisible(true);
        }
        void requestStop(){stop.set(true);stopButton.setEnabled(false);status.setText("正在停止采集并保存地图…");}
        void refresh(){Display display=latest.get();if(display==null||display==shown)return;shown=display;
            birdEye.update(display.map,display.snapshot,display.result.pose);left.image(display.left);right.image(display.right);
            StereoSlam.Result result=display.result;
            status.setText("状态："+result.state+"    "+(result.loopClosed?"回环已修正    ":"")
                +(result.mapFull?"地图容量已满，停止扩展    ":"")+"当前坐标："+(result.pose==null?"不可用":result.pose.toString()));
            details.setText(String.format(Locale.ROOT,"处理 %.1f FPS / %.1f ms    内点 %d    关键帧 %d    表面点 %d    栅格 %d",
                display.fps,display.milliseconds,result.inliers,result.keyframes,display.map.visiblePoints,display.map.cells.size()));
            if(display.map.occupancy!=null){OccupancySnapshot occ=display.map.occupancy;
                details.setText(String.format(Locale.ROOT,"处理 %.1f FPS / %.1f ms    内点 %d    OCC 占据 %d / 空闲 %d / 观测 %d 帧%s",
                    display.fps,display.milliseconds,result.inliers,occ.occupiedCount,occ.freeCount,occ.activeFrames,
                    occ.capacityLimited?"    已触发体素容量限制":""));}
        }
        void complete(String message){refresh();finished.set(true);timer.stop();stopButton.setEnabled(true);stopButton.setText("关闭");status.setText(message);}
    }
    private interface Source extends AutoCloseable {
        boolean read(Mat left,Mat right)throws IOException;
        double timestamp();
        @Override void close()throws IOException;
    }
    private static final class CameraSource implements Source {
        private final StereoCamera camera;private final long start=System.nanoTime();private double time;
        CameraSource(int device,StereoCalibration calibration,LiveSlamConfig config){camera=new StereoCamera(device,calibration,config);}
        public boolean read(Mat left,Mat right){boolean ok=camera.read(left,right);time=(System.nanoTime()-start)/1e9;
            if(!ok)throw new IllegalStateException("摄像头读取失败或已断开");return true;}
        public double timestamp(){return time;}public void close(){camera.close();}
    }
    private static final class DatasetSource implements Source {
        private final BufferedReader reader;private final Path root;private double time;
        DatasetSource(Path manifest)throws IOException{reader=Files.newBufferedReader(manifest);root=manifest.toAbsolutePath().getParent();}
        public boolean read(Mat left,Mat right)throws IOException{
            String line;while((line=reader.readLine())!=null){line=line.trim();if(line.isEmpty()||line.startsWith("#"))continue;
                String[] parts=line.split("\\s+");if(parts.length!=3)throw new IOException("清单格式：timestamp left right");
                time=Double.parseDouble(parts[0]);Mat l=Imgcodecs.imread(root.resolve(parts[1]).toString()),r=Imgcodecs.imread(root.resolve(parts[2]).toString());
                try{if(l.empty()||r.empty())throw new IOException("无法读取图像："+line);l.copyTo(left);r.copyTo(right);return true;}
                finally{l.release();r.release();}}
            return false;
        }
        public double timestamp(){return time;}public void close()throws IOException{reader.close();}
    }
    public static void main(String[] args)throws Exception{
        if(args.length<4){System.out.println("camera|dataset-view <calibration.properties> <device|manifest> <output-directory> [live.properties]");return;}
        if(!args[0].equals("camera")&&!args[0].equals("dataset-view"))throw new IllegalArgumentException("Use camera or dataset-view");
        OpenCvNative.load();LiveSlamConfig config=LiveSlamConfig.load(args.length>4?Paths.get(args[4]):null);
        Path output=Paths.get(args[3]).toAbsolutePath();Files.createDirectories(output);
        AtomicReference<Dashboard> reference=new AtomicReference<>();SwingUtilities.invokeAndWait(()->reference.set(new Dashboard(config,output)));
        Dashboard dashboard=reference.get();Thread worker=new Thread(()->{
            String message;
            try{run(args,config,output,dashboard);message="已停止，地图与轨迹已保存："+output;}
            catch(Exception error){message="运行失败："+error.getMessage();error.printStackTrace();}
            final String finalMessage=message;SwingUtilities.invokeLater(()->dashboard.complete(finalMessage));
        },"stereo-slam-capture");worker.start();
    }
    private static void run(String[] args,LiveSlamConfig config,Path output,Dashboard dashboard)throws Exception{
        SlamConfig slamConfig=new SlamConfig();slamConfig.maxKeyframes=config.maxKeyframes;
        slamConfig.minDepth=config.minDepth;slamConfig.maxDepth=config.maxDepth;
        try(StereoRig rig=StereoRig.load(Paths.get(args[1]));StereoSlam slam=new StereoSlam(rig.calibration,slamConfig);
            DenseStereoMapper dense=config.dense?new DenseStereoMapper(rig.calibration,config):null){
            Mat left=new Mat(),right=new Mat(),rectLeft=new Mat(),rectRight=new Mat();
            List<SlamSnapshot.MapPoint> environment=Collections.emptyList();
            BirdEyeMap map=BirdEyeMap.build(environment,config);int mappedKeys=0;long lastPublish=0;Display last=null;
            StereoSlam.Result lastResult=null;double averageMilliseconds=0,lastMilliseconds=0;
            TemporalOccupancyMapper occupancy=dense!=null&&config.occupancy.enabled?
                new TemporalOccupancyMapper(config.occupancy,config.minY,config.maxY):null;
            List<Pose> anchorPoses=Collections.emptyList();int frameNumber=0;
            try(Source source=args[0].equals("camera")?new CameraSource(Integer.parseInt(args[2]),rig.calibration,config):new DatasetSource(Paths.get(args[2]))){
                while(!dashboard.stop.get()&&source.read(left,right)){
                    long start=System.nanoTime();rig.rectify(left,right,rectLeft,rectRight);
                    StereoSlam.Result result=slam.process(rectLeft,rectRight,source.timestamp());lastResult=result;
                    boolean mapChanged=result.keyframes!=mappedKeys||result.loopClosed;
                    boolean newKeyframe=result.keyframes>mappedKeys;
                    boolean occupancyMeasurement=occupancy!=null&&result.pose!=null&&
                        (newKeyframe||frameNumber%config.occupancy.frameInterval==0);frameNumber++;
                    SlamSnapshot snapshot=null;
                    if(mapChanged){snapshot=slam.snapshot(dense==null);anchorPoses=snapshot.keyframes;}
                    List<TemporalOccupancyMapper.DepthSample> depths=Collections.emptyList();
                    if(dense!=null&&(newKeyframe||occupancyMeasurement)){
                        depths=dense.measure(rectLeft,rectRight);
                        if(newKeyframe){List<SlamSnapshot.MapPoint> points=new ArrayList<>();
                            for(TemporalOccupancyMapper.DepthSample sample:depths)points.add(new SlamSnapshot.MapPoint(sample.x,sample.y,sample.z));
                            dense.rememberKeyframe(result.keyframes-1,points);}
                    }
                    if(occupancy!=null){
                        if(occupancyMeasurement){int anchor=result.keyframes-1;
                            Pose relative=anchorPoses.get(anchor).inverse().multiply(result.pose);
                            occupancy.observe(source.timestamp(),anchor,relative,depths,anchorPoses,result.pose);
                        }else occupancy.refresh(source.timestamp(),anchorPoses,result.pose);
                    }
                    long now=System.nanoTime();
                    if(mapChanged||occupancyMeasurement||now-lastPublish>=100_000_000L){
                        if(snapshot==null)snapshot=slam.snapshot(false);
                        if(mapChanged){environment=dense==null?snapshot.points:dense.worldPoints(anchorPoses);
                            if(occupancy==null)map=BirdEyeMap.build(environment,config);mappedKeys=result.keyframes;}
                        if(occupancy!=null)map=BirdEyeMap.fromOccupancy(occupancy.snapshot(),config.minY,config.maxY);
                        double ms=(System.nanoTime()-start)/1e6;
                        last=new Display(bufferedImage(rectLeft),bufferedImage(rectRight),map,snapshot,result,ms,1000/Math.max(.001,averageMilliseconds==0?ms:averageMilliseconds));
                        dashboard.latest.set(last);lastPublish=now;
                    }
                    lastMilliseconds=(System.nanoTime()-start)/1e6;
                    averageMilliseconds=averageMilliseconds==0?lastMilliseconds:.85*averageMilliseconds+.15*lastMilliseconds;
                    // Keep replay useful to the viewer; live camera timing is controlled by capture.
                    if(args[0].equals("dataset-view"))Thread.sleep(30);
                }
            }finally{
                OccupancySnapshot finalOccupancy=occupancy==null?null:occupancy.snapshot();
                if(finalOccupancy!=null)map=BirdEyeMap.fromOccupancy(finalOccupancy,config.minY,config.maxY);
                if(lastResult!=null&&!rectLeft.empty()&&!rectRight.empty()){
                    last=new Display(bufferedImage(rectLeft),bufferedImage(rectRight),map,slam.snapshot(false),lastResult,lastMilliseconds,
                        1000/Math.max(.001,averageMilliseconds));dashboard.latest.set(last);
                }
                left.release();right.release();rectLeft.release();rectRight.release();
                if(finalOccupancy!=null){finalOccupancy.writeCsv(output.resolve("occupancy.csv"));
                    finalOccupancy.writeOccupiedPly(output.resolve("occupancy.ply"));}
                slam.writeTrajectory(output.resolve("trajectory.tum"));slam.writeMap(output.resolve("map.ply"));
                writeEnvironment(output.resolve("environment.ply"),environment);
                // Export the complete map rather than only the last followed viewport.
                final Display finalDisplay=last;
                if(finalDisplay!=null)SwingUtilities.invokeAndWait(()->{
                    BirdEyePanel export=new BirdEyePanel(config.pixelsPerMetre);export.setSize(1200,900);
                    export.update(finalDisplay.map,finalDisplay.snapshot,finalDisplay.result.pose);export.fitMap();
                    try{ImageIO.write(export.image(1200,900),"png",output.resolve("bird-eye.png").toFile());}
                    catch(IOException error){throw new java.io.UncheckedIOException(error);}
                });
            }
        }
    }
    static BufferedImage bufferedImage(Mat source){
        Mat converted=new Mat();try{
            if(source.channels()==4)Imgproc.cvtColor(source,converted,Imgproc.COLOR_BGRA2BGR);else source.copyTo(converted);
            BufferedImage image=new BufferedImage(converted.cols(),converted.rows(),converted.channels()==1?BufferedImage.TYPE_BYTE_GRAY:BufferedImage.TYPE_3BYTE_BGR);
            byte[] pixels=((DataBufferByte)image.getRaster().getDataBuffer()).getData();converted.get(0,0,pixels);return image;
        }finally{converted.release();}
    }
    private static void writeEnvironment(Path path,List<SlamSnapshot.MapPoint> points)throws IOException{
        try(BufferedWriter writer=Files.newBufferedWriter(path)){
            writer.write("ply\nformat ascii 1.0\nelement vertex "+points.size()+"\nproperty float x\nproperty float y\nproperty float z\nend_header\n");
            for(SlamSnapshot.MapPoint p:points)writer.write(String.format(Locale.ROOT,"%.6f %.6f %.6f%n",p.x,p.y,p.z));
        }
    }
}
