package winw.ai.slam;

import java.awt.*;
import java.awt.event.*;
import java.awt.image.BufferedImage;
import java.io.*;
import java.nio.file.*;
import java.util.*;
import java.util.concurrent.*;
import java.util.concurrent.atomic.*;
import javax.swing.*;

/** Settings stay on the EDT; camera sessions and calibration jobs run sequentially on one worker. */
final class AutoStereoWindow {
    private final JFrame window=new JFrame("实时双目自动匹配 · 相机与调试设置");
    private final ExecutorService worker=Executors.newSingleThreadExecutor(r->new Thread(r,"auto-stereo-session"));
    private final AtomicReference<AutoStereoDemo.Frame> latest=new AtomicReference<>();
    private final AtomicLong generation=new AtomicLong();
    private volatile boolean closed;
    private AtomicBoolean sessionStop=new AtomicBoolean();
    private volatile Path currentOutput;
    private volatile AutoStereoSettings active;
    private String notice;
    private boolean calibrating;
    private final Controls controls;
    private final JLabel status=new JLabel("相机启动后自动匹配并显示视差，无需录制或离线标定");
    private final JLabel location=new JLabel(" ");
    private final JButton apply=new JButton("应用并重启相机"),stop=new JButton("停止并保存"),calibrate=new JButton("高级：离线米制自标定");
    private final Preview left=new Preview("左图"),right=new Preview("右图"),disparity=new Preview("视差 / 像素（不表示米）");
    private final javax.swing.Timer timer;

    AutoStereoWindow(AutoStereoSettings settings){
        controls=new Controls(settings);
        JPanel pictures=new JPanel(new GridLayout(1,3,8,0));pictures.add(left);pictures.add(right);pictures.add(disparity);
        JPanel header=new JPanel(new BorderLayout(8,8));header.setBorder(BorderFactory.createEmptyBorder(8,12,8,12));
        header.add(controls,BorderLayout.CENTER);
        JPanel buttons=new JPanel(new FlowLayout(FlowLayout.LEFT));buttons.add(apply);buttons.add(stop);buttons.add(calibrate);
        JButton folder=new JButton("打开本次输出目录");buttons.add(folder);header.add(buttons,BorderLayout.SOUTH);
        apply.addActionListener(e->{try{restart(controls.read());}catch(Exception error){showError(error);}});
        stop.addActionListener(e->{sessionStop.set(true);stop.setEnabled(false);notice="正在停止采集并保存…";});
        calibrate.setToolTipText("仅用于尝试米制 SLAM；实时视差无需点击此按钮");
        calibrate.addActionListener(e->calibrate());
        folder.addActionListener(e->{try{if(currentOutput!=null)Desktop.getDesktop().open(currentOutput.toFile());}
            catch(Exception error){showError(error);}});
        JPanel footer=new JPanel(new GridLayout(2,1,0,5));footer.setBorder(BorderFactory.createEmptyBorder(8,12,8,12));
        footer.add(status);footer.add(location);window.add(header,BorderLayout.NORTH);window.add(pictures);window.add(footer,BorderLayout.SOUTH);
        window.setSize(1450,790);window.setMinimumSize(new Dimension(1180,720));window.setLocationByPlatform(true);
        window.setDefaultCloseOperation(WindowConstants.DO_NOTHING_ON_CLOSE);
        window.addWindowListener(new WindowAdapter(){@Override public void windowClosing(WindowEvent e){close();}});
        window.getRootPane().registerKeyboardAction(e->{sessionStop.set(true);notice="正在停止采集并保存…";},
            KeyStroke.getKeyStroke(KeyEvent.VK_ESCAPE,0),JComponent.WHEN_IN_FOCUSED_WINDOW);
        timer=new javax.swing.Timer(100,e->refresh());
    }
    void show(){window.setVisible(true);timer.start();restart(controls.read());}
    private void restart(AutoStereoSettings settings){
        settings.validate();if(closed||calibrating)return;
        sessionStop.set(true);AtomicBoolean token=new AtomicBoolean();sessionStop=token;
        long id=generation.incrementAndGet();AutoStereoSettings snapshot=settings.copy();
        latest.set(null);currentOutput=null;notice="正在保存上一会话并按新参数打开相机…";
        stop.setEnabled(true);clearPreviews();
        worker.submit(()->{
            if(closed||generation.get()!=id)return;
            String message;Path output=snapshot.outputDirectory.toAbsolutePath().resolve("auto-"+UUID.randomUUID());
            try{
                Files.createDirectories(output.resolve("images/left"));Files.createDirectories(output.resolve("images/right"));
                SwingUtilities.invokeLater(()->{if(!closed&&generation.get()==id){
                    currentOutput=output;active=snapshot;latest.set(null);
                    if(!calibrating)notice=null;
                }});
                AutoStereoDemo.capture(snapshot,output,token,latest);
                message="采集已停止，序列已保存；可调整参数后重新启动："+output;
            }catch(Exception error){message="运行失败："+error.getMessage()+"；可修改参数后重试";error.printStackTrace();}
            String completion=message;
            SwingUtilities.invokeLater(()->{if(!closed&&generation.get()==id&&!calibrating){notice=completion;stop.setEnabled(false);}});
        });
    }
    private void refresh(){
        AutoStereoDemo.Frame frame=latest.get();if(frame!=null){left.image=frame.left;right.image=frame.right;disparity.image=frame.disparity;
            left.repaint();right.repaint();disparity.repaint();}
        status.setText(notice!=null?notice:frame==null?"正在等待双目图像…":frame.status);
        location.setText(currentOutput==null?"每次应用参数会新建独立录制目录；单眼尺寸与相机实际模式必须匹配":"本次输出："+currentOutput);
    }
    private void clearPreviews(){left.image=null;right.image=null;disparity.image=null;left.repaint();right.repaint();disparity.repaint();}
    private void showError(Exception error){JOptionPane.showMessageDialog(window,error.getMessage(),"参数或运行错误",JOptionPane.ERROR_MESSAGE);}
    private void close(){
        closed=true;generation.incrementAndGet();sessionStop.set(true);
        timer.stop();worker.shutdown();window.dispose();
    }
    private void calibrate(){
        if(calibrating||closed)return;
        try{controls.baseline.commitEdit();}catch(java.text.ParseException error){showError(new IllegalArgumentException("请输入有效的实测基线数值"));return;}
        double baseline=((Number)controls.baseline.getValue()).doubleValue();
        if(!Double.isFinite(baseline)||baseline<=0||baseline>1000){showError(new IllegalArgumentException("请先填写实测左右镜头光心间距（毫米），例如实测为 61 mm 就填 61。"));return;}
        Path output=currentOutput;AutoStereoSettings session=active;
        if(output==null||session==null){showError(new IllegalArgumentException("请先成功启动并录制至少 25 对原始图像。"));return;}
        AutoStereoSettings snapshot=session.copy();snapshot.baselineMm=baseline;
        calibrating=true;sessionStop.set(true);stop.setEnabled(false);apply.setEnabled(false);calibrate.setEnabled(false);
        notice="正在结束采集，随后自动估计标定；进度日志写入本次输出目录…";
        // Queued behind capture, so the camera is released and manifest flushed before calibration.
        worker.submit(()->{
            String completion;boolean mapOpened=false;
            Path log=output.resolve("self-calibration.log");
            try{
                if(closed)return;
                CalibrationRuntime.log(log,"Starting calibration; project="+Paths.get("").toAbsolutePath()+"; java="+System.getProperty("java.version"));
                long pairs;
                try(java.util.stream.Stream<Path> files=Files.list(output.resolve("images/left"))){pairs=files.filter(p->p.toString().endsWith(".png")).count();}
                if(pairs<25)throw new IOException("已录制 "+pairs+" 对，需要至少 25 对；请重新启动并录制更多场景");
                progress("正在用 Java/OpenCV 估计标定并独立验证…");
                Path calibration=output.resolve("scene-calibration-"+UUID.randomUUID()+".properties");
                try(NaturalSceneCalibration estimator=new NaturalSceneCalibration(snapshot.width,snapshot.height,snapshot.features,snapshot.validationTolerance)){
                    estimator.run(output,baseline/1000,calibration,log,()->closed,this::progress);
                }
                if(closed)return;
                Path live=output.resolve("ui-live.properties");saveCameraSettings(snapshot,live);
                LiveStereoSlam.main(new String[]{"camera",calibration.toString(),Integer.toString(snapshot.device),output.resolve("metric-slam").toString(),live.toString()});
                mapOpened=true;
                completion="自然场景标定通过，已打开米制 SLAM 地图窗口；标定保存在："+calibration;
            }catch(Exception error){try{CalibrationRuntime.failure(log,error);}catch(IOException logging){error.addSuppressed(logging);}completion="自标定失败："+error.getMessage()+"；原始序列保留，详细原因见 self-calibration.log";error.printStackTrace();}

            String message=completion;boolean transferred=mapOpened;
            SwingUtilities.invokeLater(()->{if(!closed){
                if(transferred){close();return;} // The map window now owns the camera.
                calibrating=false;notice=message;apply.setEnabled(true);calibrate.setEnabled(true);
            }});
        });
    }
    private void progress(String text){SwingUtilities.invokeLater(()->{if(!closed)notice=text;});}
    static void saveCameraSettings(AutoStereoSettings s,Path path)throws IOException{
        Properties p=new Properties();p.setProperty("camera.layout",s.layout.name());p.setProperty("camera.swapEyes",Boolean.toString(s.swapEyes));
        p.setProperty("camera.backend",Integer.toString(s.backend));p.setProperty("camera.fourcc",s.fourcc);
        p.setProperty("camera.fps",Double.toString(s.fps));p.setProperty("map.disparities",Integer.toString(s.disparities));
        try(Writer writer=Files.newBufferedWriter(path)){p.store(writer,"Camera settings selected in the stereo UI");}
    }
    private static final class Preview extends JPanel {
        BufferedImage image;final String title;
        Preview(String title){this.title=title;setPreferredSize(new Dimension(400,260));setBackground(new Color(28,35,44));}
        @Override protected void paintComponent(Graphics g){super.paintComponent(g);g.setColor(Color.WHITE);g.drawString(title,12,20);
            if(image!=null){double scale=Math.min((double)getWidth()/image.getWidth(),(double)Math.max(1,getHeight()-30)/image.getHeight());
                int w=(int)(image.getWidth()*scale),h=(int)(image.getHeight()*scale);g.drawImage(image,(getWidth()-w)/2,30,w,h,null);}}
    }
    static final class Controls extends JPanel {
        final JSpinner device,width,height,fps,disparities,features,block,unique,observation,recordInterval,samples,baseline,tolerance;
        final JComboBox<String> layout=new JComboBox<>(new String[]{"左右拼接","上下拼接"});
        final JComboBox<String> backend=new JComboBox<>(new String[]{"自动 (0)","DSHOW (700)","MSMF (1400)"});
        final JCheckBox swap=new JCheckBox("交换左右眼"),record=new JCheckBox("录制原始图像");
        final JTextField fourcc=new JTextField(),output=new JTextField(),cache=new JTextField();
        private final AutoStereoSettings initial;
        Controls(AutoStereoSettings s){
            initial=s.copy();setLayout(new BorderLayout(6,6));
            device=integer(s.device,0,100,1);width=integer(s.width,160,8192,16);height=integer(s.height,120,8192,16);
            fps=decimal(s.fps,1,120,1);disparities=integer(s.disparities,16,4096,16);features=integer(s.features,500,20000,500);
            block=integer(s.blockSize,3,21,2);unique=integer(s.uniquenessRatio,0,100,1);
            observation=integer(s.observationMillis,100,10000,100);recordInterval=integer(s.recordMillis,100,10000,100);
            samples=integer(s.maxSamples,25,1000,10);baseline=decimal(s.baselineMm,0,1000,.1);tolerance=decimal(s.validationTolerance,.25,5,.25);
            layout.setSelectedIndex(s.layout==LiveSlamConfig.Layout.SIDE_BY_SIDE?0:1);
            backend.setSelectedIndex(s.backend==700?1:s.backend==1400?2:0);
            if(s.backend!=0&&s.backend!=700&&s.backend!=1400){backend.addItem("自定义 ("+s.backend+")");backend.setSelectedIndex(3);}
            swap.setSelected(s.swapEyes);record.setSelected(s.record);fourcc.setText(s.fourcc);
            output.setText(s.outputDirectory.toString());cache.setText(s.rectificationCache==null?"":s.rectificationCache.toString());
            JPanel values=new JPanel(new GridLayout(3,6,10,6));
            field(values,"设备编号",device);field(values,"单眼宽度",width);field(values,"单眼高度",height);field(values,"拼接方式",layout);field(values,"采集后端",backend);field(values,"编码 / FourCC",fourcc);
            field(values,"帧率 / FPS",fps);field(values,"视差范围 / px",disparities);field(values,"特征数量",features);field(values,"匹配窗口",block);field(values,"唯一性阈值",unique);field(values,"验证误差 / px",tolerance);
            field(values,"校正更新间隔 / ms",observation);field(values,"录制间隔 / ms",recordInterval);field(values,"录制上限 / 对",samples);field(values,"实测基线 / mm（0 未填）",baseline);
            values.add(swap);values.add(record);add(values,BorderLayout.CENTER);
            JPanel paths=new JPanel(new GridLayout(2,1,0,4));
            paths.add(pathRow("输出目录",output,true));paths.add(pathRow("校正缓存（可选）",cache,false));add(paths,BorderLayout.SOUTH);
            setBorder(BorderFactory.createTitledBorder("实时匹配无需录制和基线；修改参数后点击“应用并重启相机”"));
        }
        AutoStereoSettings read(){
            // Commit typed spinner text too, not only values entered with arrow buttons.
            for(JSpinner spinner:new JSpinner[]{device,width,height,fps,disparities,features,block,unique,observation,recordInterval,samples,baseline,tolerance}){
                try{spinner.commitEdit();}catch(java.text.ParseException error){throw new IllegalArgumentException("请输入有效数字",error);}}
            AutoStereoSettings s=initial.copy();s.device=value(device);s.width=value(width);s.height=value(height);s.fps=number(fps);
            s.disparities=value(disparities);s.features=value(features);s.blockSize=value(block);s.uniquenessRatio=value(unique);
            s.observationMillis=value(observation);s.recordMillis=value(recordInterval);s.maxSamples=value(samples);
            s.baselineMm=number(baseline);s.validationTolerance=number(tolerance);s.swapEyes=swap.isSelected();s.record=record.isSelected();
            s.layout=layout.getSelectedIndex()==0?LiveSlamConfig.Layout.SIDE_BY_SIDE:LiveSlamConfig.Layout.TOP_BOTTOM;
            s.backend=backend.getSelectedIndex()==1?700:backend.getSelectedIndex()==2?1400:backend.getSelectedIndex()==3?initial.backend:0;
            s.fourcc=fourcc.getText().trim();if(output.getText().trim().isEmpty())throw new IllegalArgumentException("请选择输出目录");
            s.outputDirectory=Paths.get(output.getText().trim());s.rectificationCache=cache.getText().trim().isEmpty()?null:Paths.get(cache.getText().trim());
            s.validate();return s;
        }
        private static JSpinner integer(int value,int min,int max,int step){return new JSpinner(new SpinnerNumberModel(value,min,max,step));}
        private static JSpinner decimal(double value,double min,double max,double step){return new JSpinner(new SpinnerNumberModel(value,min,max,step));}
        private static int value(JSpinner spinner){return ((Number)spinner.getValue()).intValue();}
        private static double number(JSpinner spinner){return ((Number)spinner.getValue()).doubleValue();}
        private static void field(JPanel target,String label,JComponent control){JPanel panel=new JPanel(new BorderLayout(0,3));panel.add(new JLabel(label),BorderLayout.NORTH);panel.add(control);target.add(panel);}
        private JPanel pathRow(String label,JTextField field,boolean directory){
            JPanel row=new JPanel(new BorderLayout(8,0));JLabel name=new JLabel(label);name.setPreferredSize(new Dimension(140,24));row.add(name,BorderLayout.WEST);row.add(field);
            JButton browse=new JButton("选择…");row.add(browse,BorderLayout.EAST);browse.addActionListener(e->{JFileChooser chooser=new JFileChooser();
                chooser.setFileSelectionMode(directory?JFileChooser.DIRECTORIES_ONLY:JFileChooser.FILES_ONLY);
                if(chooser.showOpenDialog(this)==JFileChooser.APPROVE_OPTION)field.setText(chooser.getSelectedFile().getAbsolutePath());});return row;
        }
    }
}
