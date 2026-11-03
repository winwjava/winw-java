package winw.ai.slam;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.nio.file.*;
import java.util.*;
import java.util.List;
import java.util.stream.Collectors;
import javax.imageio.ImageIO;
import javax.swing.*;

/** Desktop scene replay for inspecting actual OCC observations without opening a camera. */
public final class SceneReplayDemo {
    private SceneReplayDemo(){}
    static OccupancySnapshot load(Path file)throws Exception {
        List<OccupancySnapshot.Voxel> voxels=new ArrayList<>();double resolution=.05;
        for(String line:Files.readAllLines(file)){
            if(line.startsWith("#")){int pos=line.indexOf("resolution=");if(pos>=0)resolution=Double.parseDouble(line.substring(pos+11));continue;}
            if(line.startsWith("x,")||line.trim().isEmpty())continue;String[] f=line.split(",");
            int x=(int)Math.floor(Double.parseDouble(f[0])/resolution),y=(int)Math.floor(Double.parseDouble(f[1])/resolution),z=(int)Math.floor(Double.parseDouble(f[2])/resolution);
            voxels.add(new OccupancySnapshot.Voxel(new OccupancySnapshot.VoxelKey(x,y,z),resolution,Double.parseDouble(f[3]),Integer.parseInt(f[5]),Integer.parseInt(f[6]),OccupancySnapshot.State.valueOf(f[4])));
        }
        return new OccupancySnapshot(voxels,resolution,1,false);
    }
    private static void updateScene(Scene3DPanel panel,Path file,String status){
        try{if(file.toString().endsWith(".surf"))panel.updateSurface(OccupancySurfaceMesh.read(file),status+" · 含低置信推断");else panel.update(load(file),status);}catch(Exception e){throw new IllegalStateException(e);}
    }
    public static void main(String[] args)throws Exception {
        if(args.length<1)throw new IllegalArgumentException("OCC replay directory [--headless]");
        Path root=Paths.get(args[0]);List<Path> volumes;
        try(java.util.stream.Stream<Path> files=Files.list(root.resolve(Files.isDirectory(root.resolve("surfaces"))?"surfaces":"volumes"))){volumes=files.filter(p->p.toString().endsWith(".csv")||p.toString().endsWith(".surf")).sorted().collect(Collectors.toList());}
        if(volumes.isEmpty())throw new IllegalArgumentException("No OCC frames");
        int selected=Math.min(12,volumes.size()-1);Path preview=volumes.get(selected);
        SwingUtilities.invokeAndWait(()->{
            Scene3DPanel scene=new Scene3DPanel();updateScene(scene,preview,"实际录制图片 · 第 "+selected+" 帧 · 相机局部几何");
            BufferedImage output=new BufferedImage(1500,900,BufferedImage.TYPE_INT_RGB);Graphics2D g=output.createGraphics();
            try{g.setColor(new Color(5,7,10));g.fillRect(0,0,1500,900);g.drawImage(scene.image(1160,900),0,0,null);
                BufferedImage source=ImageIO.read(root.resolve(String.format(Locale.ROOT,"frames/%03d.png",selected)).toFile());g.drawImage(source,1170,90,320,240,null);
                g.setColor(Color.LIGHT_GRAY);g.setFont(new Font("Microsoft YaHei",Font.PLAIN,13));g.drawString("原始左眼图像 / 第 "+selected+" 帧",1170,70);
                g.drawString("连续表面由双目图像重建",1170,360);g.drawString("未进行车辆、行人等语义识别",1170,385);
                ImageIO.write(output,"png",root.resolve("scene-preview.png").toFile());
            }catch(Exception e){throw new RuntimeException(e);}finally{g.dispose();}
            if(args.length>1&&args[1].equals("--headless"))return;
            JFrame window=new JFrame("实时三维环境感知 · 实拍序列回放");window.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
            JLabel source=new JLabel();source.setPreferredSize(new Dimension(340,280));
            JPanel controls=new JPanel(new BorderLayout());JSlider slider=new JSlider(0,volumes.size()-1,selected);JButton play=new JButton("播放 / 暂停"),fit=new JButton("适应三维场景");
            JLabel state=new JLabel();controls.add(slider);JPanel actions=new JPanel();actions.add(play);actions.add(fit);actions.add(state);
            JCheckBox color=new JCheckBox("原图着色",true),trusted=new JCheckBox("仅高置信表面");
            color.addActionListener(e->scene.setTextured(color.isSelected()));trusted.addActionListener(e->scene.setTrustedOnly(trusted.isSelected()));actions.add(color);actions.add(trusted);controls.add(actions,BorderLayout.SOUTH);
            Runnable refresh=()->{int index=slider.getValue();try{updateScene(scene,volumes.get(index),"实际录制图片 · 第 "+index+" 帧 · 相机局部几何");
                BufferedImage image=ImageIO.read(root.resolve(String.format(Locale.ROOT,"frames/%03d.png",index)).toFile());source.setIcon(new ImageIcon(image.getScaledInstance(320,240,Image.SCALE_FAST)));state.setText("第 "+index+" / "+(volumes.size()-1)+" 帧");
            }catch(Exception e){state.setText(e.getMessage());}};
            slider.addChangeListener(e->refresh.run());fit.addActionListener(e->scene.resetView());
            javax.swing.Timer timer=new javax.swing.Timer(500,e->slider.setValue((slider.getValue()+1)%volumes.size()));
            play.addActionListener(e->{if(timer.isRunning())timer.stop();else timer.start();});
            window.addWindowListener(new java.awt.event.WindowAdapter(){public void windowClosed(java.awt.event.WindowEvent e){timer.stop();}});
            window.add(scene);window.add(source,BorderLayout.EAST);window.add(controls,BorderLayout.SOUTH);window.setSize(1500,950);window.setLocationByPlatform(true);refresh.run();window.setVisible(true);
        });
    }
}
