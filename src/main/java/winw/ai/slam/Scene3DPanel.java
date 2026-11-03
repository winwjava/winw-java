package winw.ai.slam;

import java.awt.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.awt.image.BufferedImage;
import java.util.*;
import javax.swing.*;

/** CPU scene renderer: perspective geometry, continuous OCC surfaces, orbit/pan/zoom. EDT owned. */
public final class Scene3DPanel extends JPanel {
    private OccupancySnapshot volume;
    private OccupancySurfaceMesh mesh;
    private double[] target={0,0,1};
    private double yaw=-.5,pitch=.3,distance=3;
    private Point drag;
    private boolean framed,pan,textured=true,trustedOnly;
    private String status="等待环境观测";
    public Scene3DPanel(){
        setPreferredSize(new Dimension(1000,650));setBackground(Color.BLACK);
        MouseAdapter mouse=new MouseAdapter(){
            public void mousePressed(MouseEvent e){drag=e.getPoint();pan=SwingUtilities.isRightMouseButton(e)||e.isShiftDown();}
            public void mouseReleased(MouseEvent e){drag=null;}
            public void mouseDragged(MouseEvent e){if(drag==null)return;double dx=e.getX()-drag.x,dy=e.getY()-drag.y;
                if(pan){target[0]-=dx*distance/Math.max(100,getHeight());target[1]-=dy*distance/Math.max(100,getHeight());}
                else{yaw+=dx*.008;pitch=Math.max(-1.4,Math.min(1.4,pitch+dy*.008));}drag=e.getPoint();repaint();}
        };addMouseListener(mouse);addMouseMotionListener(mouse);
        addMouseWheelListener(e->{distance=Math.max(.15,Math.min(100,distance*Math.exp(e.getPreciseWheelRotation()*.12)));repaint();});
    }
    public void update(OccupancySnapshot volume,String status){
        if(this.volume!=volume){this.volume=volume;mesh=volume==null?null:OccupancySurfaceMesh.buildSmooth(volume);if(volume==null)framed=false;}
        this.status=status;if(!framed&&mesh!=null&&!mesh.faces.isEmpty())fitScene();repaint();
    }
    public void updateSurface(OccupancySurfaceMesh surface,String status){this.volume=null;this.mesh=surface;this.status=status;if(!framed&&surface!=null&&!surface.faces.isEmpty())fitScene();repaint();}
    public void setTextured(boolean value){textured=value;repaint();}
    public void setTrustedOnly(boolean value){trustedOnly=value;repaint();}
    public void fitScene(){if(mesh==null||mesh.faces.isEmpty())return;
        double[] lo=mesh.min.clone(),hi=mesh.max.clone();
        // A few distant bad matches must not shrink the main observed surface to a speck.
        // This changes framing only; all geometry remains available when zooming out.
        if(mesh.faces.size()>100){for(int axis=0;axis<3;axis++){double[] values=new double[mesh.faces.size()*3];int count=0;
            for(OccupancySurfaceMesh.Face face:mesh.faces)for(double[] p:face.corners){if(count==values.length)values=Arrays.copyOf(values,values.length*2);values[count++]=p[axis];}
            Arrays.sort(values,0,count);lo[axis]=values[(int)(count*.05)];hi[axis]=values[Math.min(count-1,(int)(count*.95))];}}
        double size=0;for(int i=0;i<3;i++){target[i]=(lo[i]+hi[i])/2;size+=Math.pow(hi[i]-lo[i],2);}
        distance=Math.max(.5,Math.sqrt(size)*1.4);framed=true;repaint();
    }
    public void resetView(){yaw=-.5;pitch=.3;fitScene();}
    public BufferedImage image(int width,int height){BufferedImage image=new BufferedImage(width,height,BufferedImage.TYPE_INT_RGB);Graphics2D g=image.createGraphics();try{render(g,width,height);}finally{g.dispose();}return image;}
    protected void paintComponent(Graphics graphics){super.paintComponent(graphics);Graphics2D g=(Graphics2D)graphics.create();try{render(g,getWidth(),getHeight());}finally{g.dispose();}}
    private double[] project(double[] p,int width,int height){double x=p[0]-target[0],y=-(p[1]-target[1]),z=p[2]-target[2];
        double u=x*Math.cos(yaw)+z*Math.sin(yaw),a=-x*Math.sin(yaw)+z*Math.cos(yaw);
        double v=y*Math.cos(pitch)-a*Math.sin(pitch),depth=y*Math.sin(pitch)+a*Math.cos(pitch),d=distance-depth;
        if(d<.05)return null;double scale=Math.min(width,height)*1.25/d;
        return new double[]{width*.5+u*scale,height*.5-v*scale,depth};
    }
    private static final class PolygonFace {Path2D path;double depth;Color color;PolygonFace(Path2D p,double d,Color c){path=p;depth=d;color=c;}}
    private void render(Graphics2D g,int width,int height){
        g.setColor(new Color(5,7,10));g.fillRect(0,0,width,height);g.setRenderingHint(RenderingHints.KEY_ANTIALIASING,RenderingHints.VALUE_ANTIALIAS_ON);
        double floor=mesh==null?1:mesh.max[1]+.05,grid=distance<4?.25:distance<12?1:5,radius=Math.min(40,Math.max(2,distance*2));
        g.setColor(new Color(33,38,44));g.setStroke(new BasicStroke(1));
        for(double t=-radius;t<=radius;t+=grid){line(g,new double[]{target[0]+t,floor,target[2]-radius},new double[]{target[0]+t,floor,target[2]+radius},width,height);line(g,new double[]{target[0]-radius,floor,target[2]+t},new double[]{target[0]+radius,floor,target[2]+t},width,height);}
        java.util.List<PolygonFace> visible=new ArrayList<>();
        if(mesh!=null)for(OccupancySurfaceMesh.Face face:mesh.faces){if(trustedOnly&&face.confidence<.5)continue;Path2D path=new Path2D.Double();double depth=0;boolean valid=true;
            for(int i=0;i<face.corners.length;i++){double[] p=project(face.corners[i],width,height);if(p==null){valid=false;break;}if(i==0)path.moveTo(p[0],p[1]);else path.lineTo(p[0],p[1]);depth+=p[2];}
            if(valid){path.closePath();double[] a=face.corners[0],b=face.corners[1],c=face.corners[2];
                double nx=(b[1]-a[1])*(c[2]-a[2])-(b[2]-a[2])*(c[1]-a[1]),ny=(b[2]-a[2])*(c[0]-a[0])-(b[0]-a[0])*(c[2]-a[2]),nz=(b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]);
                double norm=Math.sqrt(nx*nx+ny*ny+nz*nz);int shade=(int)(125+80*Math.abs((nx*.3-ny*.8+nz*.4)/Math.max(1e-12,norm)));
                Color color=new Color(shade,shade+2,shade+5);
                if(textured&&face.rgb>=0){double light=.78+.22*(shade-125)/80.;int rgb=face.rgb;color=new Color((int)(((rgb>>16)&255)*light),(int)(((rgb>>8)&255)*light),(int)((rgb&255)*light));}
                visible.add(new PolygonFace(path,depth/face.corners.length,color));}
        }
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING,RenderingHints.VALUE_ANTIALIAS_OFF);
        visible.sort(Comparator.comparingDouble(p->p.depth));for(PolygonFace face:visible){g.setColor(face.color);g.fill(face.path);}
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING,RenderingHints.VALUE_ANTIALIAS_ON);
        g.setFont(new Font("Microsoft YaHei",Font.PLAIN,14));g.setColor(new Color(225,233,241));g.drawString("实时三维环境感知 / 观测表面",20,30);
        g.setFont(new Font("Microsoft YaHei",Font.PLAIN,12));g.setColor(new Color(159,175,190));g.drawString(status,20,54);
        g.drawString("拖动旋转 · 右键/Shift 拖动平移 · 滚轮缩放 · 网格仅为参考面",20,height-42);
        g.drawString("含低置信平滑推断 · 类别未知 · 名义米制精度未验证",20,height-20);
        if(mesh==null||mesh.faces.isEmpty()){g.setColor(new Color(214,185,114));g.drawString("当前没有可靠的三维表面观测",width/2-110,height/2);}
        else g.drawString(mesh.faces.size()+" 连续表面面片 · "+(trustedOnly?"仅高置信表面":"显示完整正则化表面"),20,78);
    }
    private void line(Graphics2D g,double[] a,double[] b,int width,int height){double[] p=project(a,width,height),q=project(b,width,height);if(p!=null&&q!=null)g.draw(new Line2D.Double(p[0],p[1],q[0],q[1]));}
}
