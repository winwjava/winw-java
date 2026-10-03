package winw.ai.slam;

import java.awt.*;
import java.awt.event.*;
import java.awt.geom.*;
import java.awt.image.BufferedImage;
import java.util.*;
import javax.swing.JPanel;

/** EDT-owned view; X right, initial-world Z up. Rendering also works headlessly for verification. */
public final class BirdEyePanel extends JPanel {
    public enum Style { MAP, DRIVING }
    private Style style=Style.MAP;
    private double viewYaw;
    private final DrivingViewRenderer drivingRenderer=new DrivingViewRenderer();
    private BirdEyeMap map;
    private java.util.List<SlamSnapshot.TrackPoint> trajectory=Collections.emptyList();
    private java.util.List<Pose> keyframes=Collections.emptyList();
    private Pose current;
    private double centreX,centreZ,scale;
    private boolean follow=true;
    private java.awt.Point drag;
    public BirdEyePanel(double pixelsPerMetre){
        scale=pixelsPerMetre;setPreferredSize(new Dimension(780,680));setBackground(new Color(13,23,36));
        addMouseWheelListener(e->{scale=Math.max(5,Math.min(600,scale*Math.pow(1.15,-e.getPreciseWheelRotation())));repaint();});
        MouseAdapter mouse=new MouseAdapter(){
            @Override public void mousePressed(MouseEvent e){drag=e.getPoint();}
            @Override public void mouseReleased(MouseEvent e){drag=null;}
            @Override public void mouseDragged(MouseEvent e){if(drag==null)return;follow=false;
                double dx=-(e.getX()-drag.x)/scale,dz=(e.getY()-drag.y)/scale;
                if(style==Style.DRIVING){dz/=.68;centreX+=Math.cos(viewYaw)*dx+Math.sin(viewYaw)*dz;centreZ+=-Math.sin(viewYaw)*dx+Math.cos(viewYaw)*dz;}
                else{centreX+=dx;centreZ+=dz;}drag=e.getPoint();repaint();}
        };addMouseListener(mouse);addMouseMotionListener(mouse);
    }
    public void update(BirdEyeMap map,SlamSnapshot snapshot,Pose pose){
        this.map=map;trajectory=snapshot.trajectory;keyframes=snapshot.keyframes;current=pose;
        if(follow&&pose!=null){double[] t=pose.translation();centreX=t[0];centreZ=t[2];viewYaw=DrivingProjection.yaw(pose);}repaint();
    }
    public void setStyle(Style style){this.style=Objects.requireNonNull(style);if(style==Style.DRIVING)followCamera();repaint();}
    public Style style(){return style;}
    public void followCamera(){follow=true;if(current!=null){double[] p=current.translation();centreX=p[0];centreZ=p[2];viewYaw=DrivingProjection.yaw(current);}repaint();}
    public void showOrigin(){follow=false;centreX=centreZ=0;repaint();}
    public void fitMap(){
        style=Style.MAP;
        double minX=0,maxX=0,minZ=0,maxZ=0;
        if(map!=null)for(BirdEyeMap.Cell c:map.cells){minX=Math.min(minX,c.x);maxX=Math.max(maxX,c.x);minZ=Math.min(minZ,c.z);maxZ=Math.max(maxZ,c.z);}
        for(SlamSnapshot.TrackPoint sample:trajectory){double[] t=sample.pose.translation();minX=Math.min(minX,t[0]);maxX=Math.max(maxX,t[0]);minZ=Math.min(minZ,t[2]);maxZ=Math.max(maxZ,t[2]);}
        centreX=(minX+maxX)/2;centreZ=(minZ+maxZ)/2;follow=false;
        scale=Math.max(5,Math.min(600,Math.min(Math.max(100,getWidth()-100)/Math.max(2,maxX-minX),Math.max(100,getHeight()-150)/Math.max(2,maxZ-minZ))));repaint();
    }
    public BufferedImage image(int width,int height){
        BufferedImage image=new BufferedImage(width,height,BufferedImage.TYPE_INT_RGB);Graphics2D g=image.createGraphics();
        try{render(g,width,height);}finally{g.dispose();}return image;
    }
    @Override protected void paintComponent(Graphics g){super.paintComponent(g);Graphics2D copy=(Graphics2D)g.create();
        try{render(copy,getWidth(),getHeight());}finally{copy.dispose();}}
    private double screenX(double x,int width){return width*.5+(x-centreX)*scale;}
    private double screenY(double z,int height){return height*.5-(z-centreZ)*scale;}
    private void render(Graphics2D g,int width,int height){
        if(style==Style.DRIVING){drivingRenderer.render(g,width,height,map,trajectory,current,centreX,centreZ,viewYaw,scale,follow);return;}
        g.setColor(new Color(13,23,36));g.fillRect(0,0,width,height);
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING,RenderingHints.VALUE_ANTIALIAS_ON);
        g.setFont(new Font(Font.SANS_SERIF,Font.PLAIN,12));
        double grid=scale<20?5:scale>150?.5:1;
        double xmin=centreX-width/(2*scale),xmax=centreX+width/(2*scale),zmin=centreZ-height/(2*scale),zmax=centreZ+height/(2*scale);
        g.setColor(new Color(34,49,65));
        for(double x=Math.floor(xmin/grid)*grid;x<=xmax;x+=grid)g.draw(new Line2D.Double(screenX(x,width),0,screenX(x,width),height));
        for(double z=Math.floor(zmin/grid)*grid;z<=zmax;z+=grid)g.draw(new Line2D.Double(0,screenY(z,height),width,screenY(z,height)));
        if(map!=null)for(BirdEyeMap.Cell c:map.freeCells){
            double x=screenX(c.x,width),y=screenY(c.z,height),size=Math.max(2,map.cellSize*scale);
            g.setColor(new Color(42,140,111,160));g.fill(new Rectangle2D.Double(x-size/2,y-size/2,size,size));
        }
        if(map!=null)for(BirdEyeMap.Cell c:map.cells){
            double x=screenX(c.x,width),y=screenY(c.z,height);if(x<0||y<0||x>width||y>height)continue;
            double size=Math.max(2,map.cellSize*scale);int brightness=Math.min(220,90+c.observations*9);
            g.setColor(map.occupancy==null?new Color(45,brightness,170):new Color(236,153,73));g.fill(new Rectangle2D.Double(x-size/2,y-size/2,size,size));
        }
        g.setColor(new Color(94,164,255));g.setStroke(new BasicStroke(2));
        Pose previous=null;
        for(SlamSnapshot.TrackPoint track:trajectory){
            if(previous!=null&&!track.startsSegment){double[] a=previous.translation(),b=track.pose.translation();
                g.draw(new Line2D.Double(screenX(a[0],width),screenY(a[2],height),screenX(b[0],width),screenY(b[2],height)));}previous=track.pose;
        }
        g.setColor(new Color(230,181,84));
        for(Pose p:keyframes){double[] t=p.translation();g.fill(new Ellipse2D.Double(screenX(t[0],width)-2,screenY(t[2],height)-2,4,4));}
        double ox=screenX(0,width),oy=screenY(0,height);g.setColor(new Color(216,225,235));
        g.draw(new Line2D.Double(ox-7,oy,ox+7,oy));g.draw(new Line2D.Double(ox,oy-7,ox,oy+7));g.drawString("origin",(int)ox+9,(int)oy-7);
        if(current!=null){double[] p=current.translation(),r=current.rotation();double x=screenX(p[0],width),y=screenY(p[2],height);
            double norm=Math.hypot(r[2],r[8]);if(norm>1e-6){double dx=r[2]/norm,dy=-r[8]/norm;
                Path2D arrow=new Path2D.Double();arrow.moveTo(x+dx*17,y+dy*17);arrow.lineTo(x-dx*10-dy*9,y-dy*10+dx*9);
                arrow.lineTo(x-dx*10+dy*9,y-dy*10-dx*9);arrow.closePath();g.setColor(new Color(255,112,103));g.fill(arrow);}
        }
        g.setColor(new Color(13,23,36,230));g.fillRoundRect(12,12,370,60,8,8);
        boolean occupancy=map!=null&&map.occupancy!=null;
        g.setColor(new Color(227,235,244));g.setFont(new Font(Font.SANS_SERIF,Font.BOLD,16));g.drawString(occupancy?"OCCUPANCY MAP  /  X-Z":"BIRD'S-EYE MAP  /  X-Z",24,36);
        g.setFont(new Font(Font.SANS_SERIF,Font.PLAIN,12));g.drawString(occupancy?"Green: free columns / amber: occupied / dark: unknown":"Green: surfaces   Blue: track   Coral: camera",24,57);
        g.setColor(new Color(227,235,244));g.drawString("+Z / initial forward",width-150,25);
        g.drawString(String.format(Locale.ROOT,"Grid %.1f m | %s | wheel: zoom / drag: pan",grid,follow?"following":"manual"),18,height-20);
        if(current==null){g.setColor(new Color(255,177,95));g.drawString("No current pose - initializing or tracking lost",18,height-42);}
    }
}
