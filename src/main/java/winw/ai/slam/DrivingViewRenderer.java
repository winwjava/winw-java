package winw.ai.slam;

import java.awt.*;
import java.awt.geom.*;
import java.util.*;
import java.util.List;

/** Driving-screen style, using observed geometry only. No invented lanes or classified traffic. */
final class DrivingViewRenderer {
    private BirdEyeMap lastMap;
    private List<Surface> surfaces=Collections.emptyList();
    private static final class Surface {
        double minX=Double.POSITIVE_INFINITY,minZ=Double.POSITIVE_INFINITY,maxX=Double.NEGATIVE_INFINITY,maxZ=Double.NEGATIVE_INFINITY;
        double minY=Double.POSITIVE_INFINITY,maxY=Double.NEGATIVE_INFINITY;
        int count;
        void add(BirdEyeMap.Cell cell,double padding){minX=Math.min(minX,cell.x-padding);maxX=Math.max(maxX,cell.x+padding);
            minZ=Math.min(minZ,cell.z-padding);maxZ=Math.max(maxZ,cell.z+padding);
            minY=Math.min(minY,cell.minY);maxY=Math.max(maxY,cell.maxY);count+=cell.observations;}
    }
    private List<Surface> group(BirdEyeMap map){
        double resolution=Math.max(.3,map.cellSize);
        Map<Long,List<BirdEyeMap.Cell>> bins=new HashMap<>();
        for(BirdEyeMap.Cell cell:map.cells){int x=(int)Math.floor(cell.x/resolution),z=(int)Math.floor(cell.z/resolution);
            bins.computeIfAbsent(key(x,z),ignored->new ArrayList<>()).add(cell);}
        List<Surface> result=new ArrayList<>();ArrayDeque<Long> queue=new ArrayDeque<>();
        while(!bins.isEmpty()){
            long start=bins.keySet().iterator().next();queue.add(start);Surface surface=new Surface();int cells=0;
            while(!queue.isEmpty()){
                long next=queue.remove();List<BirdEyeMap.Cell> bin=bins.remove(next);if(bin==null)continue;
                int x=(int)(next>>32),z=(int)next;cells+=bin.size();
                for(BirdEyeMap.Cell cell:bin)surface.add(cell,map.cellSize/2);
                for(int dx=-1;dx<=1;dx++)for(int dz=-1;dz<=1;dz++)if(dx!=0||dz!=0){long neighbor=key(x+dx,z+dz);if(bins.containsKey(neighbor))queue.add(neighbor);}
            }
            if(cells>=4)result.add(surface);
        }
        return result;
    }
    private long key(int x,int z){return((long)x<<32)|(z&0xffffffffL);}
    void render(Graphics2D g,int width,int height,BirdEyeMap map,List<SlamSnapshot.TrackPoint> trajectory,Pose pose,
            double centreX,double centreZ,double yaw,double scale,boolean follow){
        DrivingProjection projection=new DrivingProjection(centreX,centreZ,yaw,scale,width,height);
        g.setPaint(new GradientPaint(0,0,new Color(245,247,250),0,height,new Color(223,228,234)));g.fillRect(0,0,width,height);
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING,RenderingHints.VALUE_ANTIALIAS_ON);
        // Range guides are metric reference curves, never inferred lane markings.
        g.setStroke(new BasicStroke(1));g.setFont(new Font(Font.SANS_SERIF,Font.PLAIN,11));
        for(int range=5;range<=40;range+=5){
            Path2D curve=new Path2D.Double();for(int i=-80;i<=80;i++){
                double angle=Math.toRadians(i);Point2D p=projection.projectLocal(range*Math.sin(angle),range*Math.cos(angle),0);
                if(i==-80)curve.moveTo(p.getX(),p.getY());else curve.lineTo(p.getX(),p.getY());}
            g.setColor(new Color(195,202,211,125));g.draw(curve);
            Point2D label=projection.projectLocal(0,range,0);if(label.getY()>115){g.setColor(new Color(129,140,154));g.drawString(range+" m",width/2+9,(int)label.getY()-4);}
        }
        if(map!=null&&map.occupancy!=null){
            for(BirdEyeMap.Cell cell:map.freeCells){double r=map.cellSize/2;
                Path2D tile=polygon(projection,new double[][]{{cell.x-r,cell.z-r},{cell.x+r,cell.z-r},{cell.x+r,cell.z+r},{cell.x-r,cell.z+r}});
                g.setColor(new Color(77,177,147,55));g.fill(tile);
            }
            List<OccupancySnapshot.Voxel> voxels=new ArrayList<>();
            for(OccupancySnapshot.Voxel voxel:map.occupancy.voxels)if(voxel.state==OccupancySnapshot.State.OCCUPIED&&projection.local(voxel.x,voxel.z)[1]>-9)voxels.add(voxel);
            // Paint the nearest 20k cells; distant cells remain in the map/export.
            voxels.sort(Comparator.comparingDouble(v->Math.hypot(v.x-centreX,v.z-centreZ)));
            if(voxels.size()>20000)voxels=new ArrayList<>(voxels.subList(0,20000));
            voxels.sort(Comparator.comparingDouble(v->-projection.local(v.x,v.z)[1]));
            double r=map.occupancy.resolution/2;Surface voxelSurface=new Surface();
            for(OccupancySnapshot.Voxel voxel:voxels){
                voxelSurface.minX=voxel.x-r;voxelSurface.maxX=voxel.x+r;voxelSurface.minZ=voxel.z-r;voxelSurface.maxZ=voxel.z+r;
                voxelSurface.minY=voxel.y-r;voxelSurface.maxY=voxel.y+r;
                drawEnvelope(g,projection,voxelSurface);
            }
        }else if(map!=null){
            if(map!=lastMap){lastMap=map;surfaces=group(map);}
            // Draw connected observed footprints. The outline is a geometric envelope, not an object classifier.
            List<Surface> sorted=new ArrayList<>(surfaces);sorted.sort(Comparator.comparingDouble(s->-projection.local((s.minX+s.maxX)/2,(s.minZ+s.maxZ)/2)[1]));
            for(Surface surface:sorted){
                double cx=(surface.minX+surface.maxX)/2,cz=(surface.minZ+surface.maxZ)/2;
                if(projection.local(cx,cz)[1]<-9)continue;
                // Avoid enclosing large L-shaped walls in a filled rectangular area.
                double area=(surface.maxX-surface.minX)*(surface.maxZ-surface.minZ);
                if(area>12)continue;
                Path2D outline=polygon(projection,new double[][]{{surface.minX,surface.minZ},{surface.maxX,surface.minZ},
                    {surface.maxX,surface.maxZ},{surface.minX,surface.maxZ}});
                g.setColor(new Color(155,168,181,55));g.fill(outline);g.setColor(new Color(123,141,159,150));g.setStroke(new BasicStroke(1.2f));g.draw(outline);
                if(surface.maxY-surface.minY>.1)drawEnvelope(g,projection,surface);
            }
            for(BirdEyeMap.Cell cell:map.cells){double[] local=projection.local(cell.x,cell.z);if(local[1]<-9)continue;
                Point2D p=projection.projectLocal(local[0],local[1],cell.minY);if(p.getX()<0||p.getX()>width||p.getY()<100||p.getY()>height)continue;
                double size=Math.max(1.5,map.cellSize*scale/Math.max(.45,1+local[1]*.035));
                int alpha=Math.min(215,75+cell.observations*12);g.setColor(new Color(85,108,132,alpha));
                g.fill(new Ellipse2D.Double(p.getX()-size/2,p.getY()-size/2,size,size));
            }
        }
        Pose previous=null;g.setStroke(new BasicStroke(3,BasicStroke.CAP_ROUND,BasicStroke.JOIN_ROUND));g.setColor(new Color(59,135,239,180));
        for(SlamSnapshot.TrackPoint track:trajectory){
            double[] b=track.pose.translation();
            if(previous!=null&&!track.startsSegment){double[] a=previous.translation();
                if(projection.local(a[0],a[2])[1]>-15&&projection.local(b[0],b[2])[1]>-15){
                    Point2D p=projection.project(a[0],a[2]),q=projection.project(b[0],b[2]);g.draw(new Line2D.Double(p,q));}}
            previous=track.pose;
        }
        if(pose!=null){double[] t=pose.translation();Point2D p=projection.project(t[0],t[2]);
            Graphics2D vehicle=(Graphics2D)g.create();try{
                vehicle.translate(p.getX(),p.getY());vehicle.rotate(DrivingProjection.yaw(pose)-yaw);
                drawVehicle(vehicle,Math.max(38,Math.min(85,scale*1.65)));
            }finally{vehicle.dispose();}}
        drawHud(g,width,height,pose,map,follow);
    }
    private Path2D polygon(DrivingProjection projection,double[][] points){Path2D shape=new Path2D.Double();
        for(int i=0;i<points.length;i++){Point2D p=projection.project(points[i][0],points[i][1]);if(i==0)shape.moveTo(p.getX(),p.getY());else shape.lineTo(p.getX(),p.getY());}
        shape.closePath();return shape;}
    private Point2D point(DrivingProjection projection,double x,double y,double z){double[] local=projection.local(x,z);return projection.projectLocal(local[0],local[1],y);}
    private void face(Graphics2D g,Color colour,Point2D... points){Path2D shape=new Path2D.Double();
        for(int i=0;i<points.length;i++){Point2D p=points[i];if(i==0)shape.moveTo(p.getX(),p.getY());else shape.lineTo(p.getX(),p.getY());}
        shape.closePath();g.setColor(colour);g.fill(shape);g.setColor(new Color(110,128,147,170));g.draw(shape);}
    /** Bounding envelope of measured point heights, never an inferred semantic object model. */
    private void drawEnvelope(Graphics2D g,DrivingProjection p,Surface s){
        Point2D[] top={point(p,s.minX,s.minY,s.minZ),point(p,s.maxX,s.minY,s.minZ),point(p,s.maxX,s.minY,s.maxZ),point(p,s.minX,s.minY,s.maxZ)};
        Point2D[] bottom={point(p,s.minX,s.maxY,s.minZ),point(p,s.maxX,s.maxY,s.minZ),point(p,s.maxX,s.maxY,s.maxZ),point(p,s.minX,s.maxY,s.maxZ)};
        face(g,new Color(157,170,183,180),top[0],top[1],bottom[1],bottom[0]);
        face(g,new Color(179,190,202,170),top[1],top[2],bottom[2],bottom[1]);
        face(g,new Color(196,207,219,205),top);
    }
    /** Generic camera carrier glyph; styling dimensions do not imply measured vehicle dimensions. */
    private void drawVehicle(Graphics2D g,double width){
        double height=width*1.85;
        g.setColor(new Color(60,70,85,35));g.fill(new RoundRectangle2D.Double(-width*.6,-height*.44,width*1.2,height,24,24));
        g.setColor(new Color(42,47,54));
        for(int side:new int[]{-1,1})for(int axle:new int[]{-1,1})g.fill(new RoundRectangle2D.Double(side*width*.5-width*.08,axle*height*.27-height*.1,width*.16,height*.2,6,6));
        Path2D body=new Path2D.Double();body.moveTo(-width*.35,-height*.5);body.curveTo(-width*.5,-height*.48,-width*.51,-height*.2,-width*.5,height*.35);
        body.quadTo(-width*.47,height*.5,-width*.32,height*.5);body.lineTo(width*.32,height*.5);
        body.quadTo(width*.47,height*.5,width*.5,height*.35);body.curveTo(width*.51,-height*.2,width*.5,-height*.48,width*.35,-height*.5);body.closePath();
        g.setPaint(new GradientPaint((float)-width/2,0,new Color(211,216,222),(float)width/2,0,Color.WHITE));g.fill(body);
        g.setColor(new Color(123,134,147));g.setStroke(new BasicStroke(1.2f));g.draw(body);
        g.setColor(new Color(47,60,77));g.fill(new RoundRectangle2D.Double(-width*.34,-height*.18,width*.68,height*.43,12,12));
        g.setPaint(new GradientPaint(0,(float)(-height*.18),new Color(94,122,151),0,(float)(height*.25),new Color(31,44,59)));
        g.fill(new RoundRectangle2D.Double(-width*.29,-height*.15,width*.58,height*.35,9,9));
        g.setColor(new Color(230,236,242));g.setStroke(new BasicStroke(2));g.draw(new Line2D.Double(-width*.3,-height*.21,width*.3,-height*.21));
        g.setColor(new Color(250,253,255));g.fill(new RoundRectangle2D.Double(-width*.35,-height*.43,width*.18,4,3,3));g.fill(new RoundRectangle2D.Double(width*.17,-height*.43,width*.18,4,3,3));
        g.setColor(new Color(222,74,74));g.fill(new RoundRectangle2D.Double(-width*.36,height*.43,width*.2,4,2,2));g.fill(new RoundRectangle2D.Double(width*.16,height*.43,width*.2,4,2,2));
    }
    private void drawHud(Graphics2D g,int width,int height,Pose pose,BirdEyeMap map,boolean follow){
        g.setColor(new Color(250,252,254,240));g.fillRect(0,0,width,98);
        boolean occupancy=map!=null&&map.occupancy!=null;
        g.setFont(new Font(Font.SANS_SERIF,Font.BOLD,20));g.setColor(new Color(42,53,68));g.drawString(occupancy?"3D OCCUPANCY / DRIVING":"DRIVING VIEW",25,34);
        g.setFont(new Font(Font.SANS_SERIF,Font.PLAIN,12));g.setColor(new Color(110,122,137));g.drawString(occupancy?"Stereo depth  /  temporal voxel fusion":"Stereo geometry  /  heading aligned",25,57);
        g.drawString(occupancy?"Green: fully observed free columns / grey: occupied voxels":"Observed surfaces + recorded trajectory",25,78);
        g.setColor(pose==null?new Color(197,130,51):new Color(48,146,113));g.fillOval(width-188,25,7,7);
        g.setColor(new Color(66,80,98));g.setFont(new Font(Font.SANS_SERIF,Font.BOLD,12));g.drawString(pose==null?"POSE UNAVAILABLE":"LOCALIZED",width-173,33);
        g.setFont(new Font(Font.SANS_SERIF,Font.PLAIN,11));g.drawString((map==null?0:map.visiblePoints)+(occupancy?" occupied voxels":" surface samples"),width-188,57);
        g.setColor(new Color(247,250,253,232));g.fillRoundRect(16,height-64,width-32,49,12,12);
        g.setColor(new Color(82,100,122));g.drawString(follow?"FOLLOWING HEADING":"MANUAL VIEW",30,height-43);
        g.drawString(occupancy?"Unobserved = unknown / geometric OCC, no semantic classes":"Carrier icon is schematic  /  range curves are not lanes",30,height-26);
        if(pose==null){g.setColor(new Color(197,130,51));g.drawString("Tracking lost or initializing; no current vehicle position",25,120);}
    }
}
