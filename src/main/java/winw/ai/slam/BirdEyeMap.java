package winw.ai.slam;

import java.util.*;

/** Bird-eye projection of either legacy surface density or conservative 3D occupancy columns. */
public final class BirdEyeMap {
    public static final class Cell {
        public final double x,z;
        public final double minY,maxY;
        public final double probability;
        public final int observations;
        Cell(double x,double z,int observations,double minY,double maxY){this(x,z,observations,minY,maxY,Double.NaN);}
        Cell(double x,double z,int observations,double minY,double maxY,double probability){
            this.x=x;this.z=z;this.observations=observations;this.minY=minY;this.maxY=maxY;this.probability=probability;}
    }
    private static final class Accumulator {
        int count;double minY=Double.POSITIVE_INFINITY,maxY=Double.NEGATIVE_INFINITY;
        void add(double y){count++;minY=Math.min(minY,y);maxY=Math.max(maxY,y);}
    }
    public final List<Cell> cells;
    public final List<Cell> freeCells;
    public final OccupancySnapshot occupancy;
    public final double cellSize;
    public final int visiblePoints;
    private BirdEyeMap(List<Cell> cells,double cellSize,int visiblePoints){
        this(cells,cellSize,visiblePoints,Collections.emptyList(),null);
    }
    private BirdEyeMap(List<Cell> cells,double cellSize,int visiblePoints,List<Cell> freeCells,OccupancySnapshot occupancy){
        this.cells=Collections.unmodifiableList(cells);this.cellSize=cellSize;this.visiblePoints=visiblePoints;
        this.freeCells=Collections.unmodifiableList(freeCells);this.occupancy=occupancy;
    }
    public static BirdEyeMap build(List<SlamSnapshot.MapPoint> points,LiveSlamConfig config){
        Map<Long,Accumulator> counts=new HashMap<>();int visible=0;
        for(SlamSnapshot.MapPoint p:points){
            if(!Double.isFinite(p.x)||!Double.isFinite(p.y)||!Double.isFinite(p.z)||p.y<config.minY||p.y>config.maxY)continue;
            double dx=Math.floor(p.x/config.cellSize),dz=Math.floor(p.z/config.cellSize);
            if(Math.abs(dx)>Integer.MAX_VALUE||Math.abs(dz)>Integer.MAX_VALUE)continue;
            int x=(int)dx,z=(int)dz;long key=((long)x<<32)|(z&0xffffffffL);
            counts.computeIfAbsent(key,ignored->new Accumulator()).add(p.y);visible++;
        }
        List<Cell> cells=new ArrayList<>();
        for(Map.Entry<Long,Accumulator> entry:counts.entrySet()){
            long key=entry.getKey();int x=(int)(key>>32),z=(int)key;
            Accumulator a=entry.getValue();cells.add(new Cell((x+.5)*config.cellSize,(z+.5)*config.cellSize,a.count,a.minY,a.maxY));
        }
        return new BirdEyeMap(cells,config.cellSize,visible);
    }
    private static final class Column {
        int hits,freeLayers;double minY=Double.POSITIVE_INFINITY,maxY=Double.NEGATIVE_INFINITY,probability;
    }
    /** Occupied if any selected layer is occupied; free only if ALL selected layers are observed free. */
    public static BirdEyeMap fromOccupancy(OccupancySnapshot snapshot,double minY,double maxY){
        Map<Long,Column> columns=new HashMap<>();double r=snapshot.resolution;
        int first=(int)Math.ceil(minY/r-.5),last=(int)Math.floor(maxY/r-.5),required=last-first+1;
        for(OccupancySnapshot.Voxel voxel:snapshot.voxels){if(voxel.iy<first||voxel.iy>last)continue;
            long key=((long)voxel.ix<<32)|(voxel.iz&0xffffffffL);Column column=columns.computeIfAbsent(key,ignored->new Column());
            if(voxel.state==OccupancySnapshot.State.OCCUPIED){column.hits+=voxel.hitFrames;column.probability=Math.max(column.probability,voxel.probability);
                column.minY=Math.min(column.minY,voxel.y-r/2);column.maxY=Math.max(column.maxY,voxel.y+r/2);}
            if(voxel.state==OccupancySnapshot.State.FREE)column.freeLayers++;
        }
        List<Cell> occupied=new ArrayList<>(),free=new ArrayList<>();
        for(Map.Entry<Long,Column> entry:columns.entrySet()){
            int x=(int)(entry.getKey()>>32),z=(int)(long)entry.getKey();Column c=entry.getValue();
            if(c.hits>0)occupied.add(new Cell((x+.5)*r,(z+.5)*r,c.hits,c.minY,c.maxY,c.probability));
            else if(required>0&&c.freeLayers==required)free.add(new Cell((x+.5)*r,(z+.5)*r,c.freeLayers,minY,maxY,0));
        }
        return new BirdEyeMap(occupied,r,snapshot.occupiedCount,free,snapshot);
    }
}
