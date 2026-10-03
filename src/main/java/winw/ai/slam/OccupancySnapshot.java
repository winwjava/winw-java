package winw.ai.slam;

import java.io.*;
import java.nio.file.*;
import java.util.*;

/** Detached probabilistic 3D occupancy. Unobserved cells have prior p=0.5 and UNKNOWN state. */
public final class OccupancySnapshot {
    public enum State { UNKNOWN, FREE, OCCUPIED }
    public static final class Voxel {
        public final int ix,iy,iz,hitFrames,freeFrames;
        public final double x,y,z,probability;
        public final State state;
        Voxel(VoxelKey key,double resolution,double probability,int hits,int misses,State state){
            ix=key.x;iy=key.y;iz=key.z;x=(ix+.5)*resolution;y=(iy+.5)*resolution;z=(iz+.5)*resolution;
            this.probability=probability;hitFrames=hits;freeFrames=misses;this.state=state;
        }
    }
    static final class VoxelKey {
        final int x,y,z;
        VoxelKey(int x,int y,int z){this.x=x;this.y=y;this.z=z;}
        @Override public int hashCode(){return (x*73856093)^(y*19349663)^(z*83492791);}
        @Override public boolean equals(Object o){if(!(o instanceof VoxelKey))return false;VoxelKey b=(VoxelKey)o;return x==b.x&&y==b.y&&z==b.z;}
    }
    public final List<Voxel> voxels;
    public final double resolution;
    public final int activeFrames,occupiedCount,freeCount;
    /** True if observations were dropped to stay within the configured voxel budget. */
    public final boolean capacityLimited;
    private final Map<VoxelKey,Voxel> index;
    OccupancySnapshot(List<Voxel> voxels,double resolution,int activeFrames,boolean limited){
        this.voxels=Collections.unmodifiableList(new ArrayList<>(voxels));this.resolution=resolution;this.activeFrames=activeFrames;capacityLimited=limited;
        Map<VoxelKey,Voxel> map=new HashMap<>();int occupied=0,free=0;
        for(Voxel v:voxels){map.put(new VoxelKey(v.ix,v.iy,v.iz),v);if(v.state==State.OCCUPIED)occupied++;if(v.state==State.FREE)free++;}
        index=Collections.unmodifiableMap(map);occupiedCount=occupied;freeCount=free;
    }
    public State stateAt(double x,double y,double z){Voxel v=at(x,y,z);return v==null?State.UNKNOWN:v.state;}
    public double probabilityAt(double x,double y,double z){Voxel v=at(x,y,z);return v==null?.5:v.probability;}
    private Voxel at(double x,double y,double z){
        if(!Double.isFinite(x)||!Double.isFinite(y)||!Double.isFinite(z))return null;
        return index.get(new VoxelKey((int)Math.floor(x/resolution),(int)Math.floor(y/resolution),(int)Math.floor(z/resolution)));
    }
    public void writeCsv(Path file)throws IOException{
        try(BufferedWriter w=Files.newBufferedWriter(file)){
            w.write("# geometric occupancy; unlisted cells UNKNOWN; resolution="+resolution+"\n");
            w.write("x,y,z,occupancy_probability,state,hit_frames,free_frames\n");
            for(Voxel v:voxels)w.write(String.format(Locale.ROOT,"%.5f,%.5f,%.5f,%.6f,%s,%d,%d%n",v.x,v.y,v.z,v.probability,v.state,v.hitFrames,v.freeFrames));
        }
    }
    public void writeOccupiedPly(Path file)throws IOException{
        try(BufferedWriter w=Files.newBufferedWriter(file)){
            w.write("ply\nformat ascii 1.0\nelement vertex "+occupiedCount+"\nproperty float x\nproperty float y\nproperty float z\nproperty float occupancy_probability\nend_header\n");
            for(Voxel v:voxels)if(v.state==State.OCCUPIED)w.write(String.format(Locale.ROOT,"%.5f %.5f %.5f %.6f%n",v.x,v.y,v.z,v.probability));
        }
    }
}
