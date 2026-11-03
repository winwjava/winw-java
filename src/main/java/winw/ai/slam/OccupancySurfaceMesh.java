package winw.ai.slam;

import java.util.*;

/** Greedy surface extraction from observed OCC cells. Never fills unknown interiors or convex hulls. */
public final class OccupancySurfaceMesh {
    public static final class Face {
        public final double[][] corners;
        public final int axis,sign;
        public final int rgb;
        public final double confidence;
        Face(double[][] corners,int axis,int sign){this(corners,axis,sign,-1,1);}
        Face(double[][] corners,int axis,int sign,int rgb,double confidence){this.corners=corners;this.axis=axis;this.sign=sign;this.rgb=rgb;this.confidence=confidence;}
    }
    private static final class Plane {
        final int axis,sign,coordinate;
        Plane(int a,int s,int c){axis=a;sign=s;coordinate=c;}
        public int hashCode(){return Objects.hash(axis,sign,coordinate);}
        public boolean equals(Object o){if(!(o instanceof Plane))return false;Plane p=(Plane)o;return axis==p.axis&&sign==p.sign&&coordinate==p.coordinate;}
    }
    public final List<Face> faces;
    public final int occupiedCells;
    public final double[] min,max;
    private OccupancySurfaceMesh(List<Face> faces,int count,double[] min,double[] max){this.faces=Collections.unmodifiableList(faces);occupiedCells=count;this.min=min;this.max=max;}
    static OccupancySurfaceMesh surface(List<Face> faces,double[] min,double[] max){return new OccupancySurfaceMesh(faces,0,min,max);}
    public void write(java.nio.file.Path path)throws java.io.IOException {
        try(java.io.DataOutputStream out=new java.io.DataOutputStream(new java.io.BufferedOutputStream(java.nio.file.Files.newOutputStream(path)))){
            out.writeInt(0x53555246);out.writeInt(faces.size());for(Face face:faces){out.writeInt(face.corners.length);for(double[] p:face.corners)for(double x:p)out.writeFloat((float)x);out.writeInt(face.rgb);out.writeFloat((float)face.confidence);}
        }
    }
    public static OccupancySurfaceMesh read(java.nio.file.Path path)throws java.io.IOException {
        List<Face> faces=new ArrayList<>();double[] min={Double.POSITIVE_INFINITY,Double.POSITIVE_INFINITY,Double.POSITIVE_INFINITY},max={Double.NEGATIVE_INFINITY,Double.NEGATIVE_INFINITY,Double.NEGATIVE_INFINITY};
        try(java.io.DataInputStream in=new java.io.DataInputStream(new java.io.BufferedInputStream(java.nio.file.Files.newInputStream(path)))){
            if(in.readInt()!=0x53555246)throw new java.io.IOException("Invalid surface file");int count=in.readInt();if(count<0||count>1000000)throw new java.io.IOException("Invalid face count");
            for(int i=0;i<count;i++){int n=in.readInt();if(n!=3&&n!=4)throw new java.io.IOException("Invalid face");double[][] corners=new double[n][3];for(double[] p:corners)for(int k=0;k<3;k++){p[k]=in.readFloat();if(!Double.isFinite(p[k]))throw new java.io.IOException("Invalid vertex");min[k]=Math.min(min[k],p[k]);max[k]=Math.max(max[k],p[k]);}faces.add(new Face(corners,0,1,in.readInt(),in.readFloat()));}
        }
        if(faces.isEmpty()){Arrays.fill(min,0);Arrays.fill(max,0);}return surface(faces,min,max);
    }
    /** Half-cell isosurface of measured occupied cells. Unknown is not treated as observed free. */
    public static OccupancySurfaceMesh buildSmooth(OccupancySnapshot snapshot){
        OccupancySurfaceMesh bounds=build(snapshot);
        Set<OccupancySnapshot.VoxelKey> nodes=new HashSet<>(),cubes=new HashSet<>();
        for(OccupancySnapshot.Voxel v:snapshot.voxels)if(v.state==OccupancySnapshot.State.OCCUPIED){
            nodes.add(new OccupancySnapshot.VoxelKey(v.ix,v.iy,v.iz));
            for(int x=-1;x<=0;x++)for(int y=-1;y<=0;y++)for(int z=-1;z<=0;z++)cubes.add(new OccupancySnapshot.VoxelKey(v.ix+x,v.iy+y,v.iz+z));
        }
        int[][] tetra={{0,1,3,7},{0,3,2,7},{0,2,6,7},{0,6,4,7},{0,4,5,7},{0,5,1,7}};
        List<Face> faces=new ArrayList<>();
        for(OccupancySnapshot.VoxelKey cube:cubes){double[][] p=new double[8][3];boolean[] inside=new boolean[8];
            for(int i=0;i<8;i++){int x=cube.x+(i&1),y=cube.y+((i>>1)&1),z=cube.z+((i>>2)&1);p[i]=new double[]{(x+.5)*snapshot.resolution,(y+.5)*snapshot.resolution,(z+.5)*snapshot.resolution};inside[i]=nodes.contains(new OccupancySnapshot.VoxelKey(x,y,z));}
            for(int[] tet:tetra){List<Integer> in=new ArrayList<>(),out=new ArrayList<>();for(int i:tet)(inside[i]?in:out).add(i);
                if(in.isEmpty()||out.isEmpty())continue;
                if(in.size()==1||out.size()==1){int pivot=(in.size()==1?in:out).get(0);List<Integer> other=in.size()==1?out:in;double[][] triangle=new double[3][];for(int i=0;i<3;i++)triangle[i]=mid(p[pivot],p[other.get(i)]);faces.add(new Face(triangle,0,1));}
                else{double[] a=mid(p[in.get(0)],p[out.get(0)]),b=mid(p[in.get(0)],p[out.get(1)]),c=mid(p[in.get(1)],p[out.get(0)]),d=mid(p[in.get(1)],p[out.get(1)]);
                    faces.add(new Face(new double[][]{a,b,d},0,1));faces.add(new Face(new double[][]{a,d,c},0,1));}
            }
        }
        return new OccupancySurfaceMesh(faces,bounds.occupiedCells,bounds.min,bounds.max);
    }
    private static double[] mid(double[] a,double[] b){return new double[]{(a[0]+b[0])/2,(a[1]+b[1])/2,(a[2]+b[2])/2};}
    private static long key(int u,int v){return ((long)u<<32)|(v&0xffffffffL);}
    public static OccupancySurfaceMesh build(OccupancySnapshot snapshot){
        Set<OccupancySnapshot.VoxelKey> occupied=new HashSet<>();
        double[] min={Double.POSITIVE_INFINITY,Double.POSITIVE_INFINITY,Double.POSITIVE_INFINITY},max={Double.NEGATIVE_INFINITY,Double.NEGATIVE_INFINITY,Double.NEGATIVE_INFINITY};
        for(OccupancySnapshot.Voxel voxel:snapshot.voxels)if(voxel.state==OccupancySnapshot.State.OCCUPIED)occupied.add(new OccupancySnapshot.VoxelKey(voxel.ix,voxel.iy,voxel.iz));
        Map<Plane,TreeSet<Long>> masks=new HashMap<>();
        for(OccupancySnapshot.VoxelKey voxel:occupied){int[] p={voxel.x,voxel.y,voxel.z};
            for(int a=0;a<3;a++){min[a]=Math.min(min[a],p[a]*snapshot.resolution);max[a]=Math.max(max[a],(p[a]+1)*snapshot.resolution);
                for(int sign:new int[]{-1,1}){int[] n=p.clone();n[a]+=sign;
                    if(!occupied.contains(new OccupancySnapshot.VoxelKey(n[0],n[1],n[2])))masks.computeIfAbsent(new Plane(a,sign,p[a]+(sign>0?1:0)),unused->new TreeSet<>()).add(key(p[(a+1)%3],p[(a+2)%3]));}
            }
        }
        List<Face> faces=new ArrayList<>();
        for(Map.Entry<Plane,TreeSet<Long>> entry:masks.entrySet()){Plane plane=entry.getKey();Set<Long> mask=entry.getValue();
            while(!mask.isEmpty()){long first=entry.getValue().first();int u=(int)(first>>32),v=(int)first,width=1,height=1;
                while(mask.contains(key(u+width,v)))width++;
                boolean row=true;while(row){for(int x=0;x<width;x++)if(!mask.contains(key(u+x,v+height))){row=false;break;}if(row)height++;}
                for(int x=0;x<width;x++)for(int y=0;y<height;y++)mask.remove(key(u+x,v+y));
                double[][] corners=new double[4][3];int[][] offsets={{0,0},{width,0},{width,height},{0,height}};
                for(int i=0;i<4;i++){corners[i][plane.axis]=plane.coordinate*snapshot.resolution;corners[i][(plane.axis+1)%3]=(u+offsets[i][0])*snapshot.resolution;corners[i][(plane.axis+2)%3]=(v+offsets[i][1])*snapshot.resolution;}
                faces.add(new Face(corners,plane.axis,plane.sign));
            }
        }
        if(occupied.isEmpty()){Arrays.fill(min,0);Arrays.fill(max,0);}
        return new OccupancySurfaceMesh(faces,occupied.size(),min,max);
    }
}
