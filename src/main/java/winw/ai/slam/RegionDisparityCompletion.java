package winw.ai.slam;

import java.util.*;
import org.opencv.core.*;
import org.opencv.ximgproc.*;

/** Conservative image-region disparity models. Model-filled pixels remain explicitly low confidence. */
final class RegionDisparityCompletion {
    private static final class Region {double b,g,r,x,y;int count;List<double[]> seeds=new ArrayList<>();Set<Integer> neighbors=new HashSet<>();double[] plane;int origin=-1;}
    static boolean[] complete(Mat image,short[] disparity,float[] confidence,byte[] bgr,int width,int height){
        Mat labels=new Mat();SuperpixelSLIC slic=Ximgproc.createSuperpixelSLIC(image,Ximgproc.SLICO,28,10);
        try{slic.iterate(4);slic.enforceLabelConnectivity(20);slic.getLabels(labels);int[] id=new int[width*height];labels.get(0,0,id);
            Region[] regions=new Region[slic.getNumberOfSuperpixels()];for(int i=0;i<regions.length;i++)regions[i]=new Region();
            for(int y=0;y<height;y++)for(int x=0;x<width;x++){int i=y*width+x;Region region=regions[id[i]];region.count++;region.b+=bgr[i*3]&255;region.g+=bgr[i*3+1]&255;region.r+=bgr[i*3+2]&255;region.x+=x;region.y+=y;
                if(confidence[i]>=180&&disparity[i]>16&&x%2==0&&y%2==0)region.seeds.add(new double[]{x,y,disparity[i]/16.});
                if(x>0&&id[i-1]!=id[i])region.neighbors.add(id[i-1]);if(x+1<width&&id[i+1]!=id[i])region.neighbors.add(id[i+1]);if(y>0&&id[i-width]!=id[i])region.neighbors.add(id[i-width]);if(y+1<height&&id[i+width]!=id[i])region.neighbors.add(id[i+width]);
            }
            ArrayDeque<Integer> queue=new ArrayDeque<>();Random random=new Random(73);
            for(int i=0;i<regions.length;i++){Region r=regions[i];if(r.count==0)continue;r.b/=r.count;r.g/=r.count;r.r/=r.count;r.x/=r.count;r.y/=r.count;r.plane=fit(r.seeds,random);if(r.plane!=null){r.origin=i;queue.add(i);}}
            while(!queue.isEmpty()){int from=queue.removeFirst();Region a=regions[from],origin=regions[a.origin];
                for(int to:a.neighbors){Region b=regions[to];if(b.plane!=null||b.count==0)continue;
                    double color=Math.sqrt(Math.pow(a.b-b.b,2)+Math.pow(a.g-b.g,2)+Math.pow(a.r-b.r,2));
                    if(color>22||Math.sqrt(Math.pow(origin.b-b.b,2)+Math.pow(origin.g-b.g,2)+Math.pow(origin.r-b.r,2))>22||Math.hypot(b.x-origin.x,b.y-origin.y)>100)continue;
                    b.plane=a.plane;b.origin=a.origin;queue.add(to);
                }
            }
            boolean[] supported=new boolean[disparity.length];
            for(int y=0;y<height;y++)for(int x=0;x<width;x++){int i=y*width+x;if(confidence[i]>=128&&disparity[i]>16){supported[i]=true;continue;}
                Region region=regions[id[i]];if(region.plane==null)continue;double[] p=region.plane;double d=p[0]*x+p[1]*y+p[2];if(d<1||d>width/3.)continue;
                disparity[i]=(short)Math.round(d*16);confidence[i]=32; supported[i]=true;
            }
            return supported;
        }finally{labels.release();}
    }
    private static double[] fit(List<double[]> points,Random random){
        if(points.size()<20)return null;double[] best=null;int bestCount=0;
        for(int trial=0;trial<30;trial++){double[] a=points.get(random.nextInt(points.size())),b=points.get(random.nextInt(points.size())),c=points.get(random.nextInt(points.size()));
            double det=(b[0]-a[0])*(c[1]-a[1])-(c[0]-a[0])*(b[1]-a[1]);if(Math.abs(det)<10)continue;
            double u=((b[2]-a[2])*(c[1]-a[1])-(c[2]-a[2])*(b[1]-a[1]))/det,v=((b[0]-a[0])*(c[2]-a[2])-(c[0]-a[0])*(b[2]-a[2]))/det;
            if(Math.abs(u)>.5||Math.abs(v)>.5)continue;double[] plane={u,v,a[2]-u*a[0]-v*a[1]};int count=0;
            for(double[] p:points)if(Math.abs(plane[0]*p[0]+plane[1]*p[1]+plane[2]-p[2])<1.25)count++;
            if(count>bestCount){bestCount=count;best=plane;}
        }
        if(bestCount<20||bestCount<points.size()*.7)return null;
        // Refine only robust support; a small normal-equation solve avoids a heavyweight optimizer.
        double[][] h=new double[3][4];for(double[] p:points)if(Math.abs(best[0]*p[0]+best[1]*p[1]+best[2]-p[2])<1.25){double[] a={p[0],p[1],1};for(int j=0;j<3;j++){for(int k=0;k<3;k++)h[j][k]+=a[j]*a[k];h[j][3]+=a[j]*p[2];}}
        for(int j=0;j<3;j++){int pivot=j;for(int k=j+1;k<3;k++)if(Math.abs(h[k][j])>Math.abs(h[pivot][j]))pivot=k;double[] temp=h[j];h[j]=h[pivot];h[pivot]=temp;if(Math.abs(h[j][j])<1e-8)return best;double scale=h[j][j];for(int k=j;k<4;k++)h[j][k]/=scale;for(int r=0;r<3;r++)if(r!=j){double f=h[r][j];for(int k=j;k<4;k++)h[r][k]-=f*h[j][k];}}
        return new double[]{h[0][3],h[1][3],h[2][3]};
    }
}
