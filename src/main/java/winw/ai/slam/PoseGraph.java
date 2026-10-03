package winw.ai.slam;

import java.util.ArrayList;
import java.util.List;
import org.apache.commons.math3.linear.*;

/** Small dense pose graph: fixed first pose, numerical Jacobians, robust loop edges. */
public final class PoseGraph {
    private static final class Edge {
        int a,b; Pose relative; boolean loop;
        Edge(int a,int b,Pose p,boolean loop){this.a=a;this.b=b;relative=p;this.loop=loop;}
    }
    private final List<Pose> poses=new ArrayList<>();
    private final List<Edge> edges=new ArrayList<>();
    public int addPose(Pose pose){poses.add(pose);return poses.size()-1;}
    public Pose pose(int i){return poses.get(i);}
    public int size(){return poses.size();}
    public void addEdge(int a,int b,Pose relative,boolean loop) {
        if(a<0||b<0||a>=size()||b>=size()||a==b) throw new IllegalArgumentException("Invalid edge");
        edges.add(new Edge(a,b,relative,loop));
    }
    private double[] residual(Edge e,Pose a,Pose b){return e.relative.inverse().multiply(a.inverse().multiply(b)).vector();}
    private double cost(List<Pose> values) {
        double cost=0;
        for(Edge e:edges){double n=0;for(double x:residual(e,values.get(e.a),values.get(e.b)))n+=x*x;
            cost+= e.loop&&n>1 ? 2*Math.sqrt(n)-1:n;}
        return cost;
    }
    public double cost(){return cost(poses);}
    public void optimize(int iterations) {
        int n=(size()-1)*6;if(n==0)return;
        double lambda=1e-4;
        for(int iteration=0;iteration<iterations;iteration++) {
            double[][] h=new double[n][n];double[] g=new double[n];
            for(Edge e:edges){
                double[] res=residual(e,pose(e.a),pose(e.b));double norm=0;for(double v:res)norm+=v*v;
                double weight=e.loop&&norm>1?1/Math.sqrt(norm):1;
                int[] ids={e.a,e.b};double[][][] jac=new double[2][6][6];
                for(int s=0;s<2;s++) if(ids[s]>0) for(int k=0;k<6;k++) {
                    double[] d=new double[6];d[k]=1e-6;
                    Pose changed=pose(ids[s]).multiply(Pose.fromVector(d));
                    double[] rr=residual(e,s==0?changed:pose(e.a),s==1?changed:pose(e.b));
                    for(int r=0;r<6;r++)jac[s][r][k]=(rr[r]-res[r])/1e-6;
                }
                for(int s=0;s<2;s++)if(ids[s]>0)for(int k=0;k<6;k++) {
                    int row=(ids[s]-1)*6+k;
                    for(int r=0;r<6;r++)g[row]+=weight*jac[s][r][k]*res[r];
                    for(int u=0;u<2;u++)if(ids[u]>0)for(int l=0;l<6;l++)for(int r=0;r<6;r++)
                        h[row][(ids[u]-1)*6+l]+=weight*jac[s][r][k]*jac[u][r][l];
                }
            }
            for(int i=0;i<n;i++){h[i][i]+=lambda;g[i]=-g[i];}
            RealVector step=new QRDecomposition(new Array2DRowRealMatrix(h,false)).getSolver().solve(new ArrayRealVector(g,false));
            List<Pose> next=new ArrayList<>();next.add(pose(0));
            for(int i=1;i<size();i++){double[] d=new double[6];for(int k=0;k<6;k++)d[k]=step.getEntry((i-1)*6+k);
                next.add(pose(i).multiply(Pose.fromVector(d)));}
            if(cost(next)<cost()){poses.clear();poses.addAll(next);lambda=Math.max(1e-8,lambda*.3);
                if(step.getNorm()<1e-6)break;
            }else lambda*=10;
        }
    }
}
