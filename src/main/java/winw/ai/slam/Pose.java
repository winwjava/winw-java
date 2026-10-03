package winw.ai.slam;

import java.util.Arrays;

/** Immutable rigid transform. Camera poses are T_world_camera, distances in metres. */
public final class Pose {
    private final double[] r, t;
    public Pose(double[] rotation, double[] translation) {
        if (rotation.length != 9 || translation.length != 3) throw new IllegalArgumentException("Pose dimensions");
        r = rotation.clone(); t = translation.clone();
        for (double v : r) if (!Double.isFinite(v)) throw new IllegalArgumentException("Nonfinite rotation");
        for (double v : t) if (!Double.isFinite(v)) throw new IllegalArgumentException("Nonfinite translation");
    }
    public static Pose identity() { return new Pose(new double[]{1,0,0,0,1,0,0,0,1}, new double[3]); }
    public double[] rotation() { return r.clone(); }
    public double[] translation() { return t.clone(); }
    public double[] transform(double[] p) {
        return new double[]{r[0]*p[0]+r[1]*p[1]+r[2]*p[2]+t[0],
            r[3]*p[0]+r[4]*p[1]+r[5]*p[2]+t[1], r[6]*p[0]+r[7]*p[1]+r[8]*p[2]+t[2]};
    }
    public Pose multiply(Pose b) {
        double[] a = new double[9];
        for(int i=0;i<3;i++) for(int j=0;j<3;j++) for(int k=0;k<3;k++) a[3*i+j]+=r[3*i+k]*b.r[3*k+j];
        return new Pose(a, transform(b.t));
    }
    public Pose inverse() {
        double[] a = {r[0],r[3],r[6],r[1],r[4],r[7],r[2],r[5],r[8]};
        double[] b = new double[3];
        for(int i=0;i<3;i++) for(int j=0;j<3;j++) b[i]-=a[i*3+j]*t[j];
        return new Pose(a,b);
    }
    /** Rotation-vector plus translation parameterization, used by the pose graph. */
    public static Pose fromVector(double[] v) {
        double theta=Math.sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
        double a=theta<1e-8 ? 1-theta*theta/6 : Math.sin(theta)/theta;
        double b=theta<1e-8 ? .5-theta*theta/24 : (1-Math.cos(theta))/(theta*theta);
        double[] k={0,-v[2],v[1],v[2],0,-v[0],-v[1],v[0],0}, rr=new double[9];
        for(int i=0;i<3;i++) for(int j=0;j<3;j++) {
            rr[i*3+j]=(i==j?1:0)+a*k[i*3+j];
            for(int n=0;n<3;n++) rr[i*3+j]+=b*k[i*3+n]*k[n*3+j];
        }
        return new Pose(rr,new double[]{v[3],v[4],v[5]});
    }
    public double[] vector() {
        // Quaternion extraction remains stable close to a 180 degree rotation.
        double[] q=quaternion(); double s=Math.sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]);
        double a=s<1e-10?2:2*Math.atan2(s,q[3])/s;
        return new double[]{q[0]*a,q[1]*a,q[2]*a,t[0],t[1],t[2]};
    }
    /** x,y,z,w; chooses the nonnegative w hemisphere. */
    public double[] quaternion() {
        double[] q=new double[4]; double trace=r[0]+r[4]+r[8];
        if(trace>0) {
            double s=2*Math.sqrt(1+trace); q[3]=s/4;
            q[0]=(r[7]-r[5])/s; q[1]=(r[2]-r[6])/s; q[2]=(r[3]-r[1])/s;
        } else {
            int i=r[0]>r[4]?(r[0]>r[8]?0:2):(r[4]>r[8]?1:2), j=(i+1)%3, k=(i+2)%3;
            double s=2*Math.sqrt(1+r[3*i+i]-r[3*j+j]-r[3*k+k]);
            q[i]=s/4; q[j]=(r[3*j+i]+r[3*i+j])/s; q[k]=(r[3*k+i]+r[3*i+k])/s;
            q[3]=(r[3*k+j]-r[3*j+k])/s;
        }
        if(q[3]<0) for(int i=0;i<4;i++) q[i]=-q[i];
        return q;
    }
    public double distance(Pose b) { double s=0; for(int i=0;i<3;i++) s+=(t[i]-b.t[i])*(t[i]-b.t[i]); return Math.sqrt(s); }
    @Override public String toString() { return Arrays.toString(t); }
}
