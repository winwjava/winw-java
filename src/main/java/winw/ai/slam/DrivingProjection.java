package winw.ai.slam;

import java.awt.geom.Point2D;

/** Heading-aligned schematic perspective. Ground plane is the initial camera's X-Z plane. */
public final class DrivingProjection {
    private final double x,z,sin,cos,scale;
    private final int width,height;
    public DrivingProjection(double x,double z,double yaw,double scale,int width,int height){
        this.x=x;this.z=z;sin=Math.sin(yaw);cos=Math.cos(yaw);this.scale=scale;this.width=width;this.height=height;
    }
    /** Local x points right and local z points forward. */
    public double[] local(double worldX,double worldZ){
        double dx=worldX-x,dz=worldZ-z;return new double[]{cos*dx-sin*dz,sin*dx+cos*dz};
    }
    public Point2D.Double project(double worldX,double worldZ){double[] p=local(worldX,worldZ);return projectLocal(p[0],p[1],0);}
    public Point2D.Double projectLocal(double right,double forward,double down){
        double perspective=1/Math.max(.45,1+forward*.035);
        return new Point2D.Double(width*.5+right*scale*perspective,height*.77-forward*scale*.68*perspective+down*scale*.65*perspective);
    }
    public static double yaw(Pose pose){double[] r=pose.rotation();return Math.atan2(r[2],r[8]);}
}
