package winw.ai.slam;

import java.io.IOException;
import java.nio.file.Path;
import java.util.*;
import java.util.function.DoubleUnaryOperator;
import org.apache.commons.math3.linear.*;
import org.apache.commons.math3.optim.*;
import org.apache.commons.math3.optim.nonlinear.scalar.*;
import org.apache.commons.math3.optim.nonlinear.scalar.noderiv.BOBYQAOptimizer;

/** A weak eye may borrow constraints from the other eye, but both focal scales must be observable. */
final class JointFocalEstimator {
    private JointFocalEstimator() {}

    static double[] estimate(List<RealMatrix> left,List<RealMatrix> right,RealMatrix stereo,
                            int width,int height,Path log)throws IOException {
        if(left.size()<6||right.size()<6)throw new IOException("联合焦距估计需要每眼至少 6 组时序约束");
        double lo=.3*Math.max(width,height),hi=3*Math.max(width,height);
        for(int eye=0;eye<2;eye++){
            List<RealMatrix> frames=eye==0?left:right;
            DoubleUnaryOperator marginal=f->NaturalSceneCalibration.score(frames,f,width,height);
            double f=minimum(marginal,lo,hi),cost=marginal.applyAsDouble(f);
            double separation=Math.min(marginal.applyAsDouble(f*.8),marginal.applyAsDouble(f*1.2))-cost;
            log(log,"FOCAL_SEED eye="+(eye==0?"left":"right")+" f="+f+" residual="+cost+" separation="+separation
                +" weak="+(separation<1e-5)+"; final decision uses joint profile");
        }
        org.apache.commons.math3.analysis.MultivariateFunction objective=x->
            NaturalSceneCalibration.score(left,x[0],width,height)+NaturalSceneCalibration.score(right,x[1],width,height)
                +stereoScore(stereo,x[0],x[1],width,height);
        // Search the full physical range. A weak single-eye seed must not impose +/-20% bounds.
        List<double[]> seeds=new ArrayList<>();int grid=31;
        double[] focal=new double[grid],lc=new double[grid],rc=new double[grid];
        for(int i=0;i<grid;i++){
            focal[i]=lo*Math.pow(hi/lo,(double)i/(grid-1));
            lc[i]=NaturalSceneCalibration.score(left,focal[i],width,height);
            rc[i]=NaturalSceneCalibration.score(right,focal[i],width,height);
        }
        for(int i=0;i<grid;i++)for(int j=0;j<grid;j++)seeds.add(new double[]{focal[i],focal[j],lc[i]+rc[j]+stereoScore(stereo,focal[i],focal[j],width,height)});
        seeds.sort(Comparator.comparingDouble(x->x[2]));
        double[] best=null;double bestCost=Double.POSITIVE_INFINITY;List<double[]> tried=new ArrayList<>();
        for(double[] seed:seeds){
            if(tried.stream().anyMatch(x->Math.abs(Math.log(seed[0]/x[0]))+Math.abs(Math.log(seed[1]/x[1]))<.15))continue;
            tried.add(seed);
            try{
                PointValuePair solution=new BOBYQAOptimizer(5,lo*.05,1e-5).optimize(new MaxEval(600),
                    new InitialGuess(new double[]{seed[0],seed[1]}),new SimpleBounds(new double[]{lo,lo},new double[]{hi,hi}),
                    GoalType.MINIMIZE,new ObjectiveFunction(objective));
                if(Double.isFinite(solution.getValue())&&solution.getValue()<bestCost){bestCost=solution.getValue();best=solution.getPoint();}
            }catch(org.apache.commons.math3.exception.MathIllegalStateException error){log(log,"FOCAL_JOINT seed failed: "+error.getMessage());}
            if(tried.size()==4)break;
        }
        if(best==null)throw new IOException("联合焦距优化未收敛");
        double separation=Double.POSITIVE_INFINITY;
        // Profile out the other focal. Testing each axis while keeping the other fixed can
        // falsely declare a pure-translation rig observable: only their ratio is constrained.
        for(int eye=0;eye<2;eye++)for(double factor:new double[]{.8,1.2}){
            final int fixedEye=eye;final double fixed=best[eye]*factor;
            DoubleUnaryOperator profile=free->objective.value(fixedEye==0?new double[]{fixed,free}:new double[]{free,fixed});
            double free=minimum(profile,lo,hi),delta=profile.applyAsDouble(free)-bestCost;
            separation=Math.min(separation,delta);
            log(log,"FOCAL_PROFILE fixedEye="+(eye==0?"left":"right")+" factor="+factor+" fixed="+fixed+" optimizedOther="+free+" delta="+delta);
        }
        log(log,"FOCAL_JOINT left="+best[0]+" right="+best[1]+" residual="+bestCost+" profileSeparation="+separation+" seeds="+tried.size());
        if(best[0]<lo*1.05||best[1]<lo*1.05||best[0]>hi/1.05||best[1]>hi/1.05||bestCost>.005||separation<1e-5)
            throw new IOException("联合焦距仍不可可靠确定：left="+best[0]+", right="+best[1]+", residual="+bestCost
                +", profileSeparation="+separation+"；已评估双眼和双目约束，未导出米制标定");
        return best;
    }

    private static double stereoScore(RealMatrix f,double left,double right,int w,int h){
        RealMatrix e=NaturalSceneCalibration.intrinsic(right,w,h).transpose().multiply(f).multiply(NaturalSceneCalibration.intrinsic(left,w,h));
        double[] s=new SingularValueDecomposition(e).getSingularValues();
        double mismatch=(s[0]-s[1])/Math.max(1e-15,s[0]+s[1]);return 20*mismatch*mismatch;
    }
    private static double minimum(DoubleUnaryOperator objective,double lo,double hi){
        double best=lo,cost=Double.POSITIVE_INFINITY;int grid=160;
        for(int i=0;i<=grid;i++){double x=lo*Math.pow(hi/lo,(double)i/grid),value=objective.applyAsDouble(x);if(value<cost){best=x;cost=value;}}
        double a=Math.max(lo,best/1.03),b=Math.min(hi,best*1.03);
        for(int i=0;i<35;i++){double x=a+(b-a)/3,y=b-(b-a)/3;if(objective.applyAsDouble(x)<objective.applyAsDouble(y))b=y;else a=x;}
        return (a+b)/2;
    }
    private static void log(Path path,String message)throws IOException{if(path!=null)CalibrationRuntime.log(path,message);}
}
