package winw.ai.slam;

import java.util.*;
import winw.ai.slam.OccupancySnapshot.*;

/**
 * Sliding-window 3D occupancy, ray clearing, scan deduplication and keyframe-anchored replay.
 * Full 3D DDA traverses measured rays only; no invalid-depth rays or inferred occluded volume.
 */
public final class TemporalOccupancyMapper {
    public static final class DepthSample {
        public final double x,y,z,confidence,depthSigma;
        public DepthSample(double x,double y,double z,double confidence){
            this(x,y,z,confidence,0);
        }
        public DepthSample(double x,double y,double z,double confidence,double depthSigma){
            if(!Double.isFinite(x)||!Double.isFinite(y)||!Double.isFinite(z)||z<=0||!(confidence>0&&confidence<=1))
                throw new IllegalArgumentException("Invalid depth sample");
            if(!Double.isFinite(depthSigma)||depthSigma<0)throw new IllegalArgumentException("Invalid depth uncertainty");
            this.x=x;this.y=y;this.z=z;this.confidence=confidence;this.depthSigma=depthSigma;
        }
    }
    private static final class Evidence {
        double weight;boolean hit;
        Evidence(double weight,boolean hit){this.weight=weight;this.hit=hit;}
    }
    private static final class Counts {
        double hits,misses;int hitFrames,freeFrames;
    }
    private static final class Observation {
        double time;int anchor;Pose relative,world;
        List<DepthSample> samples;Map<VoxelKey,Evidence> evidence;
        Observation(double time,int anchor,Pose relative,List<DepthSample> samples){
            this.time=time;this.anchor=anchor;this.relative=relative;this.samples=new ArrayList<>(samples);
        }
    }
    private final OccupancyConfig config;
    private final double minY,maxY,hitLog,missLog;
    private final ArrayDeque<Observation> observations=new ArrayDeque<>();
    private final Map<VoxelKey,Counts> grid=new HashMap<>();
    private double time=Double.NEGATIVE_INFINITY,centreX,centreZ,replayCentreX,replayCentreZ;
    private boolean limited;
    public TemporalOccupancyMapper(OccupancyConfig config,double minY,double maxY){
        config.validate();if(!Double.isFinite(minY)||!Double.isFinite(maxY)||minY>=maxY)throw new IllegalArgumentException("Height interval");
        this.config=config.copy();this.minY=minY;this.maxY=maxY;hitLog=logOdds(config.hitProbability);missLog=logOdds(config.missProbability);
    }
    private double logOdds(double p){return Math.log(p/(1-p));}
    public void observe(double timestamp,int anchor,Pose anchorFromCamera,List<DepthSample> samples,List<Pose> keyframes,Pose current){
        if(timestamp<=time)throw new IllegalArgumentException("Occupancy observation time must increase");
        if(anchor<0||anchor>=keyframes.size())throw new IllegalArgumentException("Invalid occupancy anchor");
        refresh(timestamp,keyframes,current);
        Observation observation=new Observation(timestamp,anchor,Objects.requireNonNull(anchorFromCamera),samples);
        observation.world=keyframes.get(anchor).multiply(anchorFromCamera);observation.evidence=cast(observation);
        observations.addLast(observation);apply(observation.evidence,1);
        while(observations.size()>config.maxFrames)removeOldest();enforceCapacity();
    }
    /** Age out stale evidence even during LOST, and replay corrected poses after a loop. */
    public void refresh(double timestamp,List<Pose> keyframes,Pose current){
        if(!Double.isFinite(timestamp)||timestamp<time)throw new IllegalArgumentException("Occupancy time must be finite and nondecreasing");
        time=timestamp;
        if(current!=null){double[] t=current.translation();centreX=t[0];centreZ=t[2];}
        while(!observations.isEmpty()&&timestamp-observations.peekFirst().time>config.maxAgeSeconds)removeOldest();
        boolean replay=Math.hypot(centreX-replayCentreX,centreZ-replayCentreZ)>config.resolution;
        for(Observation o:observations){
            if(o.anchor>=keyframes.size())throw new IllegalArgumentException("Missing occupancy anchor");
            Pose corrected=keyframes.get(o.anchor).multiply(o.relative);
            if(!same(o.world,corrected))replay=true;o.world=corrected;
        }
        // Replaying the retained window prevents stale free-space evidence after loop corrections.
        if(replay){grid.clear();for(Observation o:observations){o.evidence=cast(o);apply(o.evidence,1);}replayCentreX=centreX;replayCentreZ=centreZ;}
        enforceCapacity();
    }
    private boolean same(Pose a,Pose b){return a.distance(b)<1e-7&&Arrays.equals(a.rotation(),b.rotation());}
    private void enforceCapacity(){while(grid.size()>config.maxVoxels&&!observations.isEmpty()){limited=true;removeOldest();}}
    private void removeOldest(){Observation oldest=observations.removeFirst();apply(oldest.evidence,-1);}
    private void apply(Map<VoxelKey,Evidence> evidence,int sign){
        for(Map.Entry<VoxelKey,Evidence> entry:evidence.entrySet()){
            Counts counts=grid.computeIfAbsent(entry.getKey(),ignored->new Counts());Evidence e=entry.getValue();
            if(e.hit){counts.hits+=sign*e.weight;counts.hitFrames+=sign;}else{counts.misses+=sign*e.weight;counts.freeFrames+=sign;}
            if(counts.hitFrames==0&&counts.freeFrames==0)grid.remove(entry.getKey());
        }
    }
    private boolean inside(VoxelKey key){double x=(key.x+.5)*config.resolution,y=(key.y+.5)*config.resolution,z=(key.z+.5)*config.resolution;
        return y>=minY&&y<=maxY&&Math.abs(x-centreX)<=config.radius&&Math.abs(z-centreZ)<=config.radius;}
    private Map<VoxelKey,Evidence> cast(Observation observation){
        Map<VoxelKey,Evidence> evidence=new HashMap<>();double[] origin=observation.world.translation();
        for(DepthSample sample:observation.samples){
            if(sample.confidence<config.minDepthConfidence)continue;
            double[] end=observation.world.transform(new double[]{sample.x,sample.y,sample.z});
            ray(origin,end,sample.confidence,sample.depthSigma,evidence);
        }
        return evidence;
    }
    private void add(Map<VoxelKey,Evidence> scan,VoxelKey key,boolean hit,double weight){
        if(!inside(key))return;
        Evidence existing=scan.get(key);
        // Hit wins over a crossing free ray in the same scan; duplicate pixels cannot inflate confidence.
        if(existing==null)scan.put(key,new Evidence(weight,hit));
        else if(hit&&!existing.hit){existing.hit=true;existing.weight=weight;}
        else if(hit==existing.hit)existing.weight=Math.max(existing.weight,weight);
    }
    private void ray(double[] start,double[] end,double confidence,double sigma,Map<VoxelKey,Evidence> scan){
        double resolution=config.resolution;int[] cell=new int[3],target=new int[3],step=new int[3];
        double[] tMax=new double[3],tDelta=new double[3];
        double length=0;for(int axis=0;axis<3;axis++){double direction=end[axis]-start[axis];length+=direction*direction;
            cell[axis]=(int)Math.floor(start[axis]/resolution);target[axis]=(int)Math.floor(end[axis]/resolution);
            step[axis]=direction>0?1:direction<0?-1:0;
            if(step[axis]==0){tMax[axis]=tDelta[axis]=Double.POSITIVE_INFINITY;}
            else{double boundary=(cell[axis]+(step[axis]>0?1:0))*resolution;
                tMax[axis]=(boundary-start[axis])/direction;tDelta[axis]=resolution/Math.abs(direction);}
        }
        // Reject implausibly long external measurements to bound traversal cost.
        if(Math.sqrt(length)>config.radius*3)return;
        int limit=Math.abs(target[0]-cell[0])+Math.abs(target[1]-cell[1])+Math.abs(target[2]-cell[2])+4;
        for(int iteration=0;iteration<limit;iteration++){
            boolean endpoint=Arrays.equals(cell,target);VoxelKey key=new VoxelKey(cell[0],cell[1],cell[2]);
            if(endpoint){add(scan,key,true,confidence);return;}
            // Camera origin is not an observed free voxel.
            // Preserve an UNKNOWN band before the uncertain depth endpoint instead of over-clearing.
            double distance=0;for(int axis=0;axis<3;axis++){double d=(cell[axis]+.5)*resolution-end[axis];distance+=d*d;}
            if(iteration>0&&Math.sqrt(distance)>2*sigma+resolution*.8661)add(scan,key,false,confidence);
            double next=Math.min(tMax[0],Math.min(tMax[1],tMax[2]));
            for(int axis=0;axis<3;axis++)if(tMax[axis]<=next+1e-12){cell[axis]+=step[axis];tMax[axis]+=tDelta[axis];}
        }
    }
    public OccupancySnapshot snapshot(){
        List<Voxel> voxels=new ArrayList<>();
        for(Map.Entry<VoxelKey,Counts> entry:grid.entrySet()){
            if(!inside(entry.getKey()))continue;Counts c=entry.getValue();
            double log=Math.max(-3.5,Math.min(3.5,c.hits*hitLog+c.misses*missLog));double probability=1/(1+Math.exp(-log));
            State state=probability>=config.occupiedThreshold&&c.hitFrames>=config.minHitFrames?State.OCCUPIED:
                probability<=config.freeThreshold&&c.freeFrames>0?State.FREE:State.UNKNOWN;
            voxels.add(new Voxel(entry.getKey(),config.resolution,probability,c.hitFrames,c.freeFrames,state));
        }
        return new OccupancySnapshot(voxels,config.resolution,observations.size(),limited);
    }
}
