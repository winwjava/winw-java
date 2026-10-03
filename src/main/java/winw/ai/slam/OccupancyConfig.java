package winw.ai.slam;

/** Geometric inverse-sensor model, not a trained neural occupancy predictor. */
public final class OccupancyConfig {
    public boolean enabled=true;
    public double resolution=.25,radius=15,maxAgeSeconds=8,hitProbability=.72,missProbability=.35;
    public double occupiedThreshold=.65,freeThreshold=.35,disparitySigma=.5,minDepthConfidence=.2;
    public int maxFrames=20,maxVoxels=200000,frameInterval=3,minHitFrames=2;
    public void validate(){
        if(!positive(resolution)||resolution<.05||!positive(radius)||radius>100||!positive(maxAgeSeconds)
            ||maxFrames<2||maxFrames>100||maxVoxels<100||frameInterval<1||minHitFrames<1||minHitFrames>maxFrames
            ||!(hitProbability>.5&&hitProbability<1)||!(missProbability>0&&missProbability<.5)
            ||!(occupiedThreshold>.5&&occupiedThreshold<1)||!(freeThreshold>0&&freeThreshold<.5)
            ||!positive(disparitySigma)||!(minDepthConfidence>0&&minDepthConfidence<=1))
            throw new IllegalArgumentException("Invalid occupancy configuration");
    }
    private boolean positive(double value){return Double.isFinite(value)&&value>0;}
    OccupancyConfig copy(){OccupancyConfig c=new OccupancyConfig();c.enabled=enabled;c.resolution=resolution;c.radius=radius;
        c.maxAgeSeconds=maxAgeSeconds;c.hitProbability=hitProbability;c.missProbability=missProbability;
        c.occupiedThreshold=occupiedThreshold;c.freeThreshold=freeThreshold;c.disparitySigma=disparitySigma;
        c.minDepthConfidence=minDepthConfidence;c.maxFrames=maxFrames;c.maxVoxels=maxVoxels;
        c.frameInterval=frameInterval;c.minHitFrames=minHitFrames;return c;}
}
