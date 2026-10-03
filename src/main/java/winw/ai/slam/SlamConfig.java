package winw.ai.slam;

/** Defaults for small, static scenes. Create a new engine after changing configuration. */
public final class SlamConfig {
    public int features=1800, minInliers=20, keyframeInterval=10, loopGap=20, maxKeyframes=80;
    public double ratio=.75, epipolarTolerance=1.5, minDepth=.3, maxDepth=60;
    public double reprojectionError=3, minInlierRatio=.35, keyframeTranslation=.15;
    void validate() {
        if(features<100 || minInliers<6 || keyframeInterval<1 || loopGap<2 || maxKeyframes<2
            || !(ratio>0&&ratio<1) || !(epipolarTolerance>0) || !(minDepth>0&&maxDepth>minDepth)
            || !(reprojectionError>0) || !(minInlierRatio>0&&minInlierRatio<=1) || !(keyframeTranslation>0))
            throw new IllegalArgumentException("Invalid SLAM configuration");
    }
    SlamConfig copy() {
        SlamConfig c=new SlamConfig(); c.features=features;c.minInliers=minInliers;c.keyframeInterval=keyframeInterval;
        c.loopGap=loopGap;c.maxKeyframes=maxKeyframes;c.ratio=ratio;c.epipolarTolerance=epipolarTolerance;
        c.minDepth=minDepth;c.maxDepth=maxDepth;c.reprojectionError=reprojectionError;
        c.minInlierRatio=minInlierRatio;c.keyframeTranslation=keyframeTranslation;return c;
    }
}
