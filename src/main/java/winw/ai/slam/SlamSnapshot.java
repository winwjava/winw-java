package winw.ai.slam;

import java.util.*;

/** Detached immutable world-coordinate data for GUI rendering; no native Mat crosses threads. */
public final class SlamSnapshot {
    public static final class MapPoint {
        public final double x,y,z;
        public MapPoint(double x,double y,double z){this.x=x;this.y=y;this.z=z;}
    }
    public static final class TrackPoint {
        public final Pose pose;
        public final boolean startsSegment;
        public TrackPoint(Pose pose,boolean startsSegment){this.pose=pose;this.startsSegment=startsSegment;}
    }
    public final List<MapPoint> points;
    public final List<TrackPoint> trajectory;
    public final List<Pose> keyframes;
    public SlamSnapshot(List<MapPoint> points,List<TrackPoint> trajectory,List<Pose> keyframes){
        this.points=Collections.unmodifiableList(new ArrayList<>(points));
        this.trajectory=Collections.unmodifiableList(new ArrayList<>(trajectory));
        this.keyframes=Collections.unmodifiableList(new ArrayList<>(keyframes));
    }
    public static SlamSnapshot empty(){return new SlamSnapshot(Collections.emptyList(),Collections.emptyList(),Collections.emptyList());}
}
