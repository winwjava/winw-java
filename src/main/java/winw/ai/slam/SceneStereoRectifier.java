package winw.ai.slam;

import java.util.*;
import java.io.*;
import java.nio.file.*;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.features2d.*;
import org.opencv.imgproc.Imgproc;

/** Targetless projective rectification. Homographies are NOT metric camera calibration. */
public final class SceneStereoRectifier implements AutoCloseable {
    private final ORB orb;
    private final BFMatcher matcher=BFMatcher.create(Core.NORM_HAMMING,false);
    private final int width,height;
    private final double validationTolerance;
    private Mat leftH=new Mat(),rightH=new Mat(),pendingLeft=new Mat(),pendingRight=new Mat();
    private int confirmations,failures;
    public int matches,inliers;
    public double verticalError=Double.NaN;
    public String status="等待有纹理、不同距离的静态场景";
    private double minimumDisparity;
    private boolean trusted;
    public SceneStereoRectifier(int width,int height){
        this(width,height,3500,1.5);
    }
    public SceneStereoRectifier(int width,int height,int features,double validationTolerance){
        if(width<160||height<120)throw new IllegalArgumentException("Per-eye size must be at least 160x120");
        if(features<500||features>20000||!Double.isFinite(validationTolerance)||validationTolerance<.25||validationTolerance>5)
            throw new IllegalArgumentException("Invalid feature count or validation tolerance");
        this.width=width;this.height=height;this.validationTolerance=validationTolerance;orb=ORB.create(features);orb.setFastThreshold(10);
    }
    public boolean ready(){return trusted;}
    public int minimumDisparity(){return (int)Math.floor(minimumDisparity)-16;}
    /** Cache is a candidate only: three live observations must verify it again. */
    public void loadCandidate(Path path)throws IOException{
        Properties p=new Properties();try(Reader reader=Files.newBufferedReader(path)){p.load(reader);}
        if(!"projective-not-metric".equals(p.getProperty("mode"))||Integer.parseInt(p.getProperty("width"))!=width
            ||Integer.parseInt(p.getProperty("height"))!=height)throw new IllegalArgumentException("Rectification cache mode/size mismatch");
        Mat a=readMatrix(p,"left.H"),b=null;
        try{b=readMatrix(p,"right.H");if(!usable(a)||!usable(b))throw new IllegalArgumentException("Invalid cached homography");
            a.copyTo(pendingLeft);b.copyTo(pendingRight);confirmations=0;trusted=false;failures=0;
            status="已读取候选缓存，等待当前画面验证";
        }finally{a.release();if(b!=null)b.release();}
    }
    private static Mat readMatrix(Properties p,String key){
        String[] tokens=p.getProperty(key,"").split(",");if(tokens.length!=9)throw new IllegalArgumentException("Invalid "+key);
        double[] values=new double[9];for(int i=0;i<9;i++){values[i]=Double.parseDouble(tokens[i]);
            if(!Double.isFinite(values[i]))throw new IllegalArgumentException("Nonfinite "+key);}
        Mat m=new Mat(3,3,CvType.CV_64F);m.put(0,0,values);return m;
    }
    public void save(Path path)throws IOException{
        if(!ready())throw new IllegalStateException("Only verified rectification may be saved");
        Properties p=new Properties();p.setProperty("mode","projective-not-metric");p.setProperty("width",Integer.toString(width));p.setProperty("height",Integer.toString(height));
        p.setProperty("left.H",matrixText(leftH));p.setProperty("right.H",matrixText(rightH));
        try(Writer writer=Files.newBufferedWriter(path)){p.store(writer,"Natural-scene homographies; NOT a StereoRig metric calibration");}
    }
    private static String matrixText(Mat m){double[] values=new double[9];m.get(0,0,values);StringJoiner text=new StringJoiner(",");
        for(double v:values)text.add(Double.toString(v));return text.toString();}
    private void validate(Mat image){
        if(image.empty()||image.cols()!=width||image.rows()!=height||image.depth()!=CvType.CV_8U
            ||(image.channels()!=1&&image.channels()!=3&&image.channels()!=4))
            throw new IllegalArgumentException("Expected per-eye size 8-bit gray/BGR/BGRA input");
    }
    static Mat gray(Mat image){Mat g=new Mat();if(image.channels()==1)image.copyTo(g);
        else Imgproc.cvtColor(image,g,image.channels()==4?Imgproc.COLOR_BGRA2GRAY:Imgproc.COLOR_BGR2GRAY);return g;}
    /** Call periodically; candidate needs three independently acquired frames before use. */
    public boolean observe(Mat left,Mat right){
        validate(left);validate(right);
        matches=0;inliers=0;verticalError=Double.NaN;
        Mat l=gray(left),r=gray(right),ld=new Mat(),rd=new Mat(),mask=new Mat();
        MatOfKeyPoint lk=new MatOfKeyPoint(),rk=new MatOfKeyPoint();
        List<MatOfDMatch> forward=new ArrayList<>(),reverse=new ArrayList<>();
        try{
            orb.detectAndCompute(l,mask,lk,ld);orb.detectAndCompute(r,mask,rk,rd);
            if(ld.empty()||rd.empty())return reject("纹理不足，请对准书架或有远近层次的物体");
            matcher.knnMatch(ld,rd,forward,2);matcher.knnMatch(rd,ld,reverse,2);
            int[] back=new int[rd.rows()];Arrays.fill(back,-1);
            for(MatOfDMatch row:reverse){DMatch[] m=row.toArray();if(m.length==2&&m[0].distance<.75*m[1].distance)back[m[0].queryIdx]=m[0].trainIdx;}
            KeyPoint[] a=lk.toArray(),b=rk.toArray();List<Point> lp=new ArrayList<>(),rp=new ArrayList<>();
            for(MatOfDMatch row:forward){DMatch[] m=row.toArray();if(m.length==2&&m[0].distance<.75*m[1].distance&&back[m[0].trainIdx]==m[0].queryIdx){
                lp.add(a[m[0].queryIdx].pt);rp.add(b[m[0].trainIdx].pt);}}
            matches=lp.size();return estimate(lp,rp);
        }finally{for(Mat m:new Mat[]{l,r,ld,rd,mask,lk,rk})m.release();
            for(MatOfDMatch m:forward)m.release();for(MatOfDMatch m:reverse)m.release();}
    }
    // Package-visible geometric entry point allows testing real perspective camera correspondences.
    boolean estimate(List<Point> left,List<Point> right){
        matches=left.size();inliers=0;verticalError=Double.NaN;
        if(left.size()!=right.size()||left.size()<8)return reject("有效匹配不足（需要至少 8 对），请让特征分散在画面中");
        List<Point> trainL=new ArrayList<>(),trainR=new ArrayList<>(),testL=new ArrayList<>(),testR=new ArrayList<>();
        for(int i=0;i<left.size();i++){boolean test=i%4==0;
            (test?testL:trainL).add(left.get(i));(test?testR:trainR).add(right.get(i));}
        MatOfPoint2f a=new MatOfPoint2f(),b=new MatOfPoint2f(),il=new MatOfPoint2f(),ir=new MatOfPoint2f();
        Mat mask=new Mat(),f=null,h1=new Mat(),h2=new Mat();
        try{
            // A horizontal rig only needs a vertical offset, not an unconstrained F.
            // Fit on training points and verify on held-out points and later frames.
            if(left.size()<80&&coverage(trainL)&&coverage(trainR)){
                double[] offsets=new double[trainL.size()];
                for(int i=0;i<offsets.length;i++)offsets[i]=trainL.get(i).y-trainR.get(i).y;
                Arrays.sort(offsets);double dy=offsets[offsets.length/2];
                h1.release();h2.release();
                h1=Mat.eye(3,3,CvType.CV_64F);h2=Mat.eye(3,3,CvType.CV_64F);h2.put(1,2,dy);
                List<Point> consistentL=new ArrayList<>(),consistentR=new ArrayList<>();
                for(int i=0;i<trainL.size();i++)
                    if(Math.abs(trainL.get(i).y-trainR.get(i).y-dy)<=validationTolerance){
                        consistentL.add(trainL.get(i));consistentR.add(trainR.get(i));
                    }
                int consistent=consistentL.size();
                if(Math.abs(dy)<=height*.05&&consistent>=Math.max(6,Math.ceil(offsets.length*.8))
                    &&coverage(consistentL)&&coverage(consistentR)
                    &&error(h1,h2,testL,testR)<=validationTolerance){
                    inliers=consistent;verticalError=error(h1,h2,testL,testR);
                    return accept(h1,h2,testL,testR,consistentL,consistentR,"少点水平校正（近似、非米制）");
                }
                h1.release();h2.release();h1=new Mat();h2=new Mat();
            }
            if(trainL.size()<12)return reject("少量匹配不符合水平双目模型，等待更多分散特征");
            a.fromList(trainL);b.fromList(trainR);
            f=Calib3d.findFundamentalMat(a,b,Calib3d.FM_RANSAC,1.5,.999,mask);
            if(f.empty()||f.rows()!=3||f.cols()!=3)return reject("极线估计失败");
            List<Point> goodL=new ArrayList<>(),goodR=new ArrayList<>();
            for(int i=0;i<trainL.size();i++)if(mask.get(i,0)[0]!=0){goodL.add(trainL.get(i));goodR.add(trainR.get(i));}
            inliers=goodL.size();
            if(inliers<12||inliers<trainL.size()*.55||!coverage(goodL)||!coverage(goodR))return reject("匹配分布不足或误匹配过多");
            // A dominant plane does not constrain a unique stereo fundamental matrix.
            Mat planeMask=new Mat(),plane=null;
            try{il.fromList(goodL);ir.fromList(goodR);plane=Calib3d.findHomography(il,ir,Calib3d.RANSAC,2,planeMask);
                if(!plane.empty()&&Core.countNonZero(planeMask)>.90*inliers)return reject("场景接近平面，请加入近处与远处物体");
            }finally{planeMask.release();if(plane!=null)plane.release();}
            if(!Calib3d.stereoRectifyUncalibrated(il,ir,f,new Size(width,height),h1,h2,3))return reject("双目校正失败");
            if(!usable(h1)||!usable(h2))return reject("校正形变过大，等待更好的场景");
            double error=error(h1,h2,testL,testR);
            if(!Double.isFinite(error)||error>validationTolerance)return reject("独立验证误差过大，可能存在畸变或不同步");
            verticalError=error;
            return accept(h1,h2,testL,testR,goodL,goodR,"自然场景校正（非米制）");
        }finally{for(Mat m:new Mat[]{a,b,il,ir,mask,h1,h2})m.release();if(f!=null)f.release();}
    }
    private boolean accept(Mat h1,Mat h2,List<Point> testL,List<Point> testR,
                           List<Point> goodL,List<Point> goodR,String mode){
            failures=0;
            if(trusted&&error(leftH,rightH,testL,testR)<=validationTolerance){status=mode+"已验证";return false;}
            if(pendingLeft.empty()||error(pendingLeft,pendingRight,testL,testR)>validationTolerance){
                h1.copyTo(pendingLeft);h2.copyTo(pendingRight);confirmations=1;
                status="候选校正：等待后续画面验证 1/3";return false;
            }
            confirmations++;
            if(confirmations<3){status="候选校正：等待后续画面验证 "+confirmations+"/3";return false;}
            pendingLeft.copyTo(leftH);pendingRight.copyTo(rightH);trusted=true;confirmations=0;
            double[] d=new double[goodL.size()];
            for(int i=0;i<d.length;i++)d[i]=project(leftH,goodL.get(i)).x-project(rightH,goodR.get(i)).x;
            Arrays.sort(d);minimumDisparity=d[d.length/20];
            pendingLeft.release();pendingRight.release();status=mode+"已启用";return true;
    }
    private boolean reject(String reason){
        confirmations=0;pendingLeft.release();pendingRight.release();
        if(++failures>=3)trusted=false;
        status=reason+(trusted?"；暂时保留上次校正":"；暂停视差");return false;
    }
    private boolean coverage(List<Point> points){boolean[] cells=new boolean[12];
        for(Point p:points)if(p.x>=0&&p.y>=0&&p.x<width&&p.y<height)cells[Math.min(2,(int)(p.y*3/height))*4+Math.min(3,(int)(p.x*4/width))]=true;
        int n=0;for(boolean c:cells)if(c)n++;double minX=width,maxX=0,minY=height,maxY=0;
        for(Point p:points){minX=Math.min(minX,p.x);maxX=Math.max(maxX,p.x);minY=Math.min(minY,p.y);maxY=Math.max(maxY,p.y);}
        return n>=4&&maxX-minX>=width*.25&&maxY-minY>=height*.25;}
    static Point project(Mat h,Point p){double w=h.get(2,0)[0]*p.x+h.get(2,1)[0]*p.y+h.get(2,2)[0];
        return new Point((h.get(0,0)[0]*p.x+h.get(0,1)[0]*p.y+h.get(0,2)[0])/w,
            (h.get(1,0)[0]*p.x+h.get(1,1)[0]*p.y+h.get(1,2)[0])/w);}
    private boolean usable(Mat h){
        if(!Core.checkRange(h)||Math.abs(Core.determinant(h))<1e-12)return false;
        double sign=0;
        for(Point p:Arrays.asList(new Point(0,0),new Point(width,0),new Point(0,height),new Point(width,height))){
            double w=h.get(2,0)[0]*p.x+h.get(2,1)[0]*p.y+h.get(2,2)[0];
            if(Math.abs(w)<1e-9||sign*w<0)return false;sign=w;
            Point q=project(h,p);if(!Double.isFinite(q.x)||!Double.isFinite(q.y)||Math.abs(q.x)>3*width||Math.abs(q.y)>3*height)return false;}
        Mat white=new Mat(height,width,CvType.CV_8UC1,new Scalar(255)),warped=new Mat();
        try{Imgproc.warpPerspective(white,warped,h,new Size(width,height),Imgproc.INTER_NEAREST);
            return Core.countNonZero(warped)>.55*width*height;
        }finally{white.release();warped.release();}
    }
    private double error(Mat h1,Mat h2,List<Point> l,List<Point> r){
        double[] e=new double[l.size()];int n=0;
        for(int i=0;i<l.size();i++){Point a=project(h1,l.get(i)),b=project(h2,r.get(i));
            if(Double.isFinite(a.x)&&Double.isFinite(b.x)&&Double.isFinite(a.y)&&Double.isFinite(b.y)
                &&a.x>=0&&a.x<width&&b.x>=0&&b.x<width&&a.y>=0&&a.y<height&&b.y>=0&&b.y<height)e[n++]=Math.abs(a.y-b.y);}
        if(n<2||n<l.size()*.8)return Double.POSITIVE_INFINITY;
        Arrays.sort(e,0,n);return e[(int)Math.ceil(.80*(n-1))];
    }
    public void rectify(Mat left,Mat right,Mat l,Mat r){
        validate(left);validate(right);if(!ready())throw new IllegalStateException("No validated natural-scene rectification");
        Imgproc.warpPerspective(left,l,leftH,new Size(width,height));Imgproc.warpPerspective(right,r,rightH,new Size(width,height));
    }
    /** Warp masks prevent black borders being interpreted as observed scene pixels. */
    public void masks(Mat leftMask,Mat rightMask){
        if(!ready())throw new IllegalStateException("Not ready");Mat white=new Mat(height,width,CvType.CV_8UC1,new Scalar(255));
        try{Imgproc.warpPerspective(white,leftMask,leftH,new Size(width,height),Imgproc.INTER_NEAREST);
            Imgproc.warpPerspective(white,rightMask,rightH,new Size(width,height),Imgproc.INTER_NEAREST);
        }finally{white.release();}
    }
    @Override public void close(){leftH.release();rightH.release();pendingLeft.release();pendingRight.release();
        // Feature2D.clear is not implemented by every JNI build; wrappers own algorithm lifetime.
    }
}
