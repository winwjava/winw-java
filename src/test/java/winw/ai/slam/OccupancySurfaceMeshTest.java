package winw.ai.slam;

import java.awt.image.BufferedImage;
import java.util.*;
import org.junit.Test;
import static org.junit.Assert.*;

public class OccupancySurfaceMeshTest {
    private OccupancySnapshot volume(int[][] cells){List<OccupancySnapshot.Voxel> list=new ArrayList<>();for(int[] p:cells)list.add(new OccupancySnapshot.Voxel(new OccupancySnapshot.VoxelKey(p[0],p[1],p[2]),.1,.9,2,0,OccupancySnapshot.State.OCCUPIED));return new OccupancySnapshot(list,.1,2,false);}
    @Test public void solidBlockMergesToSixExteriorFaces(){
        List<int[]> cells=new ArrayList<>();for(int x=0;x<2;x++)for(int y=0;y<2;y++)for(int z=0;z<2;z++)cells.add(new int[]{x,y,z});
        OccupancySurfaceMesh mesh=OccupancySurfaceMesh.build(volume(cells.toArray(new int[0][])));
        assertEquals(8,mesh.occupiedCells);assertEquals(6,mesh.faces.size());
        assertArrayEquals(new double[]{.2,.2,.2},mesh.max,1e-12);
    }
    @Test public void disconnectedObservationsDoNotFillGap(){
        OccupancySurfaceMesh mesh=OccupancySurfaceMesh.build(volume(new int[][]{{-1,0,0},{1,0,0}}));
        assertEquals(12,mesh.faces.size());
        for(OccupancySurfaceMesh.Face face:mesh.faces)if(face.axis!=0)for(double[] p:face.corners)assertFalse(p[0]>.00001&&p[0]<.09999);
    }
    @Test public void rendererWorksWithoutCameraOrGpu(){
        Scene3DPanel panel=new Scene3DPanel();panel.update(volume(new int[][]{{0,0,0},{1,0,0},{1,1,0}}),"synthetic renderer test");
        BufferedImage image=panel.image(900,680);int bright=0;
        for(int y=100;y<580;y++)for(int x=100;x<800;x++)if((image.getRGB(x,y)&255)>90)bright++;
        assertTrue(bright>500);
        panel.update(null,"lost");assertNotNull(panel.image(320,240));
    }
    @Test public void smoothSurfacePreservesUnobservedGap(){
        OccupancySurfaceMesh mesh=OccupancySurfaceMesh.buildSmooth(volume(new int[][]{{-1,0,0},{1,0,0}}));
        assertFalse(mesh.faces.isEmpty());
        for(OccupancySurfaceMesh.Face face:mesh.faces){double min=Double.POSITIVE_INFINITY,max=Double.NEGATIVE_INFINITY;
            for(double[] p:face.corners){min=Math.min(min,p[0]);max=Math.max(max,p[0]);}
            assertTrue("No face may bridge the unknown gap",max<=1e-12||min>=.1-1e-12);
        }
    }
}
