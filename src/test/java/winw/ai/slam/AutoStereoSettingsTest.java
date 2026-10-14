package winw.ai.slam;

import static org.junit.Assert.*;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.nio.file.*;
import javax.imageio.ImageIO;
import javax.swing.*;
import org.junit.Test;

public class AutoStereoSettingsTest {
    @Test public void typedValuesAreCommittedAndCaptureSnapshotIsIndependent()throws Exception{
        SwingUtilities.invokeAndWait(()->{
            AutoStereoSettings original=new AutoStereoSettings();
            AutoStereoWindow.Controls panel=new AutoStereoWindow.Controls(original);
            ((JSpinner.DefaultEditor)panel.width.getEditor()).getTextField().setText("800");
            ((JSpinner.DefaultEditor)panel.baseline.getEditor()).getTextField().setText("61");
            panel.layout.setSelectedIndex(1);panel.swap.setSelected(true);panel.backend.setSelectedIndex(1);
            AutoStereoSettings snapshot=panel.read();
            assertEquals(800,snapshot.width);assertEquals(61,snapshot.baselineMm,0);
            assertEquals(640,original.width);assertEquals(0,original.baselineMm,0);
            panel.width.setValue(960);assertEquals(800,snapshot.width);
            assertEquals(LiveSlamConfig.Layout.TOP_BOTTOM,snapshot.cameraConfig().layout);
            assertTrue(snapshot.cameraConfig().swapEyes);assertEquals(700,snapshot.cameraConfig().backend);
            // Render the actual form for visual inspection without opening a physical camera.
            panel.setSize(1400,260);layout(panel);
            BufferedImage image=new BufferedImage(1400,260,BufferedImage.TYPE_INT_RGB);Graphics2D g=image.createGraphics();panel.printAll(g);g.dispose();
            try{ImageIO.write(image,"png",Paths.get("target/auto-stereo-settings-preview.png").toFile());}
            catch(java.io.IOException error){throw new java.io.UncheckedIOException(error);}
        });
    }
    @Test public void rejectsInvalidCombinationBeforeCameraRestart()throws Exception{
        SwingUtilities.invokeAndWait(()->{
            AutoStereoWindow.Controls panel=new AutoStereoWindow.Controls(new AutoStereoSettings());
            panel.width.setValue(160);
            try{panel.read();fail("Disparity range is too large for this width");}
            catch(IllegalArgumentException expected){assertTrue(expected.getMessage().contains("视差"));}
            panel.width.setValue(640);panel.block.setValue(4);
            try{panel.read();fail("Even matching window must be rejected");}
            catch(IllegalArgumentException expected){assertTrue(expected.getMessage().contains("奇数"));}
        });
    }
    @Test public void metricCameraReceivesTheActualCaptureSettings()throws Exception{
        AutoStereoSettings settings=new AutoStereoSettings();settings.device=2;settings.backend=1400;
        settings.layout=LiveSlamConfig.Layout.TOP_BOTTOM;settings.swapEyes=true;settings.fps=15;
        settings.fourcc="";settings.disparities=64;
        Path file=Files.createTempFile("ui-live-settings-", ".properties");
        try{AutoStereoWindow.saveCameraSettings(settings,file);LiveSlamConfig loaded=LiveSlamConfig.load(file);
            assertEquals(settings.layout,loaded.layout);assertTrue(loaded.swapEyes);assertEquals(1400,loaded.backend);
            assertEquals(15,loaded.fps,0);assertEquals("",loaded.fourcc);assertEquals(64,loaded.disparities);
        }finally{Files.deleteIfExists(file);}
    }
    private static void layout(Container container){container.doLayout();for(Component child:container.getComponents())if(child instanceof Container)layout((Container)child);}
}
