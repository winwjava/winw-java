package winw.ai.perception.visual;

import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.nio.charset.Charset;

import javax.swing.JButton;
import javax.swing.JFrame;

import com.github.sarxos.webcam.Webcam;
import com.github.sarxos.webcam.WebcamPanel;

public class CameraVisionDemo implements ActionListener {

	static Webcam webcam;

	@Override
	public void actionPerformed(ActionEvent event) {
		webcam.open();
		BufferedImage image = webcam.getImage();

		try {
			stereoVision(image);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	public void stereoVision(BufferedImage stereo) throws IOException {
		// TODO 需要从两侧裁剪一段。视野未重叠部分不能做视差计算。
		int crop = 20;// 两侧分别裁剪10个像素
		BufferedImage leftImage = stereo.getSubimage(crop, 0, 640 - crop * 2, 480);
		BufferedImage rightImage = stereo.getSubimage(640 + crop, 0, 640 - crop * 2, 480);// 两侧分别裁剪10个像素

		long t0 = System.currentTimeMillis();
//		BufferedImage image = ImageIO.read(new File(im));
//		FileOutputStream output = new FileOutputStream(new File(ot));
		StereoVision.disparityImage(leftImage, rightImage);

		long t1 = System.currentTimeMillis();
		System.out.println("Visual Blob, cost: " + (t1 - t0) + "ms.");

	}

	public static void main(String[] args) {
		System.out.println("CAM Total: " +
				Webcam.getWebcams().size());
		for (Webcam cam : Webcam.getWebcams()) {
			System.out.println("CAM: " +
					cam.getName() + " : " + cam.getViewSize().getHeight() + "," + cam.getViewSize().getWidth());
		}
		webcam = Webcam.getWebcams().get(0);
		webcam.setCustomViewSizes(new Dimension(2560, 720), new Dimension(1280, 480));
		webcam.setViewSize(new Dimension(1280, 480));

		JButton b = new JButton(new String("photograph".getBytes(), Charset.defaultCharset()));
		b.setPreferredSize(new Dimension(100, 30));
		b.addActionListener(new CameraVisionDemo());
		WebcamPanel panel = new WebcamPanel(webcam);
		panel.setFPSDisplayed(true);
		panel.setDisplayDebugInfo(true);
		panel.setImageSizeDisplayed(true);
//		panel.setMirrored(true);
		panel.add(b);
		JFrame window = new JFrame("Stereo Vision");
		window.pack();
		window.add(panel);
		window.setVisible(true);
		window.setResizable(true);
		window.setExtendedState(JFrame.MAXIMIZED_BOTH);
		window.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	}

}
