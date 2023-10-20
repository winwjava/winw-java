package winw.ai.perception.visual;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javax.imageio.ImageIO;

/**
 * 形状。
 * 
 * 最佳连接。怎么避免周围线条干扰？前馈网络算法。
 * 
 * @author winw
 * @see VisualEdge
 *
 */
public class VisualForm {

	/**
	 * 复杂细胞感受野，由简单细胞相连（也可以是虚接相连），线条拟合。
	 * 
	 * <Point>
	 * 复杂细胞的感受野就是由相连的一系列有特定朝向的简单细胞来构建。
	 * <Point>
	 * 复杂细胞的感受野来自于一系列空间倾向性特定排列的简单细胞输入的转换。
	 * <Point>
	 * 在视觉皮层里还存在一类细胞，这类细胞不存在相互拮抗的开关区域。但是和简单细胞一样，它们也能对特定方向的条形光斑刺激产生反应。
	 * 而且不管是黑底白带的刺激类型还是白底黑带的刺激类型，都能产生类似的反应。这类细胞通常较简单细胞有更大的感受野，特定方向的条形光斑只要落在这个感受野上
	 * 它们都能对刺激产生反应。这类细胞被称为复杂细胞，可以认为，这类细胞与简单细胞相比是更加抽象的线条或者边缘的感受器。
	 * 
	 * <Point>
	 * 初级视皮质的细胞也存在水平连接，兴奋可以在细胞之间传播，这使得图像可以连接在一起被处理。可以看到，一组横线方向一致时会引起更大的反应。
	 */
	public static void complexCellReceptiveField(BufferedImage image, LineGrid[][] lineGridArray) {
		// 模拟V1中有方向选择性的复杂细胞的反应：
		// 复杂细胞层（Complex cells, C1）：将相邻子区域（如1和2）中相同方向简单细胞的反应取最大值。
		// 编码更大感受野范围内的方向特征，只要该区域内存在特定方向的视觉刺激，该神经元就会有特异性的反应。

		// 事实上，线性加和也可以将较小的感受野整合成更大的感受野。但与之相比，取最大值能提高神经网络抗背景噪声干扰的能力，从而保证检测特定特征，而不会因接收位置和场景而混淆。
		// 前馈算法。不同的组合可以构成多种形状，根据形状特征比较最大的组合可能性，前馈到组合上（否定原有的组合）。

		Map<Line, Line> lineLink = new HashMap<Line, Line>();

		Graphics graphics = image.getGraphics();
		graphics.setColor(Color.GREEN);
		for (int j = 1; j < lineGridArray.length - 1; j++) {
			for (int k = 1; k < lineGridArray[0].length - 1; k++) {
				if (lineGridArray[j][k] == null) {
					continue;
				}

				// 相邻关系的线条合并，将线条放进 50*50的网格里，然后在周围网格里搜索。
				List<Line> lineList = new ArrayList<Line>();
				lineList.addAll(lineGridArray[j][k].getLineList());
				if (lineGridArray[j + 1][k] != null) {
					lineList.addAll(lineGridArray[j + 1][k].getLineList());
				}
				if (lineGridArray[j + 1][k + 1] != null) {
					lineList.addAll(lineGridArray[j + 1][k + 1].getLineList());
				}
				if (lineGridArray[j - 1][k - 1] != null) {
					lineList.addAll(lineGridArray[j - 1][k - 1].getLineList());
				}
				if (lineGridArray[j][k + 1] != null) {
					lineList.addAll(lineGridArray[j][k + 1].getLineList());
				}
				if (lineGridArray[j - 1][k] != null) {
					lineList.addAll(lineGridArray[j - 1][k].getLineList());
				}
				if (lineGridArray[j][k - 1] != null) {
					lineList.addAll(lineGridArray[j][k - 1].getLineList());
				}

				// 用每个线条与周围线条比较。
				for (int m = 0; m < lineList.size() - 1; m++) {
					Line lineA = lineList.get(m);
					for (int n = m + 1; n < lineList.size() - 1; n++) {
						Line lineB = lineList.get(n + 1);
						// 寻找周围相同方向的线条。

						// 角度相近，并且两条线条的x1y1组成的线条的角度也相近。

						// 每个线条只会属于一条线。
						// TODO 需要用到前馈神经网络。

						// 每一种组合是一种可能，看哪种可能更优。

						// TODO 当多个线条方向相同，并线的权重越高，

						// TODO 只连接一次？最佳连接。怎么避免周围线条干扰？前馈网络算法。
						if (Math.abs(lineA.getOrientation() - lineB.getOrientation()) > 2) {
							continue;
						}

						double degree = Math
								.abs(Math.atan2(lineA.getY1() - lineB.getY1(), lineA.getX1() - lineB.getX1()) * 180
										/ Math.PI);
						System.out.println("lineA: " + lineA.getOrientation() + ", lineB: " + lineB.getOrientation()
								+ ", merge degree: " + degree);

						if (Math.abs(lineA.getOrientation() - degree) < 2) {// 相差5度，认为在一条线上。
							System.out.println("lineA: " + lineA.getOrientation() + ", lineB: " + lineB.getOrientation()
									+ ", merge");

							// 在两条线上做标记
							graphics.drawLine(lineA.getX1(), lineA.getY1(), lineB.getX2(), lineB.getY2());

							// 记录一次 A线条与B线条在相邻的一条线上。
							// 将可能在一条直线的线条，放进Map
							if (!lineLink.containsKey(lineA)) {
								lineLink.put(lineA, lineB);
							} else if (!lineLink.containsKey(lineB)) {
								lineLink.put(lineB, lineA);
							}

							// TODO ，一定存在一个线与几个线都可能有关系。
						}
					}

				}
			}
		}

		// TODO 找到最佳连接方式。
		// 如果两个线条连起来，权重是1。则三个线条连起来，则权重是2。四个线条连起来，则权重是3。

		for (int j = 1; j < lineGridArray.length - 1; j++) {
			for (int k = 1; k < lineGridArray[0].length - 1; k++) {
				if (lineGridArray[j][k] == null) {
					continue;
				}

				List<Line> lineList = new ArrayList<Line>();
				for (Line lineA : lineGridArray[j][k].getLineList()) {
					Line lineB = lineLink.get(lineA);
					lineExtends(lineLink, lineList, lineA, lineB);

					System.out.println(lineList.size());
				}

			}
		}
	}

	private static void lineExtends(Map<Line, Line> lineLink, List<Line> lineList, Line lineA, Line lineB) {
		// A 与 B 关联的 C 也在一条线上。
		
		// TODO 有限递归？
		
		// 向一个方向递归，否则会栈溢出
		
		
		// TODO 这里应该用拟合？最大拟合？
		
		
		// 方向相同，
		
		if (lineB != null && lineLink.containsKey(lineB)
				&& Math.abs(lineLink.get(lineB).getOrientation() - lineA.getOrientation()) < 2) {
			lineList.add(lineLink.get(lineB));
			lineExtends(lineLink, lineList, lineA, lineLink.get(lineLink.get(lineB)));
		}
	}

	public static BufferedImage show(BufferedImage image) {
		BufferedImage result = new BufferedImage(image.getWidth(), image.getHeight(), image.getType());
		int[][] grayImage = VisualEdge.brightnessReceptiveField(image);// 灰度处理，边缘增强，返回二值化二维数组，存储亮度0~255
		LineGrid[][] lineGridArray = VisualEdge.edgeReceptiveField(result, grayImage);// 在V1或V2，线条感受野，当两个有交集时，可以合并。

		complexCellReceptiveField(result, lineGridArray);
		return result;
	}

	public static void main(String[] args) throws IOException {
		BufferedImage bufferedImage = ImageIO.read(new File("E:/IMG/2040.jpg"));
		long t0 = System.currentTimeMillis();
		BufferedImage result = show(bufferedImage);

		long t1 = System.currentTimeMillis();
		System.out.println("Visual Form, cost: " + (t1 - t0) + "ms.");
		FileOutputStream output = new FileOutputStream(new File("E:/IMG/2040-edge.jpg"));

		ImageIO.write(result, "jpg", output);
		output.flush();
		output.close();

		// TODO 将亮暗边界的视野扩大，扩大到整个视野，将小视野的明暗边界连接为更大视野的明暗边界。

		// TODO V1到V2 明暗边界到物体形状，是拼凑的吗？
		// 非经典感受野的意义？

	}

	// 距离形状生成，还差多少？还差10天。距离三维形状生成还差20天。
	// 距离训练、成功识别物体，还差半年以上。到能够应用，需要1年以上。

	
	// TODO 用 DBSCAN（Density-Based Spatial Clustering ofApplications with Noise）聚类算法，将近似相同方向，在一条线的线段合并起来。

	// 霍夫变换 lines = cv2.HoughLinesP(edges,1,np.pi/180,160,minLineLength=100,maxLineGap=10)
}
