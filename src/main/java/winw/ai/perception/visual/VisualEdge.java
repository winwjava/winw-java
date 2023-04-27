package winw.ai.perception.visual;

import java.awt.image.BufferedImage;

/**
 * 像视网膜一样，每9个像素（中间一个点、8个方向围绕8个点）组成一个边缘检测方格，像素可以重复使用。
 * 
 * <p>
 * 第二层，根据第一层的计算结果，再做合并。
 * <p>
 * 经过5层就可以实现边缘形状的检测。
 * 
 * <p>
 * 深度网络
 * 
 * <p>
 * 神经节细胞 感受野，同心圆。
 * 
 * 人的视网膜上共有600万-700万视锥细胞（3种，篮绿红三种颜色）、1.1-1.3亿个视杆细胞（明暗），传出神经元——神经节细胞的数目大约只有感受细胞的百分之一。数十个-数百个感受细胞通过双极细胞以及水平细胞和无长突细胞与一个神经节细胞相联系。
 * 
 * 
 * <p>
 * 知觉恒常性 人能在一定范围内不随知觉条件的改变而保持对客观事物相对稳定特性的组织加工，这是人们知觉客观事物的一个重要特性。
 * 
 * 1.
 * 大小恒常性。影响因素有：①刺激条件。条件越复杂，则越表现出常性，当刺激条件减少，则常性现象减少；②距离因素。距离很远时，常性消失；③水平观察时，常性表现大。垂直观察时，常性表现小。此外，在用人工瞳孔时，大小恒常性消失。参考庞佐错觉、缪勒-莱尔错觉、相对大小错觉。
 * 
 * 2. 形状恒常性。
 * 
 * 3. 颜色恒常性。
 * 
 * 4. 距离恒常性。
 * 
 * 5. 明度恒常性。
 * 
 * <p>
 * 视觉（视网膜）图像的稳定性 庭视觉反射作用（Vestibulo-Ocular
 * Reflex，VOR）是眼球运行系统的一部分，其作用是让眼球向与头转动相反的方向运动，以维持视觉（视网膜）图像的稳定性。VOR由前庭核酸的前端神经元调节，前端神经元从前庭感知神经元中接受头部旋转信息并处理，将结果告知眼球肌肉的运动神经元。VOR的递归网络模型能重现和解释调节VOR的神经元的处理信息时的静态、动态、非线性和分布性许多方面的特性。
 * 
 * @author winw
 *
 */
public class VisualEdge {

	/**
	 * 表面亮度
	 * <p>
	 * 在V1输入层（L4C）中，神经元对表面亮度的反应很强，尽管弱于边界反应，但其编码表面亮度信息的能力是强于边界反应的。
	 * 
	 * <p>
	 * 综上，这项研究完整刻画了物体表面亮度信息在V1的编码形式和潜在的神经机制：层间的前馈连接同时驱动了表面反应和边界反应，前馈信号的局部整合提高了边界反应对亮度信息的编码能力。视皮层内的抑制主要抑制了表面反应，并将更多的亮度信息分配给了边界反应。这两个皮层过程结合在一起，将表面亮度信息整合到边界反应中，从而实现基于边界反应的效率编码。该文揭示的亮度信息处理方式，与邢大军课题组在2020年揭示的有关朝向信息（客体形状纹理相关）的皮层处理方式高度一致10。
	 * 
	 * http://keyanyuan.bnu.edu.cn/yxfc/243329.html
	 */
	public void surfaceBrightness() {

	}

	/**
	 * 视网膜感受野，分为：边缘、运动方向、明暗变化
	 * 
	 * 
	 * 模拟视网膜同心圆感受野，根据光线明暗对比度，分析出边界。
	 * <p>
	 * 同心圆结构模型：中心区域若干像素（7个像素），周围相邻圆环若干像素（）。
	 * <p>
	 * 根据中间一个像素点和周围8个，或者
	 * 
	 * <p>
	 * 70%的视网膜神经节细胞具有明显的方位选择性
	 * 
	 * <P>
	 * 加拿大神经生理学家David Hunter Hubel和瑞典神经科学家Torsten Nils
	 * Wiesel在20世纪50年代和60年代开始研究视觉机制，他们将图像投射到屏幕上，将测量神经元活动的电线插入猫的大脑，通过固定猫的头部来控制视网膜上的成像，测试生物细胞对线条、直角、边缘线等图形的反应。
	 * 
	 * 研究显示有些细胞对某些处在一个角度上的线条、垂直线条、直角或者明显的边缘线，都有特别的反应，这就是绝大多数视皮层细胞都具有的强烈的方位选择性。不仅如此，要引起这个细胞反应，直线的朝向还只能落在一个很小的角度范围里，也就是该细胞的感受野内。这样以上两个研究就统一起来了。
	 * 
	 * 从1960年到1980年，两人合作了20多年，细致科学地研究了人眼视觉的机制，因此他们被认为是现代视觉科学之父，并于1981年一起获得了诺贝尔生理学与医学奖。
	 *
	 * <p>
	 * https://www.cnblogs.com/koala999/articles/16989285.html
	 * 
	 * https://daily.zhihu.com/story/9314332?yidian_docid=0FwfRRDb&utm_id=0
	 * 
	 */
	public void retinaReceptiveField() {

	}

	/**
	 * 视皮层感受野
	 * <p>
	 * 视皮层神经元的感受野对应视网膜上的一个更大的区域，因为它是由若干个外膝体细胞的感受野共同会聚到一个视皮层细胞的感受野上的（图3）。他们宣称，简单细胞的感受野再汇聚成复杂细胞的感受野，后者再进一步汇聚成超复杂细胞的感受野。
	 *
	 * <p>
	 * 沿着Hubel and
	 * Wiesel指引的方向，有人将视觉场景的加工进一步细分为三个层次——低水平加工是简单特征（如方位、颜色、对比度、视差、运动方向等）的分析，中等水平的加工用低水平特征解析视觉场景，包括轮廓整合、表面特征、形状区分、对象运动等，高水平加工就是运用表面和轮廓来辨识对象（Kandel
	 * et al. 2013）。
	 *
	 * https://blog.csdn.net/hjwang1/article/details/80627766
	 */
	public void cortexReceptiveField(BufferedImage image) {
		// 感受野同心圆，检测出单根线条、垂直线条、直角。

//		Graph graph = new Graph();
//		BufferedImage edgeImage = new BufferedImage(image.getWidth(), image.getHeight(), BufferedImage.TYPE_INT_BGR);
//
//		for (int i = image.getMinX(); i < image.getWidth(); i++) {
//			for (int j = image.getMinY(); j < image.getHeight(); j++) {
//				// 循环X和Y坐标，逐个像素比较。
//
//				if (i == image.getMinX() || j == image.getMinY() || i == image.getWidth() - 1
//						|| j == image.getHeight() - 1) {
//					continue;// 四条边的像素点不用比较。
//				}
//
//				// 同心圆的 中心区域
//
//				// 同心圆的 圆环区域
//
//				int[] rgb = rgbValue(image.getRGB(i, j));
//				int[] rgbR = rgbValue(image.getRGB(i + 1, j));
//				int[] rgbL = rgbValue(image.getRGB(i - 1, j));
//				int[] rgbT = rgbValue(image.getRGB(i, j + 1));
//				int[] rgbD = rgbValue(image.getRGB(i, j - 1));
//
//				// 明暗对比
//				if (rgbSimilar(rgb, rgbR) || rgbSimilar(rgb, rgbL) || rgbSimilar(rgb, rgbT) || rgbSimilar(rgb, rgbD)) {
//
////					graph.addNode(i, j);
//
//					edgeImage.setRGB(i, j, 6522);
//				} else {
//					edgeImage.setRGB(i, j, image.getRGB(i, j));
//				}
//			}
//		}
//		return edgeImage;

	}

	/**
	 * 视觉皮层功能柱
	 * 
	 * <p>
	 * Hubel和Wiesel根据对视觉刺激的反应特征，在视皮层发现了多种神经元，分别叫做简单细胞、复杂细胞以及超复杂细胞。他们后续的研究以及后来大量科研工作者的实验对这些不同细胞的功能进行了深入的探索。他们的另外一项重要发现则是在视皮层中证实了之前由Vernon
	 * Mountcastle(1918--2015)根据其在躯体感觉皮层的研究提出的皮层功能柱的结构。他们的发现可以简单描述为许多具有相同特性的皮层细胞，在视皮层内按照一定的规则在空间上排列起来，这种按功能排列的皮层结构，即皮层的功能构筑，沿着皮层的不同层次呈现柱状分布，例如方向柱、方位柱、眼优势柱、空间频率柱以及颜色柱等。这一结构的形成对于皮层内感觉信息的处理具有重要的影响。
	 * 
	 * <p>
	 * 有4种类型视皮层神经元。
	 * 
	 * 1．简单细胞。感受野面积较小，给光区和撤光区分离，有较明显的空间总合，反应具有线性特征，没有或很少有自发放电。具有特定方向和在视野中有固定位置的刺激，最能激发简单细胞。
	 * 
	 * 》》简单型：感受野呈狭长型，分给光区和撤光区，最佳刺激是线条。具有最佳朝向的线条（与给光区朝向相同）能诱发最强反应，反之则抑制。
	 * 
	 * 2．复杂细胞。给光区和撤光区重叠，反应具有非线性特征，空间总合不明显，自发放电强。相比简单细胞：①反应要求一定方位的线性刺激，但不管在视野中的部位如何；②当光线移过视野时，能继续激发对适当方位的线性刺激。由此，复杂细胞对于适当方位的移动的直线刺激能继续激发，可以认为它们接受大量的简单细胞输入的刺激。
	 * 》》复杂型：感受野较大，无明显光区、线条方向检测，只对朝向敏感，对位置不敏感。
	 * 
	 * 3．超复杂细胞。能从几种复杂细胞中接受兴奋性和抑制性的输入信息。反应特点同复杂细胞，也反应特殊方位的线性刺激，但有明显的端点抑制，这种刺激不能超过某种长度。
	 * 》》超复杂型：感受野的反应特征与复杂型相似，但有明显的终端抑制，即长方形的长度超过一定的限度则有抑制效应（检测端点）。
	 * 4．极高度复杂细胞。反应移过视野的边，只要是这边有一特定的宽度。有些极高度复杂细胞特别反应两个边形成的直角，这种细胞也称为角探测器。
	 * 
	 * 视皮层神经元对视觉刺激的各种静态和动态特征都具有高度选择性。一是方位/方向选择性。只有当刺激线条或边缘处在适宜的方位角并按一定的方向移动时，才表现出最大兴奋（最佳方位或最佳方向）。二是空间频率选择性。每一个视皮层细胞都有一定的空间频率调谐。在同一皮层区内，不同细胞也有不同的空间频率调谐。三是速度选择性。对移动图形的反应比对静止的闪烁图形要强得多。而且对某一最佳速度的反应最大，移动速度高于或低于这一速度时，反应都会减小。四是双眼视差选择性。与外膝体细胞不同，大部分视皮层细胞接受双眼输入，在左、右视网膜上分别有一个感受野。这一对感受野在视网膜上的位置差（相对于注视点）称为“视差”（disparity）。根据视差可判断该细胞的调谐距离，从而形成深度视觉。五是颜色选择性。与外膝体细胞一样，皮层细胞也具有颜色选择性。与皮层下的单颉顽式感受野不同，视皮层的颜色感受野具有双颉顽式结构。例如对于R-G型感受野，其颜色结构可能有两种形式：感受野中心可能被绿视锥细胞的输入兴奋，同时被红视锥细胞输入抑制，或者相反；外周对颜色的反应性质正好与中心相反。因此，该细胞通过感受野中心的颜色颉顽能分辨红色和绿色，通过中心与外周的相互作用能使红—绿对比的边缘得到增强。
	 *
	 * <p>
	 * 在大脑视觉皮层中，具有相同视功能特性（相同图像特征选择性和相同感受野）的皮层细胞，以垂直于大脑表面的方式排列称柱状结构，被称为视皮层功能柱。大体有两种功能柱理论，即特征提取功能柱和空间频率功能柱。特征提取功能柱包括：方位柱、眼优势柱、颜色柱等。
	 * 
	 * <p>
	 * 研究表明，功能柱系统正好与各种特征检测功能一一对应。所有功能柱都垂直于皮层表面，排列成片层状。
	 * 
	 * 1. 方位柱。位于17区和18区。细胞的敏感方位总是很有规律地按顺时针或逆时针方向变化。
	 * 
	 * 2. 眼优势柱。左眼优势细胞与右眼优势细胞通过一定的间隔交替出现。
	 * 
	 * 3. 空间频率柱。皮层细胞的最佳空间频率也是有规则地以柱的形式垂直于皮层表层排列。试验证明，猫皮层17区存在该结构。
	 * 
	 * 4. 颜色柱。试验发现，有颜色特异性的细胞和没有颜色特异性的细胞成串交替出现。同一柱内所有的细胞具有相同的光谱特性。
	 * 
	 * 猕猴大脑皮层由约109个神经元组成，这些神经元构成了约105个功能柱结构，每个功能柱结构中包含了约104个神经元。先前的解剖学证据表明，某一个功能柱内的神经元（这些神经元具有相似的功能特异性，例如偏好某一颜色、朝向、运动方向、深度信息等）倾向于与其他具有相同功能特性的功能柱产生连接。
	 * 
	 * <p>
	 * http://www.ziint.zju.edu.cn/index.php/event/details.html?tid=415
	 */
	public void functionColumn() {

		// 功能柱，是卷积？
	}

	/**
	 * 纹理
	 * <p>
	 * 4．
	 * 寿天德等人还发现，经典感受野中心以外直径9度范围内的所谓大外周抑制区也具有独立的对光栅刺激的方位选择性反应。这一方位选择性的平均强度与经典感受野中心相近。大外周与中心的方位选择性可能不同，对整个大范围的复合感受野的方位选择性贡献也不同。在复合感受野方位选择性的形成中具有中心决定、大外周决定和中心与大外周共同决定等模式。并且大外周刺激的存在将调制中心的方位选择性，反之亦然。这使得神经节细胞可能完成更复杂的感知功能，如纹理质地的感知等。
	 * 
	 * <p>
	 * 
	 * 提取图像特征点是图像处理的基础。经典感受野外大外周的发现，有可能改变提取图像特征点的数学方法。我们寄希望于它能够由此推动改变整个图像识别的数学方法。如果真是这样，我们就可以欢欣鼓舞了。
	 * 
	 */
	public void texture() {

	}

	/**
	 * 双眼视差，外侧膝状体，
	 * 
	 * 四是双眼视差选择性。与外膝体细胞不同，大部分视皮层细胞接受双眼输入，在左、右视网膜上分别有一个感受野。这一对感受野在视网膜上的位置差（相对于注视点）称为“视差”（disparity）。根据视差可判断该细胞的调谐距离，从而形成深度视觉。
	 *
	 * 
	 * 灵长类的外膝体由6层细胞构成。两眼的输入分别投射到3个不同层次。从同侧眼来的纤维终止于5、3、2层，从对侧眼来的纤维终止于6、4、1层。这些相互重叠的每一个层次都与视网膜有点对点的投射关系。这种对应关系有利于建立双眼间的相互联系，为在视皮层形成双眼视差（深度和立体感）调谐奠定了基础。
	 */
	public void disparity() {

	}
}
