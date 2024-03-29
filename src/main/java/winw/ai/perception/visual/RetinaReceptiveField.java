package winw.ai.perception.visual;

import java.awt.image.BufferedImage;

/**
 * 像视网膜一样，每9个像素（中间一个点、8个方向围绕8个点）组成一个边缘检测方格，像素可以重复使用。
 * 
 * <Point>
 * 第二层，根据第一层的计算结果，再做合并。
 * <Point>
 * 经过5层就可以实现边缘形状的检测。
 * 
 * <Point>
 * 深度网络
 * 
 * <Point>
 * 神经节细胞 感受野，同心圆。
 * 
 * 人的视网膜上共有600万-700万视锥细胞（3种，篮绿红三种颜色）、1.1-1.3亿个视杆细胞（明暗），传出神经元——神经节细胞的数目大约只有感受细胞的百分之一。数十个-数百个感受细胞通过双极细胞以及水平细胞和无长突细胞与一个神经节细胞相联系。
 * 
 * 
 * <Point>
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
 * <Point>
 * 视觉（视网膜）图像的稳定性 庭视觉反射作用（Vestibulo-Ocular
 * Reflex，VOR）是眼球运行系统的一部分，其作用是让眼球向与头转动相反的方向运动，以维持视觉（视网膜）图像的稳定性。VOR由前庭核酸的前端神经元调节，前端神经元从前庭感知神经元中接受头部旋转信息并处理，将结果告知眼球肌肉的运动神经元。VOR的递归网络模型能重现和解释调节VOR的神经元的处理信息时的静态、动态、非线性和分布性许多方面的特性。
 * 
 * @author winw
 *
 */
public class RetinaReceptiveField {

	/**
	 * 表面亮度
	 * <Point>
	 * 在V1输入层（L4C）中，神经元对表面亮度的反应很强，尽管弱于边界反应，但其编码表面亮度信息的能力是强于边界反应的。
	 * 
	 * <Point>
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
	 * <Point>
	 * 同心圆结构模型：中心区域若干像素（7个像素），周围相邻圆环若干像素（）。
	 * <Point>
	 * 根据中间一个像素点和周围8个，或者
	 * 
	 * <Point>
	 * 70%的视网膜神经节细胞具有明显的方位选择性
	 * 
	 * <Point>
	 * 加拿大神经生理学家David Hunter Hubel和瑞典神经科学家Torsten Nils
	 * Wiesel在20世纪50年代和60年代开始研究视觉机制，他们将图像投射到屏幕上，将测量神经元活动的电线插入猫的大脑，通过固定猫的头部来控制视网膜上的成像，测试生物细胞对线条、直角、边缘线等图形的反应。
	 * 
	 * 研究显示有些细胞对某些处在一个角度上的线条、垂直线条、直角或者明显的边缘线，都有特别的反应，这就是绝大多数视皮层细胞都具有的强烈的方位选择性。不仅如此，要引起这个细胞反应，直线的朝向还只能落在一个很小的角度范围里，也就是该细胞的感受野内。这样以上两个研究就统一起来了。
	 * 
	 * 从1960年到1980年，两人合作了20多年，细致科学地研究了人眼视觉的机制，因此他们被认为是现代视觉科学之父，并于1981年一起获得了诺贝尔生理学与医学奖。
	 *
	 * <Point>
	 * https://www.cnblogs.com/koala999/articles/16989285.html
	 * 
	 * https://daily.zhihu.com/story/9314332?yidian_docid=0FwfRRDb&utm_id=0
	 * 
	 */
	public void retinaReceptiveField() {

	}

	/**
	 * 视皮层感受野
	 * <Point>
	 * 视皮层神经元的感受野对应视网膜上的一个更大的区域，因为它是由若干个外膝体细胞的感受野共同会聚到一个视皮层细胞的感受野上的（图3）。他们宣称，简单细胞的感受野再汇聚成复杂细胞的感受野，后者再进一步汇聚成超复杂细胞的感受野。
	 *
	 * <Point>
	 * 沿着Hubel and
	 * Wiesel指引的方向，有人将视觉场景的加工进一步细分为三个层次——低水平加工是简单特征（如方位、颜色、对比度、视差、运动方向等）的分析，中等水平的加工用低水平特征解析视觉场景，包括轮廓整合、表面特征、形状区分、对象运动等，高水平加工就是运用表面和轮廓来辨识对象（Kandel
	 * et al. 2013）。
	 *
	 * https://blog.csdn.net/hjwang1/article/details/80627766
	 */
	public void cortexReceptiveField(BufferedImage image) {

	}

	/**
	 * 纹理
	 * <Point>
	 * 4．
	 * 寿天德等人还发现，经典感受野中心以外直径9度范围内的所谓大外周抑制区也具有独立的对光栅刺激的方位选择性反应。这一方位选择性的平均强度与经典感受野中心相近。大外周与中心的方位选择性可能不同，对整个大范围的复合感受野的方位选择性贡献也不同。在复合感受野方位选择性的形成中具有中心决定、大外周决定和中心与大外周共同决定等模式。并且大外周刺激的存在将调制中心的方位选择性，反之亦然。这使得神经节细胞可能完成更复杂的感知功能，如纹理质地的感知等。
	 * 
	 * <Point>
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
