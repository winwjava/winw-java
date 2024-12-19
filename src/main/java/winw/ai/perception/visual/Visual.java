package winw.ai.perception.visual;

/**
 * 视觉
 */

public class Visual {

	/**
	 * 视网膜：光电转换和初步处理
	 * 
	 * <p>
	 * 视网膜是眼球的最内层，它包含了大量的感光细胞和神经元，负责将光线转换为电信号，并进行初步的处理。感光细胞有两种类型：视锥细胞和视杆细胞。视锥细胞主要分布在视网膜中央部位，对亮度和颜色敏感，负责白天的清晰视觉；视杆细胞主要分布在视网膜周边部位，对亮度敏感，负责黑暗中的模糊视觉。感光细胞通过化学反应将光能转化为电能，并将电信号传递给神经元。
	 * <p>
	 * 神经元有多种类型，其中最重要的是双极细胞、水平细胞、星形细胞、杆-锥细胞和神经节细胞。双极细胞接收感光细胞的信号，并传递给星形细胞和神经节细胞；水平细胞在感光细胞和双极细胞之间进行水平连接，调节信号的强度；星形细胞在双极细胞和神经节细胞之间进行垂直连接，增强信号的对比度；杆-锥细胞在视杆细胞和视锥细胞之间进行连接，实现两种类型感光细胞的互补；神经节细胞是视网膜上最外层的神经元，它们将信号汇集并通过视神经传递给大脑。
	 * <p>
	 * 在这个过程中，视网膜已经开始对视觉信息进行初步的处理，例如提取边缘、方向、颜色、亮度等特征，并对信号进行放大、抑制、增强等调节。这些处理使得视网膜能够适应不同的光照条件，并突出重要的信息。
	 */
	public void retina() {
		// 视杆细胞主要分布在视网膜周边部位，对亮度敏感，负责黑暗中的模糊视觉
	}

	/**
	 * LGN：中继站和过滤器
	 * <p>
	 * LGN（外侧膝状体）是位于丘脑中的一个核团，它是视觉信息从眼睛到大脑皮层的中间站。LGN有六层，每层都接收来自一只眼睛的信号，并按照不同的特征进行分类。其中第1-2层为大细胞层（M层），主要接收来自视杆细胞的信号，负责运动、深度和低空间频率（即粗略图像）信息的处理；第3-6层为小细胞层（P层），主要接收来自视锥细胞的信号，负责形状、颜色和高空间频率（即清晰图像）信息的处理；此外还有一些位于各层之间的粒状细胞层（K层），主要接收来自非M非P型神经节细胞（即双层树突神经节细胞）的信号，负责红绿对立和蓝黄对立等颜色信息的处理。
	 * <p>
	 * LGN在传递视觉信息时，不仅起到了中继站的作用，还起到了过滤器的作用。LGN受到来自大脑皮层、丘脑其他部分以及脑干等多个来源的反馈控制，这些反馈可以调节LGN对信号的选择性和敏感性，使得LGN能够根据当前的任务、注意力和情境等因素，有选择地放大或抑制某些特征或区域的信号，并抑制噪声或冗余信息。这些过滤使得LGN能够优化视觉信息，并为后续的处理做好准备。
	 */
	public void lgn() {
		// 需要注意：低空间频率（即粗略图像）、高空间频率（即清晰图像）
	}

	/**
	 * V1：初级视皮层
	 * <p>
	 * V1（初级视皮层）是位于枕叶上的一块区域，它是大脑皮层中最早接收到视觉信息的部分。V1有六层，其中第四层又分为四个亚层（IVa, IVb, IVcα,
	 * IVcβ），每个亚层都接收来自LGN不同层次的投射。V1中最重要的功能单元是简单细胞和复杂细胞。简单细胞位于第三层和第四层，它们对特定位置和方向上具有一定宽度和长度的刺激敏感，并具有明显的兴奋区和抑制区；复杂细胞位于第二层和第五层，它们对特定方向上具有一定速度和长度的刺激敏感，但不受刺激位置的影响。简单细胞和复杂细胞都能够对边缘、方向、运动等特征进行检测，并形成了一种类似于拼图的视觉表征。
	 * 
	 * <p>
	 * V1在处理视觉信息时，不仅起到了初级的特征提取的作用，还起到了一些高级的功能，例如双眼立体视、颜色恒常性和图形连续性等。双眼立体视是指通过比较两只眼睛接收到的不同视角的图像，计算出物体距离观察者的深度信息；颜色恒常性是指通过调节对不同光照条件下的颜色反射率的感知，使得物体的颜色在不同环境中保持一致；图形连续性是指通过填补图像中的空缺或遮挡，使得物体的形状在不完整或不清晰时仍然能够被识别。
	 */
	public void v1() {

	}

	/**
	 * V2：次级视皮层
	 * 
	 * <p>
	 * V2（次级视皮层）是位于V1周围的一块区域，它是大脑皮层中第二个接收到视觉信息的部分。V2也有六层，其中第四层又分为四个亚层（IVa, IVb,IVcα,
	 * IVcβ），每个亚层都接收来自V1相应亚层的投射。V2中最重要的功能单元是超复杂细胞。超复杂细胞位于各层，它们对特定方向上具有一定长度和宽度的刺激敏感，但对刺激位置和速度不敏感，并且具有端点抑制或侧抑制等特性。超复杂细胞能够对角度、长度、宽度、形状等特征进行检测，并形成了一种类似于轮廓的视觉表征。
	 * 
	 * <p>
	 * V2在处理视觉信息时，不仅起到了次级的特征提取的作用，还起到了一些高级的功能，例如双眼立体视、颜色恒常性和图形连续性等。双眼立体视是指通过比较两只眼睛接收到的不同视角的图像，计算出物体距离观察者的深度信息；颜色恒常性是指通过调节对不同光照条件下的颜色反射率的感知，使得物体的颜色在不同环境中保持一致；图形连续性是指通过填补图像中的空缺或遮挡，使得物体的形状在不完整或不清晰时仍然能够被识别。
	 */
	public void v2() {

	}

	/**
	 * V3高级视皮层
	 * <p>
	 * V3-V5（高级视皮层）是位于V2后方或侧方的一些区域。V3主要负责对形状和大小进行编码；
	 */
	public void v3() {

	}

	/**
	 * V4高级视皮层（猕猴视觉皮层V4中表征中等复杂度形状特征的功能区）
	 * <p>
	 * V3-V5（高级视皮层）是位于V2后方或侧方的一些区域。V4主要负责对颜色和纹理进行编码；
	 * 
	 * <p>
	 * 课题组在前期工作中测量了V4神经元对于大量随机自然图片的反应，通过降维和聚类分析，发现存在大量对圆弧和对角特异性响应的神经元，而且这些神经元在空间上聚集。本研究中，课题组直接使用含有圆弧和角的几何图形刺激集，用双光子成像扫描猕猴V4的一大片区域，发现一些区域可被圆弧或者角特异性激活。课题组直接测量了这些区域中的神经元，发现其中的大部分神经元，都对圆弧或角选择性反应，而且这种选择性不能单纯用局部朝向或者朝向不连续解释。课题组测量了神经元对介于圆弧和角之间的折线形的反应，发现这些神经元不仅仅通过局部朝向的简单加和来编码和区分圆弧与角，而是对曲线整体的平滑性敏感。这说明复杂特征的视觉编码不能用简单的前馈加和来解释，特征功能区的形成也应依赖复杂的水平连接。
	 * 
	 * <p>
	 * 总而言之，课题组在猕猴中间视皮层V4中发现了编码圆弧和角的功能区，为视觉形状特征的处理过程补充了一块剩余的拼图，即中间复杂度形状特征的表征。这一发现帮助完善了课题组对于由简单到复杂的视觉信息整合过程的认知。同时，由于自然环境中动物往往含有较多的平滑弧线，而静物如石头、树枝有更多的棱角，圆弧区和角区的明显分离可能有助于动物对自然物体的分类处理和响应。
	 * 
	 * 
	 * <p>
	 * 颜色知觉，全色盲是由V4区受损引起的，对所有颜色均无法感知。V4还是参与形状知觉的次级视觉区，颜色可提供物体形状的重要信息。患者无法识别
	 * 渐变颜色的细微差别，从而丧失了对莫奈作品中人物的形状知觉。
	 * 
	 * <p>
	 * 参考： https://www.research.pku.edu.cn/bdkyjz/1345976.htm
	 */
	public void v4() {

	}

	/**
	 * 面部识别
	 */
	public void it() {

	}

	public void pit() {

	}

	public void cit() {

	}

	public void ait() {

	}

	/**
	 * 
	 * MT/V5 高级视皮层
	 * <p>
	 * V3-V5（高级视皮层）是位于V2后方或侧方的一些区域。V5（或称MT）主要负责对运动和方向进行编码。
	 * <p>
	 * 运动知觉，朝特定方向运动时，兴奋性最强，兴奋性与运 动速度有关。单侧半球MT区损伤对运动知觉影响较小，双侧半球损伤会导致运 动知觉丧失。
	 * 
	 */
	public void mt() {

	}

	public void dp() {

	}

	public void mip() {

	}

	public void po() {

	}

	public void pip() {

	}

	public void lip() {

	}

	public void mst() {

	}

	public void vip() {

	}

	public void _7a() {

	}

	/**
	 * 下颞叶皮层（IT）中除面孔脑区
	 * 
	 * 面孔识别是指通过分析面部特征和表情，识别出人物的身份、情绪和意图；
	 */
	public void face() {

	}

	/**
	 * 场景理解是指通过分析场景中的物体、空间和关系，理解出场景的含义、目标和动机；
	 */
	public void understand() {

	}

	/**
	 * 注意力控制是指通过选择性地关注某些特征或区域，提高对重要信息的处理效率，并抑制无关信息。
	 */
	public void attention() {

	}
	

}