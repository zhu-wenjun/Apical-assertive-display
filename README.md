# Apical-assertive-display

```
diff --git a/arch/arm/boot/dts/nufront-tl7790-phone-test.dts b/arch/arm/boot/dts/nufront-tl7790-phone-test.dts
index 9bc8c09..ba7bce7 100644
--- a/arch/arm/boot/dts/nufront-tl7790-phone-test.dts
+++ b/arch/arm/boot/dts/nufront-tl7790-phone-test.dts
@@ -43,7 +43,7 @@
 		};
 
 		VGA: VGA {
-			panel = "1280x800@60";
+			panel = "1920x1080@60";
 		};
 
 		Wireless: nufront-wifi {
diff --git a/arch/arm/boot/dts/nufront-tl7790.dtsi b/arch/arm/boot/dts/nufront-tl7790.dtsi
index 080059e..b3ab1f9 100644
--- a/arch/arm/boot/dts/nufront-tl7790.dtsi
+++ b/arch/arm/boot/dts/nufront-tl7790.dtsi
@@ -391,6 +391,12 @@
 			clocks = <&clks TL7790_PWM_CLK>, <&clks TL7790_PWM_PCLK>;
 			clock-names = "pwm_clk", "pwm_pclk";
 		};
+
+		ad: ad@05010000 {
+			compatible = "nufront, assertive display";
+			reg = <0x05010000 0x1000>;
+		};
+
 		g3d@05030000 {
 			compatible = "arm,mali-400", "arm,mali-utgard";
 			reg = <0x05030000 0x10000>;
diff --git a/drivers/clk/nufront/clk-tl7790.c b/drivers/clk/nufront/clk-tl7790.c
index 8bf652a..7d31ca8 100644
--- a/drivers/clk/nufront/clk-tl7790.c
+++ b/drivers/clk/nufront/clk-tl7790.c
@@ -436,9 +436,9 @@ static void __init tl7790_clocks_init(struct device_node *prcm_node)
 	clk_prepare_enable(clk[TL7790_DDR_PHY_PCLK]);
 
 	//clk_set_parent(clk[TL7790_CPU_CLK], clk[TL7790_PLL_DDR_FIXED]);
-	//clk_set_rate(clk[TL7790_PLL0], 1200000000);
+	clk_set_rate(clk[TL7790_PLL0], 1200000000);
 	//clk_set_rate(clk[TL7790_CPU_DIV0], 600000000);
-	//clk_set_parent(clk[TL7790_CPU_CLK], clk[TL7790_CPU_DIV0]);
+	clk_set_parent(clk[TL7790_CPU_CLK], clk[TL7790_PLL0_FIXED]);
 
 	clk_prepare_enable(clk[TL7790_CPU_CLK]);


diff --git a/arch/arm/boot/dts/nufront-tl7790.dtsi b/arch/arm/boot/dts/nufront-tl7790.dtsi
index 02ea2aa..bcac730 100644
--- a/arch/arm/boot/dts/nufront-tl7790.dtsi
+++ b/arch/arm/boot/dts/nufront-tl7790.dtsi
@@ -362,6 +362,13 @@
 			clock-names = "isp_m_hclk" , "isp_s_hclk" , "isp_clk" , "isp_jpeg_clk";
 		};
 
+		ad: apical_ad@05010000 {
+			compatible = "nufront, assertive display";
+			reg = <0x05010000 0x1000>;
+			ab_present = <1>;
+			bl_level = <30>;
+		};
+
 		lcd: nusmartfb@05140000 {
 			compatible = "nufront,nusmartfb";
 			id = <0>;
diff --git a/drivers/video/nusmart/Makefile b/drivers/video/nusmart/Makefile
index 8adc024..e5bd097 100644
--- a/drivers/video/nusmart/Makefile
+++ b/drivers/video/nusmart/Makefile
@@ -1 +1,2 @@
+#obj-$(CONFIG_NU7TL_DISPLAY)		+= n7tl-lcdc.o nu7tl_dsi.o tl7790_ad.o panel/
 obj-$(CONFIG_NU7TL_DISPLAY)		+= n7tl-lcdc.o nu7tl_dsi.o panel/
```
