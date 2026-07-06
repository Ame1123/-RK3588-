TFTLCD 驱动使用文档
适用工程：STM32F10x（ZET6），驱动文件 tftlcd.c / tftlcd.h / font.h
一、硬件连接与配置
接口方式
LCD 通过 FSMC（Bank1 Sector4） 连接，内存映射地址为 0x6C000000，CPU 以读写内存的方式操作屏幕，
速度快且不占用 CPU 轮询时间。
信号 
STM32 引脚 
说明
数据总线 D0-D15 
PD0/1/8/9/10/14/15, PE7-15 
16位并行数据
片选 CS 
PG12 
FSMC NE4
读使能 RD 
PD4 
FSMC NOE
写使能 WR 
PD5 
FSMC NWE
命令/数据 RS 
PG0 
FSMC A10（地址线）
背光 LED 
PB0 
高电平亮
复位 RST 
— 
接系统复位或独立引脚
选择屏幕型号
在 tftlcd.h 开头，保留且只保留一个宏定义，其余注释掉：
// 根据你的屏幕型号取消对应注释，其余保持注释
#define TFTLCD_HX8357DN   // 默认，320×480 竖屏
//#define TFTLCD_ILI9341
//#define TFTLCD_ILI9486
//#define TFTLCD_NT35510
// ... 其他型号见 tftlcd.h
显示方向
#define TFTLCD_DIR  0   // 0=竖屏（portrait），1=横屏（landscape）
二、初始化
TFTLCD_Init
TFTLCD使用文档.md 
2026-06-01
1 / 11
void TFTLCD_Init(void);
必须在所有 LCD 操作之前调用一次。
完成 FSMC 时钟使能、GPIO 配置、FSMC 时序配置、屏幕控制器初始化序列、背光开启。
初始化完成后 tftlcd_data 结构体中的 width、height、id、dir 字段即可使用。
// 示例
SysTick_Init(72);
TFTLCD_Init();
// 之后才可以调用其他 LCD 函数
当前工程中的 UI 层已开启触摸输入与动态刷新：`Hardware/ui.c` 中 `UI_TOUCH_ENABLE` 和 `UI_DYNAMIC_REFRESH_ENABLE` 均为 1。
tftlcd_data 结构体
初始化后可读取屏幕基本信息：
extern _tftlcd_data tftlcd_data;
tftlcd_data.width;   // 屏幕宽度（像素）
tftlcd_data.height;  // 屏幕高度（像素）
tftlcd_data.id;      // LCD 控制器 ID（如 0x8357）
tftlcd_data.dir;     // 当前方向，0=竖屏，1=横屏
三、颜色
前景色与背景色全局变量
extern u16 FRONT_COLOR;  // 前景色（文字、点、线的颜色），默认黑色
extern u16 BACK_COLOR;   // 背景色（文字底色），默认白色
直接赋值即可改变后续绘制颜色：
FRONT_COLOR = RED;
BACK_COLOR  = WHITE;
预定义颜色常量（RGB565 格式）
WHITE    0xFFFF    BLACK    0x0000    RED      0xF800
GREEN    0x07E0    BLUE     0x001F    YELLOW   0xFFE0
TFTLCD使用文档.md 
2026-06-01
2 / 11
CYAN     0x7FFF    MAGENTA  0xF81F    GRAY     0x8430
DARKBLUE 0x01CF    BROWN    0xBC40    ORANGE   0xFD20
RGB565 自定义颜色公式：color = ((R & 0x1F) << 11) | ((G & 0x3F) << 5) | (B & 0x1F)
四、清屏与填充
LCD_Clear — 整屏填充单色
void LCD_Clear(u16 Color);
参数 
说明
Color 填充颜色，用预定义颜色常量或自定义 RGB565 值
LCD_Clear(WHITE);   // 清为白色
LCD_Clear(BLACK);   // 清为黑色
LCD_Fill — 矩形区域填充单色
void LCD_Fill(u16 xStart, u16 yStart, u16 xEnd, u16 yEnd, u16 color);
参数 
说明
xStart 左上⻆ X 坐标
yStart 左上⻆ Y 坐标
xEnd 
右下⻆ X 坐标（含）
yEnd 
右下⻆ Y 坐标（含）
color 填充颜色
// 绘制蓝色标题栏（全宽40像素高）
LCD_Fill(0, 0, tftlcd_data.width - 1, 39, DARKBLUE);
LCD_Color_Fill — 矩形区域填充颜色数组
TFTLCD使用文档.md 
2026-06-01
3 / 11
void LCD_Color_Fill(u16 sx, u16 sy, u16 ex, u16 ey, u16 *color);
将一个 u16 数组按像素顺序（逐行）填入指定矩形区域，适合显示小图标或自定义图形。
五、基本绘图
LCD_DrawPoint — 画单个像素点
void LCD_DrawPoint(u16 x, u16 y);
以当前 FRONT_COLOR 颜色在 (x, y) 位置画一个点。
FRONT_COLOR = RED;
LCD_DrawPoint(100, 100);
LCD_DrawFRONT_COLOR — 指定颜色画点
void LCD_DrawFRONT_COLOR(u16 x, u16 y, u16 color);
不改变全局 FRONT_COLOR，直接以指定颜色画一个点，性能比先赋值再 DrawPoint 更高。
LCD_DrawFRONT_COLOR(50, 50, BLUE);
LCD_ReadPoint — 读取像素颜色
u16 LCD_ReadPoint(u16 x, u16 y);
返回 (x, y) 位置当前的 RGB565 颜色值，超出屏幕范围返回 0。
LCD_DrawLine — 画直线
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2);
以当前 FRONT_COLOR 从 (x1,y1) 到 (x2,y2) 画一条直线（Bresenham 算法）。
TFTLCD使用文档.md 
2026-06-01
4 / 11
FRONT_COLOR = BLACK;
LCD_DrawLine(0, 0, 319, 479);  // 对角线
LCD_DrawLine_Color — 指定颜色画直线
void LCD_DrawLine_Color(u16 x1, u16 y1, u16 x2, u16 y2, u16 color);
不依赖全局 FRONT_COLOR，直接指定颜色。
LCD_DrawRectangle — 画空心矩形
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2);
以当前 FRONT_COLOR 画矩形边框，(x1,y1) 为左上⻆，(x2,y2) 为右下⻆。
FRONT_COLOR = BLUE;
LCD_DrawRectangle(10, 50, 310, 200);
LCD_Draw_Circle — 画空心圆
void LCD_Draw_Circle(u16 x0, u16 y0, u8 r);
以 (x0, y0) 为圆心，r 为半径，以当前 FRONT_COLOR 画圆（Bresenham 算法）。
FRONT_COLOR = GREEN;
LCD_Draw_Circle(160, 240, 50);
LCD_DrowSign — 画十字准星标记
void LCD_DrowSign(uint16_t x, uint16_t y, uint16_t color);
在 (x, y) 位置画一个 9×9 的实心方块加水平/垂直各 9 个像素的十字线，用于触摸校准标记等场景。
TFTLCD使用文档.md 
2026-06-01
5 / 11
六、文字显示
字体大小通过 size 参数指定，支持 3 种：
size 值 字体规格 字符像素宽×高
12 
小号 
6 × 12
16 
中号 
8 × 16
24 
大号 
12 × 24
LCD_ShowChar — 显示单个 ASCII 字符
void LCD_ShowChar(u16 x, u16 y, u8 num, u8 size, u8 mode);
参数 说明
x, y 字符左上⻆坐标
num 
ASCII 字符，范围 ' '（0x20）到 '~'（0x7E）
size 字体大小：12 / 16 / 24
mode 
0 = 非叠加模式（背景色填充）；1 = 叠加模式（背景透明）
FRONT_COLOR = BLACK;
BACK_COLOR  = WHITE;
LCD_ShowChar(10, 10, 'A', 24, 0);
LCD_ShowString — 显示 ASCII 字符串
void LCD_ShowString(u16 x, u16 y, u16 width, u16 height, u8 size, u8 *p);
参数 
说明
x, y 
起始坐标（左上⻆）
width 显示区域宽度（像素），超出自动换行
height 显示区域高度（像素），超出裁剪
size 
字体大小：12 / 16 / 24
*p 
字符串指针（ASCII，需转为 u8*）
TFTLCD使用文档.md 
2026-06-01
6 / 11
FRONT_COLOR = WHITE;
BACK_COLOR  = DARKBLUE;
LCD_ShowString(10, 10, 200, 30, 24, (u8 *)"Hello World!");
注意：只支持 ASCII 可打印字符（空格到~），中文需要使用 LCD_ShowFontHZ。
LCD_ShowNum — 显示无符号整数（高位补空格）
void LCD_ShowNum(u16 x, u16 y, u32 num, u8 len, u8 size);
参数 说明
x, y 起始坐标
num 要显示的数值，范围 0 ~ 4294967295
len 显示位数（总共几位，不足高位补空格）
size 字体大小：12 / 16 / 24
// 显示温度值，保留 3 位，如 "25 "
FRONT_COLOR = RED;
LCD_ShowNum(110, 70, temperature, 3, 24);
LCD_ShowxNum — 显示无符号整数（高位可补零）
void LCD_ShowxNum(u16 x, u16 y, u32 num, u8 len, u8 size, u8 mode);
mode 位 
说明
bit7 = 1 高位补 '0'（如 025）
bit7 = 0 高位补 ' '（如 25）
bit0 = 1 叠加模式（背景透明）
bit0 = 0 非叠加模式（背景色填充）
// 高位补零，显示 "025"
LCD_ShowxNum(10, 50, 25, 3, 24, 0x80);
TFTLCD使用文档.md 
2026-06-01
7 / 11
// 高位补空格，非叠加
LCD_ShowxNum(10, 50, 25, 3, 24, 0x00);
LCD_ShowFontHZ — 显示汉字
void LCD_ShowFontHZ(u16 x, u16 y, u8 *cn);
显示 32×29 点阵汉字，字模数据在 font.h 中的 CnChar32x29[] 数组。
每个汉字占 cn 指针的 2 个字节（GBK 编码）。
当前 font.h 中内置的汉字字库有限，如需显示更多汉字需向字库中添加对应字模数据。
FRONT_COLOR = BLACK;
BACK_COLOR  = WHITE;
LCD_ShowFontHZ(10, 60, (u8 *)"温度");
七、图片显示
LCD_ShowPicture — 显示图片
void LCD_ShowPicture(u16 x, u16 y, u16 wide, u16 high, u8 *pic);
参数 说明
x, y 图片左上⻆坐标
wide 图片宽度（像素）
high 图片高度（像素）
*pic 图片数据指针，格式为 RGB565 小端（低字节在前）
图片数据通常放在 picture.h 中作为 const u8 数组。
#include "picture.h"
LCD_ShowPicture(0, 0, 320, 140, (u8 *)gImage_pic);
图片转换工具推荐：Image2Lcd，导出格式选 "C 数组"，颜色格式选 "RGB565"，勾选 "字节倒序"（小
端）。
八、高级控制
TFTLCD使用文档.md 
2026-06-01
8 / 11
LCD_Set_Window — 设置绘图窗口
void LCD_Set_Window(u16 sx, u16 sy, u16 width, u16 height);
设置后续 LCD_WriteData_Color 写入的目标矩形区域，连续写入颜色数据会按行自动递增坐标。
一般由驱动内部调用，手动使用时需注意配合 LCD_WriteData_Color。
LCD_Display_Dir — 切换显示方向
void LCD_Display_Dir(u8 dir);
dir 效果
0 
竖屏（Portrait）
1 
横屏（Landscape）
切换后 tftlcd_data.width 和 tftlcd_data.height 自动更新。
LCD_Display_Dir(1);  // 切换为横屏
LCD_WriteData_Color — 连续写入像素颜色
void LCD_WriteData_Color(u16 color);
在已设置好窗口（LCD_Set_Window）的前提下，连续调用此函数逐个写入像素颜色，适合批量填充或显示图
片。
九、背光控制
LCD_LED = 1;   // 开背光（PB0 高电平）
LCD_LED = 0;   // 关背光
十、完整使用示例
#include "system.h"
#include "SysTick.h"
TFTLCD使用文档.md 
2026-06-01
9 / 11
#include "tftlcd.h"
int main(void)
{
    SysTick_Init(72);
    TFTLCD_Init();         // 初始化
    LCD_Clear(WHITE);      // 清白屏
    // 蓝色标题栏
    LCD_Fill(0, 0, tftlcd_data.width - 1, 39, DARKBLUE);
    FRONT_COLOR = WHITE;
    BACK_COLOR  = DARKBLUE;
    LCD_ShowString(10, 8, 200, 30, 24, (u8 *)"STM32 TFTLCD Demo");
    // 红色边框
    FRONT_COLOR = RED;
    LCD_DrawRectangle(10, 50, tftlcd_data.width - 11, 200);
    // 显示文字
    FRONT_COLOR = BLACK;
    BACK_COLOR  = WHITE;
    LCD_ShowString(20, 65,  100, 24, 24, (u8 *)"Temp:");
    LCD_ShowString(20, 100, 100, 24, 24, (u8 *)"Humi:");
    // 显示数值
    FRONT_COLOR = RED;
    LCD_ShowNum(110, 65,  26, 3, 24);   // 温度 26°C
    FRONT_COLOR = BLUE;
    LCD_ShowNum(110, 100, 58, 3, 24);   // 湿度 58%
    // 单位
    FRONT_COLOR = BLACK;
    LCD_ShowString(170, 65,  40, 24, 24, (u8 *)"C");
    LCD_ShowString(170, 100, 60, 24, 24, (u8 *)"%RH");
    while (1) {}
}
十一、常⻅问题
现象 
排查方向
屏幕全白/全黑，无任何显
示
检查背光 PB0 是否接好；检查 TFTLCD_HX8357DN 宏是否与实际屏幕型号一
致
屏幕花屏或颜色错乱 
FSMC 时序参数与屏幕不匹配，或 tftlcd.h 中选错了屏幕型号宏
文字显示位置偏移 
坐标原点 (0,0) 在左上⻆，x 向右，y 向下，检查传入坐标是否正确
LCD_ShowNum 显示不全 
len 参数要与数值实际位数匹配，如显示 3 位数应传 len=3
TFTLCD使用文档.md 
2026-06-01
10 / 11
现象 
排查方向
汉字显示乱码或空白 
font.h 中 CnChar32x29 数组里没有对应汉字字模，需要自行添加
图片颜色偏色 
Image2Lcd 导出时颜色格式须选 RGB565，且勾选"字节倒序"
TFTLCD使用文档.md 
