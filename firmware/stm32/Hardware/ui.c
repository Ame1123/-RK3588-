#include "ui.h"
#include "tftlcd.h"
#include "touch.h"
#include "Communication.h"
#include "Servo.h"
#include <stdio.h>
#include <string.h>

/* ===================== 布局与配色 ===================== */
#define SCR_W   320
#define SCR_H   480
#define HDR_H   34            /* 顶部标题栏高度 */
#define NAV_Y   434           /* 底部导航栏起始 y */
#define TOAST_Y 414           /* 提示条 y */

#define C_BG     WHITE
#define C_HDR    DARKBLUE     /* 标题栏底色 */
#define C_HDRTX  WHITE
#define C_TX     BLACK
#define C_PANEL  LIGHTGRAY
#define C_OK     GREEN
#define C_WARN   YELLOW
#define C_ALARM  RED
#define C_BTN    GRAYBLUE
#define C_BTNTX  WHITE

/* 页面枚举 */
enum { PG_MAIN = 0, PG_MANUAL, PG_AUTO, PG_DEVICE, PG_SENSOR, PG_ALARM, PG_SET, PG_COUNT };

#define DYN_REFRESH_TICKS 20
#define UI_DYNAMIC_REFRESH_ENABLE 1
#define UI_TOUCH_ENABLE 1

static const char *PAGE_TITLE[PG_COUNT] = {
	"Main Status", "Manual Ctrl", "Auto Mode", "Device Ctrl",
	"Sensors", "Alarm / Log", "Settings"
};

/* ===================== 运行态 ===================== */
static uint8_t  s_page      = PG_MAIN;
static uint8_t  s_full      = 1;   /* 需整页重绘 */
static uint8_t  s_forcedyn  = 0;   /* 需立即刷新动态值 */
static uint8_t  s_dyncnt    = 0;   /* 动态刷新节流计数 */

/* 提示条 */
static char     s_toast[40] = "";
static uint32_t s_toast_t   = 0;

/* 确认对话框 */
static uint8_t  s_dlg       = 0;
static char     s_dlgmsg[44];
static char     s_dlgcmd[56];
static uint8_t  s_dlgspec   = 0;   /* 0=下发命令 1=恢复默认 2=触摸校准 */

/* 事件时间戳（由 diff 得到，供日志页显示） */
static uint8_t  pv_mode = 1, pv_relay = 0, pv_box = 0;
static uint32_t t_mode = 0, t_relay = 0, t_box = 0;

/* ===================== 基础绘制助手 ===================== */
static uint8_t hit(uint16_t tx, uint16_t ty, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
	return (tx >= x) && (tx < x + w) && (ty >= y) && (ty < y + h);
}
static void fillr(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t c)
{
	LCD_Fill(x, y, x + w - 1, y + h - 1, c);
}
static void rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t c)
{
	FRONT_COLOR = c;
	LCD_DrawRectangle(x, y, x + w - 1, y + h - 1);
}
static void str_at(uint16_t x, uint16_t y, uint8_t size, uint16_t fg, uint16_t bg, const char *s)
{
	FRONT_COLOR = fg;
	BACK_COLOR  = bg;
	LCD_ShowString(x, y, SCR_W, size, size, (uint8_t *)s);
}
/* 值字段：先清背景再画（用于动态刷新，避免残影） */
static void field(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t size, uint16_t fg, uint16_t bg, const char *s)
{
	fillr(x, y, w, h, bg);
	FRONT_COLOR = fg;
	BACK_COLOR  = bg;
	LCD_ShowString(x, y, w, h, size, (uint8_t *)s);
}
/* 按钮：填充+边框+居中标签（16号字） */
static void button(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const char *label, uint16_t bg, uint16_t fg)
{
	uint16_t lw = (uint16_t)strlen(label) * 8;
	uint16_t lx = (w > lw) ? x + (w - lw) / 2 : x + 2;
	fillr(x, y, w, h, bg);
	rect(x, y, w, h, BLACK);
	str_at(lx, y + (h > 16 ? (h - 16) / 2 : 0), 16, fg, bg, label);
}
/* 状态灯：14x14 色块 */
static void led(uint16_t x, uint16_t y, uint16_t c)
{
	fillr(x, y, 14, 14, c);
	rect(x, y, 14, 14, BLACK);
}

/* ===================== 通用格式化 ===================== */
static void fmt_uptime(char *b, uint32_t s)
{
	sprintf(b, "%02u:%02u:%02u", (unsigned)(s / 3600), (unsigned)((s / 60) % 60), (unsigned)(s % 60));
}
/* 系统总体状态：0正常 1注意 2报警 */
static uint8_t sys_state(void)
{
	if (!g_sensor_ok) return 2;
	if (g_temp >= g_th_temp_high || g_humi >= g_th_humi_high || g_wind >= g_th_wind_high) return 2;
	if (g_last_cmd_ok != 1 && (g_uptime_sec - g_last_cmd_sec) < 10) return 1;
	return 0;
}
static uint16_t state_color(uint8_t st)
{
	return (st == 2) ? C_ALARM : (st == 1) ? C_WARN : C_OK;
}

/* ===================== 动作下发 ===================== */
static uint8_t ui_send(const char *json)
{
	char buf[64];
	uint8_t i, r;
	for (i = 0; i < sizeof(buf) - 1 && json[i]; i++) buf[i] = json[i];
	buf[i] = 0;
	r = ProcessJSONCommand(buf);
	for (i = 0; i < sizeof(g_last_cmd) - 1 && json[i]; i++) g_last_cmd[i] = json[i];
	g_last_cmd[i] = 0;
	g_last_cmd_sec = g_uptime_sec;
	g_last_cmd_ok  = r ? 1 : 0;
	return r;
}
static void toast(const char *m)
{
	uint8_t i;
	for (i = 0; i < sizeof(s_toast) - 1 && m[i]; i++) s_toast[i] = m[i];
	s_toast[i] = 0;
	s_toast_t = g_uptime_sec;
}

/* ===================== 确认对话框 ===================== */
static void ui_confirm(const char *msg, const char *cmd, uint8_t spec)
{
	uint8_t i;
	for (i = 0; i < sizeof(s_dlgmsg) - 1 && msg[i]; i++) s_dlgmsg[i] = msg[i];
	s_dlgmsg[i] = 0;
	s_dlgcmd[0] = 0;
	if (cmd)
	{
		for (i = 0; i < sizeof(s_dlgcmd) - 1 && cmd[i]; i++) s_dlgcmd[i] = cmd[i];
		s_dlgcmd[i] = 0;
	}
	s_dlgspec = spec;
	s_dlg = 1;
	s_full = 1;
}
static void restore_defaults(void)
{
	g_th_temp_high = 40.0f;
	g_th_humi_high = 90.0f;
	g_th_wind_high = 20.0f;
}
static void draw_dialog(void)
{
	fillr(30, 168, 260, 148, C_PANEL);
	rect(30, 168, 260, 148, BLACK);
	rect(31, 169, 258, 146, BLACK);
	str_at(46, 184, 16, C_TX, C_PANEL, "Confirm:");
	str_at(46, 212, 16, C_ALARM, C_PANEL, s_dlgmsg);
	button(48, 258, 96, 44, "YES", C_OK, BLACK);
	button(176, 258, 96, 44, "NO", C_ALARM, WHITE);
}
static void dialog_tap(uint16_t x, uint16_t y)
{
	if (hit(x, y, 48, 258, 96, 44))       /* YES */
	{
		if (s_dlgspec == 0)      ui_send(s_dlgcmd);
		else if (s_dlgspec == 1) { restore_defaults(); toast("Defaults restored"); }
		else if (s_dlgspec == 2) { TP_Adjust(); }
		s_dlg = 0;
		s_full = 1;
	}
	else if (hit(x, y, 176, 258, 96, 44)) /* NO */
	{
		s_dlg = 0;
		s_full = 1;
	}
}

/* ===================== 头部 / 导航 / 提示条 ===================== */
static void draw_header(void)
{
	char b[8];
	fillr(0, 0, SCR_W, HDR_H, C_HDR);
	str_at(8, 9, 16, C_HDRTX, C_HDR, PAGE_TITLE[s_page]);
	sprintf(b, "%d/%d", s_page + 1, PG_COUNT);
	str_at(SCR_W - 32, 9, 16, C_HDRTX, C_HDR, b);
}
static void draw_nav(void)
{
	fillr(0, NAV_Y, SCR_W, SCR_H - NAV_Y, C_HDR);
	button(4,   NAV_Y + 2, 100, 42, "< PREV", C_BTN, C_BTNTX);
	button(110, NAV_Y + 2, 100, 42, "HOME",   C_BTN, C_BTNTX);
	button(216, NAV_Y + 2, 100, 42, "NEXT >", C_BTN, C_BTNTX);
}
static void nav_tap(uint16_t x)
{
	if (hit(x, NAV_Y + 2, 4, 0, 100, 44))        s_page = (s_page == 0) ? PG_COUNT - 1 : s_page - 1;
	else if (hit(x, NAV_Y + 2, 110, 0, 100, 44)) s_page = PG_MAIN;
	else if (hit(x, NAV_Y + 2, 216, 0, 100, 44)) s_page = (s_page + 1) % PG_COUNT;
	s_full = 1;
}
static void draw_toast(void)
{
	if (s_toast[0] && (g_uptime_sec - s_toast_t) < 5)
		field(0, TOAST_Y, SCR_W, 18, 16, C_ALARM, C_BG, s_toast);
	else
		fillr(0, TOAST_Y, SCR_W, 18, C_BG);
}

/* ===================== 页 0：主界面 ===================== */
static void page_main(uint8_t full)
{
	char b[40];
	uint8_t st = sys_state();

	/* 系统状态大横幅（颜色随状态变化，故每次都画） */
	fillr(6, 38, 308, 40, state_color(st));
	rect(6, 38, 308, 40, BLACK);
	str_at(14, 42, 16, BLACK, state_color(st), "SYSTEM");
	str_at(14, 58, 16, BLACK, state_color(st),
	       st == 2 ? "ALARM" : st == 1 ? "ATTENTION" : "RUNNING");

	/* 模式横幅 */
	fillr(6, 82, 308, 30, moshiqiehuan ? C_OK : GBLUE);
	rect(6, 82, 308, 30, BLACK);
	sprintf(b, "MODE: %s", moshiqiehuan ? "MANUAL" : "AUTO");
	str_at(14, 89, 16, BLACK, moshiqiehuan ? C_OK : GBLUE, b);

	if (full)
	{
		str_at(10, 122, 16, C_TX, C_BG, "S1:");
		str_at(165, 122, 16, C_TX, C_BG, "S2:");
		str_at(10, 148, 16, C_TX, C_BG, "S3:");
		str_at(165, 148, 16, C_TX, C_BG, "S4:");
		str_at(10, 178, 16, C_TX, C_BG, "Relay:");
		str_at(165, 178, 16, C_TX, C_BG, "Box:");
		str_at(10, 204, 16, C_TX, C_BG, "Temp:");
		str_at(165, 204, 16, C_TX, C_BG, "Humi:");
		str_at(10, 230, 16, C_TX, C_BG, "Wind:");
		str_at(165, 230, 16, C_TX, C_BG, "Dir:");
		str_at(10, 256, 16, C_TX, C_BG, "Cmd:");
		str_at(10, 288, 16, C_TX, C_BG, "Alarms:");
	}

	/* 舵机 */
	sprintf(b, "%.0f", Servo_GetAngle1()); field(46, 122, 90, 18, 16, BLUE, C_BG, b);
	sprintf(b, "%.0f", Servo_GetAngle2()); field(200, 122, 90, 18, 16, BLUE, C_BG, b);
	sprintf(b, "%.0f", Servo_GetAngle3()); field(46, 148, 90, 18, 16, BLUE, C_BG, b);
	sprintf(b, "%.0f", Servo_GetAngle4()); field(200, 148, 90, 18, 16, BLUE, C_BG, b);
	/* 继电器 / box */
	field(66, 178, 90, 18, 16, relay_switch ? C_ALARM : C_TX, C_BG, relay_switch ? "ON" : "OFF");
	field(200, 178, 90, 18, 16, box_value ? BLUE : C_TX, C_BG, box_value ? "OPEN" : "CLOSE");
	/* 温湿度 */
	sprintf(b, "%.1fC", g_temp); field(60, 204, 95, 18, 16, g_temp >= g_th_temp_high ? C_ALARM : C_TX, C_BG, b);
	sprintf(b, "%.1f%%", g_humi); field(210, 204, 95, 18, 16, g_humi >= g_th_humi_high ? C_ALARM : C_TX, C_BG, b);
	/* 风速风向 */
	sprintf(b, "%.1fm/s", g_wind); field(60, 230, 95, 18, 16, g_wind >= g_th_wind_high ? C_ALARM : C_TX, C_BG, b);
	field(200, 230, 90, 18, 16, C_TX, C_BG, g_wind_dir);
	/* 命令状态 */
	sprintf(b, "%s  %us ago", g_last_cmd_ok == 1 ? "OK" : g_last_cmd_ok == 2 ? "BADJSON" : "FAIL",
	        (unsigned)(g_uptime_sec - g_last_cmd_sec));
	field(52, 256, 258, 18, 16, g_last_cmd_ok == 1 ? C_TX : C_ALARM, C_BG, b);

	/* 报警区 */
	{
		uint16_t y = 306;
		fillr(6, 304, 308, 100, C_BG);
		if (!g_sensor_ok)         { str_at(14, y, 16, C_ALARM, C_BG, "! Sensor read failure"); y += 20; }
		if (g_temp >= g_th_temp_high) { str_at(14, y, 16, C_ALARM, C_BG, "! High temperature"); y += 20; }
		if (g_humi >= g_th_humi_high) { str_at(14, y, 16, C_ALARM, C_BG, "! High humidity"); y += 20; }
		if (g_wind >= g_th_wind_high) { str_at(14, y, 16, C_ALARM, C_BG, "! Wind too high"); y += 20; }
		if (y == 306) str_at(14, y, 16, C_OK, C_BG, "No active alarms");
	}
}

/* ===================== 页 1：手动控制 ===================== */
static void manual_step(uint8_t servo, float delta)
{
	char b[24];
	float cur, nv, lo = 0, hi = 180;
	switch (servo)
	{
		case 1:
			if (moshiqiehuan == 0) { toast("Auto mode: S1 locked"); return; }
			cur = Servo_GetAngle1(); break;
		case 2: cur = Servo_GetAngle2(); break;
		case 3: cur = Servo_GetAngle3(); lo = 90; break;
		default: cur = Servo_GetAngle4(); break;
	}
	nv = cur + delta;
	if (nv < lo) nv = lo;
	if (nv > hi) nv = hi;
	sprintf(b, "{\"servo%d\":%.0f}", servo, nv);
	ui_send(b);
}
static void stepper_row(uint8_t full, uint16_t y, const char *name, float ang)
{
	char b[8];
	if (full)
	{
		str_at(8, y + 12, 16, C_TX, C_BG, name);
		button(96, y, 46, 44, "-", C_BTN, C_BTNTX);
		button(214, y, 46, 44, "+", C_BTN, C_BTNTX);
	}
	sprintf(b, "%3.0f", ang);
	field(150, y + 8, 56, 24, 24, BLUE, C_BG, b);
}
static void page_manual(uint8_t full)
{
	stepper_row(full, 40,  "S1(0-180)", Servo_GetAngle1());
	stepper_row(full, 94,  "S2(0-180)", Servo_GetAngle2());
	stepper_row(full, 148, "S3(90-180)", Servo_GetAngle3());
	stepper_row(full, 202, "S4(0-180)", Servo_GetAngle4());
	if (full)
	{
		str_at(8, 258, 16, C_TX, C_BG, "Presets (all servos):");
		button(6,   278, 68, 40, "G1", C_BTN, C_BTNTX);
		button(82,  278, 68, 40, "G2", C_BTN, C_BTNTX);
		button(158, 278, 68, 40, "G3", C_BTN, C_BTNTX);
		button(234, 278, 68, 40, "G4", C_BTN, C_BTNTX);
		button(6,   326, 304, 40, "ALL HOME", C_WARN, BLACK);
	}
}
static void manual_tap(uint16_t x, uint16_t y)
{
	/* 舵机加减：步进 10 度 */
	if (hit(x, y, 96, 40, 46, 44))  manual_step(1, -10);
	else if (hit(x, y, 214, 40, 46, 44)) manual_step(1, +10);
	else if (hit(x, y, 96, 94, 46, 44))  manual_step(2, -10);
	else if (hit(x, y, 214, 94, 46, 44)) manual_step(2, +10);
	else if (hit(x, y, 96, 148, 46, 44)) manual_step(3, -10);
	else if (hit(x, y, 214, 148, 46, 44)) manual_step(3, +10);
	else if (hit(x, y, 96, 202, 46, 44)) manual_step(4, -10);
	else if (hit(x, y, 214, 202, 46, 44)) manual_step(4, +10);
	/* 预设/归位（确认执行） */
	else if (hit(x, y, 6, 278, 68, 40))   ui_confirm("Preset G1 all?", "{\"command\":\"ALL1\"}", 0);
	else if (hit(x, y, 82, 278, 68, 40))  ui_confirm("Preset G2 all?", "{\"command\":\"ALL2\"}", 0);
	else if (hit(x, y, 158, 278, 68, 40)) ui_confirm("Preset G3 all?", "{\"command\":\"ALL3\"}", 0);
	else if (hit(x, y, 234, 278, 68, 40)) ui_confirm("Preset G4 all?", "{\"command\":\"ALL4\"}", 0);
	else if (hit(x, y, 6, 326, 304, 40))  ui_confirm("Home all servos?", "{\"command\":\"ALLHOME\"}", 0);
}

/* ===================== 页 2：自动模式 ===================== */
static void page_auto(uint8_t full)
{
	char b[32];
	/* 模式横幅 */
	fillr(6, 40, 308, 40, moshiqiehuan ? C_OK : GBLUE);
	rect(6, 40, 308, 40, BLACK);
	sprintf(b, "MODE: %s", moshiqiehuan ? "MANUAL" : "AUTO");
	str_at(14, 52, 16, BLACK, moshiqiehuan ? C_OK : GBLUE, b);

	/* 舵机1自动旋转状态 */
	fillr(6, 90, 308, 34, (moshiqiehuan == 0 && servo1stop == 0) ? C_OK : C_WARN);
	rect(6, 90, 308, 34, BLACK);
	sprintf(b, "S1 auto: %s", moshiqiehuan ? "N/A" : servo1stop ? "PAUSED" : "RUNNING");
	str_at(14, 99, 16, BLACK, (moshiqiehuan == 0 && servo1stop == 0) ? C_OK : C_WARN, b);

	if (full)
	{
		str_at(10, 134, 16, C_TX, C_BG, "S1 angle:");
		button(6, 176, 304, 44, "TOGGLE AUTO / MANUAL", C_BTN, C_BTNTX);
		button(6, 230, 148, 44, "PAUSE S1", C_BTN, C_BTNTX);
		button(162, 230, 148, 44, "RESUME S1", C_BTN, C_BTNTX);
		str_at(10, 288, 16, C_TX, C_BG, "Cycle: 0-180-0 loop");
		str_at(10, 312, 16, C_ALARM, C_BG, "Auto: S1 not manual-ctrl");
	}
	sprintf(b, "%.0f", Servo_GetAngle1());
	field(96, 134, 90, 18, 16, BLUE, C_BG, b);
}
static void auto_tap(uint16_t x, uint16_t y)
{
	if (hit(x, y, 6, 176, 304, 44))
	{
		if (moshiqiehuan) ui_confirm("Enter AUTO mode?", "{\"auto\":0}", 0);
		else              ui_confirm("Enter MANUAL mode?", "{\"auto\":1}", 0);
	}
	else if (hit(x, y, 6, 230, 148, 44))
	{
		if (moshiqiehuan) toast("Only in Auto mode");
		else ui_send("{\"servo1stop\":1}");
	}
	else if (hit(x, y, 162, 230, 148, 44))
	{
		if (moshiqiehuan) toast("Only in Auto mode");
		else ui_send("{\"servo1stop\":0}");
	}
}

/* ===================== 页 3：设备控制 ===================== */
static void page_device(uint8_t full)
{
	char b[32];
	/* 继电器横幅 */
	fillr(6, 40, 308, 34, relay_switch ? C_ALARM : C_PANEL);
	rect(6, 40, 308, 34, BLACK);
	sprintf(b, "Relay: %s", relay_switch ? "ON" : "OFF");
	str_at(14, 49, 16, relay_switch ? WHITE : C_TX, relay_switch ? C_ALARM : C_PANEL, b);
	/* box横幅 */
	fillr(6, 150, 308, 34, box_value ? GBLUE : C_PANEL);
	rect(6, 150, 308, 34, BLACK);
	sprintf(b, "Box: %s", box_value ? "OPEN" : "CLOSE");
	str_at(14, 159, 16, C_TX, box_value ? GBLUE : C_PANEL, b);

	if (full)
	{
		button(6, 82, 148, 44, "RELAY ON", C_BTN, C_BTNTX);
		button(162, 82, 148, 44, "RELAY OFF", C_BTN, C_BTNTX);
		button(6, 192, 148, 44, "BOX OPEN", C_BTN, C_BTNTX);
		button(162, 192, 148, 44, "BOX CLOSE", C_BTN, C_BTNTX);
		str_at(10, 252, 16, C_TX, C_BG, "Servo5:");
		str_at(165, 252, 16, C_TX, C_BG, "Servo6:");
		str_at(10, 278, 16, C_TX, C_BG, "Status:");
	}
	sprintf(b, "%.0f", Servo5_Angle); field(72, 252, 80, 18, 16, BLUE, C_BG, b);
	sprintf(b, "%.0f", Servo6_Angle); field(228, 252, 80, 18, 16, BLUE, C_BG, b);
	field(72, 278, 200, 18, 16, BoxServo_InPosition() ? C_OK : C_WARN, C_BG,
	      BoxServo_InPosition() ? "DONE (in position)" : "MOVING...");
}
static void device_tap(uint16_t x, uint16_t y)
{
	if (hit(x, y, 6, 82, 148, 44))        ui_confirm("Turn relay ON?", "{\"switch\":1}", 0);
	else if (hit(x, y, 162, 82, 148, 44)) ui_confirm("Turn relay OFF?", "{\"switch\":0}", 0);
	else if (hit(x, y, 6, 192, 148, 44))  ui_confirm("Open the box?", "{\"box\":1}", 0);
	else if (hit(x, y, 162, 192, 148, 44)) ui_confirm("Close the box?", "{\"box\":0}", 0);
}

/* ===================== 页 4：传感器监控 ===================== */
static void sensor_line(uint8_t full, uint16_t y, const char *name, float val, const char *unit,
                        float th, float mn, float mx)
{
	char b[40];
	if (full) str_at(10, y, 16, C_TX, C_BG, name);
	led(140, y, (val >= th) ? C_ALARM : C_OK);
	sprintf(b, "%.1f%s", val, unit);
	field(160, y, 90, 18, 16, (val >= th) ? C_ALARM : C_TX, C_BG, b);
	sprintf(b, "th>=%.0f", th);
	field(258, y, 60, 18, 16, GRAY, C_BG, b);
	sprintf(b, "min %.1f  max %.1f", mn > 900 ? 0 : mn, mx < -900 ? 0 : mx);
	field(10, y + 18, 300, 16, 16, GRAY, C_BG, b);
}
static void page_sensor(uint8_t full)
{
	char b[40];
	sensor_line(full, 44,  "Temperature", g_temp, "C", g_th_temp_high, g_temp_min, g_temp_max);
	sensor_line(full, 96,  "Humidity",    g_humi, "%", g_th_humi_high, g_humi_min, g_humi_max);
	sensor_line(full, 148, "Wind speed",  g_wind, "m/s", g_th_wind_high, 0, g_wind_max);
	if (full)
	{
		str_at(10, 206, 16, C_TX, C_BG, "Wind dir:");
		str_at(10, 240, 16, C_TX, C_BG, "Updated:");
		str_at(10, 266, 16, C_TX, C_BG, "Status:");
	}
	field(96, 206, 90, 18, 16, BLUE, C_BG, g_wind_dir);
	sprintf(b, "%us ago", (unsigned)(g_uptime_sec - g_sensor_sec));
	field(88, 240, 120, 18, 16, C_TX, C_BG, b);
	field(80, 266, 200, 18, 16, g_sensor_ok ? C_OK : C_ALARM, C_BG, g_sensor_ok ? "NORMAL" : "READ FAIL");
}

/* ===================== 页 5：报警 / 日志 ===================== */
static void page_alarm(uint8_t full)
{
	char b[48];
	if (full)
	{
		str_at(10, 40, 16, C_TX, C_BG, "Last cmd:");
		str_at(10, 88, 16, C_TX, C_BG, "Last error:");
		str_at(10, 136, 16, C_TX, C_BG, "Err JSON/Unk/Sensor:");
		str_at(10, 172, 16, C_TX, C_BG, "Uptime:");
		str_at(10, 200, 16, C_TX, C_BG, "Last mode chg:");
		str_at(10, 226, 16, C_TX, C_BG, "Last relay:");
		str_at(10, 252, 16, C_TX, C_BG, "Last box:");
	}
	field(10, 60, 300, 18, 16, C_TX, C_BG, g_last_cmd);
	sprintf(b, "%s  %us ago", g_last_cmd_ok == 1 ? "OK" : g_last_cmd_ok == 2 ? "BADJSON" : "FAIL",
	        (unsigned)(g_uptime_sec - g_last_cmd_sec));
	field(120, 40, 190, 18, 16, g_last_cmd_ok == 1 ? C_OK : C_ALARM, C_BG, b);
	field(10, 108, 300, 18, 16, C_ALARM, C_BG, g_last_error);
	sprintf(b, "%u / %u / %u", g_err_invalid_json, g_err_unknown_cmd, g_err_sensor);
	field(10, 154, 300, 18, 16, C_TX, C_BG, b);
	fmt_uptime(b, g_uptime_sec); field(88, 172, 120, 18, 16, C_TX, C_BG, b);
	sprintf(b, "%us ago (%s)", (unsigned)(g_uptime_sec - t_mode), moshiqiehuan ? "MANUAL" : "AUTO");
	field(140, 200, 170, 18, 16, C_TX, C_BG, b);
	sprintf(b, "%us ago (%s)", (unsigned)(g_uptime_sec - t_relay), relay_switch ? "ON" : "OFF");
	field(120, 226, 190, 18, 16, C_TX, C_BG, b);
	sprintf(b, "%us ago (%s)", (unsigned)(g_uptime_sec - t_box), box_value ? "OPEN" : "CLOSE");
	field(96, 252, 214, 18, 16, C_TX, C_BG, b);

	/* 报警汇总 */
	{
		uint16_t y = 300;
		if (full) str_at(10, 280, 16, C_TX, C_BG, "Active alarms:");
		fillr(6, 298, 308, 108, C_BG);
		if (!g_sensor_ok)             { str_at(14, y, 16, C_ALARM, C_BG, "! Sensor read failure"); y += 20; }
		if (g_temp >= g_th_temp_high) { str_at(14, y, 16, C_ALARM, C_BG, "! High temperature"); y += 20; }
		if (g_humi >= g_th_humi_high) { str_at(14, y, 16, C_ALARM, C_BG, "! High humidity"); y += 20; }
		if (g_wind >= g_th_wind_high) { str_at(14, y, 16, C_ALARM, C_BG, "! Wind too high"); y += 20; }
		if (y == 300) str_at(14, y, 16, C_OK, C_BG, "None");
	}
}

/* ===================== 页 6：设置 ===================== */
static void thr_row(uint8_t full, uint16_t y, const char *name, float val, const char *unit)
{
	char b[24];
	if (full)
	{
		str_at(8, y + 10, 16, C_TX, C_BG, name);
		button(150, y, 40, 40, "-", C_BTN, C_BTNTX);
		button(272, y, 40, 40, "+", C_BTN, C_BTNTX);
	}
	sprintf(b, "%.0f%s", val, unit);
	field(196, y + 8, 72, 24, 24, BLUE, C_BG, b);
}
static void page_set(uint8_t full)
{
	if (full)
	{
		str_at(8, 40, 16, C_TX, C_BG, "Baud: 115200 (fixed)");
		str_at(8, 62, 16, C_TX, C_BG, "Init: S1=90 S2=0 S3=90 S4=0");
		str_at(8, 84, 16, C_TX, C_BG, "Limit: S3 = 90..180");
	}
	thr_row(full, 110, "TempHi", g_th_temp_high, "C");
	thr_row(full, 156, "HumiHi", g_th_humi_high, "%");
	thr_row(full, 202, "WindHi", g_th_wind_high, "");
	if (full)
	{
		button(6, 250, 148, 40, "BL ON", C_BTN, C_BTNTX);
		button(162, 250, 148, 40, "BL OFF", C_BTN, C_BTNTX);
		button(6, 296, 304, 40, "CALIBRATE TOUCH", C_WARN, BLACK);
		button(6, 342, 304, 40, "RESTORE DEFAULTS", C_ALARM, WHITE);
	}
}
static void set_tap(uint16_t x, uint16_t y)
{
	if (hit(x, y, 150, 110, 40, 40))      { g_th_temp_high -= 1; if (g_th_temp_high < 0) g_th_temp_high = 0; }
	else if (hit(x, y, 272, 110, 40, 40)) { g_th_temp_high += 1; if (g_th_temp_high > 125) g_th_temp_high = 125; }
	else if (hit(x, y, 150, 156, 40, 40)) { g_th_humi_high -= 1; if (g_th_humi_high < 0) g_th_humi_high = 0; }
	else if (hit(x, y, 272, 156, 40, 40)) { g_th_humi_high += 1; if (g_th_humi_high > 100) g_th_humi_high = 100; }
	else if (hit(x, y, 150, 202, 40, 40)) { g_th_wind_high -= 1; if (g_th_wind_high < 0) g_th_wind_high = 0; }
	else if (hit(x, y, 272, 202, 40, 40)) { g_th_wind_high += 1; if (g_th_wind_high > 60) g_th_wind_high = 60; }
	else if (hit(x, y, 6, 250, 148, 40))  { LCD_LED = 1; toast("Backlight ON"); }
	else if (hit(x, y, 162, 250, 148, 40)) { LCD_LED = 0; toast("Backlight OFF"); }
	else if (hit(x, y, 6, 296, 304, 40))  ui_confirm("Calibrate touch?", 0, 2);
	else if (hit(x, y, 6, 342, 304, 40))  ui_confirm("Restore defaults?", 0, 1);
}

/* ===================== 渲染 / 轮询调度 ===================== */
static void draw_page(uint8_t full)
{
	if (full) fillr(0, HDR_H, SCR_W, NAV_Y - HDR_H, C_BG);
	switch (s_page)
	{
		case PG_MAIN:   page_main(full);   break;
		case PG_MANUAL: page_manual(full); break;
		case PG_AUTO:   page_auto(full);   break;
		case PG_DEVICE: page_device(full); break;
		case PG_SENSOR: page_sensor(full); break;
		case PG_ALARM:  page_alarm(full);  break;
		case PG_SET:    page_set(full);    break;
		default: break;
	}
	draw_toast();
}

static void page_tap(uint16_t x, uint16_t y)
{
	switch (s_page)
	{
		case PG_MANUAL: manual_tap(x, y); break;
		case PG_AUTO:   auto_tap(x, y);   break;
		case PG_DEVICE: device_tap(x, y); break;
		case PG_SET:    set_tap(x, y);    break;
		default: break;                    /* 主界面/传感器/日志页无控件 */
	}
}

/* 触摸上升沿检测：一次按下产生一次 tap */
static uint8_t read_tap(uint16_t *x, uint16_t *y)
{
	static uint8_t was = 0;
	tp_dev.scan(0);
	if (tp_dev.sta & TP_PRES_DOWN)
	{
		if (!was)
		{
			was = 1;
			if (tp_dev.x[0] < SCR_W && tp_dev.y[0] < SCR_H)
			{
				*x = tp_dev.x[0];
				*y = tp_dev.y[0];
				return 1;
			}
		}
	}
	else
	{
		was = 0;
	}
	return 0;
}

/* 事件跟踪：diff 出模式/继电器/box 变化并打时间戳 */
static void track_events(void)
{
	if (moshiqiehuan != pv_mode) { pv_mode = moshiqiehuan; t_mode = g_uptime_sec; }
	if (relay_switch != pv_relay) { pv_relay = relay_switch; t_relay = g_uptime_sec; }
	if (box_value != pv_box)     { pv_box = box_value;     t_box = g_uptime_sec; }
}

void UI_Init(void)
{
#if UI_TOUCH_ENABLE
	TP_Init();            /* 触摸屏初始化 + 首次校准（校准参数存内部 Flash） */
#endif
	pv_mode = moshiqiehuan;
	pv_relay = relay_switch;
	pv_box = box_value;
	s_full = 1;
	LCD_Clear(C_BG);
#if !UI_TOUCH_ENABLE
	(void)&dialog_tap;
	(void)&nav_tap;
	(void)&page_tap;
	(void)&read_tap;
#endif
#if !UI_DYNAMIC_REFRESH_ENABLE
	(void)s_dyncnt;
#endif
}

void UI_Poll(void)
{
#if UI_TOUCH_ENABLE
	uint16_t x, y;
	track_events();
	if (!read_tap(&x, &y)) return;
	if (s_dlg)          { dialog_tap(x, y); return; }
	if (y >= NAV_Y)     { nav_tap(x); return; }
	page_tap(x, y);
	s_forcedyn = 1;      /* 交互后立即刷新数值 */
#else
	track_events();
#endif
}

void UI_Render(void)
{
#if UI_DYNAMIC_REFRESH_ENABLE
	uint8_t periodic_page;
#endif

	if (s_dlg)
	{
		if (s_full) { draw_dialog(); s_full = 0; }
		return;
	}
	if (s_full)
	{
		draw_header();
		draw_page(1);
		draw_nav();
		s_full = 0;
		s_dyncnt = 0;
		return;
	}
#if UI_DYNAMIC_REFRESH_ENABLE
	periodic_page = (s_page == PG_MAIN) || (s_page == PG_AUTO) ||
	                (s_page == PG_DEVICE) || (s_page == PG_SENSOR) ||
	                (s_page == PG_ALARM);
	if (!(s_forcedyn || (periodic_page && ++s_dyncnt >= DYN_REFRESH_TICKS))) return;
	if (s_forcedyn || ++s_dyncnt >= 8)   /* 约 1s 刷新一次动态值 */
	{
		draw_page(0);
		s_dyncnt = 0;
		s_forcedyn = 0;
	}
#else
	if (s_forcedyn)
	{
		draw_page(0);
		s_dyncnt = 0;
		s_forcedyn = 0;
	}
#endif
}
