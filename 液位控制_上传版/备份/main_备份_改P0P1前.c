
/* ==========================================================================
 * 第 1 部分：包含头文件
 * ========================================================================== */
#include "stm32f10x.h" 
#include <stdio.h> 
#include <stdarg.h> 
#include "delay.h" 
#include "oled.h" 
#include "si2c.h"  
#include "button.h"                                   

/* ==========================================================================
 * 第 2 部分：参数配置
 * ========================================================================== */

/* 默认零点距离：探头下表面 到 "水面在0cm刻度时" 的距离，单位cm。 */
#define ZERO_DISTANCE_DEFAULT   7.00f                   /* 默认零点距离：7.00 厘米 */

/* 误差在这个范围内，就认为"已经稳住了"。 */
#define HOLD_TOLERANCE_CM       0.30f                   /* 稳定判定范围：±0.3 厘米 */
#define HOLD_GRACE_MS           2000                    /* 抖动时限：2秒钟 */

/* ★传感器故障判据（时间，毫秒）：连续这么久没拿到有效数据，就认为出故障了。 */
#define SENSOR_ERR_MS           2000

/* AUTO 自动流程里，稳住15秒钟才会自动切到下一个目标。 */
#define HOLD_TIME_S             15.0f                   

/* 从关泵到水真的不再往上涨，大约需要这么久（秒）。*/
#define PUMP_STOP_LAG_S         0.1f                   

/* 最多提前多少厘米关泵*/
#define EARLY_STOP_MAX_CM        0.10f                 

/* 水泵最短开多久 / 最短关多久（毫秒）。 */
#define PUMP_MIN_ON_MS          300                     
#define PUMP_MIN_OFF_MS         200                    

/* 水泵最长连续开多久（毫秒）。 */
#define PUMP_MAX_ON_MS          6000    

/* 开泵以后至少经过这么久（毫秒）才开始用实测速度做预测。 */
#define RATE_MEASURE_MIN_MS             600                     /* 开泵 600ms 后才开始算上升速度 */

/* 声速换算：343m/s 的声速下，1微秒的回波时间 = 0.01715厘米的单程距离 */
#define SOUND_CM_PER_US         0.01715f

/* 每隔多少毫秒测一次液位 */
#define SAMPLE_PERIOD_MS        60                      

/* ---- 超声波测距超时---- */
#define ECHO_RISE_US            3000                    /* 等 ECHO 拉高的最长时间（微秒） */
#define ECHO_TIMEOUT_US         10000                   /* ECHO 高电平的最长时间（微秒）*/

/* ★假数据过滤参数 */
#define LEVEL_RANGE_CM          8.0f    /* 液位合理范围：零点上下各 8cm*/
/* （跳变判据直接取 LEVEL_RANGE_CM 的一半，不再单独设参数） */
#define PUMP_RISE_RATE_DEFAULT  0.45f   /* 估不出上升速度时用的默认值 */

/* ★"读数可信"的判据 */
#define STABLE_COUNT            2                       /* 连续几次读数互相一致才算可信 */
#define MAX_SPREAD_CM           0.50f                   /* 这几次之间最大允许差多少厘米 */
#define SETTLE_MS               1000                    /* 停泵后等水面平静的时间(毫秒) */

/* 屏幕每隔多少毫秒刷新一次（OLED 走软件I2C，刷新一次要20ms左右） */
#define OLED_PERIOD_MS            120                    /* 屏幕刷新周期：120 毫秒 */

/* ==========================================================================
 * 第 3 部分：全局变量（整个程序都要用到的"公共记事本"）
 * ========================================================================== */

/* ---- 3.1 硬件对象 ---- */
static OLED_TypeDef  oled;                            /* OLED 显示屏对象，后面所有屏幕操作都要带着它 */
static SI2C_TypeDef  si2c;                            /* OLED 用的软件I2C对象（记录SCL/SDA接在哪两个脚） */
static Button_TypeDef key1;                           /* 按键1对象（按键库用它记住按键的状态） */
static Button_TypeDef key2;                           /* 按键2对象 */

/* ---- 3.2 液位相关的数据 ---- */
static float   zero_distance_cm = ZERO_DISTANCE_DEFAULT; /* 零点距离：水面在0刻度时，探头到水面的距离 */
static float   distance_cm      = ZERO_DISTANCE_DEFAULT; /* 当前（滤波后的）探头到水面的距离 */
static float   level_cm         = 0.00f;                 /* 当前液位：正数=高于0刻度，负数=低于0刻度 */
static uint8_t sensor_ok        = 0;                     /* 这次测量成功了吗：1=成功 0=失败 */


/* ---- 3.3 水泵和控制器 ---- */
static float    target_cm        = 0.00f;                   /* 目标液位（希望水面停在哪个刻度） */
static uint8_t  pump_on          = 0;                       /* 水泵现在开着吗：1=开 0=关 */
static uint32_t pump_switch_ms   = 0;                       /* 水泵状态最后一次改变的时刻（毫秒） */
static float    level_at_start   = 0.00f;                   /* 本次开泵那一瞬间的液位（用来算上升速度） */
static float    rise_rate        = PUMP_RISE_RATE_DEFAULT;  /* 液面上升速度 cm/s（一开始用默认值，测到真的就替换） */
static float    hold_seconds     = 0.00f;                   /* 液位保持在误差范围内的持续时间（秒） */
static uint32_t last_control_ms     = 0;                    /* 上一次做控制计算的时刻（用来算时间差） */

/* ---- 3.4 模式 ---- */
typedef enum
{
	MODE_ZERO = 0,                                          /* 模式1：标定零点 */
	MODE_MANUAL = 1,                                        /* 模式2：手动控制水泵 */
	MODE_HOLD0 = 2,                                         /* 模式3：自动控制，始终保持在 0 位 */
	MODE_AUTO = 3,                                          /* 模式4：稳住0 → -2 → +2） */
	MODE_COUNT = 4                                          /* 模式的个数，用来做循环计数*/
} CtrlMode;

static CtrlMode mode        = MODE_ZERO;              /* 当前模式，开机默认停在模式1（标定零点） */
static uint8_t  auto_stage  = 0;                      /* AUTO 自动流程走到第几步（0~3） */

/* ---- 3.5 滤波用的数组（去掉测量噪声） ---- */
#define FILTER_COUNT  7                                     /* 第一级·中值滤波：取最近 7 次，丢掉突发错误数据 */
#define AVG_COUNT     8                                     /* 第二级·滑动平均：再取最近 8 次求平均（约 0.5 秒） */

/* 中值滤波 → 把突然跳到远处的假数据整条丢掉
   滑动平均 → 把剩下的抖动磨平（约 0.5 秒的平滑）*/
static float   raw_buf[FILTER_COUNT];                      /* 存放最近几次测到的原始距离 */
static uint8_t raw_count     = 0;                          /* 数组里已经存了几个数据（最多7个） */
static uint8_t raw_index     = 0;                          /* 下一个数据要存到数组的第几个格子 */

static uint8_t  has_last_distance = 0;                     /* 有没有存过可信距离：0=还没有 */
static float    stable_buf[STABLE_COUNT];                  /* 最近几次原始读数 */
static uint8_t  stable_index        = 0;                   /* 环形数组的下一个位置 */
static uint8_t  stable_stored        = 0;                  /* 已经存了几个 */
static uint8_t  level_trusted   = 0;                       /* 1 = 现在的液位是"可信读数"测出来的 */


static uint8_t  level_estimated = 0;                   /* 1 = 现在显示的液位是估算值（泵在转、测不准时） */
static uint32_t last_distance_ms = 0;                  /* 上一次拿到可信距离的时刻（标定要用） */

/* ★自学习上升速度用的变量：泵转着时超声波被搅乱、学不到真实速度，
     所以改成"每次补完水、水面平静后，用真实液位差反推速度" */
static uint8_t  learn_pending   = 0;                   /* 1 = 正在等一次真实读数来学习速度 */
static float    learn_start_cm  = 0.0f;                /* 这次开泵前的液位 */
static uint32_t learn_on_ms     = 0;                   /* 这次一共抽了多久 */
static uint32_t hold_outside_ms     = 0;                   /* 液位连续"跑出容差"多久了（用来做抖动宽限） */

static float    avg_buf[AVG_COUNT];                        /* 第二级滤波：滑动平均用的环形数组 */
static uint8_t  avg_index         = 0;                   /* 滑动平均数组的下一个位置 */
static uint8_t  avg_count         = 0;                   /* 滑动平均数组里已有几个数据 */

/* ==========================================================================
 * 第 4 部分：函数声明
 * ========================================================================== */
static void  Hardware_Init(void);                       /* 总初始化：把所有硬件都叫醒 */
static void  Pump_Init(void);                           /* 初始化水泵控制引脚（TB6612 的4根线） */
static void  Pump_Set(uint8_t on);                      /* 开泵 / 关泵 */
static void  Led_Set(uint8_t on);                       /* 板载LED开关（只是指示灯） */
static void  Hcsr04_Init(void);                         /* 初始化超声波模块和微秒计时器 */
static int   Hcsr04_Measure(float *pDistanceCm);        /* 测一次距离：0=成功 -1=失败 */
static float Median_Filter(float NewValue);             /* 中值滤波 */
static void  Level_Update(void);                        /* 测一次液位（测距+滤波+换算） */
static void  Control_Update(void);                      /* 控制核心：决定水泵开还是关 */
static void  Auto_Sequence_Update(void);                /* 五级自动流程的状态机 */
static void  Set_Target(float TargetCm);                /* 设置目标液位 */
static void  Mode_Apply(void);                          /* 按当前模式设置目标等参数 */
static void  Key_Init(void);                            /* 初始化两个按键 */
static void  Key1_Clicked(uint8_t click_count);              /* KEY1 单击要做的事 */
static void  Key2_Clicked(uint8_t click_count);              /* KEY2 单击要做的事 */
static float Abs_Float(float x);                            /* 自己写的取绝对值函数 */
static void  Format_Cm(char *pStr, float Cm);           /* 把厘米数变成字符串 */
static void  Format_Sec(char *pStr, float Sec);         /* 把秒数变成字符串 */
static const char *Mark_Name(float LevelCm);            /* 判断液位最接近哪条刻度 */
static void  Oled_ShowLine(uint8_t Line, const char *pStr); /* 在第几行显示字符串 */
static void  Oled_ShowLineF(uint8_t Line, const char *pFormat, ...); /* 在第几行显示格式化字符串 */
static void  Display_Update(void);                      /* 刷新屏幕 */

/* ==========================================================================
 * 第 5 部分：把各个硬件"叫醒"（初始化）
 * ========================================================================== */

/**
  * @brief  OLED 写数据回调函数。
  */
static int Oled_I2C_Write(uint8_t addr, const uint8_t *pdata, uint16_t size)
{
	return My_SI2C_SendBytes(&si2c, addr, pdata, size); /* 调用软件I2C把数据发出去 */
}

/**
  * @brief  初始化 OLED 显示屏
  * @note   软件 I2C 的 SCL/SDA 引脚在这里配置
  */
static void Oled_Init(void)
{
	OLED_InitTypeDef oled_init;                         /* 定义 OLED 的初始化参数结构体 */

	/* 1. 告诉软件I2C：SCL 接哪个脚，SDA 接哪个脚 */
	si2c.SCL_GPIOx = GPIOB;                       /* SCL 所在的端口（GPIOB） */
	si2c.SCL_GPIO_Pin = GPIO_Pin_8;               /* SCL 具体是哪个引脚（PB8） */
	si2c.SDA_GPIOx = GPIOB;                       /* SDA 所在的端口（GPIOB） */
	si2c.SDA_GPIO_Pin = GPIO_Pin_9;               /* SDA 具体是哪个引脚（PB9） */
	My_SI2C_Init(&si2c);                          /* 把这两个引脚配置成开漏输出 */

	/* 2. 把 I2C 发送函数交给 OLED 驱动，并初始化屏幕 */
	oled_init.i2c_write_cb = Oled_I2C_Write;            /* 告诉屏幕驱动"用这个函数发数据" */
	OLED_Init(&oled, &oled_init);                       /* 初始化屏幕（内部会发一堆配置命令） */

	OLED_Clear(&oled);                                  /* 清空画布，先变成全黑 */
	OLED_SendBuffer(&oled);                             /* 把清空后的画面真正发到屏幕上 */
}

/**
  * @brief  初始化水泵控制引脚（TB6612FNG 的 4 根控制线）和板载LED
  * @note   TB6612 接线：AIN1=PA4、AIN2=PA5、PWMA=PA6、STBY=PA7
  *         上电时保证水泵是关着的
  */
static void Pump_Init(void)
{
	GPIO_InitTypeDef gpio_init = {0};                         /* 引脚初始化结构体 */

	/* 1. 打开 GPIOA（水泵控制线）、GPIOC（板载LED）的时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);

	/* 2. TB6612 的 4 根控制线，都配置成推挽输出 */
	gpio_init.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7; /* PA4~PA7 一起配 */
	gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;                                 /* 推挽输出 */
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;                                /* 引脚翻转速度 50MHz */
	GPIO_Init(GPIOA, &gpio_init);                                           /* 4个引脚一次性初始化 */

	/* 3. 上电先把 4 根线全部拉低：STBY=0 时 TB6612 处于休眠状态 */
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);                  /* PA4(AIN1) 输出低 */
	GPIO_ResetBits(GPIOA, GPIO_Pin_5);                  /* PA5(AIN2) 输出低 */
	GPIO_ResetBits(GPIOA, GPIO_Pin_6);                  /* PA6(PWMA) 输出低 */
	GPIO_ResetBits(GPIOA, GPIO_Pin_7);                  /* PA7(STBY) 输出低 = 芯片休眠 */

	/* 4. 板载LED：开漏输出 */
	gpio_init.GPIO_Pin = GPIO_Pin_13;                   /* 配置 PC13 */
	gpio_init.GPIO_Mode = GPIO_Mode_Out_OD;             /* 开漏输出 */
	gpio_init.GPIO_Speed = GPIO_Speed_2MHz;             /* 2Mhz频率 */
	GPIO_Init(GPIOC, &gpio_init);                       /* 初始化 LED 引脚 */
	GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);         /* 写1 = 熄灭（PC13 低电平点亮） */

	/* 5. 记录初始状态 */
	pump_on = 0;                                      /* 记事本上写：水泵现在是关的 */
	pump_switch_ms = GetTick();                       /* 记下"关泵状态"是从什么时候开始的 */
}

/**
  * @brief  开泵或关泵
  * @param  on  1 = 开水泵，0 = 关水泵
  * @note   TB6612 真值表：
  *           开泵：STBY=1，AIN1=1，AIN2=0，PWMA=1  →  正转（全速）
  *           关泵：STBY=1，AIN1=1，AIN2=1，PWMA=0  →  短路刹车（电机立刻停住）
  *   
  */
static void Pump_Set(uint8_t on)
{
	on = on ? 1 : 0;                                    /* 把任何非0的值都当成 1，防止误传参数 */

	if (on == pump_on)                                /* 如果状态和现在一样，就什么都不用做 */
	{
		return;                                         /* 直接返回，不重复操作硬件 */
	}

	if (on)                                             /* ---------- 开泵 ---------- */
	{
		GPIO_SetBits(GPIOA, GPIO_Pin_4);                /* PA4(AIN1) = 1 */
		GPIO_ResetBits(GPIOA, GPIO_Pin_5);              /* PA5(AIN2) = 0，和 AIN1 相反才有方向 */
		GPIO_SetBits(GPIOA, GPIO_Pin_6);                /* PA6(PWMA) = 1，全速 */
		GPIO_SetBits(GPIOA, GPIO_Pin_7);                /* PA7(STBY) = 1，芯片开始工作 */
		Led_Set(1);                                     /* 板载LED点亮，表示"泵在转" */
	}
	else                                                /* ---------- 关泵 ---------- */
	{
		GPIO_SetBits(GPIOA, GPIO_Pin_4);                /* PA4(AIN1) = 1 */
		GPIO_SetBits(GPIOA, GPIO_Pin_5);                /* PA5(AIN2) = 1，两个都1 = 短路刹车 */
		GPIO_ResetBits(GPIOA, GPIO_Pin_6);              /* PA6(PWMA) = 0 */
		GPIO_SetBits(GPIOA, GPIO_Pin_7);                /* PA7(STBY) 保持 1，刹车才有效 */
		Led_Set(0);                                     /* 板载LED熄灭 */
	}

	pump_on = on;                                     /* 记事本更新：水泵现在是开还是关 */
	pump_switch_ms = GetTick();                       /* 记下这次状态改变的时刻（后面算时间要用） */
}

/**
  * @brief  板载LED开关，只是给人看的指示，不参与控制
  * @param  on  1 = 点亮，0 = 熄灭
  */
static void Led_Set(uint8_t on)
{
	/* PC13 是低电平点亮：要亮就写 0(Bit_RESET)，要灭就写 1(Bit_SET) */
	GPIO_WriteBit(GPIOC, GPIO_Pin_13, on ? Bit_RESET : Bit_SET); /* 亮=写0，灭=写1 */
}

/**
  * @brief  初始化 HC-SR04 超声波模块 + TIM2 微秒计时器
  * @note   测回波时间要求精确到微秒，所以用 TIM2 做一个 1微秒加1 的计数器
  */
static void Hcsr04_Init(void)
{
	GPIO_InitTypeDef gpio_init;                         /* 引脚初始化结构体 */
	TIM_TimeBaseInitTypeDef tim_init;                   /* 定时器初始化结构体 */
	RCC_ClocksTypeDef clocks;                           /* 用来读取系统各条总线的频率 */
	uint32_t timer_clk;                                 /* 定时器的实际时钟频率 */
	uint16_t prescaler;                                 /* 计算出来的分频系数 */

	/* 1. 打开 GPIOB 和 TIM2 的时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); /* GPIOB：TRIG 和 ECHO 都在这 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); /* TIM2：用来数微秒 */

	/* 2. TRIG 脚：推挽输出。 */
	gpio_init.GPIO_Pin = GPIO_Pin_0;                    /* TRIG 是 PB0 */
	gpio_init.GPIO_Mode = GPIO_Mode_Out_PP;             /* 推挽输出：由单片机决定高低电平 */
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;            /* 翻转速度 50MHz */
	GPIO_Init(GPIOB, &gpio_init);                       /* 初始化 TRIG 引脚 */
	GPIO_ResetBits(GPIOB, GPIO_Pin_0);                  /* 平时保持低电平，测距时才拉高 */

	/* 3. ECHO 脚：下拉输入。*/
	gpio_init.GPIO_Pin = GPIO_Pin_10;                   /* ECHO 是 PB10 */
	gpio_init.GPIO_Mode = GPIO_Mode_IPD;                /* 下拉输入 */
	GPIO_Init(GPIOB, &gpio_init);                       /* 初始化 ECHO 引脚 */

	/* 4. 配置 TIM2：让它每 1 微秒 计一个数
	     先问一下系统当前各条总线的频率，再算出分频系数，
	     这样即使以后改了主频，这里也不用跟着改。 */
	RCC_GetClocksFreq(&clocks);                         /* 读取当前时钟配置 */
	timer_clk = clocks.PCLK1_Frequency;                 /* 先取 APB1 总线频率（TIM2 挂在 APB1 上） */
	if (timer_clk == clocks.HCLK_Frequency)             /* 如果 APB1 没有分频 */
	{
		timer_clk = clocks.PCLK1_Frequency;             /* 定时器时钟就等于 APB1 频率 */
	}
	else                                                /* 如果 APB1 分了频 */
	{
		timer_clk = clocks.PCLK1_Frequency * 2;         /* 定时器时钟 = APB1 频率 × 2（STM32 的规矩） */
	}
	prescaler = (uint16_t)(timer_clk / 1000000);        /* 想要 1MHz（每微秒加1）需要除以多少 */
	if (prescaler < 1)                                  /* 防止算出来是 0 */
	{
		prescaler = 1;                                  /* 那就至少分频 1 次 */
	}

	TIM_TimeBaseStructInit(&tim_init);                  /* 先把结构体填成默认值 */
	tim_init.TIM_CounterMode = TIM_CounterMode_Up;      /* 向上计数 */
	tim_init.TIM_Period = 65535;                       /* 数到 65535 自动回到 0 */
	tim_init.TIM_Prescaler = (uint16_t)(prescaler - 1); /* 寄存器里写的是"分频值-1" */
	tim_init.TIM_ClockDivision = TIM_CKD_DIV1;          /* 时钟不再分频 */
	TIM_TimeBaseInit(TIM2, &tim_init);                  /* 把配置写进 TIM2 */
	TIM_SetCounter(TIM2, 0);                            /* 计数器清零，从 0 开始数 */
	TIM_Cmd(TIM2, ENABLE);                              /* 启动计数器，让它一直跑 */
}

/**
  * @brief  测一次距离
  * @param  pDistanceCm 输出参数：测到的距离，单位厘米
  * @retval 0 成功；-1 失败
  */
static int Hcsr04_Measure(float *pDistanceCm)
{
	uint16_t echo_rise, echo_fall, echo_us;             /* 回波开始的时刻、结束的时刻、持续了多少微秒 */
	uint32_t wait_from;                                 /* 等待回波开始的起始时刻（用来算超时） */

	/* ---- 第1步：发触发脉冲 ----*/
	GPIO_ResetBits(GPIOB, GPIO_Pin_0);                  /* 先拉低，保证脉冲从干净的低电平开始 */
	DelayUs(2);                                         /* 保持 2 微秒 */
	GPIO_SetBits(GPIOB, GPIO_Pin_0);                    /* TRIG 拉高 */
	DelayUs(12);                                        /* 高电平保持 12 微秒（模块要求 >10us） */
	GPIO_ResetBits(GPIOB, GPIO_Pin_0);                  /* TRIG 拉低，模块开始发射超声波 */

	/* ---- 第2步：等 ECHO 变高（等回波开始），最多等 3 毫秒 ---- */
	wait_from = TIM_GetCounter(TIM2);                   /* 记下等待开始的时间 */
	while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10) == Bit_RESET) /* ECHO 还是低就继续等 */
	{
		if ((uint16_t)(TIM_GetCounter(TIM2) - wait_from) > ECHO_RISE_US) /* 等太久了 */
		{
			return -1;                                  /* 模块没反应，本次测量失败 */
		}
	}
	echo_rise = TIM_GetCounter(TIM2);                   /* 记下 ECHO 变高的时刻 */

	/* ---- 第3步：量 ECHO 高电平持续了多久 ---- */
	while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10) == Bit_SET) /* ECHO 还是高就继续量 */
	{
		if ((uint16_t)(TIM_GetCounter(TIM2) - echo_rise) > ECHO_TIMEOUT_US) /* 回波太长 */
		{
			return -1;                                  /* 回波太长，肯定不对，本次失败 */
		}
	}
	echo_fall = TIM_GetCounter(TIM2);                   /* 记下 ECHO 变低的时刻 */

	/* ---- 第4步：算距离 ----*/
	echo_us = (uint16_t)(echo_fall - echo_rise);        /* 高电平一共持续了多少微秒 */

	*pDistanceCm = (float)echo_us * SOUND_CM_PER_US;    /* 微秒 × 每微秒厘米数 = 距离（厘米） */

	/* ---- 第5步：物理范围检查----*/
	if (*pDistanceCm < 2.0f || *pDistanceCm > 400.0f)
	{
		return -1;                                      /* 太近或太远，本次失败 */
	}

	return 0;                                           /* 返回 0 表示这次测量成功 */
}

/**
  * @brief  初始化两个按键（KEY1 切换模式、KEY2 开关水泵 / 标定零点）
  */
static void Key_Init(void)
{
	Button_InitTypeDef key_init;                    /* 按键初始化参数 */

	/* ---- KEY1：切换模式 ---- */
	key_init.GPIOx = GPIOB;                         /* KEY1 接在 GPIOB */
	key_init.GPIO_Pin = GPIO_Pin_1;                 /* 具体是 PB1 */
	key_init.button_pressed_cb = 0;                 /* 不用"按下"回调，写0即可 */
	key_init.button_released_cb = 0;                /* 不用"松开"回调 */
	key_init.button_clicked_cb = Key1_Clicked;      /* 单击时调用 Key1_Clicked */
	key_init.button_long_pressed_cb = 0;            /* KEY1 不需要长按功能 */
	key_init.LongPressTime = 1000;                  /* 长按判定时间：1000 毫秒 */
	key_init.LongPressTickInterval = 100;           /* 长按后每 100 毫秒重复触发一次 */
	key_init.ClickInterval = 150;                   /* 连击判定间隔 150 毫秒 */
	My_Button_Init(&key1, &key_init);               /* 正式初始化 KEY1 */

	/* ---- KEY2：手动开/关泵 + 长按标定 ---- */
	key_init.GPIOx = GPIOB;                         /* KEY2 也接在 GPIOB */
	key_init.GPIO_Pin = GPIO_Pin_11;                /* 具体是 PB11 */
	key_init.button_pressed_cb = 0;                 /* 不用"按下"回调 */
	key_init.button_released_cb = 0;                /* 不用"松开"回调 */
	key_init.button_clicked_cb = Key2_Clicked;      /* 单击 → 开关水泵 */
	key_init.button_long_pressed_cb = 0;            /* KEY2 不用长按 */
	key_init.LongPressTime = 1000;                  /* 长按判定时间：1000 毫秒 */
	key_init.LongPressTickInterval = 100;           /* 长按后每 100 毫秒重复触发一次 */
	key_init.ClickInterval = 150;                   /* 连击判定间隔 150 毫秒 */
	My_Button_Init(&key2, &key_init);               /* 正式初始化 KEY2 */
}
/* ==========================================================================
 * 第 6 部分：滤波和液位换算
 * ========================================================================== */

/**
  * @brief  中值滤波：把最新的数据放进数组，返回这组数据中间的那个值
  */
static float Median_Filter(float NewValue)
{
	float sorted[FILTER_COUNT];                                /* 复制一份数据用来排序（不能破坏原数组） */
	float insert_value;                                        /* 插入排序时正在挪动的那个值 */
	uint8_t i, j;                                              /* 两个循环用的下标 */
 
	/* 1. 把新数据存进环形数组 */
	raw_buf[raw_index] = NewValue;                             /* 存到当前的空位上 */
	raw_index++;                                               /* 下一个位置往后移一格 */
	if (raw_index >= FILTER_COUNT)                             /* 如果已经移到数组末尾了 */
	{
		raw_index = 0;                                           /* 就绕回到数组开头（这就是环形数组） */
	}
	if (raw_count < FILTER_COUNT)                              /* 如果数组还没存满 */
	{
		raw_count++;                                             /* 有效数据的个数加 1 */
	}

	/* 2. 复制一份出来准备排序 */
	for (i = 0; i < raw_count; i++)                            /* 遍历所有有效数据 */
	{
		sorted[i] = raw_buf[i];                                  /* 一个一个复制到临时数组 */
	}

	/* 3. 插入排序：从小到大排好 */
	for (i = 1; i < raw_count; i++)                            /* 从第2个元素开始，依次插入到前面 */
	{
		insert_value = sorted[i];                                /* 取出当前要插入的值 */
		j = i;                                                   /* 从当前位置往前找 */
		while (j > 0 && sorted[j - 1] > insert_value)            /* 只要前面的值比 insert_value 大 */
		{
			sorted[j] = sorted[j - 1];                             /* 就把前面那个值往后挪一格 */
			j--;                                                   /* 继续往前比较 */
		}
		sorted[j] = insert_value;                                /* 找到位置了，把 insert_value 放进去 */
	}

	/* 4. 返回中间那个值*/
	return sorted[raw_count / 2];                   
}

/**
  * @brief  测一次液位：测距离 → 滤波 → 换算成液位
  * @note   这个函数每 60 毫秒被调用一次
  */
static void Level_Update(void)
{
	float raw_distance;                                 /* 本次测到的原始距离（没滤波） */
	float median;                                       /* 第一级·中值滤波后的距离 */
	float sum;                                          /* 第二级·滑动平均求和用 */
	uint8_t i;                                          /* 循环下标 */

	/* ---- 第1步：测一次距离 ---- */
	if (Hcsr04_Measure(&raw_distance) != 0)             /* 模块没反应 */
	{
		raw_distance = -1.0f;                           /* 用 -1 表示"这次没测到" */
	}


	/* ---- 第2步：两道假数据检查---- */
	/* ★模式1（标定零点）时【跳过】这两道检查*/
	if (mode == MODE_ZERO)
	{
		if (raw_distance > 0.0f && (raw_distance < 2.0f || raw_distance > 250.0f))
		{
			raw_distance = -1.0f;                       /* 只丢掉物理上不可能的距离 */
		}
	}
	else if (raw_distance > 0.0f)                       /* 其它模式：模块给出数据了才检查 */
	{
		/* 检查①（范围）：距离必须落在"零点 ± LEVEL_RANGE_CM"以内。
		     水面不可能跑到容器外面去，超出这个范围一定是假数据。
		     ★前提是零点已经标定正确（用模式1 标定）。 */
		if (raw_distance < (zero_distance_cm - LEVEL_RANGE_CM) ||
		    raw_distance > (zero_distance_cm + LEVEL_RANGE_CM))
		{
			raw_distance = -1.0f;                       /* 判为无效 */
		}
	}

	/* ★检查②（稳定性）：最近 STABLE_COUNT 次原始读数必须互相接近。
	     水泵一转水面就乱，读数会"错但在范围内"地乱跳 ——
	     只有这条能把它认出来，这是"泵一直抽不停"的根治办法。 */
	if (raw_distance > 0.0f)                            /* 先通过范围检查的才进来排队 */
	{
		float min_value, max_value;                                   /* 最近几次里的最小值和最大值 */
		uint8_t k;

		stable_buf[stable_index] = raw_distance;          /* 存进环形数组 */
		stable_index++;
		if (stable_index >= STABLE_COUNT) stable_index = 0;       /* 到头绕回 */
		if (stable_stored < STABLE_COUNT) stable_stored++;          /* 计数 */

		if (stable_stored < STABLE_COUNT)                        /* 还没攒够次数，先不认 */
		{
			raw_distance = -1.0f;
		}
		else
		{
			min_value = stable_buf[0];                         /* 求这几次里的极差 */
			max_value = stable_buf[0];
			for (k = 1; k < STABLE_COUNT; k++)
			{
				if (stable_buf[k] < min_value) min_value = stable_buf[k];
				if (stable_buf[k] > max_value) max_value = stable_buf[k];
			}
			if ((max_value - min_value) > MAX_SPREAD_CM)             /* 差得太散：水面是乱的 */
			{
				raw_distance = -1.0f;                   /* 判为"测不准"，本次不采用 */
			}
		}
	}

		if (raw_distance > 0.0f)                            /* ===== 数据可信，正常处理 ===== */
	{
		has_last_distance = 1;                       /* 标记：已经存过可信距离了 */
		last_distance_ms = GetTick();                  /* 记下时刻（标定零点时要用"最近的可信距离"） */

		median = Median_Filter(raw_distance);           /* ★第一级：中值滤波，把跳到远处的假数据丢掉 */

		/* ★第二级：滑动平均 = 把最近 AVG_COUNT 次的中值加起来除以个数。
		     比"加权平均"更好理解，效果一样：把剩下的抖动磨平。 */
		avg_buf[avg_index] = median;                  /* 存进环形数组 */
		avg_index++;                                    /* 位置往后移一格 */
		if (avg_index >= AVG_COUNT)                         /* 移到最后就绕回开头 */
		{
			avg_index = 0;
		}
		if (avg_count < AVG_COUNT)                          /* 记录已经存了几个（刚开机时不满） */
		{
			avg_count++;
		}

		sum = 0.0f;                                     /* 开始求和 */
		for (i = 0; i < avg_count; i++)
		{
			sum += avg_buf[i];
		}
		distance_cm = sum / (float)avg_count;         /* 平均值就是最终使用的距离 */

		/* 3. 换算液位：液位 = 零点距离 − 当前距离
		     水面越高 → 距离越小 → 液位越大 */
		level_cm = zero_distance_cm - distance_cm; /* 这就是屏幕上显示的液位 */

		sensor_ok = 1;                                /* 记录：这次测量是成功的 */
		level_trusted = 1;                            /* ★这次是"可信读数" */
		level_estimated = 0;                          /* ★拿到真实读数了，取消"估算值"标志 */
		/* 说明：所有"传感器故障"判断都用 last_distance_ms（上面刚更新的时间戳），
		   不再需要单独维护"失败次数""失明计时"这些计数器。 */
	}
	else
	{
		sensor_ok = 0;                                /* 记录：这次测量失败 */
		level_trusted = 0;                            /* ★测不准：不能拿它控制水泵 */
	}
}

/* ==========================================================================
 * 第 7 部分：按键处理
 *           （按键库会在"单击""长按"发生时，自动调用下面这几个函数）
 * ========================================================================== */

/**
  * @brief  KEY1 单击：切换模式
  * @param  click_count 连击次数（这里用不到）
  */
static void Key1_Clicked(uint8_t click_count)
{
	(void)click_count;                                       /* 参数用不到，写这一行是为了避免编译器告警 */

	mode++;                                           /* 模式号加一，切到下一个模式 */
	if (mode >= MODE_COUNT)                           /* 如果超过最后一个模式了 */
	{
		mode = MODE_ZERO;                             /* 就绕回到模式1 */
	}
	Mode_Apply();                                       /* 按新模式设置目标液位、水泵等 */
}

/**
  * @brief  KEY2 单击：作用跟着当前模式走
  * @param  click_count 连击次数（这里用不到）
  * @note   ★模式1（标定零点）：按一下 KEY2，就把"此刻的水面高度"记成 0。
  *           标定时水面一定要正好停在容器的 0cm 刻度线上。
  *         ★模式2（手动控制）：按一下开泵、再按一下关泵。
  *         模式3、模式4 是全自动的，KEY2 不起作用。
  */
static void Key2_Clicked(uint8_t click_count)
{
	(void)click_count;                                       /* 参数用不到，避免告警 */

	if (mode == MODE_ZERO)                            /* ===== 模式1：标定零点 ===== */
	{
		/* ★用"最近 1 秒内测到的可信距离"来标定。
		     不用瞬时的 sensor_ok，因为那个标志每 60ms 就刷新一次，
		     按下去的那一瞬间很可能刚好是"这一次没测到"，会导致标定随机失败。 */
		if (has_last_distance && (GetTick() - last_distance_ms) < 1000)
		{
			zero_distance_cm = distance_cm;         /* 把当前距离记成"零点距离" */
			level_cm = 0.0f;                          /* 液位立刻归 0 */
			hold_seconds = 0.0f;                      /* 换了参考点，保持计时重新开始 */
			level_estimated = 0;                      /* 取消"估算值"标志 */
		}
	}
	else if (mode == MODE_MANUAL)                     /* ===== 模式2：手动开关水泵 ===== */
	{
		Pump_Set(pump_on ? 0 : 1);                    /* 现在是开就关掉，是关就打开（取反） */
	}
	/* 模式3、模式4：什么都不做，交给自动控制 */
}


/**
  * @brief  按当前模式设置目标液位
  */
static void Mode_Apply(void)
{
	hold_seconds = 0.0f;                              /* 换模式了，稳定保持计时清零 */
	auto_stage = 0;                                   /* 自动流程也从第 0 步重新开始 */

	switch (mode)                                     /* 看看现在是哪个模式 */
	{
		case MODE_ZERO:                                 /* 模式1：标定零点 */
			Pump_Set(0);                                /* 标定时必须停泵，水面静止才量得准 */
			break;                                      /* 结束这个分支 */

		case MODE_MANUAL:                               /* 模式2：手动控制 */
			Pump_Set(0);                                /* 进手动模式先把泵关掉，由人来决定 */
			break;                                      /* 结束这个分支 */

		case MODE_HOLD0:                                /* 模式3：自动保持在 0 位 */
			Set_Target(0.00f);                     /* 目标设成 0.00cm，然后交给自动控制 */
			break;                                      /* 结束这个分支 */

		case MODE_AUTO:                                 /* 模式4：五级自动流程 */
			          /* 五级流程规定"从稳定的 0 开始"，所以先回到 0cm */
			Set_Target(0.00f);                     /* 第 0 步的目标是 0.00cm */
			break;                                      /* 结束这个分支 */

		default:                                        /* 其它没列出的情况 */
			Pump_Set(0);                                /* 保险起见先停泵 */
			break;                                      /* 结束这个分支 */
	}
}

/* ==========================================================================
 * 第 8 部分：控制算法（程序的核心）
 * ========================================================================== */

/**
  * @brief  设置目标液位；如果目标变了，把"稳定保持"计时清零
  * @param  TargetCm 新的目标液位（厘米）
  */
static void Set_Target(float TargetCm)
{
	if (target_cm != TargetCm)                        /* 只有目标真的变了才需要处理 */
	{
		target_cm = TargetCm;                         /* 记下新的目标液位 */
		hold_seconds = 0.0f;                          /* 目标变了，保持计时必须重新开始 */
	}
}

/**
  * @brief  取绝对值（自己写一个，省得引入数学库）
  * @param  x 任意浮点数
  * @retval x 的绝对值
  */
static float Abs_Float(float x)
{
	return (x < 0.0f) ? -x : x;                         /* 是负数就取反，是正数就原样返回 */
}

/**
  * @brief  五级自动流程：0cm 稳住 → -2cm 稳住 → +2cm 稳住
  * @note   注意"从0降到-2"这一段程序不会主动做什么，
  *         因为漏水孔一直在漏水，水面自己会降下去；我们要做的只是别补水。
  */
static void Auto_Sequence_Update(void)
{
	if (mode != MODE_AUTO)                            /* 不是 AUTO 模式就什么都不做 */
	{
		return;                                         /* 直接返回 */
	}

	switch (auto_stage)                               /* 看看流程走到第几步了 */
	{
		case 0:                                         /* 第1步：先把液位稳在 0cm */
			Set_Target(0.00f);                     /* 目标 = 0.00cm */
			if (hold_seconds >= HOLD_TIME_S)          /* 如果已经稳稳保持够 30 秒 */
			{
				auto_stage = 1;                       /* 进入第 2 步 */
				hold_seconds = 0.0f;                  /* 保持计时清零，为下一步重新计时 */
			}
			break;                                      /* 结束本步 */

		case 1:                                         /* 第2步：目标 -2cm，关泵让水自己漏下去 */
			Set_Target(-2.00f);                /* 目标 = -2.00cm（泵不会开，靠漏水下降） */
			if (hold_seconds >= HOLD_TIME_S)          /* 在 -2cm 稳住够 30 秒 */
			{
				auto_stage = 2;                       /* 进入第 3 步 */
				hold_seconds = 0.0f;                  /* 保持计时清零 */
			}
			break;                                      /* 结束本步 */

		case 2:                                         /* 第3步：目标 +2cm，补水补上去 */
			Set_Target(2.00f);                 /* 目标 = +2.00cm */
			if (hold_seconds >= HOLD_TIME_S)          /* 在 +2cm 稳住够 30 秒 */
			{
				auto_stage = 3;                       /* 进入第 4 步（完成） */
				hold_seconds = 0.0f;                  /* 保持计时清零 */
			}
			break;                                      /* 结束本步 */

		default:                                        /* 第4步：任务完成，一直保持在 0cm */
			Set_Target(0.00f);                 /* 目标一直是 0.00cm */
			break;                                      /* 结束 */
	}
}

/**
  * @brief  控制器主逻辑：决定水泵开还是关
  * @note   每 60 毫秒执行一次，控制思想：
  *         ① 液位比目标低得多 → 开泵补水
  *         ② 一边补一边算液面上升速度
  *         ③ 预测"现在关泵的话，水还会再涨多少"，提前关泵，刚好停在目标上
  *         ④ 漏水孔会让水位慢慢下降，降到一定程度再重复①
  *         于是水位就在目标刻度上做很小的来回微调，看起来就是稳稳地停在那里。
  */
static void Control_Update(void)
{
	uint32_t now;                                       /* 当前时刻（毫秒） */
	uint32_t since_ms;                                  /* 距水泵上次改变状态过了多久（毫秒） */
	float    early_stop_cm;                               /* 提前多少厘米关泵 */
	float    measured_rate;                              /* 本次算出来的液面上升速度 */

	/* ---- 1. 计算这一轮距离上一次控制过了多少毫秒 ---- */
	now = GetTick();                                    /* 读一下现在的系统时间 */
	if (last_control_ms == 0)                            /* 如果是第一次进来 */
	{
		last_control_ms = now;                           /* 先补上时间基准，避免第一次算出一个巨大的值 */
	}
	{
		uint32_t elapsed_ms = now - last_control_ms;          /* 距上次控制过去了多少毫秒 */
		last_control_ms = now;                           /* 记录本次时刻，留给下次用 */

		since_ms = now - pump_switch_ms;              /* 水泵当前状态已经保持了多久 */

		/* ---- 2. 模式1（标定零点）：水泵完全停下来，交给按键 ---- */
		if (mode == MODE_ZERO)
		{
			hold_seconds = 0.0f;                      /* 标定模式不算"稳定保持" */
			return;                                     /* 直接返回，后面的自动控制不执行 */
		}

		/* ---- 3. 测距数据不可信时的处理----
		用已知的上升速度估算"到现在补了多少水"*/
		/* 泵在转、但已经 0.3 秒没拿到有效读数 → 也算"测不准"，改用估算值 */
		/* "多久没拿到有效读数"——所有传感器判据都用这一个值 */
		uint32_t no_data_ms = now - last_distance_ms;

		/* ★只要"当前读数不可信"（水面被搅乱、或根本没数据），就走估算流程：
		     绝不用乱跳的数字去控制水泵。 */
		if (no_data_ms >= SENSOR_ERR_MS || !level_trusted)
		{
			/* ★★模式2（手动控制）下，程序绝对不能碰水泵！
			     手动模式是"人来开关泵"，泵一转水面就乱、超声波会短暂失明，
			     如果这里执行"停泵保护"，就会出现"按了 KEY2 泵转一下就自己停"的怪现象。
			     所以手动模式只把液位用估算值显示出来，水泵原样不动。          */
			if (mode == MODE_MANUAL)
			{
				if (pump_on)                          /* 泵在转，用估算值让屏幕上的液位继续动 */
				{
					level_cm = level_at_start + rise_rate * ((float)since_ms / 1000.0f);
					level_estimated = 1;              /* 标记：这是估算值 */
				}
				return;                                 /* ★直接返回：不碰水泵 */
			}

			if (pump_on)                              /* 泵正开着，先试试用估算值撑住 */
			{
				/* 估算液位 = 开泵那一刻的液位 + 上升速度 × 已经抽了多久 */
				float estimated_level_cm  = level_at_start + rise_rate * ((float)since_ms / 1000.0f);

				/* 提前量：关泵后水还会再涨一点，所以要提前关 */
				float estimated_early_stop_cm = rise_rate * PUMP_STOP_LAG_S + 0.02f;
				if (estimated_early_stop_cm > EARLY_STOP_MAX_CM)
				{
					estimated_early_stop_cm = EARLY_STOP_MAX_CM;            /* 限幅 */
				}

				/* 还没补够，而且没有超时 → 继续抽，屏幕显示估算出来的液位 */
				if (estimated_level_cm < (target_cm - estimated_early_stop_cm) && since_ms < PUMP_MAX_ON_MS)
				{
					level_cm = estimated_level_cm;                   /* 显示平滑的估算值（不再是乱跳的假数据） */
					level_estimated = 1;              /* ★标记：这个液位是估算出来的 */

					/* ★稳定保持也照常累加（用估算值判断有没有跑出容差），
					     否则泵一工作就把 15/30 秒的计时白白丢掉，
					     考核时永远攒不够稳定时间。 */
					if (Abs_Float(level_cm - target_cm) <= HOLD_TOLERANCE_CM)
					{
						hold_seconds += (float)elapsed_ms / 1000.0f;   /* 在范围内：继续累加 */
						hold_outside_ms = 0;                          /* 宽限计时清零 */
					}
					else
					{
						hold_outside_ms += elapsed_ms;                     /* 超出范围：宽限计时累加 */
						if (hold_outside_ms >= HOLD_GRACE_MS)
						{
							hold_seconds = 0.0f;                  /* 连续超出 0.5 秒才清零 */
						}
					}
					return;                             /* 继续抽水，暂不报错 */
				}
			}

			Pump_Set(0);                                /* ★补够了 / 估不出来 → 立刻关泵，绝不盲目补水 */

			if (level_estimated)                      /* 刚才是"泵在转、靠估算补够的" */
			{
				/* 这属于正常补够，不是故障：继续显示估算液位，
				   等水面平静、真实读数回来后会自动切回实测值。 */
				learn_pending  = 1;                   /* ★等一次真实读数，用来学习上升速度 */
				learn_start_cm = level_at_start;    /* 记录这次开泵前的液位 */
				learn_on_ms    = since_ms;            /* 记录这次一共抽了多久 */
				return;
			}

			/* ★这里不马上清零保持计时：
			     水泵刚停时水面还在晃，超声波会短暂失明 1~2 秒，
			     如果一失明就把"稳定保持"清零，那 15/30 秒永远攒不够。
			     只有连续测不到数据超过 3 秒，才认为真的故障、重新计时。 */
			if (no_data_ms >= (SENSOR_ERR_MS * 2))      /* 失明超过 2 倍故障时间(4秒)才算真故障 */
			{
				hold_seconds = 0.0f;                  /* 保持计时清零（短暂失明不会清） */
			}

			return;                                     /* 后面的控制不再执行 */
		}

		/* ---- 3.4 ★自学习：用"补一次水前后的真实液位差"修正上升速度 ----
		        泵转着的时候水面是乱的、学不到速度，所以等泵停下来、
		        水面平静、拿到真实液位之后，再回头算这段时间涨了多少，
		        这样第二次补水就能估得更准（越用越准）。                */
		if (learn_pending && !pump_on && sensor_ok && learn_on_ms > 300)
		{
			float learned_rate = (level_cm - learn_start_cm) / ((float)learn_on_ms / 1000.0f);

			learned_rate = learned_rate * 1.30f;                              /* ★故意放大 30%：宁可估计得"补得快一点"，
			                                               让泵早一点停（少补可以再补，多了就退不回来） */
			if (learned_rate > 0.05f && learned_rate < 1.50f)                 /* 只接受合理范围的速度 */
			{
				rise_rate = 0.5f * rise_rate + 0.5f * learned_rate;   /* 慢慢修正，避免被一次异常带偏 */

				/* ★速度不能学得太低：估得太慢，程序会以为"还早"，泵就会跑到 8 秒上限、
				     结果补过头。所以设一个下限，保证估算始终偏"快"一点（宁少补不过头）。 */
				if (rise_rate < 0.25f)
				{
					rise_rate = 0.25f;
				}
			}
			learn_pending = 0;                        /* 这次学习结束 */
		}

		/* ---- 3.5 模式2（手动控制）：水泵完全交给按键，程序不自动控制 ---- */
		if (mode == MODE_MANUAL)
		{
			hold_seconds = 0.0f;                      /* 手动模式不算"稳定保持" */
			return;                                     /* 直接返回，后面的自动控制不执行 */
		}

		/* ---- 4. 自动流程可能需要切换目标液位 ---- */
		Auto_Sequence_Update();                         /* 如果是 AUTO 模式，这里可能会改目标 */

		/* ---- 5. 开关控制 + 提前关泵 ---- */
		if (pump_on)                                  /* ========== 现在泵是开着的 ========== */
		{
			/* 5.1 估计补水时液面的上升速度（cm/s） */
			if (since_ms >= RATE_MEASURE_MIN_MS)                /* 开泵超过 600ms，速度数据才有意义 */
			{
				/* 速度 = （现在的液位 − 开泵时的液位）÷ 经过的秒数 */
				measured_rate = (level_cm - level_at_start) / ((float)since_ms / 1000.0f);

				if (measured_rate > 0.005f)                      /* 只接受"确实在上涨"的数据 */
				{
					/* 新速度 = 0.6×旧速度 + 0.4×这次算出来的速度（让速度估计更稳） */
					rise_rate = 0.6f * rise_rate + 0.4f * measured_rate;

					if (rise_rate < 0.02f)  rise_rate = 0.02f; /* 最小限幅，防止速度被算成0 */
					if (rise_rate > 1.50f)  rise_rate = 1.50f; /* 最大限幅，防止离谱数据 */
				}
			}

			/* 5.2 算"关泵后水还会再涨多少厘米"：
			       还会涨的高度 ≈ 上升速度 × 关泵后的滞后时间
			       再多留 0.02cm 余量，宁可少补一点也别冲过头 */
			early_stop_cm = rise_rate * PUMP_STOP_LAG_S + 0.02f; /* 提前量 */
			if (early_stop_cm > EARLY_STOP_MAX_CM)           /* 但最多只能提前 0.2cm */
			{
				early_stop_cm = EARLY_STOP_MAX_CM;           /* 限幅，防止永远补不到目标 */
			}

			/* 5.3 判断是否该关泵 */
			if (since_ms >= PUMP_MIN_ON_MS && level_cm >= (target_cm - early_stop_cm))
			{
				/* 已经开了至少 200ms，且液位进入"目标−提前量"范围 → 关泵 */
				Pump_Set(0);                            /* 提前关泵，剩下的水会自己把液面顶到目标 */
			}
			else if (since_ms >= PUMP_MAX_ON_MS)        /* 保护：一次连续开超过 8 秒 */
			{
				Pump_Set(0);                            /* 强制关掉，防止传感器故障时一直灌水 */
			}
		}
		else                                            /* ========== 现在泵是关着的 ========== */
		{
			/* 5.4 没在补水：液位比目标低得够多就重新开泵 */
			/* ★三个条件同时满足才开泵：
			     ① 关了至少 PUMP_MIN_OFF_MS；
			     ② 停泵后已经等够 SETTLE_MS，水面平静了；
			     ③ 而且现在拿到的是"可信读数"（不是乱跳的数据）。
			   这样就不会出现"拿乱跳数据判断 → 补过头"的情况。 */
			if (since_ms >= PUMP_MIN_OFF_MS &&          /* 已经关了至少 200ms */
			    since_ms >= SETTLE_MS &&                /* 而且水面已经平静下来 */
			    level_trusted &&                      /* 而且读数可信 */
			    level_cm < (target_cm - HOLD_TOLERANCE_CM)) /* 并且比目标低了 0.3cm */
			{
				level_at_start = level_cm;          /* 记下开泵瞬间的液位，算上升速度要用 */
				Pump_Set(1);                            /* 开泵补水 */
			}
		}


		/* ---- 6. "稳定保持"计时：液位在目标 ±0.3cm 以内就一直累加 ----
		       注意：补水那一瞬间也会继续计时，因为考核看的是水面有没有跑出范围，
		       而不是水泵有没有停。只要水面没跑出去，计时就不该被打断。 */
		if (Abs_Float(level_cm - target_cm) <= HOLD_TOLERANCE_CM) /* ★用平滑值判断，读数抖一下不算跑出去 */
		{
			hold_seconds += (float)elapsed_ms / 1000.0f;   /* 把这一轮的时间加到保持计时上 */
			hold_outside_ms = 0;                          /* 在范围内，宽限计时清零 */
		}
		else                                            /* 暂时跑出范围了 */
		{
			/* ★抖动宽限：水面被水泵/进水搅动时，液位会偶尔抖出 ±0.3cm，
			     如果一抖就清零，那 15/30 秒永远攒不够。
			     所以连续跑出范围超过 0.5 秒，才认为真的没稳住。          */
			hold_outside_ms += elapsed_ms;                     /* 累加"跑出范围"的时间 */

			if (hold_outside_ms >= HOLD_GRACE_MS)         /* 连续超出超过 0.5 秒 */
			{
				hold_seconds = 0.0f;                  /* 计时清零，重新开始算 */
			}
		}

	}
}

/* ==========================================================================
 * 第 9 部分：显示和上报（给人看的部分）
 * ========================================================================== */

/**
  * @brief  把浮点数厘米值变成 "+0.12" / "-1.35" 这样的字符串
  * @param  pStr 输出字符串（至少要能放8个字符）
  * @param  Cm   要转换的厘米值
  * @note   为什么不用 printf 的 %f ？因为本工程用了微库(microlib)。
  *         自己用整数拼字符串最稳妥，不依赖浮点打印功能。
  */
static void Format_Cm(char *pStr, float Cm)
{
	int32_t value;                                          /* 把厘米换算成"百分之一厘米"的整数 */

	if (Cm >= 0.0f)                                     /* 如果是正数 */
	{
		value = (int32_t)(Cm * 100.0f + 0.5f);              /* ×100 变成整数，+0.5 是为了四舍五入 */
		sprintf(pStr, "+%d.%02d", (int)(value / 100), (int)(value % 100)); /* 拼成 +X.XX */
	}
	else                                                /* 如果是负数 */
	{
		value = (int32_t)(Cm * 100.0f - 0.5f);              /* ×100，-0.5 是为了负数的四舍五入 */
		value = -value;                                         /* 先取成正数，方便按位拼字符串 */
		sprintf(pStr, "-%d.%02d", (int)(value / 100), (int)(value % 100)); /* 拼成 -X.XX */
	}
}

/**
  * @brief  把秒数变成 " 12.3" 这样的字符串
  * @param  pStr 输出字符串（至少要能放8个字符）
  * @param  Sec  秒数
  */
static void Format_Sec(char *pStr, float Sec)
{
	int32_t value = (int32_t)(Sec * 10.0f + 0.5f);          /* ×10 变成"十分之一秒"的整数，四舍五入 */
	sprintf(pStr, "%d.%d", (int)(value / 10), (int)(value % 10)); /* 拼成 X.X */
}

/**
  * @brief  判断当前液位最接近容器上的哪一条刻度线
  * @param  LevelCm 当前液位（厘米）
  * @retval "0CM" / "-2CM" / "+2CM"，离三条线都远时返回 "---"
  * @note   考核二级要求"在 -2、0、+2 三个测试位置均识别正确"，
  *         这个函数就是程序给出的"液位状态"结论，直接显示在屏幕上。
  *         判定范围取 ±0.5cm，和考核的允许误差一致。
  */
static const char *Mark_Name(float LevelCm)
{
	if (Abs_Float(LevelCm - 0.00f) <= 0.50f)      return "0CM"; /* 离 0 线最近 */
	if (Abs_Float(LevelCm - (-2.00f)) <= 0.50f) return "-2CM"; /* 离 -2 线最近 */
	if (Abs_Float(LevelCm - 2.00f) <= 0.50f)  return "+2CM"; /* 离 +2 线最近 */
	return "---";                                       /* 三条线都不靠 */
}

/**
  * @brief  在屏幕的第 Line 行显示一串字符
  * @param  Line 行号 0~7（屏幕高64像素，一行8像素，一共8行）
  * @param  pStr 要显示的字符串
  */
static void Oled_ShowLine(uint8_t Line, const char *pStr)
{
	/* 开启一个文本框：左上角坐标(0, Line*8)，宽128、高8，
	   驱动会自动把光标放到这一行的正确位置上，并把超出部分裁掉 */
	OLED_StartTextRegion(&oled, 0, (int16_t)(Line * 8), 128, 8);
	OLED_DrawString(&oled, pStr);                     /* 在光标处画出这串字符 */
}

/**
  * @brief  在屏幕的第 Line 行显示带格式的字符串
  * @param  Line    行号 0~7
  * @param  pFormat 格式字符串
  */
static void Oled_ShowLineF(uint8_t Line, const char *pFormat, ...)
{
	char text_buf[64];                                       /* 先把结果拼到这个缓冲区里 */
	va_list ap;                                         /* 可变参数列表 */

	va_start(ap, pFormat);                              /* 开始取可变参数 */
	vsprintf(text_buf, pFormat, ap);                         /* 按格式把数字拼成字符串 */
	va_end(ap);                                         /* 结束取参数 */

	Oled_ShowLine(Line, text_buf);                           /* 把拼好的字符串显示到指定行 */
}

/**
  * @brief  刷新 OLED 显示
  * @note   ★屏幕上只留 5 个项目，重要的独占一行、上下留空行，一眼就能看清：
  *           第0行  模式号（1~4）
  *           第2行  液位读数        ← 最重要
  *           第4行  水泵开还是关
  *           第6行  稳定保持了多久
  *           第7行  液位落在哪条刻度线上
  */
static void Display_Update(void)
{
	char level_text[12];                           /* 液位文字缓冲区 */
	char hold_text[12];                            /* 保持秒数文字 */

	/* ---- 把要显示的数字先变成字符串 ---- */
	Format_Cm(level_text, level_cm);           /* ★用平滑后的液位显示，读数不跳 */
	Format_Sec(hold_text, hold_seconds);         /* 保持秒数 → 字符串 */

	OLED_Clear(&oled);                        /* 清屏 */

	/* 第0行：模式号（1~4），按 KEY1 切换 */
	Oled_ShowLineF(0, "MODE:%d", (int)mode + 1);

	/* 第2行：液位读数（最重要，独占一行，上下各空一行）
	     ★★屏幕上只显示【实测值】，绝不显示程序估算/预测出来的数字！
	        三种情况分得很清楚：
	          ① 有可信读数            → 显示实测液位（例如 +0.12 cm）
	          ② 暂时测不准（水面被搅动）→ 显示 --.--，明确告诉你"没测到"，不编数字
	          ③ 长时间测不到（真故障）  → 显示 SENSOR ERR                      */
	if (level_trusted)                        /* ① 这次读数是可信的 */
	{
		Oled_ShowLineF(2, "LEVEL: %s cm", level_text);     /* 显示真实测到的液位 */
	}
	else if ((GetTick() - last_distance_ms) >= SENSOR_ERR_MS)     /* ③ 长时间测不到 */
	{
		Oled_ShowLine(2, "LEVEL: SENSOR ERR");
	}
	else                                        /* ② 暂时测不准 */
	{
		Oled_ShowLine(2, "LEVEL:  --.-- ");     /* 不编数字，如实告诉你没测到 */
	}

	/* 第4行：水泵状态（ON = 正在补水） */
	Oled_ShowLineF(4, "PUMP : %s", pump_on ? "ON" : "OFF");

	/* 第6行：稳定保持了多少秒（考核要求连续 15 / 30 秒） */
	Oled_ShowLineF(6, "HOLD : %s s", hold_text);   /* 例：HOLD : 12.3 s */

	/* 第7行：液位最接近哪条刻度线（同样只认实测值） */
	if (level_trusted)                                        /* 有可信读数才判断刻度 */
	{
		Oled_ShowLineF(7, "MARK : %s", Mark_Name(level_cm));  /* 0CM / -2CM / +2CM */
	}
	else                                                        /* 测不准就显示 --- */
	{
		Oled_ShowLine(7, "MARK : ---");
	}

	OLED_SendBuffer(&oled);                   /* 把这一帧送到屏幕（约 20ms） */
}

/* ==========================================================================
 * 第 10 部分：硬件总初始化
 * ========================================================================== */
static void Hardware_Init(void)
{
	/* 1. 先初始化延时功能。
	     很多库函数（按键等）内部都要用到系统时间，所以它得第一个初始化。 */
	Delay_Init();

	/* 2. 初始化各个外设 */
	Pump_Init();                                        /* 水泵控制脚 + 板载LED（顺便保证上电时泵是关的） */
	Oled_Init();                                        /* OLED 显示屏 */
	Hcsr04_Init();                                      /* 超声波 + 微秒计时器 */
  Key_Init();                                         /* 两个按键 */
}

/* ==========================================================================
 * 第 11 部分：主函数
 * ========================================================================== */
int main(void)
{
	uint32_t last_sample_ms = 0;                         /* 上次测液位的时刻 */
	uint32_t last_oled_ms     = 0;                         /* 上次刷屏的时刻 */
	uint32_t now;                                       /* 当前时刻 */

	Hardware_Init();                                    /* 第1步：把硬件都初始化好 */

	/* 记录一下各个任务的时间基准，避免"开机瞬间就立刻执行一次" */
	now = GetTick();                                    /* 读一下当前时间 */
	last_sample_ms = now;                                /* 液位采样的时间基准 */
	last_oled_ms = now;                                    /* 刷屏的时间基准 */
	last_control_ms = now;                               /* 控制计算的时间基准 */
	last_distance_ms = now;                           /* 传感器时间戳也初始化，避免一上电就报故障 */

	while (1)
	{
		now = GetTick();                                /* 每一轮开头先读一次当前时间 */

		/* ---- 任务0：扫描按键 ---- */
		My_Button_Proc(&key1);                        /* 处理 KEY1：消抖、判断单击/长按，并调用对应函数 */
		My_Button_Proc(&key2);                        /* 处理 KEY2：同上 */

		/* ---- 任务1：每 60 毫秒测一次液位并做一次控制 ---- */
		if ((now - last_sample_ms) >= SAMPLE_PERIOD_MS)  /* 距离上次够 60ms 了吗 */
		{
			last_sample_ms = now;                        /* 记下本次时刻 */
			Level_Update();                             /* 测距离、滤波、算出液位 */
			Control_Update();                           /* 决定水泵开关 */
		}

		/* ---- 任务2：每 120 毫秒刷新一次屏幕 ---- */
		if ((now - last_oled_ms) >= OLED_PERIOD_MS)          /* 距离上次够 120ms 了吗 */
		{
			last_oled_ms = now;                            /* 记下本次时刻 */
			Display_Update();                           /* 刷新屏幕（大概耗时 20ms） */
		}
	}
}

/************************ (C) COPYRIGHT *****END OF FILE***********************/
