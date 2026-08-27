# yaw_4310 固件问题清单与修复记录

云台 yaw 轴无刷电机驱动固件的代码审查结果与修复状态。

> **修复状态（2026-08-09）**：P0 全部 4 条、P1 全部 9 条、P2 中 6 条已修复。
> 详见第七节「修复记录」，上机实测结果见第八节，**闭环上线实测见第九节**。
> **控制闭环已打开并实测通过**：串口收底盘指令 → 位置环跟随，
> 跟随误差 ≤0.7°，运动峰值电流 26 mA，回传帧折算到 [−π, π] 后读数与指令直接对齐。
> 硬件信息已对照 `04.相关资料/01.原理图pdf/foc电机驱动_2026-04-13.pdf` 核实，
> 其中电流采样极性一条已被实测推翻并更正（见 8.2），
> 编码器方向一条同样被实测推翻（见 9.1）。

## 一、工程概况

| 项目 | 实际情况 |
|---|---|
| 芯片 | STM32F103C8（64KB Flash / 20KB RAM，**无 FPU**） |
| 驱动板 | new_foc (DRV8313)，`02.嵌入式/01.硬件PCB/06.foc控制板` |
| 栅极驱动 | TI DRV8313PWPR 三路半桥，EN1/EN2/EN3 **硬件接死使能**，无软件使能线 |
| 电流采样 | 2× INA240A2（增益 50 V/V）+ 10mΩ 分流电阻，REF 接 1.65V 基准 |
| 编码器 | AS5600 磁编码器，软件 I2C（PB10=SCL / PB11=SDA），4.7k 上拉 |
| PWM | TIM1 CH1/2/3（PA8/PA9/PA10），中心对齐，ARR=1440 → 25kHz |
| 通信 | USART2（PA2/PA3），RX 用 DMA1_CH6 循环 + 软件帧同步，TX 用 DMA1_CH7 |
| CAN | TJA1050 收发器，PB8=RX / PB9=TX。驱动写了但 `can_init()` 从未调用 |
| 操作系统 | **无**。目录名叫 `freertos`，但 `Middlewares/` 是空的，裸机轮询 |

代码与 `07.pitch_4310` 同源，仅改了 PID 参数和控制目标值。

### 原理图核实的关键事实

修复过程中对照原理图确认了几件之前只能猜测的事：

1. **只有两路电流采样，测的是 A 相和 C 相**（不是 A/B）。U6 经 R80 测 `M0_OUT1`，U7 经 R81 测 `M0_OUT3`。原代码按 A/B 相做 Clarke 变换，公式是错的。
2. **PA4/PA5 在原理图上并未接到电流采样**，而原代码把 ADC 配成 4 通道并把 PA4/PA5 设为模拟模式，后两个通道采的是浮空噪声。
3. **PA6/PB5 是空脚**，`E_EN_*` / `M_EN_*` 是旧板遗留代码。DRV8313 的 EN 引脚硬件接死，**软件无法断使能**，唯一的停机手段就是关 PWM 输出。
4. **INA240A2 增益 50 V/V、分流电阻 10mΩ** —— 代码里 `amp_gain=50`、`shunt_resistor=0.01` 是对的。
5. **I2C 上拉 4.7k**（R6/R7）—— 这就是无延时软件 I2C 还能勉强工作的原因。

### 当前行为结论

**上电后不会响应底盘主控发来的数据。** 串口帧收到了、也解析了，但控制调用是注释掉的。唯一会驱动电机的动作是 `Motor_init()` 里约 2.4 秒的上电对齐，结束后停机，转子进入自由状态。

修复后这一点**没有改变** —— 详见第七节末尾的说明。

---

## 二、P0 — 必须先修

### P0-1　`delay_us()` 关闭了 SysTick，全工程时间基准失效

`Drivers/SYSTEM/delay/delay.c:8-20` 是「借用 SysTick 做精确延时」的写法：改写 `LOAD`、清 `VAL`、等 `COUNTFLAG`，**最后一行 `SysTick->CTRL &= ~(1<<0)` 把计数器彻底停掉**。而 `HAL_Delay` 又被重定向到 `delay_ms`（`delay.c:49-52`），所以 HAL 永远不会把它重新打开。

`Motor_init()` 里最后一次 `delay_ms(300)` 返回之后，`SysTick->VAL` 就**永久冻结成一个常数**。而 FOC 全部靠读 `SysTick->VAL` 算时间差：

| 位置 | 冻结后的实际结果 |
|---|---|
| `Foc/Pid.c:6-8` | `now - prev == 0` → 命中 `Ts <= 0` 兜底 → `Ts = 1e-3` |
| `Foc/Lowpass.c:24-29` | 走 else 分支算出 1.86s → 命中 `Ts > 0.5` 兜底 → `Ts = 1e-3` |
| `AS5600/as5600.c:60-65`（`Get_Velocity_L`） | 同上 → `1e-3` |
| `AS5600/as5600.c:84-104`（`Get_Velocity_H`） | 同上 → `1e-3` |
| `Foc/Motor.c:86-93`（`velocityOpenloop`） | 同上 → `1e-3` |

**整个固件的 Ts 被硬编码成 1ms，与真实循环周期完全脱钩。** 由此推导：

- 速度反馈是错的。`vel = angle_diff / Ts`，真实周期若是 3ms，速度读数被系统性放大 3 倍。
- PID 的 I 项和 D 项不可信 —— 这大概就是现在只敢用纯 P（`I=D=0`）的原因。
- `velocityOpenloop(3)` 的实际转速远低于 3 rad/s。
- **在此之前调 PID 参数是白费功夫。**

后遗症：SysTick 中断停了 → `HAL_IncTick()` 不再执行 → `HAL_GetTick()` 冻结。目前恰好没事（`HAL_ADCEx_Calibration_Start` 在第一次 `delay_ms` 之前就跑完了），但只要以后启用 CAN 或任何带 timeout 的 HAL 调用，就会死等到永远。

**修法**：拿一个空闲定时器（TIM2 或 TIM3）配成 1µs 计数、ARR=0xFFFF 自由运行，实现 `uint32_t micros(void)`，所有 Ts 从它取；`delay_us` 改成基于同一个定时器的忙等；SysTick 还给 HAL。这是后面一切工作的前提。

### P0-2　`data_change()` 里的控制调用被全部注释掉

`Foc/Data.c:76-82`：

```c
//		PositionCloseloop(0);
//		Position_VelocityCloseloop(0.5+(joy_num*0.4));
//		Position_VelocityCloseloop(0.5+yaw_pid_num);
		/* Safe default: alignment is performed once in Motor_init(), then the
		 * motor remains stopped until the closed-loop command is validated ... */
```

对应的 pitch 版本 `07.pitch_4310/Drivers/BSP/Foc/Data.c:80` 这一行是 `velocityOpenloop(3);`。

那段英文注释与工程里其余全中文的注释风格不一致，且 `Data.c` 与 `pwm.c` 的修改时间是 2026-08-08，明显晚于 `main.c`（2026-05-26）和 `FOC.c`（2026-03-18）—— 是后来改的，不是原作者的代码。

要让它响应底盘数据，最小改动是放开第 79 行。但请先修完 P0-1。

### P0-3　软件 I2C 从不检查 ACK，编码器掉线会飞车

`SYSTEM/iic.c` 里 `i2c_receive_ack()` 有返回值，但在 `i2c_read_register()`（`iic.c:109-138`）和 `i2c_write_register()`（`iic.c:89-107`）的**所有调用点都被丢弃**。

后果：编码器排线松动、供电跌落、磁铁失磁时，`as5600_read_angal()` 会返回全 0 或全 1 的垃圾值，而代码毫无察觉，继续拿这个假角度算电角度并输出 PWM。云台会直接失控。

**修法**：让 `i2c_read_register` 通过出参返回成功标志；`as5600_read_angal` 失败时置一个全局 `encoder_fault`；主循环检测到就立刻 `Motor_stop()` 并点错误灯。这是安全性问题，不是代码整洁问题。

### P0-4　启动文件用的是 F101 的，TIM1 和 CAN 的中断向量全是 0

`stm32f103.uvprojx:389` 引用的是 `startup_stm32f101xb.s`，但芯片是 F103C8。

F101 是「基本型」，没有 TIM1 高级定时器、没有 CAN/USB。对照 `startup_stm32f101xb.s:96-106`：

```
DCD     ADC1_IRQHandler            ; ADC1        ← F103 这里是 ADC1_2
DCD     0                          ; Reserved    ← F103: USB_HP_CAN1_TX
DCD     0                          ; Reserved    ← F103: USB_LP_CAN1_RX0
DCD     0                          ; Reserved    ← F103: CAN1_RX1
DCD     0                          ; Reserved    ← F103: CAN1_SCE
DCD     EXTI9_5_IRQHandler         ; EXTI Line 9..5
DCD     0                          ; Reserved    ← F103: TIM1_BRK
DCD     0                          ; Reserved    ← F103: TIM1_UP
DCD     0                          ; Reserved    ← F103: TIM1_TRG_COM
DCD     0                          ; Reserved    ← F103: TIM1_CC
DCD     TIM2_IRQHandler            ; TIM2
```

两边在这两处的空位数量刚好都是 4 个，所以从 `EXTI9_5` 往后的 DMA / USART / TIM2-4 / I2C / SPI 向量索引全都是对的 —— 这就是现在能正常跑的原因。

但这是个地雷：

- **TIM1 的四个中断向量都是 0。** 而「用 TIM1 update 中断跑定频 FOC 环」正是 P0-1 最理想的修法 —— 一旦启用就会跳到地址 0，直接 HardFault。
- **CAN 的四个中断向量都是 0。** 这解释了为什么 `can_receive_data()` 写成了轮询 FIFO 的形式。
- `ADC1_2_IRQHandler` 这个符号在 F101 启动文件里不存在，想用 ADC 中断也得先换文件。

**修法**：换成 `startup_stm32f103xb.s`（`Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/arm/` 下应该有）。这是一行工程配置的事，但必须在动 P0-1 之前做掉。

---

## 三、P1 — 功能性错误

### P1-1　SysTick 时基换算常量全错

即使 SysTick 没被关掉，这些换算也是错的：

| 位置 | 现状 | 问题 |
|---|---|---|
| `as5600.c:61,63` / `Motor.c:88,90` / `Lowpass.c:25,27` | `/9*1e-6` | 假设 SysTick 走 HCLK/8 = 9MHz |
| `as5600.c:99` | `Ts = delta / (SystemCoreClock/8)` | 同上假设 |
| `as5600.c:63` / `Motor.c:90` / `Lowpass.c:27` | `0xFFFFFF - now + prev` | 假设 LOAD = 0xFFFFFF |
| `Pid.c:7` | `(now - prev) * 1e-6f` | 既没除频率，也没处理递减方向 |

实际上 CMSIS 的 `SysTick_Config()`（`HAL_InitTick` 内部调用）设置 `CLKSOURCE = 1`，即 **SysTick 走 72MHz 处理器时钟，不是 /8**；`LOAD = 71999`（1ms），不是 `0xFFFFFF`。

所以分频系数差 8 倍、溢出补偿差三个数量级。另外 `SysTick->VAL` 只有 1ms 量程，本来也测不了超过 1ms 的间隔 —— 而软件 I2C 读一次 AS5600 就要几百 µs，速度环周期很可能就在 1ms 附近。

修 P0-1 时这些代码整体重写即可，不用逐条改。

### P1-2　`Pid.c` 的 Ts 计算方向反了

`Foc/Pid.c:6-7`：

```c
unsigned long timestamp_now = SysTick->VAL;
float Ts = (timestamp_now - pid->timestamp_prev) * 1e-6f;
```

SysTick 是**递减**计数器，正常情况下 `now < prev`，无符号相减会回绕成一个巨大的正数，乘 `1e-6` 后基本都 `> 0.5`，于是每次都命中兜底。其他几处（`Lowpass.c` / `as5600.c` / `Motor.c`）都做了 `if(now < prev)` 的方向判断，只有 PID 这里漏了。同一个工程里两种写法并存。

### P1-3　位置环 limit 远超实际可用电压，等效于 bang-bang 控制

`main.c:36-38` 原本设：

```c
PID_Pos_Set(0.8,  0.0, 0.0, 62.8);
PID_Vel_Set(0.01, 0.0, 0.0, 60);
PID_Cur_Set(0.5,  0,   0.0, 6);
```

**关键在于每个环的输出被谁消费，单位才能确定**（我第一版审查把这点说得过于笼统，这里更正）：

| 调用路径 | PID_Pos 输出单位 | PID_Vel 输出单位 |
|---|---|---|
| `PositionCloseloop()` → `setPhaseVoltage()` | **伏特** | — |
| `Position_VelocityCloseloop()` → `VelocityCloseloop()` | **rad/s**（速度给定） | 伏特 |
| `VelocityCloseloop()` → `setPhaseVoltage()` | — | **伏特** |

而 `FOC.c` 把 `Uout` 钳到 ±0.577，所以实际最大相电压只有 `0.577 × 11.7 ≈ 6.75V`。

于是：

- **`PID_Vel` 的 limit=60 是明确的错误** —— 它总是输出伏特，而 60V 是可达上限的 9 倍。速度环几乎一进入就饱和，退化成 bang-bang。
- **`PID_Pos` 的 limit=62.8 要看拓扑**。原代码注释掉的调用是级联（`Position_VelocityCloseloop`），此时 62.8 是速度限幅（62.8 rad/s = 10 转/秒），**是合理的**；但若单独用 `PositionCloseloop`，62.8 就变成了荒谬的电压限幅。
- `PID_Cur` 的 limit=6 接近 6.75，基本合理。

**修法**：`PID_Vel` 和 `PID_Cur` 的 limit 改用 `FOC_voltage_limit()`（即 `0.577 × Vbus`）算出来；`PID_Pos` 保留 62.8 并注明它是级联下的速度限幅。

### P1-4　ADC 满量程系数写错，电流值被缩小一半；且相位搞错

`adc/adc.c:89-92`：

```c
float _readADCVoltage(uint16_t ch)
{
  return (float)ch*1.65/4096;
}
```

3.3V 基准的 12 位 ADC，满量程应该是 `ch * 3.3 / 4096`。1.65V 是 INA240 的 REF 电平，不是 ADC 的满量程 —— 写成 1.65 会让所有电压读数减半，进而让电流读数减半。`Current_calibrateOffsets()` 已经单独把静态偏置减掉了，这里不需要再补偿一次。

**对照原理图后，还发现更严重的问题**：

- 板上只有 **两路** INA240A2：U6 经 R80 测 `M0_OUT1`（**A 相**），U7 经 R81 测 `M0_OUT3`（**C 相**）。
- 而 `adc.c` 把 ADC 配成 **4 通道**（PA0/PA1/PA4/PA5），PA4/PA5 在原理图上根本没接采样电路 —— 后两个通道采的是浮空噪声。
- 代码把第二路当成 **B 相**做 Clarke 变换：`I_beta = _1_SQRT3*Ia + _2_SQRT3*Ib`。实际测的是 C 相，这个公式是错的。代入 `Ib = -Ia - Ic` 可得 A/C 相下应为 **`I_beta = -(Ia + 2*Ic)/sqrt(3)`** —— 注意两项**都是负号**，符号弄反会让电流环变成正反馈。
- `gain_b = volts_to_amps_ratio * -1` 的负号在原理图上找不到依据 —— 两路 INA240 的 IN+/IN− 接法一致，符号应该相同。

也就是说，**电流环从来就没有正确工作过**（幅值差 2 倍、相位算错、符号可疑）。这解释了为什么所有带电流环的调用都是注释状态。

### P1-5　软件 I2C 没有任何时序延时

`SYSTEM/iic.c:69-70`（以及 `i2c_start` / `i2c_stop` / `i2c_receive_byte` 里同样的写法）：

```c
I2C_SCL_SET();
I2C_SCL_RESET();
```

两句之间一个 NOP 都没有。72MHz 下 `HAL_GPIO_WritePin` 大约十几个周期，SCL 高电平只有约 200ns，远低于 AS5600 fast mode 要求的 0.6µs 最小高电平时间。

现在能读出数据，纯靠开漏输出 + 上拉电阻的 RC 上升沿把速率天然拖慢了。换个上拉阻值、加长排线、或者换一批芯片就可能读错。

另外 `SDA_IN()` / `SDA_OUT()` 宏（`iic.h:25-41`）每次收发一个字节都要调用 `HAL_GPIO_Init()` 重配引脚模式 —— 这个函数里有一大堆寄存器读改写和分支，在 `i2c_receive_byte` 的循环里调用代价极高。应该改成直接操作 `CRH` 寄存器，或者干脆保持开漏输出模式（开漏输出时输出 1 就能读回外部电平，根本不用切模式）。

### P1-6　一个控制周期要读好几次编码器

`_electricalAngle()`（`FOC.c:14-16`）→ `Get_Angel_Notrack()`（`as5600.c:141-144`）每次都走一遍完整的软件 I2C 事务。

于是：

- `PositionCloseloop()`（`Motor.c:113-118`）：`Get_Angel()` 一次 + `_electricalAngle()` 一次 = **2 次 I2C**
- `Position_VelocityCloseloop()`（`Motor.c:158-163`）：位置误差 1 次 → `VelocityCloseloop` 里 `Get_Velocity()` 1 次（内部还会调 `Get_Angel()`）→ `_electricalAngle()` 1 次 = **3~4 次 I2C**
- `Velocity_CurrentCloseloop()`：还要再加上 `Get_Current()` 里的 `_electricalAngle()`

**修法**：循环开头读一次原始 raw angle 缓存起来，机械角、连续角、电角度全部从这个缓存推算。这一条能直接把控制频率翻两三倍。

### P1-7　`setPhaseVoltage` 全是双精度软件浮点，且编译器开的是 `-O0`

F103 是 Cortex-M3，**没有 FPU**。从 map 文件能直接看到 `foc.o(i.setPhaseVoltage)` 引用了 `f2d`、`d2f`、`dadd`、`dmul`、`ddiv`、`sin`、`atan2`、`sqrt` —— 全是软件模拟的双精度运算。

具体问题：

- `FOC.c:40-41` 的 `sqrt(3)` 每个周期重算两次，应该提成常量 `1.7320508f`
- `sin()` 每周期调两次，`_normalizeAngle` 里的 `fmod()` 是双精度版本
- 这些都是 `double` 版本，参数是 `float` 还要先 `f2d` 转上去、结果再 `d2f` 转回来
- `setPhaseVoltage` 编译出来 **972 字节**代码（map 第 2812 行）

而 `stm32f103.uvprojx:316` 是 `<Optim>1</Optim>`，即 **Level 0（-O0，完全不优化）**。

**修法**（收益从大到小）：

1. 编译优化开到 `-O2`（Optim = 3）
2. `sqrt`/`sin`/`fmod`/`atan2` 全换成 `sqrtf`/`sinf`/`fmodf`/`atan2f`，所有浮点常量加 `f` 后缀
3. `sin` 换成 SimpleFOC 那种 200 点查表 + 线性插值
4. `sqrt(3)` 提成编译期常量

这几步下来 `setPhaseVoltage` 能快一个数量级。

### P1-8　`Get_Velocity_H()` 单位混乱，回绕修正是死代码

`as5600.c:107-116`：

```c
float angle_c = Get_Angel();          // 返回的是弧度
angle_diff = angle_c - CODERx->angle_prev;
if(angle_diff > 180.0f)  angle_diff -= 360.0f;   // 按「度」处理
else if(angle_diff < -180.0f) angle_diff += 360.0f;
```

`Get_Angel()` 返回弧度（`as5600.c:43` 的 `/cpr*_2PI`），弧度差永远超不过 180，这两个分支永远不成立。

而且 `Get_Angel_L()` 已经用 `full_rotation_offset` 做过圈数补偿了（`as5600.c:39`），本来就不该再做回绕。这段不会造成错误，但说明单位没理清，改代码时容易踩。

同文件里 `Get_Velocity_L`（用 `/9`）和 `Get_Velocity_H`（用 `SystemCoreClock/8`）两套换算并存，`Get_Velocity2()` 走 L 版本、`Get_Velocity()` 走 H 版本，也该统一。

### P1-9　串口 TX 用循环 DMA，上位机会收到撕裂的帧

`DMA/dma.c:25` 把 USART2 TX 的 DMA 配成了 `DMA_CIRCULAR`，配合 `main.c:45` 那一次 `HAL_UART_Transmit_DMA`，20 字节帧会被无限循环重发。这是个取巧的做法，代价是：

- **TX 占用率 100%**：20 字节 @115200 ≈ 1.7ms 一帧，帧与帧之间没有任何间隔
- **数据撕裂**：`data_change()` 在 DMA 正读取 `send_angal` 的同时改写它（`Data.c:70`），上位机会收到高字节是新值、低字节是旧值的帧。角度回传会有毛刺。
- 这也是 `data_print()` 必须被注释掉的原因 —— `fputc` 直写 `USART2->DR`（`uart1.c:19-21`），会和 DMA 抢寄存器

**修法**：改成单次 DMA，在 `HAL_UART_TxCpltCallback` 里用双缓冲切换后重新发起；或者降低回传频率，用一个标志位控制。

---

## 四、P2 — 隐患与整洁性

### P2-1　`E_EN_*` / `M_EN_*` 是旧板遗留代码，且本板无法软件断使能

`Foc/Motor.c:9-41` 定义了 `E_EN_init` / `M_EN_init` / `E_EN_open` / `M_EN_open`（PA6 和 PB5），但**全工程一次都没调用过**。

**对照原理图后可以确定**：PA6 和 PB5 在 new_foc (DRV8313) 这块板上是**空脚**，压根没连到栅极驱动。DRV8313 的 EN1/EN2/EN3 是硬件接死使能的。所以这四个函数是从别的板子（可能是带 DRV8323 的版本）抄过来的残留，应整体删除。

**但这带来一个真实的安全约束**：本板**没有软件使能线**，唯一能让电机断电的手段就是关掉 TIM1 的 PWM 输出（`Motor_stop()`）。这意味着任何故障保护都必须走 PWM 通道，没有硬件后备。

另外 `pwm_init()` 里 `pwm1_config.Pulse = arr/2`（`pwm.c:19`）把三相都预置成 50% 占空比。三相相同占空比时线电压为零，电机不会转，所以这个预置本身无害 —— 但仍应在配置完成后立即 Stop，顺序才干净。

### P2-2　motor2（TIM4）在本板上不存在

- `pwm.c:70`：TIM4 用 PB6 / PB7 / PB8 做 motor2 的 PWM
- `can.c:46,52`：CAN 用 PB8（RX）/ PB9（TX）

**原理图上只有一颗 DRV8313，只有 `M0_IN1/2/3`（PA8/PA9/PA10）一组驱动输入，没有第二个电机驱动。** PB6/PB7 是空脚。所以 `pwm_init()` 里初始化 TIM4 纯属多余，而且它把 PB8 配成 AF_PP —— 一旦启用 CAN，PB8 会被 CAN 的 AF_INPUT 覆盖（或反过来把 CAN_RX 破坏掉）。

**修法**：删掉 TIM4 那一整段和 `motor2_pwm_set()`。

### P2-2b　`can.c` 的引脚配置与原理图不符（新发现）

原理图上 TJA1050 接的是 **PB8 = CAN_RX / PB9 = CAN_TX**，而 STM32F103 的 CAN1 默认引脚是 PA11/PA12，要用 PB8/PB9 必须调用 `__HAL_AFIO_REMAP_CAN1_2()` 做重映射。

`can.c` 配置了 PB8/PB9 的 GPIO，**但从未调用重映射宏**。所以即使调用 `can_init()`，CAN 外设的信号仍然连在 PA11/PA12 上，PB8/PB9 上什么都没有 —— CAN 完全不工作。

由于 `can_init()` 目前是死代码，这条暂未修复，但启用 CAN 前必须补上重映射。

### P2-3　`can_send_data()` 里有阻塞死等 + printf

`CAN/can.c:68-74`：

```c
while(HAL_CAN_GetTxMailboxesFreeLevel(&can_handle) != 3);
uint8_t i = 0;
printf("...:\r\n");
for(i=0;i<len;i++) printf("%X",buf[i]);
```

`fputc` 是查询式阻塞（`uart1.c:19`），115200 波特率下发一帧调试信息就能把控制环拖死好几毫秒。而 `while` 死等在总线上没有其他节点应答时会长时间阻塞。

`can_receive_data()`（`can.c:77-93`）里有同样的 printf。这套代码在启用 CAN 之前必须先清理干净。

另外 `can.c:14-15` 关掉了 `AutoBusOff` 和 `AutoRetransmission` —— 前者意味着一旦进入 Bus-Off 状态就再也不会自动恢复。

### P2-4　`motor1_pwm_set()` 每次调用都重新 Start 三个通道

`PWM/pwm.c:78-87`：

```c
void motor1_pwm_set(uint16_t val1,uint16_t val2,uint16_t val3)
{
	HAL_TIM_PWM_Start(&pwm1_handle,TIM_CHANNEL_1);   // 每个周期都调
	HAL_TIM_PWM_Start(&pwm1_handle,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&pwm1_handle,TIM_CHANNEL_3);
	__HAL_TIM_SET_COMPARE(...);
}
```

`HAL_TIM_PWM_Start` 内部有状态检查、CCER 读改写、MOE 使能、计数器使能等一堆操作。控制环里高频调用纯属浪费。

**修法**：`Motor_init()` 里 Start 一次，`motor1_pwm_set` 只保留三条 `__HAL_TIM_SET_COMPARE`。

### P2-5　上电对齐动作对云台过于暴力

`Motor.c:50-66` 会用 2.5V 拖着转子正转一圈、反转一圈（500 × 2ms × 2 = 2 秒），再用 3V 定位到 `_3PI_2` 保持 300ms，全程约 2.4 秒。

云台装上枪管之后这个动作幅度很大。更要紧的是：如果转子被机械限位卡住，标定出来的 `zero_electric_angle` 就是错的，之后整个电角度全偏，闭环会直接飞车。

`Data.c:40` 那行注释掉的 `float zero_electric_angle = 5.337638f;` 说明作者曾经标定过固定值。**云台上更稳妥的做法**：标定一次写死到常量，上电时只做一次轻力矩自检确认编码器读数与预期一致，不一致就报错停机。

### P2-6　没有通信超时保护

`data_change()` 只是无条件解析 `uart_recv` 的当前内容。如果底盘主控断线或数据线松了，`uart_recv` 会一直保持最后一帧的内容，云台会**保持最后一个指令角度不动**，而不是回中或停机。

**修法**：`frame_sync()` 成功时打时间戳，主循环检查距上一帧的时间，超过 100ms 就 `Motor_stop()`。

### P2-7　没有输出斜率限制

`PID` 结构体里有 `output_ramp` 字段（`Pid.h:13`）但 `PID_operator()` 从未使用它。位置环从大误差恢复时 Uq 会阶跃到饱和值，机械冲击很大。建议实现 ramp 限制，或者至少在 `setPhaseVoltage` 前对 Uq 做一阶滤波。

### P2-8　死代码与笔误

| 位置 | 问题 |
|---|---|
| `FOC.c:28` | `_sqrt()` 在整个工程的源码里**没有定义**（`Drivers/` 全树和 HAL 头文件都查过）。奇怪的是链接通过了：map 文件里既没有 `_sqrt` 符号，也没有对它的引用；但同一个 `if(Ud)` 分支里的 `atan2` 确实被引用了，说明分支没有被优化掉。可能是 armcc 把它当成了某个内建函数。无论如何，所有调用点都传 `Ud=0`，这段运行时不会执行。建议直接改成 `sqrtf` 消除隐患 |
| `FOC.h:15` | `setPhaseVoltage_new()` 声明了但没有定义 |
| `FOC.c:39` | `sector = (angle_el / _PI_3) + 1`，若 `_normalizeAngle` 因浮点舍入返回恰好 `2π`，`sector` 会等于 7 落进 `default` 分支，三相全部输出 0，产生一个瞬时力矩缺口 |
| `Data.c:143-148` | `lowpass_filter_float()` 用 `static` 单例，注释已说明只能用于单变量，目前无人调用 |
| `Lowpass.h:14-19` | `HighPassFilter` 结构体定义了但从未使用 |
| `as5600.h:22` | `sliding_avg_filter()` 声明了但没有定义 |
| `LED.c` / `main.c:25,34` | `lde_init` / `lde1_open` 是 `led` 的笔误 |
| `as5600.c` 全文 | `angal` 是 `angle` 的笔误，`Get_Angel` 是 `Get_Angle` 的笔误 |
| `dma.c:6` | `dam_handle` 是 `dma_handle` 的笔误 |
| `key.c:19-30` | `key_scan()` 整个被注释掉，`key_init()` 配置了 PC13 但无人读取 |
| 工程配置 | OLED、NRF24L01、SPI 三个模块编进了固件但从未初始化，白占 Flash |
| 目录名 | 放在 `New_rtos版本/freertos/` 下但完全没有 RTOS |

### P2-9　`Motor.c` 全局变量缺少 `static`

`shaft_angle` 和 `open_loop_timestamp`（`Motor.c:7`）是文件内部使用的状态，暴露成了全局符号。`Data.c` 里一大堆全局变量同理。不影响功能，但容易误改。

---

## 五、建议的修复顺序

严格按这个顺序做，每一步都比下一步更基础：

| 步骤 | 内容 | 对应条目 |
|---|---|---|
| 1 | 换启动文件为 `startup_stm32f103xb.s` | P0-4 |
| 2 | 编译优化开到 `-O2` | P1-7 |
| 3 | 换时间基准：TIM2 自由运行 + `micros()`，`delay_us` 改用同一定时器，SysTick 还给 HAL | P0-1、P1-1、P1-2 |
| 4 | I2C 加时序延时 + ACK 检查，失败立刻停机；`SDA_IN/OUT` 改直接操作寄存器 | P0-3、P1-5 |
| 5 | 一个周期只读一次编码器，机械角/电角度共用缓存 | P1-6 |
| 6 | `setPhaseVoltage` 单精度化 + `sin` 查表 | P1-7 |
| 7 | 核对原理图，修正 ADC 满量程系数和三相增益符号 | P1-4 |
| 8 | `PID_Pos` 的 limit 改为 6.75，按「度」重新整定 P | P1-3 |
| 9 | 加通信超时保护 + 输出斜率限制 | P2-6、P2-7 |
| 10 | 最后才放开 `data_change()` 里的闭环调用，上台架整定 | P0-2 |

**第 3 步没做完之前，任何 PID 参数整定都是无效的** —— 因为 Ts 是假的，I 项和 D 项的行为跟纸面公式对不上。

## 六、上台架前的安全提醒

- 第一次通电务必**拆掉枪管配重**，并把限流电源设到 1~2A
- 先跑开环（`velocityOpenloop`）确认三相接线顺序和 `sensor_direction` 正负，再上闭环
- `pole_pairs = 14`（`Data.c:37`）和 `voltage_power_supply = 11.7`（`Data.c:38`）要与实际电机、实际电池电压核对
- 手边留好急停（断电开关），闭环飞车时 `Motor_stop()` 是来不及按的
- 本板**没有硬件使能线**（见 P2-1），所有保护都依赖 PWM 通道，没有后备手段 —— 这一点让急停开关更重要

---

## 七、修复记录（2026-08-09）

### 新增文件

| 文件 | 作用 |
|---|---|
| `Drivers/SYSTEM/systime/systime.c/.h` | TIM2 微秒时基。`micros()` / `millis()` / `micros_since()` / `systime_delta_s()` |

`systime.c` 已加入 Keil 工程（`SYSTEM` 组），头文件路径也已加入 `IncludePath`。

### 逐条修复内容

| 条目 | 修复方式 |
|---|---|
| **P0-1** | 全工程时间基准换成 TIM2（PSC=71 → 1MHz，ARR=0xFFFF，更新中断把计数扩展成 32 位）。`Pid.c` / `Lowpass.c` / `as5600.c` / `Motor.c` 全部改用 `systime_delta_s()`，不再读 `SysTick->VAL`。三处 `0xFFFFFF` 魔数随之消失。 |
| **P0-3** | `iic.c` 重写：加 `i2c_delay()` 半周期延时（目标 1~2µs，对应约 250~350kHz，满足 AS5600 fast mode 的最小高/低电平要求）；`i2c_receive_ack` 返回状态码；所有调用点检查返回值。延时循环用 `volatile` 计数，否则 `-O2` 会把它当死代码删掉。**具体总线速率需上示波器实测后调整迭代次数。** |
| **P0-4** | 启动文件换成 `startup_stm32f103xb.s`。原来用的是 F101 的启动文件，中断向量表缺 CAN/USB 等条目。 |
| **P1-1** | `delay_us` / `delay_ms` 改为基于 TIM2 忙等，**不再触碰 SysTick**。`delay_us` 分块处理超过 16 位量程的延时。 |
| **P1-2** | 删掉 `HAL_Delay` 的重定向。SysTick 现在正常运行，`HAL_GetTick()` 和所有带 timeout 的 HAL 调用恢复可用。 |
| **P1-3** | `PID_Vel` / `PID_Cur` 的 limit 改用 `FOC_voltage_limit()`（= `0.577 × Vbus` ≈ 6.75V）。`PID_Pos` 保留 62.8 并在 `main.c` 里注明它是级联拓扑下的**速度**限幅。三个环的单位关系已写成注释。 |
| **P1-4** | `_readADCVoltage` 的 1.65 改成 3.3。ADC 通道数 4→2（PA4/PA5 不再配成模拟输入）。Clarke 变换按 **A/C 相**重写：`I_beta = -(Ia/√3 + 2·Ic/√3)`。`gain_b` 的无依据负号去掉，变量改名 `gain_a`/`gain_c`、`current_a`/`current_c` 以反映实际相位。 |
| **P1-5** | 同 P0-3。另外去掉了每次读位都切换 SDA 方向的做法 —— SDA 常驻开漏输出，靠写 1 释放总线读回电平，省掉每比特两次 `HAL_GPIO_Init`（原来一次 16 位读要调 32 次）。 |
| **P1-6** | 新增 `as5600_update()`，一个控制周期只读一次编码器，且**单次 I2C 事务连读两字节**（原来分两次事务读 0x0C/0x0D，中间转子已经动了）。机械角、连续角、速度、电角度全部读缓存。级联环的 I2C 次数从 4~5 次降到 1 次。 |
| **P1-7** | `-O0` → `-O2`。`setPhaseVoltage` 全面单精度化（`sinf`/`fmodf`/常量加 `f` 后缀），`sqrt(3)` 提成编译期常量，`sin`/`cos` 换成 129 点四分之一波查表 + 线性插值（误差 ~7e-5，远优于编码器 0.088° 分辨率）。 |
| **P1-8** | `Get_Velocity_H` 里按「度」做的 ±180 回绕修正（对弧度值是死代码）删除。`Get_Velocity_L` / `Get_Velocity2` 两套并存的换算合并成一套。 |
| **P2-1** | `E_EN_*` / `M_EN_*` 四个函数删除，并在 `Motor.c` 注明本板无软件使能线。 |
| **P2-2** | `pwm_init()` 里 TIM4 那一整段和 `motor2_pwm_set()` 删除。 |
| **P2-4** | `motor1_pwm_set()` 只写 CCR 寄存器。通道 Start/Stop 拆成 `motor1_pwm_start()` / `motor1_pwm_stop()`，在初始化和停机时各调一次。 |
| **P2-6** | `frame_sync()` 成功组帧时记录 `last_frame_us`。`data_change()` 里超过 `COMMS_TIMEOUT_US`（100ms）就断输出。时间戳存**微秒**而非毫秒 —— `millis()` 的回卷点不是 2 的幂，跨回卷做无符号相减会得到巨大的假间隔，导致每约 71 分钟误触发一次断电；比较原始微秒则天然回卷正确。用 `s_motor_enabled` 状态位避免重复调用 HAL。 |
| **P2-7** | `Pid.c` 增加 `output_ramp` 斜率限制（`main.c` 里三个环都设了值），以及 `PID_Reset()` —— 停机时清零积分器和 `error_prev`，避免重新使能瞬间被陈旧状态踢一脚。积分钳位是原代码就有的，未改动。 |
| **P2-9** | `Motor.c` 的 `shaft_angle` / `open_loop_timestamp` 加 `static`。 |

**编码器故障保护**（原清单没列，但 P0-3 的必然结果）：`as5600.c` 增加 `encoder_fault` 标志，I2C 读失败时置位，`data_change()` 检测到就立刻断输出。原来编码器掉线时代码会拿着上一次的角度继续输出 PWM —— 这是最容易飞车的场景。

### 未修复的条目

| 条目 | 原因 |
|---|---|
| **P0-2**（闭环调用被注释） | **故意保留注释状态**。见下方说明。 |
| **P1-9**（TX 循环 DMA 撕裂） | 需要新增 DMA1_CH7 + USART2 中断服务程序才能改成单次 DMA 重发，属于本轮范围外的中断改造，且无硬件无法验证。 |
| **P2-2b**（CAN 引脚重映射缺失） | `can_init()` 是死代码，改了也无法验证。启用 CAN 前需补 `__HAL_AFIO_REMAP_CAN1_2()`。 |
| **P2-3**（CAN 阻塞 + printf） | 同上。 |
| **P2-5**（`key_init` 配置 PC13，原理图上 KEY_1 不在 PC13） | `key_scan()` 已被注释，无实际影响。 |
| 笔误类（`lde_`/`angal`/`dam_handle`） | 全工程一致的笔误，批量改名风险大于收益，且会让与 `07.pitch_4310` 的 diff 变得难读。 |

### 为什么闭环仍然关着

上游全部修好了，但**PID 参数必须重新整定**：原来的 P/I/D 是在「Ts 永久卡死在 1ms」的前提下调出来的，现在 Ts 变成真实值，同一组参数的实际行为完全不同（尤其 I 项和 D 项）。带着旧参数直接放开闭环，比修复前更危险 —— 修复前 Ts 假但恒定，现在 Ts 真但参数错配。

`Data.c` 的 `data_change()` 末尾留了三行注释和启用说明，按 开环 → 速度环 → 位置环 的顺序逐级整定：

```c
//	motor_set_enabled(1);
//	velocityOpenloop(3);              // 1. 验证接线和方向
//	VelocityCloseloop(3);             // 2. 整定 PID_Vel
//	Position_VelocityCloseloop(0.5f + yaw_pid_num);  // 3. 整定 PID_Pos
```

一次只放开一行，并且先把 `motor_set_enabled(1)` 打开。

### 待验证

代码改动尚未编译验证（本轮 Keil 构建因工具链调用受阻未能完成）。首次编译时重点检查：

- `systime.c` 是否正确加入了工程组和 `IncludePath`
- `TIM2_IRQHandler` 是否与 `stm32f1xx_it.c` 冲突（应该不会 —— 原文件只定义了内核异常）
- 新的 `i2c_*` 函数返回类型改变后，是否还有漏改的调用点

---

## 八、上机实测记录（2026-08-09）

工具链：Keil MDK（`UV4.exe`，Target = `Template`）→ OpenOCD 0.12.0 + DAPLink（CMSIS-DAP）→ USART2 @115200。
全部通过 embeddedskills 的 `keil` / `openocd` / `serial` 三个 skill 执行，配置见「工具链配置」小节。

自测代码放在被测模块自己的文件里（`as5600.c` / `adc.c`），不新建源文件，避免再次手改 `.uvprojx`。
由 `main.c` 顶部三个宏控制，正常构建时全部置 0：

```c
#define SELFTEST_ENCODER    0   /* I2C 计时 + 裕度扫描，不给电机通电 */
#define SELFTEST_POLARITY   0   /* 电流采样极性，会给绕组通直流 */
#define SKIP_MOTOR_INIT     0   /* 跳过上电对齐，转子全程不动 */
```

### 8.1 I2C 时序实测 —— 修正了 P1-5 的估算

`as5600_selftest()` 用 TIM2 微秒计数器自测，不需要示波器。

**实测标定**（72 MHz，-O2，4.7k 上拉）：

```
delay_ns ≈ 514 + 111.3 × ticks
```

每次迭代约 111 ns（8 个 CPU 周期），另有 514 ns 固定开销（函数调用 + `volatile` 局部变量的栈读写）。
**我原先估的是 4~6 周期、1~2 µs，实际偏低约 2 倍** —— `volatile` 强制每轮走内存，比按寄存器操作估的贵。

| tick 数 | 半位周期 | 单次 `as5600_update()` | I2C 环频上限 |
|---|---|---|---|
| 24（初版） | 3182 ns | 439 µs | 2277 Hz |
| **10（当前）** | **1626 ns** | **236 µs** | **4237 Hz** |

**裕度扫描**：24 档一路降到 1，每档 500 次读取，**全程零 NACK、零 glitch**。
即使在 625 ns 半位周期（约 800 kHz、AS5600 标称 fast mode 的两倍）下仍无错误。

**为什么定在 10 而不是更快**：1626 ns 半位周期约 307 kHz，仍在 AS5600 标称的 400 kHz 以内，
使这个设定同时有 datasheet 和实测两个依据；再快就只剩「这块板室温下测过一次」这一条支撑。
相对实测无错点仍有 2.6 倍余量。

**改代码时发现的约束**（原清单没有）：收发两个方向的 `t_LOW` 不对称 ——
`i2c_send_byte()` 的 SCL 低电平跨两次 `i2c_delay()`，而 `i2c_receive_byte()` 只跨一次。
所以**读取路径是瓶颈**，单次 `i2c_delay()` 必须自己超过 fast mode 要求的 1.3 µs 最小低电平时间。
1626 ns 满足；若按最初设想降到 8 ticks（1403 ns）就只剩 8% 余量。

编码器读数 `raw = 3130`、`encoder_fault = 0`，两轮测试完全一致。

### 8.2 电流采样极性实测 —— **推翻了 P1-4 的一处修改**

`current_polarity_test()` 用直流注入法，不需要示波器也不需要转动电机。

方法：把电压矢量静态停在已知电角度上，读两路电流的符号。转子静止时无反电动势，
电流分配只由占空比模式和三相电阻决定，**与转子位置无关**，因此不需要标定过的电角度零点。

条件：3S 电池、限流 1A、配重已拆、`u_test = 2.0 V`。

**第一轮（`gain_c` 为正，即我修改后的版本）**：

| | A 轴测试真值 | 实测 | C 轴测试真值 | 实测 | 判定 |
|---|---|---|---|---|---|
| A 路 | +365 mA | +364.7 | −178 mA | −174.8 | 正确 |
| C 路 | −182 mA | **+184.2** | +355 mA | **−355.5** | **反相** |

A 路两次都对，C 路两次都反 —— 是 C 路极性反，且幅值全对（比值 0.505 / 0.492，应为 0.5）。

**结论：原代码的 `gain_b = volts_to_amps_ratio * -1` 是对的。**
我在 P1-4 里依据原理图判断「两路 INA240 接法一致、负号无依据」而删掉它，是错误的 ——
原理图上 INA240 的 IN+/IN− 与分流电阻的连接关系我没有读对。已改回 `gain_c = -volts_to_amps_ratio`，
注释中的依据换成实测数据表。

**第二轮（改回负号后）全部通过**：

```
A axis: i_a = +361.4 mA, i_c = -182.3 mA   ratio -0.504
        Clarke 矢量 +0.3 deg,   命令 0 deg,     误差 +0.3 deg
C axis: i_c = +354.4 mA, i_a = -175.3 mA   ratio -0.495
        Clarke 矢量 -119.7 deg, 命令 240 deg,   误差 +0.3 deg
```

`clarke_check()` 用实测电流反算电流矢量方向并与命令电角度比较，**一次验证整条链路**：
ADC 满量程系数（3.3 V 而非 1.65 V）、两路增益的绝对值与符号、
A/C 相 Clarke 变换 `I_beta = -(Ia + 2·Ic)/√3`、以及 SVPWM 实际输出的矢量方向。
任何一环出错都不可能得到 0.3° 的误差。两个相隔 240° 的独立测试点给出**同一个** +0.3° 偏差，
说明这是两路增益的微小系统性失配，不是随机噪声。

**P1-4 的「电流环从未正确工作过」至此解除** —— 这是原清单里唯一一条此前只能标「存疑」的。

顺带确认的硬件参数：
- 两路静态偏置 1.6525 / 1.6527 V，与 1.65 V 基准吻合，INA240 和基准源工作正常
- 由 `i_a = Ud / R` 反推**相电阻约 5.5 Ω**（4310 云台电机的合理值）

### 8.3 新增的安全检查

`current_polarity_test()` 在通电**之前**检查两路静态偏置是否落在 1.30~2.00 V。
偏离说明运放没供电、REF 没接或 ADC 通道浮空 —— 这些故障通电也诊断不出来，
而且会让测试报出一个其实是「通道坏了」的假 INVERTED 结论。不通过就直接返回，不启动 PWM。

`polarity_report()` 把符号判定和幅值判定**分开**报告：幅值对而符号错 = 一路接反；
幅值错 = 增益或偏置问题。第一版把两者混在一行，第二个轴打印中断时丢掉了诊断信息。

### 8.4 工具链配置

之前 skill 未生效是因为缺配置文件（Keil 装在非标准路径），已补齐：

| 文件 | 内容 |
|---|---|
| `~/.claude/skills/keil/config.json` | `uv4_exe` → `C:\Users\35252\Desktop\stm32\UV4\UV4.exe` |
| `~/.claude/skills/openocd/config.json` | `exe` → winget 安装的 openocd 0.12.0 |
| `Projects/MDK-ARM/.embeddedskills/config.json` | `interface/cmsis-dap.cfg` + `target/stm32f1x.cfg` + COM3 |

构建/烧录/抓串口现在无需传参。另外 openocd skill 内部正确处理了中文路径的 Tcl 转义，
手动调用 `openocd -c "program <path>"` 时 `\U`、`\f` 会被 Tcl 当作转义符导致失败。

### 8.5 仍未验证的部分

| 项目 | 状态 |
|---|---|
| `zero_electric_angle` 标定复现性 | **未测**。当前构建 `SKIP_MOTOR_INIT = 1`，未执行上电对齐 |
| 开环旋转 / `sensor_direction` 正负 | **未测**。需要转子连续转动 |
| 真实控制环周期（含 FOC 计算） | **未测**。目前只知道 I2C 部分占 236 µs |
| P1-9 串口 TX 撕裂 | 未修。自测构建中直接关掉了遥测 DMA 规避 |
| P2-2b CAN 引脚重映射 | 未修，`can_init()` 仍是死代码 |

**闭环仍然关闭。** PID 参数必须在真实 Ts 下重新整定，理由见第七节末尾。



---

## 九、闭环上线记录（2026-08-09）

**闭环已打开并实测通过。** 第八节末尾"闭环仍然关闭"的结论到此作废。

### 9.1 编码器方向 —— 实测推翻了代码里的默认值

`sensor_direction` 原本写死 `+1`，是从 `07.pitch_4310` 抄来的，**这一轴从未验证过**。新增
`motor_direction_test()`（`Motor.c`）用开环扫描 4 个电周期，测转轴实际走向：

```
commanded 25.13 el.rad over 400 steps
shaft moved -1.8162 rad, expected +1.7952 rad
-> sensor_direction = -1 (encoder INVERTED vs phase order)
implied pole_pairs = 13.84 (declared 14)
-> pole_pairs confirmed
```

幅值对（13.84 vs 14），**方向反**。已改为 `sensor_direction = -1`。

这是本轮最关键的一条：`sensor_direction` 进 `_electricalAngle()`，符号错了换向角就反着走，
位置环变成**正反馈**——一闭环就飞车。静态测试（编码器读数、电流极性）全都测不出来，
必须让转子动起来才暴露。

### 9.2 闭环台架测试（无上位机）

矛盾：`data_change()` 靠 `COMMS_TIMEOUT_US` 保护，台架上没接底盘就永远不使能，
闭环根本没法验证。解决办法是新增 `closedloop_bench_test()`，用**内部设定值**闭环，
而不是为了测试去削弱保护逻辑（削弱了就有随固件发出去的风险）。

±0.3 rad 阶跃，四段：

| 指标 | 实测 |
|---|---|
| 稳态误差 | ±0.005 rad（0.3°） |
| 保持电流 | 1–12 mA |
| 阶跃峰值电流 | 18–26 mA |
| 过流跳闸 | 0 次 |
| `zero_electric_angle` | 2.0954（两次上电一致） |

误差收敛、无振荡，确认 `sensor_direction = -1` 正确。

### 9.3 串口指令链路 —— 端到端实测

底盘主控没接也能验证：`fake_chassis.py` 用 PC 冒充上位机按协议发帧。

**恒定指令**（100 Hz）：

```
     seq   loop  frames     cmd   angle     err     vel  Iq(mA)  state
  345270   455u    1014  +0.500  +0.502  -0.002   +0.08       1  ENERGISED
  370201   454u    2101  +0.500  +0.500  -0.000   -0.10       1  ENERGISED  +154 frm
```

转轴从 −3.435 rad 实际转到指令位置 +0.500 rad，稳态误差 0.1°，帧率与发送端一致（100 Hz）。

**正弦跟随**（±0.35 rad，8 s 周期）：

| 指标 | 实测 |
|---|---|
| 跟随误差 | ≤0.012 rad（0.7°） |
| 运动峰值电流 | 26 mA |
| 超调 / 振荡 | 无 |
| 帧率 | 96–99 帧/采样周期 |

### 9.4 电流保守化

用户要求"电流不要太激进，太激进发热高"。两层限制：

**主限制 —— `PID_Pos` 限幅（单环拓扑下即输出电压上限，当前 4.0 V，2026-08-11 从 2.5 V 上调）。**
低速时反电动势可忽略，相电流就是 `Uq/R`，按实测相电阻 5.5 Ω 算，堵转电流被钳在约 0.73 A。
云台大部分时间在保持位置而非高速转动，所以这个 case 才是发热的主要来源。
上调原因：2.5 V 时位置环在 50° 误差处即饱和，满杆 yaw 指令（6.4 rad/s）超出轴的跟随能力，
摇杆顶端行程出现"死区"（推到最左速度不再随杆变化）；拉大到 4.0 V 后饱和误差阈值提到 80°。
持续全速时铜耗约为 2.5 V 时的 2.6 倍，注意发热。

**兜底 —— 过流监管（`overcurrent_ok()`）。** 覆盖推理覆盖不到的情况：母线高于标称 11.7 V、
绕组比标定时更凉（阻值更低）、或真实故障（相间短路）。

| 参数 | 值 | 理由 |
|---|---|---|
| `OC_LIMIT_A` | 1.20 A | q 轴电流幅值上限 |
| `OC_TRIP_US` | 200 ms | 需**连续**超限才跳闸 |
| `OC_COOLDOWN_US` | 2 s | 跳闸后强制断电时间 |

用"连续超限"而不是单次采样：快速换向时的电流尖峰会合法地超过稳态值几百微秒，
单次采样跳闸会在正常大动作时误停。

实测保持 <12 mA、运动峰值 26 mA，距 1.2 A 跳闸线有 **46 倍余量**，铜耗 <0.005 W。
后续若嫌力矩不够，优先加 `PID_Pos` 限幅（当前 4.0 V），过流线不用动。

### 9.5 SWD 实时观测 —— 解决串口不够用

**问题**：本板只有 USART2（PA2/PA3）一路可用串口。接上底盘主控后这根线跑二进制协议，
`printf` 就不能用了——调试文本会污染链路，链路的帧又会淹没文本。
**也就是说，轴进入真实配置的那一刻，恰好是串口控制台失效的那一刻。**

**方案**：调试通道搬到 SWD。DAPLink 本来就插着用于烧录，读目标 RAM 不打扰 CPU 运行，
不额外占任何引脚。

`Data.c` 里定义 `g_yaw_monitor` 结构体，`data_change()` 每个控制周期刷新：

| 字段 | 含义 |
|---|---|
| `magic` | `0x59415731` ("YAW1")，校验地址是否读对 |
| `seq` | 控制周期计数，不涨说明主循环卡死 |
| `loop_us` | 实测控制周期 |
| `frames` | 收到的合法帧数，**判断底盘链路死活的直接证据** |
| `flags` | bit0 使能 / bit1 编码器故障 / bit2 通信超时 / bit3 过流锁定 |
| `cmd/angle/err/vel/iq` | 指令、实际角、误差、速度、q 轴电流 |

**所有退出路径都刷新**（包括提前 return 的故障分支）。只在正常路径刷新会产生误导：
编码器掉线或通信超时时数据会冻结在最后一组好值，调试器看到的是"一切正常在跟随"，
而实际上电机已经停了。`flags` 字段的意义就在于说明**为什么**停。

用法：

```powershell
python yaw_monitor.py                 # 持续观测
python yaw_monitor.py --samples 20    # 采 20 次
```

地址从 `.map` 自动解析（当前 `0x20000064`）。**重新编译后 RAM 布局可能变**，
脚本会用 magic 校验，不匹配时直接报错而不是打印一堆看着合理的垃圾数据。

### 9.6 实际控制周期

| 状态 | 周期 | 频率 |
|---|---|---|
| 仅通信+编码器（未使能） | 257 µs | 3891 Hz |
| 完整闭环（使能） | 455 µs | 2200 Hz |

差值约 200 µs 是 FOC 计算 + SVPWM + 两级 PID 的开销。这是第八节留的"真实环频未知"的答案。

### 9.7 新增工具

| 文件 | 用途 |
|---|---|
| `Projects/MDK-ARM/yaw_monitor.py` | SWD 实时观测，不占串口 |
| `Projects/MDK-ARM/fake_chassis.py` | PC 冒充底盘主控发帧，无需真上位机即可验证接收链路 |

### 9.8 构建宏（当前为正式构建）

```c
#define SELFTEST_ENCODER       0
#define SELFTEST_POLARITY      0
#define COMMISSION_DIRECTION   0   /* 改动相线/编码器磁铁后需重跑 */
#define SELFTEST_CLOSEDLOOP    0
#define SKIP_MOTOR_INIT        0   /* 正式构建必须为 0，否则换向没有零点 */
```

### 9.9 回传帧归一化 + 电流采样时机（2026-08-09 追加）

**回传值折算到 [−π, π]**

`data_change()` 原来发的是 `sensor_direction * Get_Angel_Notrack()`，范围 (−2π, 0]。
轴停在 `YAW_CMD_CENTER = 0.5` 时回传读 **−5.79**。

先说清楚这**不是** bug。底盘对回传值有两个消费者，两个都能吃下整圈偏移：

| 消费者 | 位置 | 为什么不受影响 |
|--------|------|----------------|
| 跟随 PID | `01.底盘主控/data.c:62` | 用的是 `normal_yaw = normalizeAngleRad(gimble_yaw)`，已折算 |
| 小陀螺速度变换 | `01.底盘主控/data.c:93` | 用原始 `gimble_yaw`，但 `gimbal_to_chassis_speed_compute()`（同文件 182-191）是纯旋转矩阵，只用 `cosf/sinf`，**2π 周期**，所以 `cosf(−6.29)` 与 `cosf(−0.007)` 在 float 精度内相同 |

改的真正理由是**可观测性**。本板没有 console（见 9.5），回传帧是外部唯一能直接看到的
东西。发 −5.79 时，任何读它的人（示波器 / `fake_chassis.py` / 人眼）都得先在脑子里
折一圈才知道这个数是什么意思，而"差一整圈的读数"和"轴真的跑偏了"长得一模一样。
新增 `normalize_angle_rad()`（镜像底盘的 `normalizeAngleRad`），回传变成 **+0.500**，
"云台居中了吗"变成一眼可判。

实测（`fake_chassis.py --show-reply`）：

```
恒定指令 cmd=0：       reply 稳定 +0.500，与 YAW_CMD_CENTER 完全一致
正弦 ±0.35 rad / 6 s： reply = cmd + 0.500，全程跟随误差 ≤0.01 rad
                       11536 帧解码，无丢帧
```

**电流采样移到早退路径之前**

`monitor_update()` 本来就刻意在 `data_change()` 的**每条**退出路径上调用，包括断电的
那几条 —— 目的就是避免"故障时监视块冻结在最后一组正常值"。但 `last_iq` 漏了：它原先
在 `overcurrent_ok()` 里赋值，而那个函数在编码器故障 / 失联 / 已锁定这三条早退路径上
根本不会被调用。

台架上看到的症状：

```
     seq   loop  frames     cmd   angle     err     vel  Iq(mA)  state
  430754   287u    6775  +0.000  +0.299  +0.000   +0.00     -18  COMMS_TIMEOUT
```

`COMMS_TIMEOUT`（输出级已断）旁边挂着 −18 mA，读起来像"断电的轴里还在流电流"。

拆出 `current_sample()`，在 `as5600_update()` 之后、所有互锁之前调用；
`overcurrent_ok()` 改为判断已采好的 `last_iq`。ADC 读取次数不变（仍是每周期一次）。
`closedloop_bench_test()` 的循环里同步补上，否则监督器会拿到上一轮的旧值。

修复后同一状态：

```
   23637   328u       0  +0.000  +0.155  +0.000   -0.01       3  COMMS_TIMEOUT
```

3 mA = ADC 零漂，符合断电事实。

**双通道并跑实测**（SWD 与串口同时工作，互不干扰）：

```
     seq   loop  frames     cmd   angle     err     vel  Iq(mA)  state
  258692   493u    3507  +0.740  +0.755  -0.015   -0.02     -20  ENERGISED
  269661   480u    4012  +0.520  +0.502  +0.018   +0.01      20  ENERGISED
  307881   493u    5766  +0.780  +0.785  -0.005   -0.20     -14  ENERGISED
```

闭环周期 480 µs（2080 Hz），跟随误差 ≤0.018 rad，|Iq| ≤23 mA，
`oc_trips = 0`，`g_fault_record.magic = 0`（无故障记录）。

**当前正式构建**：0 error / 0 warning，Code=17728 RO=844 RW=224 ZI=1752
→ Flash 18796 B / 64 KB，RAM 1976 B / 20 KB。
符号地址未变：`g_yaw_monitor = 0x20000064`，`g_fault_record = 0x200000e0`。

### 9.10 P1-9 回传帧撕裂 —— 实测复现并修复

这一条从代码审查起就挂在"未验证"清单里，本轮实测**确认存在**并修好了。
它之所以现在必须处理：回传帧是外部唯一能观测本轴的通道（见 9.5），
而底盘会把这个值直接喂给跟随 PID。

**机理**

`send_angal[20]` 同时是循环 TX DMA 的源缓冲和控制环改写的目标，两者之间没有任何同步：

- `dma.c:25` 配的是 `DMA_CIRCULAR`，`main.c:171` 启动后无间隙连续重发
- 115200 baud → DMA1_Channel7 每 **8.7 µs** 取一个字节，永不停顿
- `data_change()` 以 ~2100 Hz 改写字节 1/2

如果写入正好落在 DMA 取走字节 1 之后、取走字节 2 之前，接收端就会拿到
"上一采样的高字节 + 下一采样的低字节"——一个轴从未到过的角度。

**过零点最危险**。scaled 从 `0xFFFF`（−0.01）翻到 `0x0000`（0.00）时两个半边差异最大。

**实测复现**（新增 `fake_chassis.py --glitch-stats`，判据：帧间跳变 >0.20 rad
不可能是真实轴运动，±1.0 rad / 5 s 正弦反复穿越 0 点，40 s）：

```
decoded 23076 reply frames
largest step between consecutive replies: 2.560 rad
TORN FRAMES: 2 of 23076 (0.009%)
  t= 24.63s  -0.010 -> -2.560  (jump +2.550)
  t= 24.63s  -2.560 -> +0.000  (jump +2.560)
```

−2.560 rad = `0xFF00`，正是高字节 `0xFF`（来自 −0.01）配低字节 `0x00`（来自 0.00）。
概率只有万分之一，但 −2.56 rad = **147°**，底盘跟随 PID 吃到一帧这种误差就是一次力矩踢腿。

**修法：写入窗口判据（`reply_publish()`）**

不用双缓冲，而是让 CPU 只在 DMA 不会碰到这两个字节时才提交。
`CNDTR` 从 20 递减并自动重装，所以 DMA 下一个要取的字节下标是 `20 - CNDTR`。
只要该下标已经越过 2，写入就安全：两条 store 是纳秒级，DMA 要再次回到字节 1
至少还有 8.7 µs。

窗口关闭时**跳过**而不是等待。在这里自旋会让 2 kHz 的控制环停顿最多 26 µs；
跳过只损失一个周期的新鲜度（≈0.5 ms，其间轴移动远小于 1 mrad），
而且下一周期几乎必定成功——窗口 85% 时间开着，且环周期与帧周期不成谐波锁定，
连续两次踩空的概率约 2%。

顺带这个做法也防住了**成对撕裂**：两个字节在同一个开窗内写完，
接收端不可能看到来自不同控制周期的两半。

**修复后同条件复测**：

```
decoded 23070 reply frames
largest step between consecutive replies: 0.010 rad   (修复前 2.560)
no torn frames detected
```

0.010 rad 就是真实轴运动的帧间量级，说明已经没有任何非物理跳变。

**在线健康检查**：`g_yaw_monitor` 新增 `reply_skips` 字段，`yaw_monitor.py`
显示为 skip 百分比。实测稳定 **15%**，与理论值 3/20 = 15% 吻合：

```
     seq   loop  frames     cmd   angle     err     vel  Iq(mA)  state
  321839   464u    4486  +0.770  +0.759  +0.011   -0.17       8  ENERGISED  +5678 cyc, +250 frm, skip 15%
  327507   457u    4735  +0.310  +0.319  -0.009   -0.15     -24  ENERGISED  +5668 cyc, +249 frm, skip 15%
```

这个数字本身是诊断信息：接近 0% 说明 DMA 根本没跑（守卫永不触发，回传是死值），
接近 100% 说明窗口从不打开。闭环性能无退化（loop 457 µs、err ≤0.013 rad、|Iq| ≤24 mA）。

> **符号地址已变**（结构体加了一个字段）：
> `g_yaw_monitor = 0x20000068`，`g_fault_record = 0x200000e8`。
> `yaw_monitor.py` 会自己从 `.map` 解析，手工传 `--addr` 时注意更新。

### 9.11 仍未验证

- **接真实底盘主控**的联调。协议已按 `01.底盘主控/Drivers/BSP/DATA/data.c` 逐字段核对，
  用模拟帧验证通过，但两块板真实对接尚未做。
- **PID 增益仍是老固件继承值**，只是限幅改保守了。当前误差 0.7° 够用，
  但没有针对真实 Ts 系统整定过。
- **装配后的负载表现**。以上全部在**拆掉配重**的空载条件下测得，
  装上枪管后惯量增大，可能需要提高 `PID_Vel` 限幅。
- P2-2b（CAN 引脚重映射）—— CAN 本轮确认不需要，串口是唯一指令链路。

（P1-9 已于 9.10 实测复现并修复，从本清单移出。）
