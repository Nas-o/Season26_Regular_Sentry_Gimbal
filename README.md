<!--
 * @Author: Nas(1319621819@qq.com)
 * @Date: 2026-03-18 05:42:16
 * @LastEditors: Nas(1319621819@qq.com)
 * @LastEditTime: 2026-06-03 19:34:10
 * @FilePath: \Season26_Regular_Sentry_Gimbal\README.md
-->
# ADAM哨兵电控统一框架

***26赛季哨兵首次采用底盘-云台模式的双板通信，应用层使用 c++ 编写，能适应大多数对双板有需求的兵种，后续对于使用传统底盘（舵轮，麦轮，全向）和双yaw亦或单yaw云台，或者是不同遥控器，本代码均能实现快速上手调试。***

## 🛠️ 编码规范 

为了保持代码整洁，并与底层 C 语言硬件驱动进行清晰的区分，项目统一采用以下 C++ 命名规范：

### 📌 命名规范总览

| 配置项 | 命名风格 | 示例 | 规范说明 |
| :--- | :--- | :--- | :--- |
| **命名空间 (Namespace)** | 全小写蛇形 | `namespace robot::gimbal` | 替代 C 语言前缀，清晰划分底盘/云台/应用等模块。 |
| **类名 / 结构体** | 大驼峰 | `class SentryChassis;` | 必须与底层 C 语言的 `struct_t` 结构体进行严格区分。 |
| **成员函数 / 方法** | 小驼峰 | `setTargetAngle()`, `init()` | 保持现代 C++ 风格，与底层硬件驱动函数区分。 |
| **私有成员变量** | 小写蛇形 + 尾部下划线 | `target_yaw_`, `last_tick_` | 读写寄存器或写控制算法时，需一眼看出哪个是类私有变量。 |
| **局部变量 / 参数** | 小写蛇形 | `current_speed` | 简短、见名知意即可。 |
| **常量 (`constexpr`)** | 小写 k + 大驼峰 | `constexpr float kPi = 3.1415f;` | 强类型常量，替代 `#define`，带有类型检查且不污染全局。 |
| **宏定义 (仅用于编译开关)**| 全大写蛇形 | `#define USE_DEBUG_LOG` | 现代 C++ 建议**仅在条件编译时**使用 `#define`。 |
---

## 软件环境

- 配置路径：[VSCode+Ozone使用方法](.doc/ch/VSCode+Ozone使用方法.md)

>后续有进一步换成cmake的想法
