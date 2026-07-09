# <font color=blue>*光电机器人*</font>
# <font color=red>*神，尼，CR7*</font>
## 一、简介
| 底板材料 | 电机           | 感知模块  |      主控      |
| -------- | -------------- | ------- | :------------: |
| 亚克力板   | 42闭环步进电机 | 颜色传感器 | STM32F0407VGT6 |


| 开发工具              | 电机           | 感知模块  |      主控      |
| --------------------- | -------------- | ------- | :------------: |
| keil5/vscode EIDE插件 | 42闭环步进电机 | TCS34725 | STM32F0407VGT6 |
| cubeMX                | 5个5V小舵机  1个大舵机|         |                |

引脚作用图
| 引脚              | 作用           |
| --------------------- | -------------- | 
| PA13 (SYS_JTMS-SWDIO) | 烧录 | 
| PA14 (SYS_JTCK-SWCLK) | 烧录 |
| USB OTG 全速外设 | USB |
| PA11 USB_OTG_FS_DM | USB 差分负数据线 |
| PA12 USB_OTG_FS_DP | USB 差分正数据线 |
| PA10 USB_OTG_FS_ID | USB 识别引脚，区分主机 / 从机 |
| PA9 USB_OTG_FS_VBUS | USB 供电检测 |
| PB6 PB7 | I2C1_SCL  I2C1_SDA 陀螺仪|
| PB10 PB11| I2C2_SCL I2C2_SDA OLED|
| PA8 PA9  | I2C3_SCL I2C3_SDA 颜色传感器|
| PD5_TX PD6_RX | USART2 |
| PD8_TX PD9_RX | USART3 |
| PC6_TX PC7_RX | USART6 循迹 |
| PA0-WKUP | TIM5_CH1，舵机1 |
| PA1 | TIM5_CH2  舵机2 |
| PA2 | TIM5_CH3  舵机3 |
| PA3 | TIM5_CH4  舵机4 |
| PE5 | TIM9_CH1  舵机5 |
| PE6 | TIM9_CH2  舵机6 |
|PA7/DIR, PA5/STP, PC4/DIR, PA6/STP| 步进电机 |
|PD11/DIR, PD12/STP|丝杆滑台|















 


 



 

















