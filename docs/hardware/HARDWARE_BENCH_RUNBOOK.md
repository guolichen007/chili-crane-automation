# ADAM / 拉绳隔离台架手册

本版只提供框架与软件工具。任何接电操作必须由有权限人员按厂家说明和现场安全程序执行。
没有台架/设备证据前 LIVE_* 全部 NOT_RUN，不能操作真实行车、接触器或抓斗。

## 1. 当前确认与待确认

初始总输入 **24DI = ADAM-6052 8DI + ADAM-6251 16DI，包括遥控器控制输入**。
用户当前指令优先于 V1.1 最低8DI示例。24 是物理容量，不等同已确认语义点位。
模板列出可选语义，数量可能超过24；部分反馈可能没有或来自其他接口。
现场必须确认24个端子的完整表，以及哪些必要语义不能提供，不能复用同一通道伪造独立证据。

每点记录：device、0-based channel、端子/线号、语义、raw闭合/断开电平、
invert、常开/常闭、断线/断电表现、验收人/日期。
遥控器接收器就绪、方向、停止、模式/安全反馈及电气隔离仍 OPEN。
模式/安全/限位/故障不能为了给遥控器腾通道而静默省略。

| 功能 | 当前框架 |
|---|---|
| 6052 DI | 通道0..7，FC01，手册00001..00008 -> PDU offset0..7 |
| 6251 DI | 通道0..15，FC01，手册00001..00016 -> PDU offset0..15 |
| 6052 DO | 通道0..7，手册00017..00024 -> PDU offset16..23 |
| DO0/1 | Y左/右 |
| DO2/3 | Z上/下 |
| DO4/5 | G开/关 |
| DO6/7 | 保留；台架可单点测试，不赋生产功能 |

DO表是已接受的框架映射，不表示实物接线已验收。
6052 dry-contact closed-GND 逻辑0；6251 closed-GND 逻辑1。
每点 invert 必须实测，不能“所有ADAM都active-high”。湿接点接法需另查厂家手册。

## 2. 责任与遥控仲裁

原厂保留硬接线急停、限位、正反转互锁、制动，以及自动/遥控先断后合物理隔离。
我们的责任边界到中间继电器干接点；继电器之后的电机/接触器/制动由原厂确认。
ADAM 是远程I/O，不是安全PLC或实时安全控制器。

遥控DI只作观测和模式仲裁，不转发自动DO。
AUTO→REMOTE 或未知/陈旧/冲突：释放自动输出，作废排队命令。
REMOTE→AUTO：验证静止、六输出全OFF读回、mode_auto/safety_ok有效、无遥控冲突，
重新接受任务，禁止续跑遥控前旧动作。
safety_ok 与 e-stop 独立，缺少急停证据仍未知；软件不替代原始急停。

本版本 normalizer 没有静止/输出OFF的真实证据，不能宣布自动READY；
supervisor所有许可false，production使用mock，不启用无人动作。

## 3. 配置与只读验证

复制安装的 config 到场地目录，保留模板文件原样；填入已确认的host/unit、
每通道极性/映射、拉绳协议和标定，所有自动输出开关保持false。
拉绳必须用 /dev/serial/by-id；波特率、奇偶校验、slave、FC03/04、
0-based寄存器、数据类型、byte/word order、比例、方向、零点、偏移、范围全显式。
禁止地址/波特率扫描或用时间代替Y位移。

```bash
ros2 launch chili_crane_bringup bench_io.launch.py config_root:=/absolute/site/config
ros2 run chili_crane_hardware adam_di_test.py --model adam6052 --config /absolute/site/config/hardware/adam6052.template.yaml
ros2 run chili_crane_hardware adam_di_test.py --model adam6251 --config /absolute/site/config/hardware/adam6251.template.yaml
ros2 run chili_crane_hardware pull_wire_probe.py --config /absolute/site/config/hardware/pull_wire.template.yaml
```

bench_io仅只读，不启动写入器。记录24个raw输入、极性转换、通信失败/采样年龄、
计数、故障、拔线/重连结果。失败采样不得刷新有效证据时间。
拉绳标定未确认时探测只能报告raw，不能报告有效米制位置。

## 4. DO单路短脉冲的独立门控

只能在**与真实执行机构物理断开的隔离台架**做灯/表负载测试。
默认配置拒绝所有写入；该工具不接受规划器或授权话题。
先按厂家工具配置设备通信WDT和FSV，再实测以下证据：

- ADAM_FSV_CONFIGURED=true：8路安全输出全LOW/OFF；
- ADAM_WATCHDOG_CONFIGURED=true：超时参数已验证，当前工具要求0<t<=2秒；
- ADAM_SAFE_OUTPUT_VERIFIED=true：断线/终止/重启后的OFF实测；
- verification_evidence 记录设备序列号、固件、接线、日期、测试步骤和结果。

手册FSV勾选代表HIGH；OFF应逐路确认LOW，不能把“已配置”当作OFF。
看门狗被客户端TCP数据刷新；必须停止其他软件、工程工具及所有读循环，
确认独占后才允许单脉冲。不能以PC程序的finally代替硬件掉线关断。

```bash
ros2 run chili_crane_hardware adam_do_test.py \
  --config /absolute/isolated-bench/adam6052.yaml \
  --channel 0 --duration-ms 200 \
  --enable-physical-output --confirm-isolated-bench --confirm-exclusive-device-session
```

配置 physical_output_enabled=true、automatic_control_enabled=false 且全部验收门控成立才执行。
每次只允许1通道，1..1000ms；先全OFF并读回、单路ON读回，finally全OFF并读回。
SIGINT/SIGTERM触发有限清理；网络丢失/SIGKILL/掉电不能保证软件OFF。

同机合作客户端按IP/port/unit加锁；写入前创建持久故障锁存，成功全OFF读回后才清除。
程序被强杀或清理失败，读循环因锁存停止请求，避免掩盖WDT。
隔离恢复必须核实设备OFF，再通过带门控工具完成OFF读回；不要手工删除锁存绕过故障。
锁只能约束同机同身份客户端，不能约束其他PC、IP别名、不同unit或厂商工具，
因此外部独占与硬件FSV/WDT验证仍必需。恢复前绝不可重连生产负载。

## 5. 验证边界与来源

Windows协议单测只使用模拟传输；Humble CI只做build/test/mock。
原生Ubuntu、现场24DI接线、真实DO、拉绳精度/故障仍需独立精确SHA证据。

厂家资料：
[ADAM-6000 Ed12手册](https://advdownload.advantech.com/productfile/Downloadfile4/1-2B6FKTG/ADAM-6000_User_Manaul_Ed.12-FINAL.pdf)
（PDF第223页地址表，第86页WDT/FSV）；
[ADAM-6251快速手册](https://advdownload.advantech.com/productfile/Downloadfile2/1-UIS5FF/ADAM-6251%E5%BF%AB%E9%80%9F%E5%85%A5%E9%97%A8%E6%89%8B%E5%86%8C.pdf)；
[6052官方型号页](https://buy.advantech.com/Buy-Online/bymodel-ADAM-6052.htm)；
[Modbus应用规范](https://www.modbus.org/file/secure/modbusprotocolspecification.pdf)。
现场材料：辣椒地池行车自动抓取系统.pdf（V2.0）、
辣椒无人行车控制接口交接说明_V1.1.docx（2026-09-22）。
附件内容作为需求证据，不自动授权接电测试或覆盖用户当前24DI要求。
