# GFOLD_KSP 与 cyno-landing 复现指南

> 分析对象：
> - 项目一：https://github.com/xdedss/GFOLD_KSP （凸优化动力下降制导 GFOLD + kRPC）
> - 项目二：https://gitee.com/cynospace/cyno-landing （分段程序制导回收，"CynoLanding"）
> - 两者都是：KSP 游戏内装 kRPC mod → Python 客户端连游戏 → 自动着陆一子级。
>
> 本地副本：`/home/tong/refs/ksp_landing/GFOLD_KSP`、`/home/tong/refs/ksp_landing/cyno-landing`
> 验证环境：本机 Docker（python:3.6 完整镜像）+ 本机 Python 3.14 venv。验证日期：2026-10-31。

---

## 0. 共同前置：KSP + kRPC（两个项目都需要）

| 步骤 | 内容 | 依据 |
|---|---|---|
| 1 | 安装 KSP 本体（付费；Steam Linux/Windows 通用） | 〔依据：Steam 商店页〕 |
| 2 | 安装 kRPC mod：release 解压进 `GameData/`，或用 CKAN 安装 | 〔依据：krpc.github.io〕 |
| 3 | 游戏内启动 kRPC server（默认监听 50000/50001 端口〔待核实：以游戏内 server 窗口为准〕） | 〔待实测〕 |
| 4 | `pip install krpc`（Python 客户端） | 〔依据：krpc.github.io〕 |

kRPC 各 release 与 KSP 版本的兼容关系：〔待实测——需对照 krpc.github.io 官方版本表与所用 KSP 版本确认〕。

---

## 1. 项目一：GFOLD_KSP（凸优化制导）

### 1.1 原理与运行时序（依据：仓库源码，逐条可追）

- 求解器实现 Blackmore 等人的 GFOLD 两段式凸优化：Problem 3 最小化着陆误差定出可行终端时间 tf，Problem 4 在 P3 解出的着陆精度约束下最大化末质量（最小燃料）〔依据：`GFOLD_direct_exec.py` 文件头注释；论文 larsblackmore.com/iee_tcst13.pdf〕。
- 凸化手段：推力约束松弛为二阶锥约束 + 质量对数变量 z=ln(m)〔依据：`GFOLD_direct_exec.py` 约束列表〕。
- 运行时序（`demo3_gfold.py`）：
  1. 连接 kRPC，以目标点建立目标系参考系；
  2. 当海拔低于 `start_altitude=1100m`，起一个线程用当前状态外推 0.5s 作为 x0 求解；
  3. 求解成功后 `nav_mode='gfold'`，沿轨迹采样点跟踪：目标加速度 = 前馈 u + PD（k_v, k_x）修正，经 `conic_clamp` 限幅（推力大小锥 + 最大倾角锥）后换算 throttle 与姿态；
  4. 进入目标附近圆柱区（高 200m、半径 20m）切 `final` 模式：纯 PID 估高节流 + 水平位置微控；
  5. 满足速度/位置/角速度阈值或高度<1m 且上升时关机退出。
- 参数全部在 `params.txt`（目标经纬高、tf、max_tilt=25°、throttle 限制、G_max=10、V_max=150、下滑角 30°、PID 增益）。
- 改自己的目标点：改 `params.txt` 的 `target_lat/target_lon/target_height`。

### 1.2 环境安装（README 钉死的版本）

README 要求：Python 3.6 推荐、setuptools≤57.5.0、scipy==1.2.1、CVXcanon==0.1.0、cvxpy==0.4.11、krpc；可选 cvxpy_codegen 生成 C 代码提速〔依据：仓库 README〕。

### 1.3 实测记录（本机，2026-10-31）

**已实测通过**——Docker 内逐条验证：

1. `scipy==1.2.1` 与 `cvxpy==0.4.11` **不兼容**：cvxpy 0.4.11 import 链里 `from scipy.misc import logsumexp`（`cvxpy/atoms/log_sum_exp.py:22`），而 `scipy.misc.logsumexp` 在 scipy 1.2.0 已移除 → ImportError。**改用 `scipy==1.1.0` 后通过**（实测 `from scipy.misc import logsumexp` OK）。← README 的 scipy 钉法有坑。
2. 安装 `scs` 时其依赖可能把 scipy 顶到 1.5.x，**装完 solver 类包后重新 pin `scipy==1.1.0`**。
3. numpy 需 `<1.20`（Python 3.6 下 pip 解析到 1.19.5）。
4. 最终可用组合（全部实测装得上、import 通过、求解收敛）：
   `python 3.6.15 | setuptools 57.5.0 | numpy 1.19.5 | scipy 1.1.0 | CVXcanon 0.1.0 | ecos 2.0.10 | scs 2.1.2 | cvxpy 0.4.11`

**已实测——求解器闭环求解**（`test_solve_direct.py`，不经 KSP、不经 codegen，用 `GFOLD_run.py` 自带 Mars 测试数据 `g=-3.71`）：

```
p3: OPTIMAL → tf_m = 49.5 s
p4: OPTIMAL（ECOS, feastol=2.0e-13）
final mass = 2031.81 kg（湿重 2300 kg → 耗燃料 268.19 kg）
final pos = [4.9e-14, 3.8e-14, 1.7e-14] ≈ 0
final vel ≈ 0
总耗时 ~2.8 s（本机现代 CPU；README 称 direct 比 codegen C 代码慢 10 倍以上〔依据：README〕）
```

可复现命令（本机）：

```bash
cd /home/tong/refs/ksp_landing/GFOLD_KSP
docker run --rm -v $PWD:/app -w /app <python:3.6 镜像> bash -c "
  pip install 'setuptools==57.5.0' 'numpy<1.20' 'scipy==1.1.0'
  pip install 'CVXcanon==0.1.0' 'ecos==2.0.10' 'scs==2.1.2'
  pip install 'scipy==1.1.0'   # 防 scs 顶包
  pip install --no-build-isolation 'cvxpy==0.4.11'
  python test_solve_direct.py"
```

### 1.4 未实测 / 待办

- [ ] **codegen 路径**（生成 C 求解器包 `gfold_solver_p3/p4`）：需 moehle/cvxpy_codegen（GitHub，README 注：py3 安装需删 setup.py 里 `"python >= 2.7"` 一行），`build.bat` 是 Windows 批处理，Linux 下需手工分步跑 `GFOLD_codegen.py → postprocess.py → setup.py install`。〔待实测〕
- [ ] KSP 内闭环飞行（需游戏 + 自制可节流回收火箭）〔待实测〕。
- [ ] kRPC mod 版本与 KSP 版本匹配〔待核实〕。

### 1.5 KSP 侧飞船要求（从源码推出的契约）

- 可节流引擎（`vessel.control.throttle` 连续控制）；
- 足够的姿态控制能力满足 `max_tilt=25°` 推力锥；
- 着陆腿（`(N-n_i)*tf/N < 10s` 或进入 final 区时 `vessel.control.gear=True`）；
- 飞船参数（Isp、max_thrust、mass）实时读入求解器，所以**任何满足推力/比冲接口的火箭均可**，无需固定 craft 文件〔依据：`demo3_gfold.py` vessel_profile1〕。

---

## 2. 项目二：cyno-landing（分段程序制导）

### 2.1 原理与运行时序（依据：源码逐条可追）

不是凸优化，是**解析轨迹预测 + 分段程序制导**：

| 阶段 | 触发/退出 | 控制 |
|---|---|---|
| before entry burn | 预测落点高度 < 25km | 姿态 retrograde，不开机 |
| entry burn | 动作组3 开机，开环节流 1.0 | 减速到 650 m/s @ 25km（预测误差驱动俯仰/偏航微调） |
| sliding | 关机滑行 | 每 5 个循环（0.5s）用 `solve_ivp` 预测落点，误差向量驱动 retrograde 姿态修正 |
| landing burn 1 | 动作组2，节流≈0.9 | 减速到 30 m/s；节流按预测触地高度误差微调 |
| landing burn 2 | 动作组1，节流≈0.9 | 减速到 5 m/s @ 30m 海拔 |
| landing burn 3 | 竖直向上姿态 | PID 控 vertical_speed = -5 m/s 匀速触地，触地（situation==0）关机 |

- 轨迹预测模型：地固系平面运动，引力（随高度平方反比）+ 二次气动阻力（Cd 分段：亚声速 0.045 / 跨声速线性 / 超声速 1.25，参考面积 7.93 m²）+ 定值推力，`scipy.solve_ivp` 积分到「海拔或速度」终止事件；着陆点火高度用二分法预测（`cycle_number=7`）〔依据：`CynoLanding.py` Trajectory/TrajectoryE 类、`flight_cfg.py`〕。
- 大气模型：分段指数拟合 Kerbin 大气（密度/音速），5km/10km 分段〔依据：`atmo_model.py`——注意是作者拟合值，非游戏原版大气精确模型〕。

### 2.2 环境（实测）

现代 Python 即可：**实测 Python 3.14 + krpc 0.7.0 + numpy 2.5.4 + scipy 1.18.1**，`py_compile` 通过。无需 pin 老版本。

```bash
pip install krpc numpy scipy
```

### 2.3 实测记录（本机，2026-10-31）

**已实测——物理核心可离线运行**（`test_physics_core.py`：mock 掉 krpc，截取类定义，用 Kerbin 参数 R=600km、g0=9.81 做无推力弹道积分）：

```
初始：海拔 10km、垂直速度 -300 m/s、质量 20t
积分 69.4s 后：海拔 -10000m（触地面以下，终止事件正常触发）
无推力质量守恒（20t 不变）✓
```

即：大气模型、阻力模型、solve_ivp 积分器——制导的核心计算链——不依赖游戏即可验证。

### 2.4 KSP 侧飞船要求（从 `flight_cfg.py` 推出的硬契约）

这是复现本项目**最大的工作量**，比跑通代码难：

1. **动作组 1/2/3 绑定的引擎组合必须与推力参数一致**（注释原文："thrust 的值用于轨迹计算……二者的值务必对应"）：
   - entry burn = 2×215kN + 1×240kN 发动机；
   - landing burn 1 = 2×168kN + 5×205kN（×0.9）；
   - landing burn 2 = 2×168kN + 1×205kN（×0.9）；
   - 这些数字对应作者的原版部件（+可能 KRE）飞船，**换飞船必须重新手填**〔依据：flight_cfg.py〕。
2. **气动参数 `reference_area`、`Cd_subsonic/supersonic` 必须实测填写**（注释原文："务必通过 debug 进行实际飞行测试并填写，不要使用 FAR 风洞数据"）〔依据：flight_cfg.py〕。
3. **栅格舵**：`use_grid_fin=True` 是为 KRE grid fin 反向力矩数据做的特判，注释明确"不能混用 grid fin 和其他控制面"〔依据：flight_cfg.py〕。
4. **TWR 与收敛性**（注释原文）："强烈建议推重比要大于 1；不启动引擎时到达 end_altitude_entry_burn 处的速度要大于 end_speed_entry_burn，否则计算可能无法收敛"〔依据：flight_cfg.py〕。
5. `landing_position` 是一组经度点（-66.0°~-69.8°），按自己存档改〔依据：flight_cfg.py〕。
6. `belly_heading` 需与发射方位角差 180°〔依据：flight_cfg.py 注释〕。

### 2.5 未实测 / 待办

- [ ] 完整闭环飞行（需游戏 + 按 2.4 搭飞船 + 实测填气动参数）〔待实测〕。
- [ ] KRE（Kerbal Reusability Expansion）mod 的 grid fin 部件获取与版本兼容〔待核实〕。

---

## 3. 两项目对比（共同契约：同任务——KSP 一子级定点回收）

| 项 | GFOLD_KSP | cyno-landing |
|---|---|---|
| 制导方法 | 凸优化在线求解（ECOS），最优性有保证 | 解析预测 + 工程分段，无最优化 |
| 环境 | Python 3.6 + 一堆 EOL pin（有坑，见 1.3） | 现代 Python 直接装 |
| 飞船耦合 | 低：参数实时读入，任意可节流火箭 | 高：推力/气动/动作组硬绑定，需按自己飞船重填 |
| 代码状态 | 单文件 demo，可读性好 | 模块化（State/Sensor/KAP/KAPEE/Trajectory），注释非常详细 |
| 离线可验证部分 | 求解器（已实测 ✓） | 物理核心（已实测 ✓） |
| 游戏内闭环 | 待实测 | 待实测 |

---

## 4. 自测问题（闭卷）

1. GFOLD 为什么分 P3/P4 两段求解？各段目标函数是什么？〔依据：GFOLD_direct_exec.py 文件头〕
2. 为什么 README 钉的 scipy==1.2.1 跑不起来？报错链是哪一行到哪一行？〔依据：1.3 实测记录〕
3. cyno-landing 的着陆点火高度是怎么预测的？二分法的输入输出各是什么？〔依据：TrajectoryE + flight_cfg cycle_number〕
4. cyno-landing 换一艘自己的飞船至少要改 flight_cfg.py 里哪几类参数？为什么注释禁止用 FAR 风洞数据？〔依据：2.4〕
5. 两个项目各自"不装游戏就能验证"的部分是什么？本机的验证脚本和原始输出在哪？〔依据：1.3 / 2.3〕

## 5. 官方依据

- GFOLD_KSP 仓库 README 与源码：https://github.com/xdedss/GFOLD_KSP （本地副本含 git 提交 2023-01-25）
- cyno-landing 源码：https://gitee.com/cynospace/cyno-landing （本地副本 v0.0.1，提交 2026-02-27）
- GFOLD 论文：http://www.larsblackmore.com/iee_tcst13.pdf （Blackmore et al., IEEE TCST）
- kRPC 官方文档：http://krpc.github.io/krpc/
- scipy.misc.logsumexp 移除：scipy 1.2.0 release notes（实测验证 import 失败/成功的两个版本）

---

## 6. kRPC 版本对齐方案（2026-10-31 实测）

**问题**：三个项目分属三个年代（GFOLD_KSP 2020 / RocketRecoveryTutorial 2023 / cyno-landing 2026），能否统一用同一个 kRPC 版本复现？

**结论：可以对齐到 kRPC 0.5.4（mod + Python 客户端同版本），KSP 用 1.12.5。GFOLD_KSP 只需一个 scipy 兼容 shim，已实测通过。**

### 6.1 版本事实（依据 + 实测）

| kRPC 版本 | 支持 KSP（官方 changelog） | Python 客户端要求 | 发布时间 |
|---|---|---|---|
| 0.4.8 | KSP 1.5.1 | 无限制（可装 py3.6） | 2018-10 |
| 0.5.0 | KSP 1.8+ | >=3.4（0.5.0）/ >=3.7（0.5.2+） | 2023-03 |
| 0.5.4 | KSP 1.8+（社区实测配 KSP 1.12） | >=3.7 | 2024-06 |
| 0.6.0 / 0.7.0 | KSP ≥1.12.5（0.7.0 changelog 原文） | >=3.10 | 2026-07 / 2026-10 |

〔依据：krpc.github.io/krpc/latest/changelog.html；PyPI krpc JSON 元数据；KRPC.MechJeb release 注记〕

三条实测得到的硬约束：
1. **官方要求客户端与服务端同 release**：0.7.0 客户端握手时协议不符直接报 "Use a client from the same kRPC release as the server"〔实测：本机 krpc 0.7.0 客户端源码 `__init__.py`〕。0.7.0 的 `PROTOCOL_VERSION = 1`，但 0.4.8 客户端**连协议协商字段都没有**（实测解包 0.4.8 sdist），跨大版本混搭不受官方支持。
2. **0.7.0 客户端默认 `use_pregenerated_stubs=True`**（实测源码），连旧服务端时会用本地新版存根，风险自担。
3. **GFOLD_KSP 的 cvxpy 0.4.11 把 Python 钉在 ≤3.7**：scipy 需要 ≤1.2.1（1.2.0 移除 `scipy.misc.logsumexp`，而 0.4.11  import 它）；scipy 1.2.1 支持 py3.7 → **Python 3.7 是同时满足 cvxpy 0.4.11 和 krpc>=0.5.4（要求 >=3.7）的最大公约数**。唯一缺口 `scipy.misc.logsumexp` 用一行 shim 补上：`scipy.misc.logsumexp = scipy.special.logsumexp`。
4. 附带发现：krpc 0.4.8 在 PyPI 只有 sdist 且用 `use_2to3`，现代 setuptools 装不了（与 cvxpy 0.4.11 同病）；protobuf 要钉 `3.20.3`（krpc 0.5.4 的新式生成代码需要 `google.protobuf.internal.builder`，<3.20 没有）。

### 6.2 实测验证（Docker python:3.7 完整镜像）

环境：`setuptools 57.5.0 | numpy 1.19.5 | scipy 1.2.1 | CVXcanon 0.1.0 | ecos 2.0.10 | scs 2.1.2 | cvxpy 0.4.11 | protobuf 3.20.3 | krpc 0.5.4`

```
shim applied: scipy.misc.logsumexp -> scipy.special.logsumexp
GFOLD solve OK: tf=49.5 final_mass=2031.8 fuel=268.2 |v_end|=1.98e-12   ← 与 py36 基线数值一致
krpc client OK: 0.5.4
PY37 UNIFIED SCHEME: PASS
```

可复现命令见 `GFOLD_KSP/test_py37_unified.py`。

### 6.3 三个项目的对齐落法

| 项目 | Python 环境 | kRPC 客户端 | 与 0.5.4 mod 混搭风险 |
|---|---|---|---|
| RocketRecoveryTutorial | 现代 Python（3.10+） | krpc==0.5.4 | 无——API 全是基础 SpaceCenter（connect/parts.with_name/control）〔依据：源码〕 |
| cyno-landing | 现代 Python（实测 3.14 装最新版也可） | krpc==0.5.4 | 无——只用 add_stream/姿态/动作组/着陆腿等基础接口〔依据：源码〕 |
| GFOLD_KSP | **Python 3.7 专用 venv**（shim + 上表 pin） | krpc==0.5.4 | 无（求解器端已实测；游戏内闭环〔待实测〕） |

mod 侧统一装 **kRPC 0.5.4**（支持 KSP 1.8+，KSP 1.12.5 可用〔依据：changelog + 社区注记；待实测确认 1.12.5 上的实际表现〕）。
若想用更新的 0.7.0 mod（要求 KSP≥1.12.5），则三个客户端都要 0.7.0——但 0.7.0 要求 Python>=3.10，GFOLD 的 cvxpy 0.4.11 装不进 3.10+（scipy≤1.2.1 无 3.10 wheel），**GFOLD 会脱队**。所以 0.5.4 是三项目能同时对齐的版本上限。

### 6.4 官方依据（本节）

- kRPC changelog（KSP 支持矩阵、0.7.0 最低 KSP 1.12.5）：https://krpc.github.io/krpc/latest/changelog.html
- PyPI krpc 版本与 requires_python：https://pypi.org/pypi/krpc/json
- 0.7.0 客户端握手/预生成存根行为：本机 site-packages/krpc/__init__.py（krpc 0.7.0）
- 0.4.8 客户端无协议协商：krpc-0.4.8.zip sdist 解包
