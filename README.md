# RocketRecovery — KSP kRPC 火箭回收教程复现指南

复现对象：[MrGEFORCE/RocketRecoveryTutorial](https://github.com/MrGEFORCE/RocketRecoveryTutorial)（B 站教程配套代码，视频 https://www.bilibili.com/video/BV1kz4y1G7x8/）。

> 姊妹篇：[GFOLD_KSP 与 cyno-landing 复现指南](./GFOLD-cyno-landing-reproduction.md)——同一任务（KSP 一子级定点回收）的另外两种实现：凸优化制导（GFOLD）与分段程序制导（CynoLanding），含本机实测记录。

本文档区分三类信息：
- ✅ **已实测**：在本机（Ubuntu / Python 3.14）实际运行并目验输出；
- 📄 **代码/文档依据**：从仓库源码、README、官方文档读出，未在本机游戏环境验证；
- 〔待实测〕：推断或依赖外部环境，未验证。

---

## 1. 项目构成

| 目录 | 内容 | 依赖 |
|---|---|---|
| `simulations/` | 7 个纯 Python 的 PID 下降物理仿真（调 Kp/Ki/Kd） | 只需 numpy/matplotlib/simple-pid，**不需要游戏** |
| `starShipLaunch.py` / `starShipRecovery.py` | kRPC Python 客户端：星舰发射 + 一级回收（格栅舵、反推、着陆 PID） | KSP + kRPC mod |
| `krpcMultiRocket_python/`、`krpcMultiRocket_cpp/` | 多火箭/多线程同步发射（Python 和 C++ 两版） | KSP + kRPC（C++ 版还需自行编译 krpc-cpp） |
| `KOS/bst-pidloop.ks` | 箭载版回收脚本（kOS mod，纯游戏内运行） | KSP + kOS mod |
| `craft/` | `B1040.7.craft`（猎鹰9一级）、`rocketDrone.craft`（四轴飞行器） | 放入存档 Ships 目录 |
| `pacifier/` | PyQt6 四轴矢量飞行器地面站（GUI） | kRPC + PySide6/pyqtgraph/pyopengl |
| `docs/krpcCppVcpkg/` | C++ 客户端编译踩坑记录（Windows/vcpkg） | — |

## 2. 第一步：纯仿真（已实测 ✅）

```bash
pip install numpy matplotlib simple-pid
cd simulations && python3 descendingModel.py
```

- ✅ 实测：7 个脚本（basicPID / descendingModel / idealPID / whiteNoise / discussOnKp / discussOnKi / discussOnKd）在 Python 3.14 下全部运行通过。
- ✅ 实测目验：`descendingModel.py` 输出曲线——80t 火箭从 1000m 以 -150m/s 下落，约 500m 处油门从 0 拉到 ~0.6，速度平滑收敛到 0。这正是 `starShipRecovery.py` 里着陆段 `PIDUsingV`（kp=0.01, kd=0.03）的物理模型。
- 备注：本机无显示器，运行时加了 `matplotlib.use("Agg")` 并把 `plt.show()` 替换为 `savefig` 输出 PNG；桌面环境直接用原版脚本即可。

## 3. 第二步：游戏环境

1. **KSP 本体**：Steam 付费，有 Linux 原生版。
2. **kRPC mod**：从官方站 https://krpc.github.io/krpc/getting-started.html 下载 release，解压到 `GameData/`。版本兼容关系以下载页为准〔待实测：未核对当前 release 号与 KSP 版本对应关系〕。
3. **kOS mod**（仅跑 `KOS/bst-pidloop.ks` 时需要）：https://ksp-kos.github.io/KOS_DOC/
4. **飞船文件**：把 `craft/B1040.7.craft` 复制到存档的 `Ships/VAB/`；`rocketDrone.craft` 同理（配 `pacifier/` 地面站）。
5. **Python 客户端**：`pip install krpc protobuf numpy simple-pid`。krpc 客户端库连接游戏内 server 后自动拉取 protobuf 定义（依据：krpc.github.io 官方 getting-started 文档）。

## 4. 第三步：运行时序（Python 回收方案，代码依据 📄）

1. 进游戏载入含 B1040.7 的存档，设为当前飞行器；
2. 游戏内启动 kRPC server。代码硬编码 `rpc_port=50000, stream_port=50001`，需与游戏内 server 端口设置一致（kRPC 默认即这两个端口〔待实测：以游戏内设置界面为准〕）；
3. `python3 starShipLaunch.py`：自动发射到 75km；分离前速度阈值 1600 m/s 是经验值，README 中 UP 主注明不保证助推器落在东边大陆；
4. 分离后把相机切到一级助推器（脚本操作 `active_vessel`），运行 `starShipRecovery.py`。

## 5. 已知坑（从代码中读出 📄）

- **游戏语言必须是中文**：`starShipLaunch.py` / `starShipRecovery.py` 按部件中文名查找（`"S3 KS-25“矢量”液体燃料引擎"`、`"大S型升降副翼1"`）。英文版 KSP 下查找静默返回空列表，引擎矢量限制逻辑失效；英文环境需改为 `"S3 KS-25 Vector Liquid Fuel Engine"` 等对应名称。
- **B1040.7 载荷不能太低**：README（20231027 补充）注明标称运力 63t，载荷太轻→轨道变化→下降弹道不同，可能需调参或少带燃料。
- **C++ 路线**：B 站置顶评论提到的 vcpkg 清单模式锁 protobuf 3.21.12，解决的是 Windows/VS2022 的坑；UP 主本人建议改用 gcc 工具链（kRPC 官方教程也是 Linux 版）。Linux 环境下优先走 Python 方案或 gcc 编译 krpc-cpp，可跳过 vcpkg。
- C++ 版 `CMakeLists.txt` 中路径为 UP 主本机 `D:/apps/...`，复现时需全部替换。

## 6. 建议复现顺序

`simulations/`（已验证）→ 装 KSP + kRPC，跑最小连接测试（`krpc.connect` 后读 `conn.krpc.get_status().version`）→ `starShipLaunch.py` → `starShipRecovery.py` → kOS 版 / 四轴地面站。

## 7. 官方依据

- kRPC 文档：https://krpc.github.io/krpc/getting-started.html
- kOS 文档：https://ksp-kos.github.io/KOS_DOC/
- 原仓库：https://github.com/MrGEFORCE/RocketRecoveryTutorial
- 教程视频：https://www.bilibili.com/video/BV1kz4y1G7x8/
