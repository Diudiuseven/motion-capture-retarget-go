# motion-capture-retarget-go

本地可复现的动作捕捉处理管线（纯 Go，无外部设备 / 网络依赖）。
原始捕捉数据经过 **归一化 → 修复 → 平滑 → 重定向 → 循环对齐 → 压缩**
六个阶段，输出可播放且逐帧可复现的动画片段。

```
RawCapture ──Normalize──▶ Clip ──Repair──▶ Clip ──Smooth──▶ Clip
                                                              │
              Clip ◀──Compress── Clip ◀──AlignLoop── Clip ◀──Retarget
```

## 数据模型

| 类型 | 说明 |
|---|---|
| `Skeleton` | 关节层级 + 静置姿态（`RestPos`/`RestRot`）。`Validate()` 拒绝重名、父引用缺失、多根与环，错误可定位到关节名 |
| `Joint` | `Name` / `Parent` / `RestPos`（相对父关节）/ `RestRot` |
| `Frame` | 一帧姿态：`Time`（秒）、`RootPos`（根位移）、`Rotations`（各关节局部旋转，按规范名索引） |
| `Clip` | 绑定骨架的帧序列 + 标称采样率 |
| `RawCapture` | 未解析的源数据：坐标约定、关节名映射、原始帧 |
| `Error` | 可定位错误，携带 `Op`（阶段）、`Frame`（帧号）、`Joint`（关节名） |

## 坐标约定

- **内部空间**：右手系，Y 轴向上，前方 +Z，右方 +X。
- **源数据**通过 `AxisConvention` 声明自身约定：

```go
AxisConvention{Up: "Z", Forward: "Y", Right: "X", LeftHand: true}
```

  `Normalize` 会校验声明的手性与轴向是否自洽（行列式符号不符即报错），
  并将位置与旋转（含镜像源的共轭变换）转换到内部空间。
- 关节名通过 `NameMap`（源名 → 规范名）归一；规范骨架中每个关节都必须
  被映射到，否则报错。

## 各阶段语义

### Repair — 丢帧 / 重复帧 / 时间戳漂移
- 按时间戳稳定排序；落在同一采样槽的重复帧只保留第一份；
- 漂移在 `DriftTol` 内的时间戳被吸附到标称采样网格；
- 缺帧由两侧采样球面插值（slerp）补齐，**缺失数据绝不按零值处理**；
  帧内缺失的关节样本从最近邻帧插值，最终回退到静置姿态而非零值；
- 缺口超过 `MaxGapFrames` 时报错并定位到具体帧号。
- 乱序 + 重复 + 漂移的输入，处理结果与完整有序输入逐帧一致。

### Retarget — 跨比例重定向
- 世界空间旋转增量传递：`dstWorld = (srcWorld · srcRestWorld⁻¹) · dstRestWorld`，
  源与目标的骨长、静置姿态、朝向约定均可不同，动作语义保持不变；
- 根位移按两骨架站立高度比缩放，避免比例失真；
- 四元数全程走短路径插值，无关节翻转；
- 骨骼映射必须覆盖目标根关节，否则拒绝。

### Smooth / Compress — 平滑与关键帧压缩
- `Smooth(clip, window)`：时域盒式滤波，四元数先做半球对齐再平均；
- `Compress(clip, tolerance)`：迭代分裂最坏区间，保证压缩结果按
  slerp/lerp 重采样后与原始片段的偏差 ≤ `tolerance`（旋转为弧度，
  根位移为长度单位）；相同输入必得相同输出。

### AlignLoop — 循环首尾对齐
- 在尾部 `SearchWindow` 帧内搜索与首帧偏差最小的接缝；
- 接缝偏差 ≤ `MaxError` 时裁剪尾部并向首帧交叉淡入，末帧为首帧的
  精确拷贝，接缝零跳变；无法满足时返回可定位错误。

### StreamProcessor — 长片段流式处理
- 内存有界：只保留 `SmoothWindow + MaxGapFrames + 常数` 帧的滑动窗口，
  与输入长度无关（测试覆盖 5000 帧）；
- `Push` 错误携带原始帧序号；`Checkpoint` 快照仅含滑动窗口，
  `Resume` 后从断点继续，输出与不间断运行逐帧一致。

## 配置项

| 配置 | 含义 | 默认 |
|---|---|---|
| `RepairConfig.MaxGapFrames` | 允许插值的最大连续缺帧数 | 显式配置 |
| `RepairConfig.DriftTol` | 时间戳漂移容限（秒） | 半个采样周期 |
| `LoopConfig.SearchWindow` | 尾部接缝搜索窗口（帧） | 全部帧 |
| `LoopConfig.MaxError` | 接缝最大允许偏差（弧度） | 显式配置 |
| `StreamConfig.SmoothWindow` | 流式平滑半窗（帧） | 0 |
| `Compress` 的 `tolerance` | 压缩误差容限（弧度） | 显式参数 |

## 运行

```sh
go test ./...   # 自动化测试：层级校验/丢帧/重复帧/漂移/重定向/压缩/循环/流式
go run ./cmd/demo   # 端到端演示，打印帧数与可复现校验和
```
