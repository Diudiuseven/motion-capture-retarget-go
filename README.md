# motion-capture-retarget-go

一套本地可复现的动作捕捉处理管线（纯 Go，无外部设备 / 网络依赖）。
原始捕捉数据经 **解析归一化 → 帧清理 → 重定向 → 平滑 → 循环对齐 → 关键帧压缩**
后输出可播放、可复现的动画片段。

## 管线总览

```
RawFrame ──NormalizeFrame──▶ NormalizedFrame ──FrameCleaner──▶ 有序无缺口帧序列
                                                              │
                          Retargeter ◀── SourceSpec/Skeleton  │
                              │                               │
                              ▼                               ▼
                     Pose 序列 ──SmoothPoses──▶ AlignLoop ──▶ CompressKeyframes
                                                              │
                                                              ▼
                                                  Clip（可采样播放）/ []Keyframe
```

流式场景使用 `StreamProcessor` 逐帧推送，内存占用有界，支持断点续跑。

## 数据模型

| 类型 | 文件 | 说明 |
|---|---|---|
| `Vec3` / `Quat` | `math3d.go` | 三维向量与单位四元数（x,y,z,w） |
| `Joint` / `Skeleton` | `skeleton.go` | 骨骼层级；`NewSkeleton` 校验重名、缺失父级、环，输出拓扑序 |
| `Pose` | `skeleton.go` | 一帧姿态：各关节局部旋转 + 根节点世界位移 |
| `SourceSpec` / `RawFrame` | `source.go` | 原始数据源描述（命名映射、坐标约定、采样率）与原始帧 |
| `NormalizedFrame` | `source.go` | 内部统一表示的帧（内部关节名、内部坐标系） |
| `Clip` / `Keyframe` | `clip.go` | 输出动画片段与压缩关键帧 |

## 坐标约定

- **内部约定**：右手系，Y 轴向上，Z 轴向前。
- 每个来源通过 `AxisConvention{Forward, Up, RightHanded}` 声明自己的坐标系，
  `ConversionQuat` 自动构造到内部约定的转换；旋转按 `R' = C·R·C⁻¹` 换基，
  根位移按 `v' = C·v` 转换。
- 四元数一律归一化；插值走最短弧（`Slerp`），避免关节翻转。

## 各阶段与配置项

### 1. 解析与校验（`source.go`, `skeleton.go`）
- `SourceSpec.NameMap`：源关节名 → 内部关节名；未映射的关节被丢弃。
- `NewSkeleton` 拒绝：重名、父级缺失、自引用、环、多根 / 无根，
  错误携带 `Joint` 字段可定位。
- 零长度四元数等非法数据返回 `ParseError{Sequence, Joint, Reason}`。

### 2. 帧清理（`frames.go`）
- `FrameCleaner{SampleRate, MaxGapFrames}`：
  - 按采集序号去重（首个出现生效）并恢复顺序——乱序 / 重复输入与完整有序输入结果一致；
  - 缺口按序号线性插值（旋转 `Slerp`、位移 `Lerp`），**缺失帧绝不当作零值**；
    缺口超过 `MaxGapFrames` 返回 `GapError{AfterSequence, Missing}`；
  - 时间戳按 `epoch + seq/SampleRate` 重新锚定，消除漂移。

### 3. 重定向（`retarget.go`）
- `NewRetargeter(src, dst, jointMap)`：映射目标关节名 → 源关节名，根关节必须映射。
- 算法：世界空间旋转增量迁移 `Gt = (Gs·Gs_rest⁻¹)·Gt_rest`，
  骨长 / 静置姿态 / 朝向约定不一致时动作语义保持不变；静置姿态映射回静置姿态。
- 根位移按髋高比自动缩放，可用 `RootMotionScale` 覆盖。

### 4. 平滑与压缩（`smooth.go`）
- `SmoothConfig{Window}`：确定性滑动平均（四元数符号对齐后加权归一）。
- `CompressConfig{Tolerance}`：递归在误差最大帧处分裂，
  **保证** 压缩后任意帧姿态偏差 ≤ `Tolerance`（弧度）；同输入逐帧一致。

### 5. 循环对齐（`loop.go`）
- `LoopConfig{MaxSeamDeviation, MaxRootSeamDistance, BlendFrames}`：
  首尾偏差超限返回 `LoopError`（含实际偏差与限值）；
  通过则把尾部 `BlendFrames` 帧向首帧渐变融合，末帧精确等于首帧，接缝零跳变。

### 6. 流式处理（`stream.go`）
- `StreamProcessor.Push/Flush`：逐帧摄入，乱序帧在 `MaxReorder` 内有界缓冲，
  重复帧丢弃；输出经 `OnEmit` 回调。
- 内存上界 `MemoryBound() = MaxReorder + Smooth.Window + 1`，与输入长度无关。
- 失败返回 `FrameError{Sequence, Err}` 可定位到具体帧；
  `Checkpoint()/Resume(cp)` 支持崩溃后从断点续跑，输出与连续处理逐帧一致。

## 测试

```sh
go test ./...
```

覆盖：骨骼校验（重名 / 缺父 / 环）、命名映射与坐标换基、丢帧插值、
重复与乱序等价性、时间戳漂移修正、重定向语义保持与根位移缩放、
压缩误差界与确定性、循环接缝闭合与拒绝、流式乱序 / 断点续跑 / 内存上界，
以及端到端可复现性（同一输入两次运行逐帧一致）。
