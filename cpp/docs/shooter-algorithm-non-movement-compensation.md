# Shooter 非移动补偿算法说明（当前实现）

本文档描述当前代码中“非移动补偿”条件下的射击算法实现流程。  
对应代码主文件：
- `src/main/cpp/subsystems/ShooterSubsystem.cpp`
- `src/main/include/subsystems/ShooterSubsystem.h`
- `src/main/include/Constants.h`

说明范围：
- 重点讲 `ShooterSubsystem` 内部基于距离的 pitch/速度拟合与执行逻辑。
- 不展开讲 `RealTimeAimDrive/PassBallCommand` 对 `angleOffsetFromDrive`、`velOffsetFromDrive` 的移动补偿细节。
- 默认假设非移动补偿场景下：`angleOffsetFromDrive = 0`、`velOffsetFromDrive = 0`。

---

## 1. 算法输入与输出

### 1.1 输入
1. 底盘输入（来自 `CommandSwerveDrivetrain`）
- `hub_distance_m`：机器人到 Hub 的距离（`GetDistanceToHub()`）。
- `pass_distance_m`：机器人到 `passTarget` 的距离（用于传球速度拟合）。
- 当前联盟色与底盘 X 坐标（用于判定是否进入 passing 模式）。

2. 机制输入（来自 `FeederSubsystem`）
- 上料目标速度：`GetComboTargetVelocity()`。
- 上料当前速度：`GetUpwardFeederVelocity()`。

3. 操作与调参输入
- `shooting`：是否启用射击（`EnableShooter/DisableShooter` 控制）。
- `onlyDefaultShoot`：是否启用默认固定射击（POV 触发）。
- Dashboard 开关：
  - `shootUseDash`（bool）
  - `shootVelWant`（double）

4. 常量输入
- 拟合系数：
  - `a=2.8, b=21.3, c=4.2, d=17.8`
  - 传球拟合：`pass_m=5, pass_b=7`
- pitch 角限制：
  - `kMinPitchAngle=0`
  - `kMaxPitchAngle=34.65`
- 其他：
  - `shooter_height_approx=0.46932`
  - `shooter_max_composite=6.5`

### 1.2 输出
1. 飞轮目标速度
- `realShootVelocity`，最终下发到 `shooter_right_up_.setvelocitytorquecurrent(realShootVelocity)`。

2. Pitch 目标角
- 计算得到 `angle` 后，转换为电机命令角 `SetShootPitchAngle(90 - angle)`。

3. 调试/观测输出
- `shoot_velocity_expected`
- `shootRealVelo`
- `shootIdealAngle`
- `shootOnMove/isPassing`
- `shooter_hub_distance_m`

---

## 2. 周期运行主流程（ShooterSubsystem::Periodic）

### Step 0：基础控制刷新
- 每周期执行：
  - `shooter_right_up_.Control()`
  - `shooter_pitch_.Control()`
  - `shooter_right_up_.Receive()`

### Step 1：Pitch 回零门控
- 若 `shooter_pitch_reset_flag_ == false`，进入 `ShootPitch_Reset()`，并 `return`，本周期不执行后续拟合与射击控制。

### Step 2：采集距离与状态
- 读取 `hub_distance_m`。
- 读取 `pass_distance_m`（当前 pose 到 `passTarget`）。
- 根据联盟与 X 坐标更新 `isPassing`：
  - 蓝方：`x >= 5.0` 进入 passing
  - 红方：`x <= 11.54` 进入 passing

### Step 3：计算目标 pitch（角度拟合）
- 调用 `CalculatePitchAngleAboveHub(hub_distance_m)`。

### Step 4：计算目标速度（速度拟合）
- 若 `isPassing == false`：用 `hub_distance_m` 拟合。
- 若 `isPassing == true`：用 `pass_distance_m` 拟合。
- 调用 `CalculateShooterSpeedRegression(...)`。

### Step 5：计算最终飞轮速度
- 调用 `getFinalVel()`，将基础拟合速度叠加 feeder 组合补偿（以及移动补偿接口值）。

### Step 6：执行机构命令
- 若 `shooting == true`：
  - 下发飞轮目标速度 `realShootVelocity`
  - 下发 pitch 角 `SetShootPitchAngle(90-angle)`
- 若 `shooting == false`：
  - 飞轮 `Stop()`
  - pitch 回到 `0.2°`

---

## 3. Pitch 拟合实现（CalculatePitchAngleAboveHub）

函数输入：`dis`（当前调用时传入 `hub_distance_m`）  
函数输出：`angle`（后续用于 `SetShootPitchAngle(90-angle)`）

### 3.1 分支逻辑
1. 默认射击模式 `onlyDefaultShoot == true`
- 直接返回固定值：`angle = 80`

2. Passing 模式 `isPassing == true`
- 固定 `Tangle = 60`
- `angle = Tangle + angleOffsetFromDrive`

3. 常规 Speaker 模式
- 距离先缩放：`dis = dis * 1.5 / 8.0`
- 高度差：`height = 1.8288 + 1.8 - shooter_height_approx`
- 理想角：`atan2(height, dis) * 180 / pi`
- 限幅到 `[75, 85]`
- 保存 `Tangle = angle`
- 最终 `angle += angleOffsetFromDrive`

非移动补偿场景下可视为：`angleOffsetFromDrive = 0`，即仅保留上述基础拟合结果。

---

## 4. 速度拟合实现（CalculateShooterSpeedRegression）

函数输入：`dis`（常规下为 `hub_distance_m`，passing 下为 `pass_distance_m`）  
函数输出：`vel`（基础飞轮速度）

### 4.1 Dashboard 覆盖逻辑
- 当 `shootUseDash == true`：直接 `vel = shootVelWant`，跳过拟合。

### 4.2 拟合逻辑（`shootUseDash == false`）
1. `onlyDefaultShoot == true`
- `vel = 40.9`

2. `isPassing == true`
- 线性：`vel = pass_m * dis + pass_b = 5 * dis + 7`

3. 常规模式 + 近距离（`dis < 2.5`）
- 线性：`vel = 2.8 * dis + 21.3`

4. 常规模式 + 远距离（`dis >= 2.5`）
- 线性：`vel = 4.2 * dis + 17.8`

---

## 5. 最终速度合成（getFinalVel）

目标：将基础速度 `vel` 叠加 feeder 状态补偿得到 `realShootVelocity`。

### 5.1 feeder 补偿项
- `target = feeder_sub_->GetComboTargetVelocity()`
- `current = feeder_sub_->GetUpwardFeederVelocity()`
- `difference = -current + target`
- `add = difference / target * shooter_max_composite`（仅当 `target > 0.01`）

### 5.2 最终公式
- `realShootVelocity = vel + velOffsetFromDrive + add`

非移动补偿场景下：
- `velOffsetFromDrive = 0`
- 则 `realShootVelocity = vel + add`

---

## 6. Pitch 电机目标角映射

`SetShootPitchAngle(target_angle)` 内部流程：
1. 角度限幅到 `[kMinPitchAngle, kMaxPitchAngle]`，即 `[0, 34.65]`
2. 归一化：
- `norm = (clamped - min) / (max - min)`
3. 调用 `SetShootPitchNormPosition(norm)`

主流程中实际下发的是 `SetShootPitchAngle(90-angle)`，因此 `angle` 越大，最终 `90-angle` 越小。

---

## 7. 与“移动补偿”边界

虽然本文聚焦非移动补偿，但当前结构预留了两个补偿入口：
- `SetAngleOffset(double)` -> `angleOffsetFromDrive`
- `SetSpeedOffset(double)` -> `velOffsetFromDrive`

它们由 `RealTimeAimDrive` / `PassBallCommand` 在边移动边射击时实时写入。  
若不运行这些命令，两个偏置默认为 0，本文件描述的即为实际生效主流程。

---

## 8. 当前实现特征总结

1. Pitch 与速度本质都是“分段线性 + 常量分支”的工程拟合，不是高阶多项式回归。
2. Pitch 计算里存在一层距离缩放 `dis * 1.5 / 8`，属于经验系数。
3. 最终飞轮速度不是纯拟合输出，还叠加了 feeder 状态相关的动态补偿 `add`。
4. 是否 passing 不是由 `startPassing/stopPassing` 直接决定，而是每周期按场上 X 位置重算。

---

## 9. 你提到的“经验系数缩放”结论

是的，当前实现里不仅距离有经验缩放，高度项也包含经验修正。

1. 距离经验缩放
- 在常规 Speaker pitch 计算前，先做：
- `dis = dis * 1.5 / 8.0`
- 这会直接改变 `atan2(height, dis)` 的输入，属于经验映射。

2. 高度经验修正
- pitch 计算用到：
- `height = 1.8288 + 1.8 - shooter_height_approx`
- 其中 `+1.8` 不是纯几何常量，而是明显的经验补偿项（与机构/出球模型调参相关）。

---

## 10. 底盘移动补偿算法（pitch 与车头角）详细流程

本节对应代码：
- `src/main/cpp/commands/RealTimeAimDrive.cpp`
- `src/main/cpp/commands/PassBallCommand.cpp`
- `src/main/cpp/commands/AutoRealTimeAimDrive.cpp`

核心思想：  
先算“若底盘在动，球在场地坐标系下应具备的等效速度向量”，再把差值反推成：
- `shootAngleOffset`（写入 `SetAngleOffset`，作用到 pitch）
- `ShooterVelOff`（写入 `SetSpeedOffset`，作用到飞轮速度）
- `targetAngleRad` 的附加偏角（作用到底盘车头朝向）

### 10.1 补偿链路输入与输出

#### 输入
1. 来自 Shooter 的基础状态
- `vel`：基础拟合后的飞轮标称速度（未加移动补偿）
- `Tangle`：基础拟合后的出射角（未加移动补偿）

2. 来自底盘/手柄的运动状态
- 当前姿态与速度 `GetState().Pose / Speeds`
- 驾驶输入（手柄或 auto PID 产生的 `nowX/nowY`）
- 目标点（Hub 或 PassTarget）

3. 常量/参数
- `shootCoeff`
  - `RealTimeAimDrive`：`0.45`
  - `PassBallCommand`：`0.17`
  - `AutoRealTimeAimDrive`：`0.17`
- 预测延迟 `latencySeconds`
  - `RealTimeAimDrive`：`0`
  - `PassBallCommand`：`0.4`
  - `AutoRealTimeAimDrive`：`0.45`

#### 输出
1. 机构补偿输出（写回 ShooterSubsystem）
- `SetAngleOffset(shootAngleOffset)`：pitch 补偿角
- `SetSpeedOffset(ShooterVelOff)`：飞轮速度补偿

2. 底盘补偿输出（写回底盘控制）
- 目标朝向 `WithTargetDirection(rott)`，其中 `rott` 来自补偿后的 `targetAngleRad`

---

### 10.2 统一计算框架（公式视角）

下列流程在三个命令中一致，仅参数不同。

#### Step A：确定“未来瞄准点”
1. 根据指令速度估计未来位置：
- `predictedX = currentX + vx_cmd * latency`
- `predictedY = currentY + vy_cmd * latency`
2. 由未来位置指向目标点（Hub 或传球点）得到基础瞄准角：
- `targetAngleRad = atan2(dy, dx)`

#### Step B：把底盘平移速度投影到“以目标连线为基准”的坐标系
1. 朝目标方向分量（径向分量）：
- `Normvx = vx_cmd * cos(-targetAngleRad) + vy_cmd * cos(PI/2 - targetAngleRad)`
2. 侧向分量（横向分量）：
- `Normvy = vx_cmd * sin(-targetAngleRad) + vy_cmd * sin(PI/2 - targetAngleRad)`

#### Step C：建立“球速向量”并与底盘速度合成
1. 基础出球向量（由 `vel` 和 `Tangle` 给出）：
- `Shootvx = vel * cos(Tangle) * shootCoeff`
- `Shootvz = vel * sin(Tangle) * shootCoeff`
2. 与底盘运动合成（代码实现）：
- `Shootvx = Shootvx - Normvx`
- `Normvy = -Normvy`

这里等价于：在场地坐标中，射击系统需要补足底盘造成的水平速度偏移。

#### Step D：反推飞轮速度补偿
- `ShooterVelOff = sqrt(Shootvx^2 + Normvy^2 + Shootvz^2) / shootCoeff - vel`
- 含义：为了达到修正后的三维速度模长，需要在基础 `vel` 上增加/减少多少。

#### Step E：反推底盘车头角补偿
1. 横向漂移引起的附加偏航角：
- `chassisAngleOffset = atan2(Normvy, Shootvx)`（通常仅在 `vel >= 10` 时启用）
2. 合成到底盘目标朝向：
- `targetAngleRad += chassisAngleOffset`
- 再减机构安装角补偿：`targetAngleRad -= angleOfShooter`
- 红方再做坐标系修正（减 `PI`）

#### Step F：反推 pitch 补偿角（写入 ShooterSubsystem）
- `shootAngleOffset = atan2(Shootvz, Shootvx) - Tangle`
- 转成角度后调用 `SetAngleOffset(shootAngleOffset)`
- 在 `ShooterSubsystem` 中通过 `angle = Tangle + angleOffsetFromDrive` 生效。

---

### 10.3 三个命令的差异点

1. `RealTimeAimDrive`（手动边移动边打 Hub）
- 目标点固定是 Hub。
- `shootCoeff = 0.45`
- `latencySeconds = 0`
- 移动速度来自手柄输入缩放后的 `nowX/nowY`。

2. `PassBallCommand`（边移动边传球）
- 目标点是传球落点（根据场地左右半区动态选点，红方镜像）。
- `shootCoeff = 0.17`
- `latencySeconds = 0.4`
- 除补偿外，还附加了喂球放行条件（位置/速度/角度门限）。

3. `AutoRealTimeAimDrive`（自动段移动瞄准）
- 平移速度由 `m_xPID/m_yPID` 生成，带速度上限。
- `shootCoeff = 0.17`
- `latencySeconds = 0.45`
- 到达目标点后命令结束。

---

### 10.4 与非移动补偿主流程如何耦合

1. 非移动补偿阶段先给出基础解
- `Tangle`（基础 pitch）
- `vel`（基础飞轮速度）

2. 移动补偿命令每周期覆盖偏置
- `SetAngleOffset(shootAngleOffset)`
- `SetSpeedOffset(ShooterVelOff)`

3. `ShooterSubsystem::Periodic()` 合成最终执行量
- pitch 侧：`angle = Tangle + angleOffsetFromDrive`
- 速度侧：`realShootVelocity = vel + velOffsetFromDrive + feeder补偿add`

所以可把当前实现理解为：
- “基础拟合解” + “运动学补偿增量解”的叠加结构。
