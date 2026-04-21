# PyBullet 模型说明

这份说明是写给刚接触建模和仿真的同学看的。你可以把这个项目理解成：

- `uav_urdf_export_v1` 是无人机模型
- `solidworks_inertia/arm_urdf_export_v2` 是机械臂模型
- `load_models_pybullet.py` 是把它们放进 PyBullet 里展示出来的 Python 程序

## 1. 这个项目里有什么

### 无人机部分

无人机模型在：

- `uav_urdf_export_v1/urdf/uav_urdf_export_v1.urdf`
- `uav_urdf_export_v1/meshes/`

这里面有：

- `body.STL`：无人机机身的三维外形
- `rotor0.STL` 到 `rotor3.STL`：4 个旋翼的三维外形

### 机械臂部分

机械臂模型在：

- `solidworks_inertia/arm_urdf_export_v2/urdf/arm_urdf_export_v2.urdf`
- `solidworks_inertia/arm_urdf_export_v2/meshes/`

这里面有：

- `arm_base.STL`：机械臂底座外形
- `arm_link_1.STL` 到 `arm_link_5.STL`：机械臂各节连杆外形

## 2. 建模文件到底在讲什么

可以把整个模型想成“积木 + 说明书”：

- `STL` 文件像积木块，负责“长什么样”
- `URDF` 文件像装配说明书，负责“这些零件怎么连起来、重量是多少、关节怎么转”

### STL 文件做什么

`STL` 只负责几何外形，也就是“看起来像什么”。

比如：

- 机身是什么形状
- 旋翼是什么形状
- 机械臂每一节是什么形状

它不负责告诉程序“这个零件有多重、怎么运动”。

### URDF 文件做什么

`URDF` 是机器人描述文件。它会写清楚：

- 每个零件叫什么名字
- 每个零件有多重
- 转动惯量是多少
- 哪个零件和哪个零件连接
- 关节是绕哪个方向转
- 零件用哪个 `STL` 文件来显示

## 3. 参数写在哪里

项目里最重要的参数主要写在两个 `URDF` 文件里。

### 3.1 外形文件写在哪里

在 `URDF` 里会看到这样的内容：

```xml
<mesh filename="package://uav_urdf_export_v1/meshes/body.STL" />
```

这句话的意思是：

- 显示这个零件时，要去找 `body.STL` 这个三维模型文件

机械臂也是同样的写法。

### 3.2 质量和惯性参数写在哪里

在 `URDF` 里会看到：

```xml
<inertial>
  <mass value="0.55051" />
  <inertia ixx="..." iyy="..." izz="..." />
</inertial>
```

这些参数的意思是：

- `mass`：质量
- `inertia`：转动惯量，决定物体转起来“费不费劲”

这些参数对仿真很重要，因为物理引擎就是靠这些数值算运动的。

### 3.3 关节参数写在哪里

在 `URDF` 里还会看到：

```xml
<joint name="joint0" type="revolute">
  <parent link="body" />
  <child link="rotor0" />
  <axis xyz="0 0 -1" />
</joint>
```

这表示：

- 这是一个转动关节
- 它把父零件和子零件连接起来
- `axis` 表示绕哪个方向转

机械臂的每一节连杆之间也是这样连接起来的。

## 4. 这个项目是怎么导入 PyBullet 的

导入工作主要由 `load_models_pybullet.py` 完成。

你可以把它理解成 5 步：

### 第 1 步：找到模型文件

程序开头先写好了两个模型文件的位置：

- 无人机：`uav_urdf_export_v1/urdf/uav_urdf_export_v1.urdf`
- 机械臂：`solidworks_inertia/arm_urdf_export_v2/urdf/arm_urdf_export_v2.urdf`

### 第 2 步：把 `package://...` 路径改成 PyBullet 能读懂的路径

SolidWorks 导出的 `URDF` 里，`STL` 路径长这样：

```xml
package://uav_urdf_export_v1/meshes/body.STL
```

但 PyBullet 更喜欢直接看到本地路径，所以程序会在运行时自动把它改成电脑上的实际路径。

注意：

- 这一步不会破坏原始模型文件
- 程序只是临时生成一个给 PyBullet 用的副本

### 第 3 步：连接到 PyBullet

程序会先启动 PyBullet：

- 有界面模式：可以看到窗口
- 无界面模式：只在后台跑

### 第 4 步：把无人机和机械臂加载进来

程序会用 `p.loadURDF(...)` 载入两个模型。

现在的做法是：

- 先分别加载无人机和机械臂
- 再用固定约束把机械臂底座挂到无人机机身下方

这样做的好处是：

- 不需要手工重新合并两个 `URDF`
- 更容易调试安装位置

### 第 5 步：显示仿真画面

程序会：

- 加入地面
- 设置重力
- 设置相机位置
- 让旋翼和机械臂关节做简单演示动作

这里的动作主要是为了“看得见模型在动”，方便确认导入成功。

## 5. 为什么现在无人机不会自己飞

因为当前脚本主要是“展示模型”，不是完整飞控仿真。

它现在做了：

- 让旋翼看起来在转
- 让机械臂关节摆动

它还没有做：

- 电机推力计算
- 姿态控制
- 升力和力矩控制

所以它现在是“能看到组合模型”，但不是“真正会飞的无人机控制系统”。

## 6. 怎么运行这个仿真

先安装依赖：

```bash
python -m pip install pybullet
```

然后进入项目根目录运行：

```bash
cd E:\srp2026\srpyjj\test_srp
python load_models_pybullet.py
```

如果只想后台运行，不打开窗口：

```bash
python load_models_pybullet.py --headless --steps 1000
```

## 7. 如果你想调整机械臂安装位置

程序里默认把机械臂挂在无人机下方。

你可以用这个参数调机械臂上下位置：

```bash
python load_models_pybullet.py --arm-offset-z -0.15
```

这个数值越小，机械臂通常会挂得越靠下。

## 8. 一句话总结

这个项目做的事情可以简单理解成：

1. 用 `STL` 保存“长什么样”
2. 用 `URDF` 保存“怎么连接、质量多大、关节怎么转”
3. 用 `load_models_pybullet.py` 把无人机和机械臂导入 PyBullet
4. 在 PyBullet 里把它们显示出来，做一个基础仿真展示

如果以后要继续升级，这个项目还能继续往下做：

- 真正的四旋翼起飞控制
- 机械臂末端抓取
- 相机传感器
- 碰撞检测
- 自动控制算法

## 测试github