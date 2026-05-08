# 开发说明

### 相关开发文档

- ROS2及系统相关源码：https://github.com/MiRoboticsLab/cyberdog_ws
- 相关开发文档：https://github.com/MiRoboticsLab/blogs
- 相关开发示例：https://github.com/baizhongxing/Cyberdog_demo

### 更新运控

- 更新代码包
  MotionControl目录中解除限制,编译后的源码
1. 拷贝 cyberdog_locomotion.tar 到NX上

2. tar -xvf *.tar
- 更新运控
  cd  cyberdog_locomotion/cyberdog_locomotion-master/scripts
  sudo ./scp_to_cyberdog.sh

### 步态更新

- 运控更新步态
  说明： 真实沙盘要使用真实世界通关代码（zzl）高台.toml 和 楼梯.toml， 只不过原始代码中这俩文件名字写反了.

- 在NX上拷贝步态文件到运控板(可以变更顺序，本身toml文件内容写反了)
  
  ```bash
  scp ./楼梯.toml root@192.168.44.233:/robot/robot-software/control/motion_list/cyberdog2/preinstalled/zzl_gaotai.toml
  scp ./高台.toml root@192.168.44.233:/robot/robot-software/control/motion_list/cyberdog2/preinstalled/zzl_louti.toml
  ```

- 进入运控板
  
  ```bash
  ssh root@192.168.44.233
  cd /robot/robot-software/control/motion_list/cyberdog2/preinstalled/
  cat zzl_gaotai.toml > cyberdog2_hiFive_LFleg_posCtrl.toml
  cat zzl_louti.toml > cyberdog2_hiFive_RFleg_posCtrl.toml
  ```
  
  -查询命名空间
  ros2 topic list 
  -替换命名空间

# 知识点

### 腿式里程计

起始位置是0,0,0,0,0,0，即x=y=z=roll=pitch=yaw=0。

- [0]正数：向前走
- [1]正数：向左转
- [0]负数：向后走
- [1]正数：向右转

**例**
起始方向

```python
# 前进0.6m，根据腿式里程计

            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos<0.6:
                # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
                robot_ctrl.update_and_publish(7)
```

左转90度

```python
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos < 1.5:
                # print(f"移动中：{y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos}")
                robot_ctrl.update_and_publish(7)
```

反向180度

```python
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[0]/fix_cos - xyz[0]/fix_cos > -0.2:
                # print(f"移动中：{y_xyz_node.xyz[0]/fix_cos-xyz[0]/fix_cos}")
                robot_ctrl.update_and_publish(42)
```

右转90度，既左转270度

```python
            xyz=y_xyz_node.xyz
            while y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos > -0.8:
                # print(f"移动中：{y_xyz_node.xyz[1]/fix_cos-xyz[1]/fix_cos}")
                robot_ctrl.update_and_publish(7)
```

### 方向计算

以起始方向为准，正前方是0，逆时针方式旋转为正数。(基于右转函数，左转是否相反，待验证)
**示例：**

```python
# 方向修正，基于右转函数
            for _ in range(5):
                fix = robot_ctrl.rpy_fix_right(scan_node,y_xyz_node,0)
            fix_cos=math.cos(fix/180*math.pi)  
```

### 通过圆柱

旋转基于前进步态，vel_des[x,y,z] 

- x:前进速度，正值向前，旋转中为线速度
- y:横向移动速度，一般为0
- z:旋转速度，为角速度，正值左转，负值右转

线速度不变，角速度越大，圆越小
角度的不变，线速度越大，圆越大

例：

```python
            while abs((y_xyz_node.y+fix+360)%360-50) > 3:
                # print(f"转弯中：{(y_xyz_node.y+fix+360)%360}")
                robot_ctrl.update_and_publish(21)
```

# 标准场地

![](docs/images/1.png)
![](docs/images/2.png)
