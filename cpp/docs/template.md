# 核心
形成一个可以直接复制/拉新分支开始 2026 的干净仓库。

# 介绍
1.以ctre swerve with pathplanner为基础
,使用ctre generate severve进行底盘控制
2.保留vision subsystem，保留全场定位技术和gb detection物体识别技术
3.保留整体架构complex commond包含subsystem，并以各自subsystem的command主合为complex common
4.增加examplesubsystem介绍文档以及保留的subsystem的使用文档

# 去除
1.constant中的二五赛季常见变量
2.去除25赛季特有子系统：elevator、upper、intake、quest
