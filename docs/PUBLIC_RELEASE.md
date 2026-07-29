# 公开发布说明

## 推荐仓库拆分

最终保持两个公开仓库：

1. `cyberdog-inspection-system`：机器狗巡检系统集成。
2. `MOCHA`：独立的运动规划与轨迹优化算法。

不要再拆出第三个 YOLO 仓库，除非已经从干净上游重新建立并确认所有修改的权属与许可。当前主仓库只展示 YOLO 的 ROS 2/运行时适配边界。

## 发布内容

- `ros2_packages/cyberdog_web_bridge`
- `ros2_packages/cyberdog_inspection` 的自有包装与执行代码
- `ros2_packages/cyberdog_nav2_lidar` 的配置、launch 和适配脚本
- `ros2_packages/sensor_data`
- `robot_vision/cyberdog_ai_pkg` 的自有运行时管理、消息服务和节点适配示例
- 架构、接口、演示图片和已知限制

## 排除内容

- `build/`、`install/`、`log/`
- `*.engine`、`*.onnx`、`*.pt`、`*.wts`、Paddle 模型
- 厂商 SDK、动态库、驱动源码和 Docker 镜像
- 来源未核清的 `thirdparty/`、`external/` 和旧公司代码
- 地图、现场视频、告警截图、训练数据
- 真实设备 IP、RTSP 账号密码、签名 URL、密钥和令牌

## 发布步骤

```powershell
powershell -ExecutionPolicy Bypass -File ./scripts/check_public_release.ps1
git init
git add .
git status
git commit -m "docs: publish CyberDog inspection showcase"
git branch -M main
git remote add origin <YOUR_GITHUB_REPOSITORY_URL>
git push -u origin main
```

执行 `git add .` 后必须再次查看 `git status`，确认没有模型、SDK、现场图片或秘密配置。

## GitHub 首页建议

- 仓库描述：`CyberDog + Jetson Orin + ROS 2 web-controlled autonomous inspection prototype`
- Topics：`cyberdog`、`jetson-orin`、`ros2`、`nav2`、`robotics`、`computer-vision`、`fastapi`
- README 首屏保留架构图、能力表和演示 GIF/短视频链接。
- Release 页面只放经过脱敏的演示视频，不放模型和现场数据。
- 在 Related Work 中链接 MOCHA，不使用 submodule。

## 发布后核查

- 在无登录浏览器中打开仓库，确认所有链接和图片可见。
- 使用 GitHub 搜索检查 `rtsp://`、`password`、`token`、内网 IP 和邮箱。
- 如果秘密曾经进入 Git 历史，仅删除工作区文件不够；需要轮换秘密并清理历史。
