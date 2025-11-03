# lib - flight control components
# Some parts modified from original repo for the RL UAV project.

## Quick Start

```python
from lib import *

# load size-based config 加载尺寸配置
config = get_preset_config("5in")  # 2in, 3in, 5in, 7in

# init components
controller = RateController3D(*config.get_pid_gains_tuple())
mixer = QuadXMixer()
esc_output = DshotOutput(motor_count=4, dshot_speed=600)
rate_mapper = RateMapper(*config.rate_mapping)

# control loop
rate_targets = rate_mapper.map_sticks_to_rates(roll, pitch, yaw)
commands = controller.update(rate_targets, gyro_rates, dt)
motors = mixer.mix(throttle, *commands)
motors = desaturate_and_clip(motors)
esc_values = esc_output.process_outputs(motors)
```

## Structure

```
lib/
├── controllers/   # PID控制器
├── mixers/        # 混控器
├── outputs/       # 电调输出
├── utils/         # 工具函数
└── configs/       # 配置系统
```

## Size Presets 尺寸预设

- **2in**: Micro quads 微型四轴 (900°/s, DShot300)
- **3in**: Racing 竞速 (800°/s, DShot600) 
- **5in**: Standard 标准 (600°/s, DShot600)
- **7in**: Long range 远航 (300°/s, PWM)

## Components

### Controllers
- `RatePID`: Single axis PID
- `RateController3D`: 3-axis rate control

### Mixers
- `QuadXMixer`: X configuration
- `QuadPlusMixer`: + configuration
- `desaturate_and_clip()`: Motor desaturation

### Outputs
- `PWMOutput`: Standard PWM
- `DshotOutput`: DShot150/300/600/1200
- `OneShot125Output`: OneShot125

### Utils
- `AxisRates`: Rate data structure
- `RateMapper`: Stick to rate mapping
- `clamp()`, `lowpass()`: Math utils

## Demo

```bash
python demo_simple.py
```

## Integration
将 `lib/` 文件夹放入项目中，导入所需组件即可。