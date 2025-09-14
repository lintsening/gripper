# gripper
台師大機電系115級機械手臂專題—夾爪部份


8.19 嘗試梳理架構邏輯
node（父類別）-> servo_controller_node （抽象類別）->  servo_press_node、servo_rotate_node（實際施做的節點）
node（父類別）-> servo_monitor_node （抽象類別）->  monitor_press_node、monitor_rotate_node（實際施做的節點）

gripper_control/
├── gripper_control/
│   ├── __init__.py
│   ├── servo_control_node.py
│   ├── servo_control           (控制程式)
│   │   ├── servo_controller.py     (父 / 抽象)
│   │   ├── servo_press_node.py 
│   │   ├── servo_rotate_node.py
│   ├── servo_monitor           （監控程式）
│       ├── servo_monitor_node.py   （父 / 抽象）
│       ├── monitor_press_node.py
│       ├── monitor_rotate_node.py
├── launch/
│   ├── gripper.launch.py
│   ├── push_motor.launch.py  # 若存在
│   ├── rotate_motor.launch.py  # 若存在
├── package.xml
├── setup.py
├── setup.cfg