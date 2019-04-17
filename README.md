# Keystroke
This package republishes local keystrokes as ROS2 messages.

# Example output
To get started, `ros2 launch keystroke listen.launch.py`

```
C:\Users\dan\Documents\ros2_ws>ros2 launch keystroke listen.launch.py
[INFO] [launch]: process[keystroke_listen.EXE-1]: started with pid [22940]
[INFO] [keystroke_listen]: To end this node, press the escape key
[INFO] [keystroke_listen]: pressed 'a'
[INFO] [keystroke_listen]: pressed 'e'
[INFO] [keystroke_listen]: pressed 'u'
[INFO] [keystroke_listen]: pressed space (32)
[INFO] [keystroke_listen]: pressed shift_r (161)
[INFO] [keystroke_listen]: pressed f1 (112)
[INFO] [keystroke_listen]: pressed esc (27)
[INFO] [keystroke_listen]: stopping listener
[INFO] [launch]: process[keystroke_listen.EXE-1]: process has finished cleanly
```

## ROS API
#### keystroke_listen Node
Run with `ros2 run keystroke kesytroke_listen`

Parameters:
- exit_on_esc (boolean, default True): Whether the escape key should cause this node to exit

Published Topics:
- /glyphkey_pressed (String) - Pressing a letter/number/symbol key
- /key_pressed (UInt32) - Pressing spacebar, ctrl, F1, etc.
    
