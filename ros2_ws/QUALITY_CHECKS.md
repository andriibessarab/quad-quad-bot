# ROS 2 Quality Checks

All workspace quality checks are run by one script:

```bash
ros2_ws/scripts/quality.sh
```

This runs only on `ros2_ws` and applies:
- Python lint/format (`ruff`)
- C/C++ format (`clang-format`)
- CMake format/lint (`cmake-format`, `cmake-lint`)

First-time setup:

```bash
python -m pip install --upgrade pip
pip install ruff cmakelang
sudo apt-get update && sudo apt-get install -y clang-format
```

CI uses non-edit mode:

```bash
ros2_ws/scripts/quality.sh check
```
