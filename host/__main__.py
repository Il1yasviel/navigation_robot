"""上位机 GUI 入口。

两种启动方式等价：
    python -m host        （在仓库根目录）
    直接运行本文件        （如 VS Code 的“运行”按钮、python host/__main__.py）
"""
import sys
from pathlib import Path

# 按路径直接执行本文件时，sys.path[0] 是 host/ 目录自身，解释器找不到
# host 包；把仓库根目录（本文件的上一级）补进搜索路径。
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from host.app import main

if __name__ == "__main__":
    main()
