---
description: 逐轮看图，把桌上的支持物体收入相机识别的托盘。
agent: gat-table
---

处理任务：$ARGUMENTS

按gat-table规则使用tools/table_session.py完成start→poll→实际读取两张图片→submit循环。
Agent选物体和托盘空位；抓取候选与完整运动由本地symbolic model规划。
先完成在线预览，再在已有任务授权内执行；用户要求仅预览时不运动。
