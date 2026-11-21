# render-camera-path

## 题面

过场动画换成样条相机后，起停一顿一顿、弯道还越走越快：端点切线被置零，相机在首尾停住；切线不归一，朝向抖动；look_at 叉积顺序反了，朝向上下颠倒；弧长用直线距离累加，弯道偏短；重采样按参数均分，速度不均；速度曲线端点不减速，起停生硬；up 与 forward 平行时不报错，矩阵奇异；重复控制点不报错，切线为 NaN；批量采样共用同一份缓冲，路径之间串数据；count<2 不校验，直接返回空。campath/path.py 是核心，tests/test_path.py 15个红。要修的是：把以上 10 个缺陷逐一修好，使 15 个测试全部通过，同时保持公开契约与容差不变。

## 运行方式

在仓库根目录（`render-camera-path/`）执行：

```text
python -m pytest tests -q
```

初始状态下恰好 15 个测试失败，且不应出现 collection、import 或 syntax error；输出尾部为：

```text
15 failed in <耗时>s
```

修复目标同样是 `python -m pytest tests -q` 输出 `15 passed`。

## 字段表

| 字段 | 取值 |
| --- | --- |
| 所属领域 | 游戏、图形与多媒体 |
| 子方向 | 图形渲染 |
| 任务类型 | Bugfix |
| 语言/框架 | Python 3, pytest |
| 环境可复现等级 | 无外部依赖 |

## 目录与约束

```text
render-camera-path/
  README.md
  conftest.py
  .gitignore
  campath/__init__.py
  campath/path.py
  tests/test_path.py
```

- 只使用标准库 `math` 与 `pytest`，离线运行，不使用随机数。
- 位置使用世界单位 `(x, y, z)`，时间相关量使用秒；切线为单位向量，朝向为右手正交基；比较容差为 `1e-6`。
- 非法或退化输入统一抛出 `CameraPathError`，不得静默返回 NaN、空列表或奇异矩阵。
- `tests/test_path.py` 是固定验收件，**不可修改**；所有修复只能发生在 `campath/path.py`。
