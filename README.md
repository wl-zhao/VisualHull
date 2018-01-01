# VisualHull

### 优化方案与步骤

- 阅读代码，了解思路
- 利用时钟，记录每一步骤的耗时
- 分步骤降低复杂度



| 步骤                  | 耗时/s  |
| ------------------- | :---- |
| 读取文件                | 0.717 |
| get model           | 6.601 |
| get surface         | 0.218 |
| save without normal | 2.685 |
| save with normal    | 5.066 |
| save mesh.ply       | 6.643 |



可见最耗时的的步骤是get model

优化顺序：

1. get model
2. save mesh.ply
3. save with normal
4. save without normal
5. 其他的可以暂不考虑

#### 阳神的：



| 步骤                  | 耗时/s  |
| ------------------- | :---- |
| 读取文件                | 0.643 |
| get model           | 0.003 |
| get surface         | 2.382 |
| save without normal | 3.007 |
| save with normal    | 7.523 |
| save mesh.ply       | 6.827 |