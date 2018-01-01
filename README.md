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



### get model 优化

事实上，原来的代码十分耗时。其原理是遍历空间中的所有点，判断是否在visualHull内部。再利用getSurface函数，找那些在visualHull内部，但是有邻居不在（或出界）的点作为表面点。这种算法有两个地方不好：

- get model 和get Surface有点重复，进行了两次三层循环
- 许多点其实是没有必要判断的，浪费时间



解决思路：

- 不用get model，直接get surface
- 首先通过循环找到一个表面点（在查找的过程中顺便标记voxel），不一定遍历所有点，只要找到一个即可。
- 从这一点开始做BFS，将途中的内部点入队
- 事实上可以进一步优化，即仅仅将表面附近点入队



前两点优化后运行时间如下：

| 步骤                  | 耗时/s   |
| ------------------- | :----- |
| 读取文件                | 0.568  |
| get model           | 0.002  |
| get surface         | 1.8    |
| save without normal | 2.199  |
| save with normal    | 5.646  |
| save mesh.ply       | 6.012  |
| total               | 16.238 |

尤其是将get model和get surface的时间和缩短了近4秒



使用surfaceList保存所有的surface点，可以提高save without normal 和save with normal 的速度。

| save without normal | 2.132 |      |      |
| ------------------- | ----- | ---- | ---- |
| save with normal    |       |      |      |



