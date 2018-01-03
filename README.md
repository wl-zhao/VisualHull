

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



前三点优化后运行时间如下：

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

进一步我利用totalInside来判断该点是不是全在visullHull内部，这里全在的意义是，只管来说，以该点为中心的一个三阶魔方的所有块全在里面。

> 一开始我判断全在内部用的是六个方向，但是这样很容易出现误判。



使用surfaceList保存所有的surface点，理论上可以提高save without normal 和save with normal 的速度。但是实际上效果并不算太好。

| save without normal | 2.132 |      |      |
| ------------------- | ----- | ---- | ---- |
| save with normal    |       |      |      |



其他可能的优化思路：

- totalInside可以进行优化，找到可以得到有效点云的最小取法。事实上我认为直接取角块就可以。waiting for testing

- save with/without normal二者并无直接联系。事实上，主线程如下：

  ```flow
  st=>start: start
  ld=>operation: load file
  e=>end: end
  gm=>operation: get model
  gs=>operation: get surface
  wn=>condition: save with normal?
  sn=>operation: save with normal
  swn=>operation: save without normal
  sm=>operation: save mesh.ply
  st->ld->gs->wn
  wn(yes)->sn
  wn(no)->swn->e
  sn->sm->e

  ```




​	





​	从图中可以看出，save with normal 与 save without normal 是并行的，可以用两个进程运行。

-  可以考虑减少surface中的有效点，例如隔一个一取。
-  计算法向时可以考虑简化计算，不要每个点都遍历一遍。
-  有一步骤需要判断inner点or neibor点，这一步骤可以综合到get surface中完成？可能会消耗大量的存储空间。
-  可采用类似哈希的线性试探法或平方试探法确定第一个点的位置。经过测试发现，如果给定了第一个点的位置，get surface 可达0.5s的量级！可见一个快速确定初始位置的算法十分有用。
-  get normal 中，原来获取内部点中心的算法有一些无用功。我使用递推的方法在一个循环中就求出了中心点。这样可以减少总的时间0.5s左右。把握好一个原则，由于总的表面点数量较多，在大循环中即便是一些微不足道的优化，乘上总数之后，优化效果依旧可观。

