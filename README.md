# 民大附中具身智能实验室 · 红方机器人代码（第 4 阶 · 代码工作室）

这里是比赛里红方机器人真实运行的行为代码（C++，B-Human 框架）。你们可以用任何 AI 编程工具
（Claude Code、Cursor、Trae、通义灵码……）修改它，然后交给平台编译。编译和 60 秒试踢都通过后，
这个版本会自动成为你们班的比赛代码。

## 怎么提交

1. 新建一个**公开**的 GitHub 或 Gitee 仓库，把整个 `WhiteFa62/` 目录放进去（目录名不要改）。
2. 改代码，推到某个分支（比如 `dev`）。
3. 在体育场的「💻 代码工作室」里粘贴分支链接，例如 `https://github.com/用户名/仓库名/tree/dev`；
   或者直接上传包含 `WhiteFa62/` 目录的 zip 压缩包。也可以对工作室里的助手说“帮我编译这个分支”。
4. 等结果：检查 → 编译（约 1 分钟）→ 试踢 60 秒。失败时会显示出错的文件、行号和原因。

## 规则

- 只有 `WhiteFa62/` 下的 `.cpp` / `.h` 会被使用，其他文件会被忽略。
- 新增或修改的代码里不允许：读写文件、联网、执行系统命令、线程、环境变量、内联汇编
  （例如 `system(`、`fopen(`、`std::thread`、`socket(`、`getenv(`、`#include <filesystem>`）。
  机器人的代码在一个没有网络、看不到服务器文件的沙箱里运行。
- 一次最多改动 3000 行；每个班每天最多提交 20 次。
- 能编译 ≠ 踢得好：先从小改动开始，每次只改一件事，提交后在比赛中心看录像。

## 从哪里改起

| 想改什么 | 看哪个文件 |
|---|---|
| 拿到球以后怎么踢（射门、传球、带球、解围） | `SkillBehaviorControl/Skills/Ball/PlayBall.cpp` |
| 射门的方向和力度 | `SkillBehaviorControl/Skills/Ball/KickAtGoal.cpp`、`GoToBallAndKick.cpp` |
| 传球 | `SkillBehaviorControl/Skills/Ball/PassToTeammate.cpp` |
| 抢球（对手靠近时） | `SkillBehaviorControl/Skills/Ball/Zweikampf.cpp` |
| 全队的决策：谁去踢球、谁接应、站在哪里 | `Strategy/WhiteFleetStrategyBehaviorControl.cpp`、`Strategy/WhiteFleetBehavior.cpp` |
| 战术沙盘和行为工坊（第 2、3 阶）是怎么接进来的 | `WhiteFleetStrategyBehaviorControl.cpp` 里的 `applyMdfzFormation`、`applyMdfzRules` |

坐标约定：单位毫米，本方球门在 x 负方向（球门线 x = −10940），对方球门 x = +10940，场地宽 ±6940。

## 给 AI 编程工具的提示词示例

> 这是 B-Human 机器人足球的红方行为代码（C++），只能修改 `WhiteFa62/` 目录下的 .cpp/.h。
> 不能读写文件、联网、用线程或系统调用；代码会在比赛服务器上编译（clang，C++20，B-Human 框架头文件已在服务器上）。
> 我想让：离对方球门 4 米以内拿到球时直接射门，不再盘带。请找到相关代码，做最小的修改，并解释你改了什么、为什么。

本地无法编译（需要完整的 B-Human 工程），以平台的编译结果为准。把平台显示的编译错误贴给 AI 工具，让它修改。
