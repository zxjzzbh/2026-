# 队员提交指南

## 1. 开始前

- 安装 Git，登录自己的 GitHub 账号。HTTPS 可使用 Git Credential Manager 或 GitHub Desktop 登录，不把密码或 token 写进仓库地址。
- 仓库管理员在 GitHub 仓库 Settings → Collaborators 中邀请队员；队员接受邀请后才有直接 push 权限。具体页面名称以 GitHub 当前界面为准。
- 没有写入权限时，可 fork 后向本仓库提 Pull Request；无需分享账号或密钥。
- 在 [团队分工表](docs/team.md) 认领模块。成员变动通过修改表格记录，不预设任何人的姓名或权限。

## 2. 首次下载仓库

在准备存放代码的文件夹打开终端，执行：

```bash
git clone https://github.com/zxjzzbh/2026-.git
cd 2026-
git status
```

如果 Git 提示未设置提交身份，用自己的姓名和邮箱设置当前仓库身份。以下引号内容必须替换，也可使用 GitHub 提供的 noreply 邮箱：

```bash
git config user.name "你的姓名或昵称"
git config user.email "你的邮箱或GitHub noreply邮箱"
```

不需要复制整个学习资料文件夹，也不要在项目内再次 `git init`。

## 3. 每个任务使用独立分支

先 `git status`，把已有工作提交在原任务分支，或妥善暂存后再切换。干净工作区执行：

```bash
git switch main
git pull --ff-only origin main
git switch -c feat/vision-video-reader
```

分支名用英文、数字和短横线。一项任务一条分支，例如：

| 工作 | 分支示例 | 主要目录 |
|---|---|---|
| 视觉录像读取 | `feat/vision-video-reader` | `vision/` |
| 模式切换 | `feat/vehicle-mode-switch` | `vehicle/` |
| 编码器读取 | `feat/firmware-encoder` | `firmware/` |
| 视频回传 | `feat/communication-video` | `communication/` |
| 接线图 | `docs/hardware-wiring` | `hardware/` |

名字重复时添加自己的昵称。已合并的任务结束后，下次从最新 `main` 新建分支。

## 4. 修改、检查、提交、上传

以视觉模块为例：

```bash
git status
git add vision/
git diff --cached
git commit -m "feat(vision): 增加录像输入与结束处理"
git push -u origin feat/vision-video-reader
```

- `git add` 只选择本次任务需要的路径；修改接口文档时再添加 `interfaces/` 等路径。
- 首次 push 用 `-u`；同一分支后续通常 `git push` 即可。
- `commit` 保存本地历史；`push` 上传分支；Pull Request 合并后才进入 `main`。
- 被忽略的文件不会随普通 add 上传。用 `git check-ignore -v 路径` 查原因；不要为上传运行产物随意强制 add。

## 5. 建立 Pull Request

打开仓库的 Pull requests → New pull request，选择 `base: main`、`compare: 自己的分支`。写清：

1. 完成了什么，修改了哪些目录。
2. 怎么运行，依赖什么硬件/环境。
3. 做了什么验证，哪些仍未测。
4. 是否改变跨模块接口，需要谁配合。

建议至少让一位相关队员看过再合并。**这是团队协作建议，目前并未自动开启分支保护或强制审批规则。** 本次仓库骨架作为初始化提交直接进入 main；后续日常开发采用上述流程。

合并后，确保工作区干净，再同步：

```bash
git switch main
git pull --ff-only origin main
```

## 6. 开发过程中同步其他人的修改

先在自己的分支保存/提交当前工作，然后：

```bash
git fetch origin
git merge origin/main
```

若冲突，逐个打开冲突文件，与相关队员确认最终内容，移除 `<<<<<<<`、`=======`、`>>>>>>>` 标记，检查后 `git add` 对应文件并 `git commit`。本次 merge 不想继续时可以 `git merge --abort`。不要使用强推、覆盖整个目录或 `reset --hard` 解决不理解的冲突。

## 7. 不使用命令行的方式

GitHub Desktop 可以 clone 仓库、新建分支、查看修改、commit、publish/push 分支，再打开 Pull Request。少量文档也可在 GitHub 网页编辑并选择新建分支。完整工程仍建议本地 Git，方便检查是否遗漏文件。

## 8. 文件与目录约定

- 目录/代码文件尽量使用英文；说明文档用中文；文本 UTF-8、换行 LF。
- 同一模块统一放在对应目录，不在根目录创建“张三最终版”“最终版2”等文件夹。
- 个人实验用 `experiments/日期-昵称-主题/`，成熟后迁移回正式模块，避免长期重复维护两套源码。
- 模块依赖由模块自己的 `requirements.txt`、工程文件等说明，不要一开始把所有语言和平台依赖混在根目录。
- 源码、依赖声明、必要的工程文件、可公开的小样例应提交；依赖安装目录、编译输出、录像、日志和模型权重不直接提交。
- 必要的第三方源文件保留许可证与来源；没有授权的资料用索引描述，不重新发布。

## 9. 提交前检查

- 目标目录正确，模块 README 有运行方法、输入/输出和当前限制。
- 做了与变更相称的验证；只改文档无需编写程序测试。
- 没有真实密码、token、Wi-Fi 连接文件或含账号信息的日志。
- 大数据用 [数据索引](data/catalog.csv)，模型用 [模型索引](models/catalog.csv)，链接应可供队员访问且不含临时凭证。
- 提交信息可以用 `feat(模块): ...`、`fix(模块): ...`、`docs(模块): ...`，清楚描述结果即可。

若敏感文件已被 Git 跟踪，添加 `.gitignore` 不能清除已有历史；停止继续上传，通知仓库管理员处置相关凭证和历史。
