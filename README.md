# 2025狗雄
以下是提交代码规范与准则

**本地VScode书写**
在本地环境书写时，请先拉取main分支最新代码
git pull origin main

**书写后，在本地创造一个新分支，名称随意**
git checkout -b branchname

**将修改文件提交到暂存区**
git add .

**确认提交情况**
git status

**然后可以commit了**
git commit -m "Your message"

**然后将这个新分支，推送到Github上，提交PR**
git push origin branchname

**Attention：**
如果代码书写了很久，请在push之前先切换到main分支，拉取一次最新代码，并merge到你当前的工作分支后，再进行提交

**提交PR：**
在push了你的代码后，回到网页 仓库 点击Pull requests，按道理就会显示你刚刚提交的新分支 详情见 https://zhuanlan.zhihu.com/p/584834288 从第七点开始看

在提交完之后，记得删除github上的分支。本地分支随意，记得同步最新代码即可。
