# basler_camera
basler_camera


  
  1,Install the Ubuntu or Kubuntu system on 20.04.4.
  
  2,Install the ros-noetic-desktop-full from https://wiki.ros.org/noetic/Installation/Ubuntu
  
  3,Insatll the qt-opensource-linux-x64-5.12.12.run Install package.
  

  chmod +x  qt-opensource-linux-x64-5.12.12.run
  
  sudo ./qt-opensource-linux-x64-5.12.12.run
  

  4，Install Basler software：
  
  To install the Basler blaze SDK:

  1. Change to the extracted directory, e.g.,:
       cd ./Basler_blaze_SDK-4.2.1.1-c7351c9c-Linux_x86_64_setup

  2. Extract Basler_blaze_SDK-4.2.1.1-c7351c9c-Linux_x86_64.tar.gz to
     /opt/Basler_blaze:  
       sudo mkdir -p /opt/Basler_blaze  
       sudo tar -C /opt/Basler_blaze \  
         -xzf ./Basler_blaze_SDK-4.2.1.1-c7351c9c-Linux_x86_64.tar.gz
         
  QT:
  
  1.Set LD_LIBRARY_PATH to /opt/Basler_blaze/lib/gentlproducer/gtl
  
  2.Set ROS_MASTER_URI to http://localhost:11311
  
  3.sudo gedit etc/ld.so.conf
  
  (add)include /etc/ld.so.conf.d/*.conf  
              	/opt/Basler_blaze/library/CPP/lib/Linux64_x64/
  
    sudo ldconfig
    
    ---
Question:Failed to load the producer file ProducerBlaze.cti: /opt/pylon/lib/pylonCXP/bin/ProducerBlaze.cti: 无法打开共享对象文件: 没有那个文件或目录
Answer:（拷贝ProducerBlaze.cti到前文件夹）  
1，jt@JT:~$ sudo find / -name ProducerBlaze.cti  
[/opt/Basler_blaze/lib/gentlproducer/gtl/ProducerBlaze.cti]  
2,add the driver of basler to etc/ld.so.conf.d/basler.conf  
```
/opt/Basler_blaze/library/CPP/lib/Linux64_x64  
/opt/Basler_blaze/lib  
/opt/Basler_blaze/lib/gentlproducer/gtl  
/opt/Basler_blaze/lib/gentlproducer/gtl/ProducerBlaze.cti  
```


  
  
  windows:
  
  https://www.baslerweb.com/cn/downloads/software-downloads/software-pylon-6-3-0-windows/
  
  https://www.baslerweb.com/cn/downloads/software-downloads/pylon-supplementary-package-for-blaze-1-2-0-windows/

  linux:
  
  https://www.baslerweb.com/cn/downloads/software-downloads/software-pylon-7-0-0-linux-x86-64bit/( we don't use this,because 7.0 has not blaze Viewer)
  
  https://www.baslerweb.com/cn/downloads/software-downloads/software-pylon-6-3-0-linux-x86-64bit-debian/
  
  https://www.baslerweb.com/cn/downloads/software-downloads/pylon-supplementary-package-for-blaze-1-2-0-linux-x86-64-debian/
  （SKD）
  https://www.baslerweb.com/baslermedia/blaze-getting-started/


  ROS:
  
  https://www.baslerweb.com/cn/company/news-press/news/pylon-camera-driver-for-robot-operating-system-2/741185/

  Book:
  
  https://docs.baslerweb.com/blaze-101.html


        mkdir test   （ 创建文件夹test）
        cd test        （ 切换到test目录） 
        touch a.md （ 新建a.md文件）

        1，git init             初始化        
        2，git status       查看状态     
        3，git add            提交缓存区：使用：git add [文件名] 或 git add .
        4，git commit      提交信息：   使用：git commit -m “first commit”
        5，git log             查看commit记录  窗口退出:'q'
        6，组合：git add . && git commit .       一般不建议，防止误提交
        7，git branch       查看分支情况
        8，git branch a    新建分支a
        9，git checkout a 切换到分支a
        10，组合：git check -b a  新建完分支a后，直接切换到分支a
        11，git merge a    第一步，先切换到master分支上，第二步执行git merge a，意思就是把a分支的代码合并过来。
        12，git branch -d a   删除a分支
        13，git branch -D a   强制姗迟a分支
        14，git tag            有了标签就可以顺利从v1.1切换到v1.0的代码，先创建v1.0：git tag v1.0  代码有所修改后，再创建v1.1：git tag v1.1
        15，git checkout v1.0  切换tag
        16，ssh                              查看是否安装ssh
        17，ssh-keygen -t rsa    指定 rsa 算法生成密钥， 接着连续三个回车键（ 不需要输入密码） ， 然后就会生成两个文件 id_rsa 和 id_rsa.pub ， 而 id_rsa 是密钥，id_rsa.pub 就是公钥。 这两文件默认分别在如下目录里生成：C:\Users\Tekken01\.ssh
        18，git push origin master     把本地代码推到远程 master 分支
        19，git pull origin master        把远程最新的代码更新到本地。 一般我们在 push 之前都会先 pull ， 这样不容易冲突。
        20，git clone git@github.com:stormzhang/test.git                           把 clone 命令理解为高级点的复制，从服务器下载代码
        21，git remote add origin git@github.com:stormzhang/test.git      在本地添加一个远程仓库
        22，git remote -v            查看当前项目有哪些远程仓库
        23，git push origin master      有了远程仓库，我们本地的仓库就可以向远程仓库递交代码了
        24，git config —global user.name "stormzhang"                                设置用户名
        25，git config —global user.email "stormzhang.dev@gmail.com"     设置邮箱
        26，git config --global user.name      查看用户名 （分为global,system,local ）
        27，git config --global user.email     查看邮箱        (glovbal文件的)              
        28，git reset 5749de    重新定位到某一次的commit           

        alias   （在git bash进行操作，输入vi .gitconfig便可看到alias所配置的内容，本地路径是：C:\Users\Tekken01\.gitconfig）
        [user]
                name = TekkenYang
                email = yangtekken@gmail.com

        [alias]
                co = checkout
                ci = commit
                st = status
                br = branch
                ...

        1，git branch -m <oldbranch> <newbranch>  修改分支名称，如：git branch -m main master
        2，git config --global alias.co checkout # 别名
        3，git config --global alias.ci commit
        4，git config --global alias.st status
        5，git config --global alias.br branch
        6，git config --global alias.psm 'push origin master'
        7，git config --global alias.plm 'pull origin master'
        8，强大的日志功能：



        git log --graph --pretty=format:'%Cred%h%Creset -%C(yellow)%d%Creset %s %Cgreen(%cr) %C(bold blue)<%an>%Creset' --abbrev-commit --date=relative
        git config --global alias.lg "log --graph --pretty=format:'%Cred%h%Creset -%C(yellow)%d%Creset %s %Cgreen(%cr) %C(bold blue)<%an>%Creset' --abbrev-commit --date=relative"

        9，  git config --global core.editor "vim" # 设置Editor使用vim
        10，git config --global color.ui true


        [color]
            ui = true

        11，git config --global core.quotepath false    # 设置显示中文文件名 当文件名中有中文的时候，git status便会出现“\344\270\212\347\”，设置不转义以后，可以显示中文。
        12，git diff  只能比较当前文件和暂存区文件差异， 暂存区就是还没有执行 git add 的文件，一旦git add之后，就不能显示出差异

        git diff <$id1> <$id2>        # 比较两次提交之间的差异
        git diff <branch1>..<branch2> # 在两个分支之间比较
        git diff --staged             # 比较暂存区和版本库差异


        13，git checkout <develop>  切换分支或者切换tagy可以使用git checkout， 稍微提下， checkout 命令只能撤销还没有 add 进暂存区的文件。

        git checkout v1.0
        git checkout ffd9f2dd68f1eb21d36cee50dbdd504e95d9c8f7 # 后面的一长串是commit_id， 是每次commit的SHA1值， 可以根据 git log 看到。
        14， git stash   把当前分支所有没有 commit 的代码先暂存起来（ 前提是我们的代码没有进行 commit ， 哪怕你执行了add 也没关系， 我们先执行git stash ）
        14， git status  发现当前分支很干净， 几乎看不到任何改动， 代码改动也看不见了， 但其实是暂存起来
        14， git stash list   会发现此时暂存区已经有了一条记录
        14， git stash apply   在切换会其他分支， 把bug修复好， 然后发布。 之后一切都解决了， 你再切换回来继续做之前没做完的功能，之前的代码还原：git stash apply
        14， git stash drop    紧接着最好需要把暂存区的这次 stash 记录删除， 执行：git stash drop      把最近一条的 stash 记录删除了
        14，  git stash pop    来代替 apply 命令， pop 跟 apply 的唯一区别就是 pop 不但会帮你把代码还原， 还自动帮你把这条 stash 记录删除， 省的自己再 drop 一次了， 为了验证你可以紧接着执行 git stash list 命令来确认是不是已经没有记录了。
        14， git stash clear    就是清空所有暂存区的记录， drop 是只删除一条， 当然后面可以跟 stash_id 参数来删除指定的某条记录， 不跟参数就是删除最近的， 而 clear 是清空。

        15， merge & rebase
        当在分支 featureA上开发完成之后，合并在master上：

        git checkout master
        git merge featureA
        其实 rebase 命令也是合并的意思， 上面的需求我们一样可以如下操作：

        git checkout master
        git rebase featureA

        rebase 跟 merge 的区别你们可以理解成有两个书架， 你需要把两个书架的书整理到一起去， 第一种做法是 merge ， 比较粗鲁暴力， 就直接腾出一块地方把另一个书架的书全部放进去， 虽然暴力， 但是这种做法你可以知道哪些书是来自另一个书架的； 第二种做法就是rebase ， 他会把两个书架的书先进行比较， 按照购书的时间来给他重新排序， 然后重新放置好， 这样做的好处就是合并之后的书架看起来很有逻辑， 但是你很难清晰的知道哪些书来自哪个书架的。只能说各有好处的， 不同的团队根据不同的需要以及不同的习惯来选择就好。

        16，解决冲突
        Git 本身他没法判断你们两个谁更改的对， 但是这个时候他会智能的提示有 conflicts ， 需要手动解决这个冲突之后再重新进行一次 commit 提
        交。
        
       
Add ssh:
```ssh-keygen -t rsa -C "yangtekken@gmail.com"```

相机IP:   192.168.100.2 

工控机IP: 192.168.100.3 

master分支： 

1. 用于更新最稳定版本的识别算法(仅田字型托盘)；

Branch_ScanWay: 

1. 最稳定版本的识别算法(仅田字型托盘，栈孔识别法)；

Branch_ScanAll: 

1. 用于兼容识别田字型托盘和川字型托盘(2023年5月5日开始进行--2023 5月6日完成)；





