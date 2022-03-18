# 找不到ceres

## 安装ceres

[ceres安装网站](http://ceres-solver.org/)

按照网站上先安装必要的依赖

然后`git checkout 1.14.0`，这里是一个tag，标记了版本1.14.0，我们要转到这个版本，然后创建新分支`git checkout -b ceres_1.14.0`，然后再cmake, make, make test, make install等等，这里之所以不用最新版是因为会报错

# 找不到llcm

大概是Blue_lcm里的报错

直接安装lcm

[lcm github地址](https://github.com/lcm-proj/lcm)

## pcap.h

这里是引用了头文件但没找到，因为你本来就没有，下一个就好了

`sudo apt-get install libpcap-dev`

# 其他问题

- roslaunch blue_lcm

Error while loading shared libraries: liblcm.so.1: cannot open shared object file: No such file or directory

Solved this by `sudo ldconfig -v`
