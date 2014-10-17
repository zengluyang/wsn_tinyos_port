make clean
make cc2430em
## 权限问题： permission denied
chmod -R 777 ./
make cc2430em
## KEIL: ERROR
make cc2430em | grep ERROR
## 修改build/cc2430em/app.c 对应位置的错误
cd build/cc2430em
ls
./CC2430-F128.bat
##.hex出现 就说明成功了 烧录
