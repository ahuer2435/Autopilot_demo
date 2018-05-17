# test 包功能：
主要用于在开发过程中，测试使用。

## 测试节点1：
 main.cpp print.c print.h 用于测试cpp调用c库或者代码中的函数时，此时需要使用
```c++
#ifdef __cplusplus
extern "C" {
#endif
#ifdef __cplusplus
}
#endif
```
告诉cpp编译器，这些函数是源于c，编译器使用c风格编译所要调用的函数名。因为c和cpp在编译时，对函数名的处理方式不同，c是一个地址名，cpp是要包含参数及其类型的（函数重载）。
