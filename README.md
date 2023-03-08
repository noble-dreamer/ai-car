# ai-car
采取了新的底层库,所以代码有所更改,主要算法还是延续上一届

更改处:
  
  1.所有typedef struct的声明全部放在了main.h等等头文件中.所有头文件用且只用来声明和宏定义
  
  2.所有用到的全局变量全部在main.c文件中定义,其他.c文件均采用extern xxx xxx的方式避免重复定义
  
  3.采用了面向对象的方法.所有全局变量可以看作是基类中变量.其余的car_control类,跟location类等从各个全
 局变量,通过指针索引方式,读写基变量,避免反复调用全局变量,造成难以修改和维护的后果.
  使用方法见motor.c文件,具体表现为实例化后,通过this指针进行读写.
  
  
  Attention: encoding with Simple Chinese GB2312. 如果看见乱码可能是因为使用vscode编写时默认使用了UFT-8编码(并非在Keil中编程,而是利用vscode编写,使用keil编译调试).
