# pcanfd

> A PCANFD firmware implement on stm32g431.

本项目的核心代码来自[moonglow/pcan_pro_x](https://github.com/moonglow/pcan_pro_x)，沿用了其中USB和PCAN协议相关的代码，将其移植到stm32g431平台，添加了对canfd的支持。

实现了基本的canfd报文收发功能，支持PCAN-View、PCANBasic API。

本项目代码可直接用于目前市面上廉价的canable 2.0硬件，但并未广泛测试。
