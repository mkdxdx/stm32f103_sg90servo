# stm32f103_sg90servo
laser turret thingy

In this project i was trying to get servos to work with STM32F103C8T6. As a nice addition, it can be controlled via USART
with command format like "Xxxx,Yyyy;" with values ranging from -127 to 127 as axis acceleration value.

I've later added HC-06 module and was able to control it with smartphone and turn it into laser turret for my cat. Cuz cats love lasers.

Unit looks like this https://www.youtube.com/watch?v=366nlRmT59c

There is also python script to control it with PC, you are gonna also need USB-RS232 converter of sorts. Baud rate is at 9600.
