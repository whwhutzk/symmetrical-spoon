x = [1,2,3,4,5,6]
print(x[0:-1])#不包括最后一个元素
print(x[4:6])#第5，6个元素

print(eval("'hello'"))

Tempstr = "102c"
print(eval(Tempstr[0:-1]))


value = eval(input("请输入需要计算的数值："))
print(value*2)

import turtle
def drawSnake(radius,angle,length):#定义函数
    turtle.seth(-40)#画笔方向
    for i in range(length):
        turtle.circle(radius,angle)#radius半径，正在左，负在右，angle为绘制弧形的角度
        turtle.circle(-radius,angle)
    turtle.circle(radius,angle/2)
    turtle.fd(40)#turtle前进距离
    turtle.circle(16,180)
    turtle.fd(40,2/3)
turtle.setup(650,350,200,200)#设置主窗体宽度、高度、上边距、左边距
turtle.penup()#抬起画笔，不绘制形状
turtle.fd(-250)
turtle.pendown()#落下画笔，绘制形状
turtle.pensize(25)#设置画笔尺寸
turtle.pencolor("purple")#设置画笔颜色
drawSnake(40,80,4)#向函数传参
turtle.done()