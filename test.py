x = [1,2,3,4,5,6]
print(x[0:-1])#不包括最后一个元素
print(x[4:6])#第5，6个元素

print(eval("'hello'"))

Tempstr = "102c"
print(eval(Tempstr[0:-1]))

'''
value = eval(input("请输入需要计算的数值："))
print(value*2)
import turtle
def drawSnake(radius,angle,length):
    turtle.seth(-40)
    for i in range(length):
        turtle.circle(radius,angle)
        turtle.circle(-radius,angle)
    turtle.circle(radius,angle/2)
    turtle.fd(40)
    turtle.circle(16,180)
    turtle.fd(40,2/3)
turtle.setup(650,350,200,200)
turtle.penup()
turtle.fd(-250)
turtle.pendown()
turtle.pensize(25)
turtle.pencolor("purple")
drawSnake(40,80,4)
turtle.down()
'''