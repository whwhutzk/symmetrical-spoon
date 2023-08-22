#join
myTuple = ("Bill", "Steve", "Elon")
x = "&".join(myTuple)
print(x)

words = ['Hello', 'world', 'this', 'is', 'Python']
sentence = ' '.join(words)
print(sentence)  # Output: "Hello world this is Python"

#split
txt = "welcome to China"
x = txt.split( )
print(x)

sentence = "Hello world this is Python"
words = sentence.split()  # 默认使用空格作为分隔符
print(words)  # Output: ['Hello', 'world', 'this', 'is', 'Python']

numbers = "1-2-3-4-5"
number_list = numbers.split('-')  # 使用'-'作为分隔符
print(number_list)  # Output: ['1', '2', '3', '4', '5']

#pow
x = pow(2, 3)  # 计算 2 的 3 次幂
print(x)  # Output: 8

x = pow(2, 8, 5)  # 计算 2 的 8 次幂对 5 取模
print(x)  # Output: 1 (2^8 % 5 = 256 % 5 = 1)
