import sys
import io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
from bug_journey import save_error,view_log
import time
"""def c(age):
    if age > 18:
       return 
         
    else :age < 18
    return False
result = c(17)
re = 111
print(111)
def judage( ):
   if  result is False:
    print("yes")
name =['a','b','c','d']
for i in range(3):
   print(name[i])
for i in range(3):
   print(name[-i ]) 
mylist = [1,2,3,3]

mylist[1] = 0
index = mylist.index(0)
print(f"{index}")
mylist.pop(1)
index = mylist.index(3)
print(f"{index}")
mylist.insert(0,3)
index =  mylist.index(3)
print(f"{index}")
del mylist[0]
mylist.insert(0,2)
print(f"{index}")
index = mylist.index(2)
print(f"{index}")
str = 'i is the god'
Value = str[1]
print(Value)
new = str.split()
print(new)
newstr = str.strip('is')
print(newstr)
集合= {'str','you','point'}
lens = len(集合)
print(f'{lens}')

def print1():
   return 1,2
x,y = print1()
print(f'{x},{y}')


def user (*args):
   print(args)

user('tom')

with open(r'F:\python-pratice\change_first_letter.py', 'r', encoding='UTF-8') as f:
    p = f.read()
    print(f'读取到的内容: [{p}]')

new = p.replace('hello world', '')  # 替换 hello world 为空
with open(r'F:\python-pratice\change_first_letter.py', 'w', encoding='UTF-8') as f:
    f.write(new) """

class student :
    red = 111

    def __init__(self,name,age):
        self.name = name
        self.age = age
        

    def __str__ (self) :#python对缩进及其敏感，不可以像c语言那样很随意的写函数定义
      return f"{self.name},{self.age}"
    
try:
   print(student.red)
except Exception as e:
   print('打印错误')
finally:
   print('测试结束')
            
       
stud = student("中介",11)
print(stud)
print(str(stud))

class stu(student):
   red  =  222

s = stu("乐乐",22)
try:
   print(stu.red)
except Exception as e:
   print('打印错误')
   save_error(str(e))
   view_log()
finally:
   print('测试结束')
print(s)

   
   



